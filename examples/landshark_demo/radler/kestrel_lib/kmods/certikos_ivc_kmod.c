/*
 * Linux kernel module to set up Certikos shared-memory IVC.
 *
 * This kernel module should be inserted into the kernel by issuing the command
 *
 * insmod certikos_ivc_kmod.ko my_vm_num=<N> buffer_sizes=size1,size2,...,sizeM
 *
 * where N is the virtual machine number (starting at 1) of the current VM, and
 * size1, size2, etc. are the sizes of the buffers that should be
 * allocated. Note that if my_vm_num=N, then sizeN is skipped, and should be 0.
 *
 * NOTE: the max number M of sizes that can be passed on the command line is
 * bounded by the constant MAX_OTHER_VMS defined below.
 *
 * This kernel module then creates the directory /proc/certikos_ivc/
 * which will contain files vm1, vm2, etc., with the exception of the
 * number N of the current VM passed as the argument my_vm_num. Each
 * file /proc/certikos_ivc/vmi can be mmap'ed into user-space in order
 * to gain access to the physical memory associated with the IVC
 * buffer for VM i.
 */

/***
 *** Compile-time options
 ***/

/* Debugging flag: setting this turns on debugging information */
#define DEBUG 1

/* The priority to use with printk for debugging messages: high priority for
   debugging (so the messages always appear in the syslog) and low priority for
   non-debugging builds. */
#if DEBUG
#define LOGLEVEL KERN_ALERT
#else
#define LOGLEVEL KERN_DEBUG
#endif /* DEBUG */

/* The max number of other VMs we can communicate with. There is no theoretical
 * limit on this number; it just needs to be known at compile time, because of the
 * way kernel module parameters work. */
#define MAX_OTHER_VMS 7

/* Whether mmap should acquire the memory semaphore */
//#define MMAP_USE_SEMAPHORE 1


/***
 *** Include files
 ***/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/version.h>
#include <linux/slab.h>

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <asm/uaccess.h>

/* Tell hypercall_vmx.h not to define the uint32_t type, as it is
 * defined in the kernel module includes above */
#define UINT32_DEFINED
//#include "hypercall_vmx.h"
#include "certikos_ivc_kmod.h"

/* Standard kernel module information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eddy Westbrook");


/***
 *** Compatibility code for different versions of Linux
 ***/

/* Define the PDE_DATA macro for older Linux kernel versions */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#define PDE_DATA(inode) (PDE(inode)->data)
#endif /* LINUX_VERSION_CODE */

/* Define the proc_set_size function for older Linux kernel versions */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
void proc_set_size(struct proc_dir_entry *de, loff_t size) {
  de->size = size;
}
#endif /* LINUX_VERSION_CODE */



/***
 *** Code to set up command-line parameters for this kernel module.
 ***/

static int my_vm_num = -1;
module_param(my_vm_num, int, 0000);
MODULE_PARM_DESC(my_vm_num, "The Certikos VM number of the current VM");

static unsigned long buffer_sizes[MAX_OTHER_VMS];
static int num_buffer_sizes = -1;
module_param_array(buffer_sizes, ulong, &num_buffer_sizes, 0000);
MODULE_PARM_DESC(buffer_sizes, "A list of the sizes of the shared memory IVC buffers for communicating with other Certikos VMs");

static int disable_hypercalls = 0;
module_param(disable_hypercalls, int, 0000);
MODULE_PARM_DESC(disable_hypercalls, "A flag which, when set, disables the actual CertiKOS hypercalls made by this module");


/***
 *** Global variables
 ***/

/* Info about a CertiKOS buffer */
typedef struct certikos_buf {
  uint32_t vm_num; /* The VM number of the other side */
  void *virt_addr; /* The kernel-space virtual address of the buffer */
  unsigned long phys_addr; /* The physical address of the buffer */
  unsigned long size; /* The size of the buffer */
  shared_mem_state state; /* The last status result seen from CertiKOS about this buffer */
  //shared_mem_ret_value offer_err; /* Error returned from offering the buffer */
} certikos_buf_t;

certikos_buf_t *certikos_buffers = NULL;
unsigned int num_certikos_buffers = 0;


/***
 *** Buffer-related functions
 ***/

/* Initialize a buffer into a recognizable "not yet set up" state */
void certikos_buffer_init (certikos_buf_t *buffer) {
  buffer->vm_num = 0;
  buffer->virt_addr = NULL;
  buffer->phys_addr = 0;
  buffer->state = SHARED_MEM_DEAD;
}

/* Offer a CertiKOS buffer to the other VM */
void certikos_buffer_offer (certikos_buf_t *buffer) {
  /* If the physical address is 0, this buffer is in a "not yet set
     up" state, leave it as "dead" */
  if (! disable_hypercalls && buffer->phys_addr != 0) {
    shared_mem_ret_value ret =
      hyp_offer_shared_memory
      (hyp_vmname_to_pid (buffer->vm_num),
       (void*)buffer->phys_addr, buffer->size);
    if ((shared_mem_state)ret == SHARED_MEM_READY
	|| (shared_mem_state)ret == SHARED_MEM_PENDING) {
      buffer->state = ret;
    } else {
      printk (KERN_ERR "CertiKOS IVC: Got error %u in offer to VM %u (phys_addr = %lx, size = %lu)",
              ret, buffer->vm_num, buffer->phys_addr, buffer->size);
      buffer->state = SHARED_MEM_DEAD;
    }
  }
}

/* Get the status of the given buffer, updating the cached status
   value if the last seen status was "pending" or re-offering the
   buffer to the other VM if the buffer is currently "dead" */
shared_mem_state certikos_buffer_status (certikos_buf_t *buffer) {
  /* If the last seen status was "pending", get the status from CertiKOS again */
  if (buffer->state == SHARED_MEM_PENDING && ! disable_hypercalls) {
    buffer->state = hyp_shared_memory_status (hyp_vmname_to_pid (buffer->vm_num));
  }

  /* If the last seen status (possibly from the call above) was
     "dead", offer the buffer again */
  if (buffer->state == SHARED_MEM_DEAD && ! disable_hypercalls) {
    certikos_buffer_offer (buffer);
  }

  return buffer->state;
}


/***
 *** Code to manage the procfs interface
 ***/

/* Read the CertiKOS IVC buffer via a read syscall. */
ssize_t certikos_procfile_read (struct file *f, char __user *u_buf, size_t size, loff_t *offset) {
  certikos_buf_t *buffer = (certikos_buf_t*)PDE_DATA(f->f_dentry->d_inode);
  int read_size = size > buffer->size - *offset
    ? buffer->size - *offset
    : size;

  /* Update the state on reads and writes */
  certikos_buffer_status (buffer);

  /* If the offset is greater than or equal to the size of the buffer, return an error */
  if (read_size <= 0) {
    return -EINVAL;
  }

  /* Copy from the IVC buffer to the user-space pointer u_buf. The copy_to_user
     function returns the number of bytes that were not transferred; if this is not
     0, there was some memory error, so return EFAULT. */
  if (copy_to_user (u_buf, buffer->virt_addr + *offset, read_size))
    return -EFAULT;

  /* Update the offset, and return the number of bytes read */
  *offset += read_size;
  return read_size;
}

/* Write to the CertiKOS IVC buffer via a write syscall. */
ssize_t certikos_procfile_write (struct file *f, const char __user *u_buf, size_t size, loff_t *offset) {
  certikos_buf_t *buffer = (certikos_buf_t*)PDE_DATA(f->f_dentry->d_inode);
  int write_size = size > buffer->size - *offset
    ? buffer->size - *offset
    : size;

  /* Update the state on reads and writes */
  certikos_buffer_status (buffer);

  /* If the offset is greater than or equal to the size of the buffer, return an error */
  if (write_size <= 0) {
    return -EINVAL;
  }

  /* Copy to the IVC buffer from the user-space pointer u_buf */
  if (copy_from_user (buffer->virt_addr + *offset, u_buf, write_size) != 0) {
    return -EFAULT;
  }

  /* Update the offset, and return the number of bytes written */
  *offset += write_size;
  return write_size;
}

/* Map an IVC buffer into user address space */
int certikos_procfile_mmap (struct file *f, struct vm_area_struct *vma) {
  certikos_buf_t *buffer = (certikos_buf_t*)PDE_DATA(f->f_dentry->d_inode);
  unsigned long page_frame = (buffer->phys_addr >> PAGE_SHIFT) + vma->vm_pgoff;

  /*
  printk (LOGLEVEL "certikos_procfile_mmap: pgoff = %lu, vm_start = %lu, vm_end = %lu, size = %lu\n",
	  vma->vm_pgoff, vma->vm_start, vma->vm_end, buffer->size);
  */

  /* Raise an error if attempting to map beyond the end of the memory region */
  if ((vma->vm_pgoff << PAGE_SHIFT) + vma->vm_end - vma->vm_start > buffer->size) {
    return -EINVAL;
  } else {
#if MMAP_USE_SEMAPHORE
    down_write (&vma->vm_mm->mmap_sem);
#endif
    return remap_pfn_range(vma, vma->vm_start, page_frame,
                           vma->vm_end - vma->vm_start,
                           vma->vm_page_prot);
#if MMAP_USE_SEMAPHORE
    up_write (&vma->vm_mm->mmap_sem);
#endif
  }
}

/* ioctl inferface, for querying buffer status */
long certikos_procfile_ioctl (struct file *f, unsigned int num, unsigned long param) {
  certikos_buf_t *buffer = (certikos_buf_t*)PDE_DATA(f->f_dentry->d_inode);
  shared_mem_state state;

  switch (num) {
  case CERTIKOS_IOCTL_STATUS:
    state = certikos_buffer_status (buffer);
    if (copy_to_user ((shared_mem_state*)param, &state, sizeof (shared_mem_state)) != 0) {
      return -EFAULT;
    }
    return 0;

  default:
    return -EINVAL;
  }
}

/* The certikos_ivc directory entry in /proc */
static struct proc_dir_entry *certikos_proc_dir = NULL;

/* The name of the certikos_ivc directory entry in /proc */
const char *certikos_proc_dir_name = "certikos_ivc";

static struct file_operations vm_proc_ops = {
  .read = certikos_procfile_read,
  .write = certikos_procfile_write,
  .mmap = certikos_procfile_mmap,
  .unlocked_ioctl = certikos_procfile_ioctl
};


/***
 *** Helper functions
 ***/

/* Get the log base 2 of an integeger, rounded up, by repeated bit
 * shifting (with the corner case int_log2(0)=0) */
unsigned long int_log2 (unsigned long x) {
  unsigned long ret = 0;
  while (x >>= 1) { ++ret; }
  return ret + 1;
}


/* Clean up and remove the Certikos shared-memory buffers */
static void certikos_ivc_cleanup(void) {
  int i;

  printk (LOGLEVEL "CertiKOS IVC module cleanup starting...\n");

  /* First unregister our procfs entries */
  if (certikos_proc_dir != NULL) {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
    /* remove_proc_subtree is only supported starting at version 3.9 */
    remove_proc_subtree (certikos_proc_dir_name, NULL);
#else
    /* remove each individual vm<N> file, and then remove the parent directory */
    for (i = 0; i < num_certikos_buffers; ++i) {
      certikos_buf_t *buffer = &certikos_buffers[i];
      char filename [5];

      sprintf (filename, "vm%i", buffer->vm_num);
      remove_proc_entry (filename, certikos_proc_dir);
    }
    remove_proc_entry (certikos_proc_dir_name, NULL);
#endif /* LINUX_VERSION_CODE */
  }

  /* Unregister our CertiKOS IVC buffers and free the associated kernel pages, if possible */
  for (i=0; i < num_certikos_buffers; ++i) {
    certikos_buf_t *buffer = &certikos_buffers[i];

    if (buffer->virt_addr != NULL) {
      shared_mem_state buffer_state = certikos_buffer_status (buffer);
      int dealloc_buffer;

      /* Determine whether we should deallocate this buffer */
      if (! disable_hypercalls) {
        /* Try to unmap the CertiKOS shared memory buffer */
        //hypercall_rm_sm_ivc (other_vm);

        /* Deallocate the buffer only if it is "dead" according to CertiKOS */
        if (buffer_state == SHARED_MEM_DEAD) {
          dealloc_buffer = 1;
        } else {
          dealloc_buffer = 0;
        }
      } else {
        /* If there is no hypervisor, then we can safely delete all buffers */
        dealloc_buffer = 1;
      }

      /* Deallocate the buffer */
      if (dealloc_buffer) {
        unsigned long order;

        printk (LOGLEVEL "CertiKOS IVC module: deallocating buffer for VM %i\n", buffer->vm_num);

        order = int_log2 (buffer_sizes[i] / PAGE_SIZE);
        free_pages ((unsigned long)buffer->virt_addr, order);

        buffer->virt_addr = NULL;
        buffer->phys_addr = 0;
      }
      else {
        printk (LOGLEVEL "CertiKOS IVC module: buffer for VM %i still in use, not deallocating\n",
                buffer->vm_num);
      }
    }
  }

  /* Deallocate the certikos_buffers array */
  kfree (certikos_buffers);
  certikos_buffers = NULL;
  num_certikos_buffers = 0;

  printk (LOGLEVEL "CertiKOS IVC module cleanup finished\n");
}


/***
 *** Top-level entrypoints
 ***/

/* Initialize Certikos shared-memory IVC */
static int __init certikos_ivc_init(void) {
  int i;
  uint32_t vm_num;

  printk (LOGLEVEL "CertiKOS IVC module starting up...\n");

  /* Count the number of CertiKOS buffers we are creating: it is the
     number of buffer sizes given on the command line minus 1, unless
     my_vm_num falls outside [1,num_buffer_sizes] */
  num_certikos_buffers =
    (1 <= my_vm_num && my_vm_num <= num_buffer_sizes) ?
      num_buffer_sizes - 1 : num_buffer_sizes;

  /* Allocate and initialize the certikos_buf structs */
  certikos_buffers = (certikos_buf_t *) kmalloc (num_certikos_buffers * sizeof (certikos_buf_t), GFP_KERNEL);
  for (i = 0, vm_num = 1; i < num_certikos_buffers; ++vm_num) {
    certikos_buf_t *buffer;

    if (vm_num == my_vm_num) continue; /* skip myself */

    buffer = &certikos_buffers[i];
    certikos_buffer_init (buffer);
    buffer->vm_num = vm_num;

    /* subtract 1 here to go from VM numbers, which start at 1, to
       array indices, which start at 0 */
    buffer->size = buffer_sizes[vm_num - 1];

    /* Only increment i at the end of an iteration, when there is no skipping */
    i++;
  }


  /* Allocate the IVC buffers and offer them to the other VMs */
  for (i = 0; i < num_certikos_buffers; ++i) {
    certikos_buf_t *buffer = &certikos_buffers[i];
    unsigned long order;

    /* Allocate physical pages in Linux for the IVC buffer */
    order = int_log2 (buffer->size / PAGE_SIZE);
    buffer->virt_addr = (void*)__get_free_pages (GFP_KERNEL, order);
    if (buffer->virt_addr == 0) {
      printk (KERN_ERR "CertiKOS IVC: Could not allocate buffer for VM %i\n", buffer->vm_num);
      certikos_ivc_cleanup ();
      return -1;
    }

    /* Store the physical address of the buffer */
    buffer->phys_addr = virt_to_phys(buffer->virt_addr);
    printk (LOGLEVEL "CertiKOS IVC module: allocated physical address %lx for VM %i\n",
            buffer->phys_addr, buffer->vm_num);

    /* For debugging: write "CertiKOS" to the beginning of the buffer */
#if DEBUG
    memcpy (buffer->virt_addr, "CertiKOS", 8);
#endif

    /* Offer the IVC shared memory buffer with CertiKOS */
    certikos_buffer_offer (buffer);
  }

  /* create the /proc/certikos_ivc directory */
  certikos_proc_dir = proc_mkdir (certikos_proc_dir_name, NULL);
  if (certikos_proc_dir == NULL) {
    printk (KERN_ERR "CertiKOS IVC: could not create /proc/%s\n",
            certikos_proc_dir_name);
    certikos_ivc_cleanup ();
    return -1;
  }

  for (i = 0; i < num_certikos_buffers; ++i) {
    certikos_buf_t *buffer = &certikos_buffers[i];
    struct proc_dir_entry *proc_file;
    char filename[10];

    sprintf (filename, "vm%i", buffer->vm_num);
    proc_file = proc_create_data (filename, 0700,
                                certikos_proc_dir, &vm_proc_ops,
                                (void*)buffer);
    if (proc_file == NULL) {
      printk (KERN_ERR "CertiKOS IVC: could not create /proc/%s/%s\n",
              certikos_proc_dir_name, filename);
      certikos_ivc_cleanup ();
      return -1;
    }
    proc_set_size (proc_file, buffer->size);
  }

  return 0;
}


/* The entry point for module removal registered with the Linux
 * kernel; just calls certikos_ivc_cleanup */
static void __exit certikos_ivc_exit(void) {
  certikos_ivc_cleanup ();
}

/* macros to declare the init and exit functions */
module_init(certikos_ivc_init);
module_exit(certikos_ivc_exit);
