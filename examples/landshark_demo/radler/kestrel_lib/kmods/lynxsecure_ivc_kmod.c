/*
 * Linux kernel module to set up LynxSecure shared-memory IVC.
 *
 * This kernel module creates the directory /proc/lynxsecure_ivc/
 * which will contain one file, XXX, for each named memory region XXX.
 * These files can be directly read or written via the usual syscalls,
 * and can also be mmap'ed into user-space in order to gain access to
 * the physical memory associated with it.
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

#include "lynxsecure_api.h"


/* Standard kernel module information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eddy Westbrook");


/***
 *** Global variables
 ***/

/* The LynxSecure "read-only page" that gives info to the guest OS */
struct ls_ro_page *lsk_ro;

/* Our own internal description of LynxSecure shared-memory buffers */
typedef struct lynxsec_buffer {
  ls_obj_t *ls_obj; /* Contains the name and index of the buffer */
  mem_map_t *mem_map; /* Contains the physical address and size */
  void *buf_ptr; /* Kernel-space pointer to buffer */
} lynxsec_buffer_t;

/* An array of lynxsec_buffer_t's */
lynxsec_buffer_t *lynxsec_buffers = NULL;

/* Number of allocated buffers */
int num_lynxsec_buffers = 0;


/***
 *** LynxSecure-specific code
 ***/

/* (this code courtesy of the LynxSecure people) */
// Lookup a memory map by number
mem_map_t *lookup_memory_map(int idx)
{
  int i;
  mem_map_t *mmp, *lsk_mem_map;

  lsk_mem_map = (mem_map_t *)(lsk_ro->mem_maps.offset + (unsigned long)lsk_ro);

  for (i = 0; i < lsk_ro->mem_maps.count; i++) {
    mmp = &lsk_mem_map[i];
    if (mmp->idx == idx)
      return mmp;
  }

  return NULL;
}


/* Whether there is a valid ith buffer */
#define BUFFER_VALID_P(i) (lsk_ro != NULL && i < num_lynxsec_buffers && \
                           lynxsec_buffers[i].mem_map != NULL && lynxsec_buffers[i].ls_obj != NULL && \
                           lynxsec_buffers[i].buf_ptr != NULL)

/* Get the ith buffer name */
#define BUFFER_NAME(i) lynxsec_buffers[i].ls_obj->mem_descr_name

/* Get the ith buffer virtual address */
#define BUFFER_PTR(i) lynxsec_buffers[i].buf_ptr

/* Get the ith buffer physical address */
#define BUFFER_PHYS_ADDR(i) lynxsec_buffers[i].mem_map->addr

/* Get the ith buffer size */
#define BUFFER_SIZE(i) lynxsec_buffers[i].mem_map->size


/***
 *** Code to manage the procfs interface
 ***/

/* Function to read the LynxSecure IVC buffer via a read syscall. */
ssize_t lynxsecure_procfile_read (struct file *f, char __user *u_buf, size_t size, loff_t *offset) {
  unsigned long i = (unsigned long)PDE_DATA(f->f_dentry->d_inode);
  int read_size;

  if (! BUFFER_VALID_P(i)) {
    return -ENXIO;
  }

  /* Compute the size we will actually read */
  read_size = size > BUFFER_SIZE(i) - *offset ? BUFFER_SIZE(i) - *offset : size;

  /* Copy from the IVC buffer to the user-space pointer u_buf. The copy_to_user
     function returns the number of bytes that were not transferred; if this is not
     0, there was some memory error, so return EFAULT. */
  if (copy_to_user (u_buf, BUFFER_PTR(i) + *offset, read_size))
    return -EFAULT;

  /* Update the offset, and return the number of bytes read */
  *offset += read_size;
  return read_size;
}

/* Function to write to the LynxSecure IVC buffer via a write syscall. */
ssize_t lynxsecure_procfile_write (struct file *f, const char __user *u_buf, size_t size, loff_t *offset) {
  unsigned long i = (unsigned long)PDE_DATA(f->f_dentry->d_inode);
  int write_size;

  if (! BUFFER_VALID_P(i)) {
    return -ENXIO;
  }

  /* Compute the size we will actually write */
  write_size = size > BUFFER_SIZE(i) - *offset ? BUFFER_SIZE(i) - *offset : size;

  /* If the offset is greater than or equal to the size of the buffer, return an error */
  if (write_size == 0) {
    return -EINVAL;
  }

  /* Copy to the IVC buffer from the user-space pointer u_buf */
  if (copy_from_user (BUFFER_PTR(i) + *offset, u_buf, write_size) != 0) {
    return -EFAULT;
  }

  /* Update the offset, and return the number of bytes written */
  *offset += write_size;
  return write_size;
}

/* Function to map an IVC buffer into user address space */
int lynxsecure_procfile_mmap (struct file *f, struct vm_area_struct *vma) {
  unsigned long i = (unsigned long)PDE_DATA(f->f_dentry->d_inode);
  unsigned long phys_addr, page_frame;

  if (! BUFFER_VALID_P(i)) {
    return -ENXIO;
  }

  phys_addr = BUFFER_PHYS_ADDR(i);
  page_frame = (phys_addr >> PAGE_SHIFT) + vma->vm_pgoff;
  return remap_pfn_range(vma, vma->vm_start, page_frame,
                         vma->vm_end - vma->vm_start,
                         vma->vm_page_prot);
}

/* The lynxsecure_ivc directory entry in /proc */
static struct proc_dir_entry *ivc_proc_dir;

/* The name of the lynxsecure_ivc directory entry in /proc */
const char *ivc_proc_dir_name = "lynxsecure_ivc";

static struct file_operations vm_proc_ops = {
  .read = lynxsecure_procfile_read,
  .write = lynxsecure_procfile_write,
  .mmap = lynxsecure_procfile_mmap
};


/***
 *** Helper functions
 ***/

/* Clean up and remove the Lynxsecure shared-memory buffers */
static void lynxsecure_ivc_cleanup(void) {
  int i;

  printk (LOGLEVEL "LynxSecure IVC module cleanup starting...\n");

  /* First unregister our procfs entries */
  if (ivc_proc_dir != NULL) {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
    /* remove_proc_subtree is only supported starting at version 3.9 */
    remove_proc_subtree (ivc_proc_dir_name, NULL);
#else
    /* remove each individual procfs file, and then remove the parent directory */
    for (i = 0; i < num_lynxsec_buffers; ++i) {
      remove_proc_entry (lynxsec_buffers[i].ls_obj->mem_descr_name, ivc_proc_dir);
    }
    remove_proc_entry (ivc_proc_dir_name, NULL);
#endif /* LINUX_VERSION_CODE */
  }

  /* Then unmap our LynxSecure IVC buffers from the kernel address space */
  for (i=0; i < num_lynxsec_buffers; ++i) {
    if (BUFFER_VALID_P(i)) {
      printk (LOGLEVEL "LynxSecure IVC module: deallocating buffer %s\n", BUFFER_NAME(i));

      iounmap (BUFFER_PTR(i));
      BUFFER_PTR(i) = NULL;
    }
  }

  /* Finally, unmap the LynxSecure RO page */
  if (lsk_ro != NULL) {
    iounmap (lsk_ro);
  }

  printk (LOGLEVEL "LynxSecure IVC module cleanup finished\n");
}


/***
 *** Top-level entrypoints
 ***/

/* Initialize Lynxsecure shared-memory IVC */
static int __init lynxsecure_ivc_init(void) {
  int i, err = 0;
  ropage_info_t info;
  ls_obj_t *lsk_objects;

  printk (LOGLEVEL "LynxSecure IVC module starting up...\n");

  /* Initialize the globals to a default state, just in case */
  ivc_proc_dir = NULL;
  lsk_ro = NULL;
  lynxsec_buffers = NULL;
  num_lynxsec_buffers = 0;

  /* Map the LynxSecure RO page into memory */
  if (VMCALL(HVCALL_GET_ROPAGE_INFO, (unsigned long)&info) != LYNXSK_SUCCESS)
    return -EPERM;
  lsk_ro = ioremap(info.paddr, info.size);

  printk (LOGLEVEL
          "LynxSecure IVC module: found %i memory maps and %i ls_objects\n",
          lsk_ro->mem_maps.count, lsk_ro->name_object_maps.count);

  /* Read the relevant info from the RO page, and cache it in our own data structures */
  lsk_objects = (ls_obj_t *)(lsk_ro->name_object_maps.offset + (unsigned long)lsk_ro);
  num_lynxsec_buffers = lsk_ro->name_object_maps.count;
  lynxsec_buffers = kmalloc (num_lynxsec_buffers * sizeof (lynxsec_buffer_t), GFP_KERNEL);
  for (i = 0; i < num_lynxsec_buffers; ++i) {
    lynxsec_buffers[i].ls_obj = &lsk_objects[i];
    lynxsec_buffers[i].buf_ptr = NULL;
    lynxsec_buffers[i].mem_map = lookup_memory_map (lsk_objects[i].index);
    if (lynxsec_buffers[i].mem_map == NULL) {
      printk (LOGLEVEL "LynxSecure IVC module: memory object %s has no corresponding memory map, skipping it\n",
              BUFFER_NAME(i));
    }
  }

  /* Report errors from previous loop */
  if (err != 0) {
    lynxsecure_ivc_cleanup ();
    return err;
  }

  /* Map the shared memory IVC buffers into kernel address space */
  for (i = 0; i < num_lynxsec_buffers; ++i) {
    /* Skip buffers that have no memory maps, or have 0 physical
       address, or whose names do not start with "SHMEM" */
    if (lynxsec_buffers[i].mem_map == NULL ||
        lynxsec_buffers[i].mem_map->addr == 0 ||
        strncmp (BUFFER_NAME(i), "SHMEM", 5) != 0) {

      /* Don't try to get the physical address if there is no mem_map */
      if (lynxsec_buffers[i].mem_map == NULL) {
        printk (LOGLEVEL "LynxSecure IVC module: skipping memory object %s with no memory mapping\n",
                BUFFER_NAME(i));
      } else {
        printk (LOGLEVEL "LynxSecure IVC module: skipping memory object %s at physical address %llx\n",
                BUFFER_NAME(i),
                BUFFER_PHYS_ADDR(i));
      }
      continue;
    }

    /* Try to map the physical address into virtual address space */
    BUFFER_PTR(i) = ioremap (BUFFER_PHYS_ADDR(i), BUFFER_SIZE(i));

    if (BUFFER_PTR(i) == NULL) {
      printk (KERN_ERR "LynxSecure IVC module: could not map IVC buffer %s from physical address %llx, skipping\n",
              BUFFER_NAME(i), BUFFER_PHYS_ADDR(i));
      continue;
    }

    /* Print a status message */
    printk (LOGLEVEL "LynxSecure IVC module: mapped physical address %llx for buffer %s to virtual address %lx\n",
            BUFFER_PHYS_ADDR(i), BUFFER_NAME(i), (unsigned long) BUFFER_PTR(i));
  }

  /* For debugging: write "LynxSecure" to the beginning of the buffer */
  /*
#if DEBUG
    memcpy (BUFFER_PTR(i), "LynxSecure", 10);
#endif
  */

  /* create the /proc/lynxsecure_ivc directory */
  ivc_proc_dir = proc_mkdir (ivc_proc_dir_name, NULL);
  if (ivc_proc_dir == NULL) {
    printk (KERN_ERR "LynxSecure IVC: could not create /proc/%s\n",
            ivc_proc_dir_name);
    lynxsecure_ivc_cleanup ();
    return -EIO;
  }

  for (i = 0; i < num_lynxsec_buffers; ++i) {
    struct proc_dir_entry *proc_file;

    /* Skip buffers that have no memory maps */
    if (! BUFFER_VALID_P(i)) continue;

    proc_file = proc_create_data (BUFFER_NAME(i), 0700,
                                  ivc_proc_dir, &vm_proc_ops,
                                  (void*)(unsigned long)(i));
    if (proc_file == NULL) {
      printk (KERN_ERR "LynxSecure IVC: could not create /proc/%s/%s",
              ivc_proc_dir_name, BUFFER_NAME(i));
      lynxsecure_ivc_cleanup ();
      return -EIO;
    }

    proc_set_size (proc_file, BUFFER_SIZE(i));
  }

  return 0;
}


/* The entry point for module removal registered with the Linux
 * kernel; just calls lynxsecure_ivc_cleanup */
static void __exit lynxsecure_ivc_exit(void) {
  lynxsecure_ivc_cleanup ();
}

/* macros to declare the init and exit functions */
module_init(lynxsecure_ivc_init);
module_exit(lynxsecure_ivc_exit);
