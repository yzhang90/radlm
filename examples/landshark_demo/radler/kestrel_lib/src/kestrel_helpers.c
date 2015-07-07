/***
 *** Hepler functions called by Kestrel code that are *not* synthesized
 ***/

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<errno.h>
#include<stdlib.h>
#include<string.h>
#include<fcntl.h>
#include<sys/mman.h>
#include<sys/socket.h>
#include<sys/stat.h>
#include<sys/ioctl.h>
#include<sys/wait.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include "kestrel_helpers.h"
#include "certikos_ivc_kmod.h"


/* Path to the modprobe tool */
char *path_to_modprobe = "/sbin/modprobe";

/* Path to the rmmod tool */
char *path_to_rmmod = "/sbin/rmmod";

/* Name of the CertiKOS kernel module */
char *certikos_kmod_name = "certikos_ivc_kmod";

/* The directory containing files used for IVC. This directory
   will contain files vm1, vm2, etc., for all VM numbers we wish to
   communicate with. */
const char *certikos_ivc_dir = "/proc/certikos_ivc";

/* Name of the LynxSecure kernel module */
char *lynxsecure_kmod_name = "lynxsecure_ivc_kmod";

/* The directory containing files used for LynxSecure IVC */
const char *lynxsecure_ivc_dir = "/proc/lynxsecure_ivc";


/* Turn off the actual CertiKOS hypercalls in the certikos_ivc_kmod kernel module */
#define DISABLE_HYPERCALLS 0

/* Return the page size in the current system */
unsigned long get_page_size () {
  /* Cache the page size in a static variable, which is initialized the first time */
  static unsigned long page_size = -1;
  if (page_size == -1) { page_size = sysconf (_SC_PAGESIZE); }
  return page_size;
}

/* Helper function: exec a program in a child process and wait for it
   to complete, exiting with status -1 on error. The arguments to this
   function give the filename of the program and the UNIX argument
   list (whose first element should be the filename and whose last
   element should be a NULL pointer); the environment used is always
   the empty environment (to avoid any environment-based attacks). If
   the child returns a non-zero exit status and die_on_error is set,
   then we exit too; otherwise, just ignore the error status. */
void exec_child_process (const char *filename, char *argv[], int die_on_error) {
  int status, res;
  char *envp[1] = { NULL };
  pid_t child_pid = vfork ();

  if (child_pid == -1) {
    fprintf (stderr, "Could not fork child process: %s\n",
             strerror (errno));
    exit (-1);
  } else if (child_pid == 0) {
    /* This is the child task; call execve */
    status = execve (filename, argv, envp);
    if (status == -1) {
      fprintf (stderr, "Could not exec %s: %s\n",
               filename, strerror (errno));
      exit (-1);
    }

    /* Should not reach here... */
    fprintf (stderr, "Returned from exec!\n");
    exit (-1);
  } else {
    /* This is the parent thread; wait for child to return, and check
     * the exit status */
    res = waitpid (child_pid, &status, 0);
    if (res == -1) {
      fprintf (stderr, "Error returned by waitpid: %s\n", strerror (errno));
      exit (-1);
    } else if (WIFEXITED (status) && WEXITSTATUS(status) != 0 && die_on_error) {
      fprintf (stderr, "Child process for %s returned non-zero exit status %i\n",
               filename, WEXITSTATUS(status));
      exit (-1);
    } else if (WIFSIGNALED (status)) {
      fprintf (stderr, "Child process for %s killed by signal %i\n",
               filename, WTERMSIG(status));
      exit (-1);
    }
  }
}

/* Run rmmod to remove the certikos_ivc_kmod.ko kernel module */
void remove_certikos_ivc_kmod () {
  char *argv[] = { path_to_rmmod, certikos_kmod_name, NULL };
  printf ("Removing CertiKOS kernel module...\n");
  exec_child_process (path_to_rmmod, argv, 1);
  printf ("Removed CertiKOS kernel module\n");
}

/* Ensure the certikos_ivc_kmod.ko kernel module has been installed */
void ensure_certikos_ivc_kmod (unsigned long *buffer_sizes,
			       unsigned int my_vm_num, unsigned int num_vms) {
  int i, err;
  char arg1[100];
  char arg2[100];
  char *arg2_ptr = &arg2[0];
  char *rmmod_argv[] = { path_to_rmmod, certikos_kmod_name, NULL };
  int first_iter_p = 1;
  struct stat ivc_dir_stat;

  /* Set option disable_hypercalls if DISABLE_HYPERCALLS is set */
#if DISABLE_HYPERCALLS
  char *argv[] = { path_to_modprobe, certikos_kmod_name, arg1, arg2, "disable_hypercalls=1", NULL };
#else
  char *argv[] = { path_to_modprobe, certikos_kmod_name, arg1, arg2, NULL };
#endif

  /* Remove any old instances of the kernel module, if any */
  //exec_child_process (path_to_rmmod, rmmod_argv, 0);

  /* Check if the IVC directory exists, and if so, the kernel module
     is already installed */
  err = stat (certikos_ivc_dir, &ivc_dir_stat);
  if (err == 0 && S_ISDIR(ivc_dir_stat.st_mode)) {
    /* The dir exists and is a directory, so we are done here and return */
    return;
  }
  else if (err == -1 && errno != ENOENT) {
    fprintf (stderr, "Stat error on %s: %s\n",
	     certikos_ivc_dir, strerror (errno));
    exit (-1);
  } else if (err == 0 && ! S_ISDIR(ivc_dir_stat.st_mode)) {
    fprintf (stderr, "IVC directory %s is not a directory!\n",
	     certikos_ivc_dir);
    exit (-1);
  }
  /* If we reach here, certikos_ivc_dir does not exist, so
     install the kernel module */

  /* Set up arguments to the kernel module */
  sprintf (arg1, "my_vm_num=%i", my_vm_num);

  arg2_ptr += sprintf (arg2, "buffer_sizes=");
  for (i = 0; i < num_vms; ++i) {
    /* skip my own VM */
    //if (i+1 == my_vm_num) continue;

    /* print a comma before all but the first buffer length */
    if (!first_iter_p) {
      arg2_ptr += sprintf (arg2_ptr, ",");
    }

    /* now print the buffer length itself */
    arg2_ptr += sprintf (arg2_ptr, "%lu", buffer_sizes[i]);

    first_iter_p = 0;
  }

  printf ("Installing CertiKOS module with these arguments:");
  for (i = 0; argv[i] != NULL; ++i) {
    printf (" %s", argv[i]);
  }
  printf ("\n");

  exec_child_process (path_to_modprobe, argv, 1);

  /* Set up an exit handler to remove the CertiKOS kernel module */
  //atexit (&remove_certikos_ivc_kmod);
}

/* Mmap a file that represents an IVC buffer, returning a pointer to
   the region where the file is mapped and, in size, the total size of
   the buffer. */
void *map_ivc_buffer (char *filename, int size) {
  int fd;
  void *ret;
  struct stat stat;

  /* Open the file */
  fd = open (filename, O_RDWR);
  if (fd == -1) {
    fprintf (stderr, "Could not open %s: %s\n", filename, strerror (errno));
    exit (-1);
  }

  /* Stat the file, to get its size */
  if (fstat (fd, &stat) != 0) {
    fprintf (stderr, "Stat error on %s: %s\n", filename, strerror (errno));
    exit (-1);
  }

  /* Check that the file is big enough */
  if (size > stat.st_size) {
    fprintf (stderr, "IVC buffer %s not big enough: expected %u bytes, found %li bytes\n",
	     filename, size, stat.st_size);
    exit (-1);
  }

  /* Map the file into our address using mmap */
  ret = mmap (NULL, stat.st_size,
	      PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
  if (ret == MAP_FAILED) {
    fprintf (stderr, "mmap failed for IVC buffer %s: %s\n",
	     filename, strerror (errno));
    exit (-1);
  }

  printf ("Mapped IVC buffer %s to local address %lx, using %u/%lu bytes\n",
	  filename, (long unsigned int)ret, size, stat.st_size);

  /* Finally, close the file, and return the pointer and the file size */
  close (fd);
  return ret;
}

/* Wait for a CertiKOS buffer to become active */
void sync_certikos_buffer (char *filename) {
  int fd, err;
  shared_mem_state state;

  /* Open the file associated with the buffer */
  fd = open (filename, O_RDWR);
  if (fd == -1) {
    fprintf (stderr, "Error opening CertiKOS buffer file %s: %s\n",
             filename, strerror (errno));
  }

  /* Keep checking the status, until it is "READY" */
  while (1) {
    err = ioctl (fd, CERTIKOS_IOCTL_STATUS, &state);
    if (err == -1) {
      fprintf (stderr, "ioctl error on file %s: %s\n",
	       filename, strerror (errno));
      exit (-1);
    }

    if (state == SHARED_MEM_READY)
      break;
    else if (state == SHARED_MEM_PENDING)
      continue;
    else if (state == SHARED_MEM_DEAD) {
      fprintf (stderr, "Got a \"dead\" status for CertiKOS buffer %s\n",
	       filename);
      exit (-1);
    }
    else {
      fprintf (stderr, "Unexpected status result for CertiKOS buffer %s: %u\n",
	       filename, state);
      exit (-1);
    }
  }
}


/* mmap all of the buffers from /dev/mem into our address space */
void map_certikos_buffers (unsigned long *buffer_sizes, unsigned int num_vms,
                           unsigned int my_vm_num, void **buffers) {
  int i;
  char filename[100];

  /* First, install the kernel module */
  ensure_certikos_ivc_kmod (buffer_sizes, my_vm_num, num_vms);

  /* Map the buffers using proc_fs */
  printf ("Mapping CertiKOS IVC buffers...\n");

  /* Iterate through all the buffers we are interested in */
  for (i = 0; i < num_vms; ++i) {
    uint32_t vm_num = i+1;

    /* Skip my own VM number */
    if (vm_num == my_vm_num) {
      buffers[i] = 0;
      continue;
    }

    /* Store the proc_fs file name we want to open in filename */
    sprintf (filename, "%s/vm%i", certikos_ivc_dir, vm_num);

    buffers[i] = map_ivc_buffer (filename, buffer_sizes[i]);
  }
}


/* Ensure the LynxSecure kernel module is installed */
void ensure_lynxsecure_ivc_kmod () {
  int err;
  struct stat ivc_dir_stat;
  char *argv[] = { path_to_modprobe, lynxsecure_kmod_name, NULL };

  /* Check if the IVC directory exists, and if so, the kernel module
     is already installed */
  err = stat (lynxsecure_ivc_dir, &ivc_dir_stat);
  if (err == 0 && S_ISDIR(ivc_dir_stat.st_mode)) {
    /* The dir exists and is a directory, so we are done here and return */
    return;
  }
  else if (err == -1 && errno != ENOENT) {
    fprintf (stderr, "Stat error on %s: %s\n",
	     lynxsecure_ivc_dir, strerror (errno));
    exit (-1);
  } else if (err == 0 && ! S_ISDIR(ivc_dir_stat.st_mode)) {
    fprintf (stderr, "IVC directory %s is not a directory!\n",
	     lynxsecure_ivc_dir);
    exit (-1);
  }

  exec_child_process (path_to_modprobe, argv, 1);
}

/* Allocate a memory buffer of size bytes that can be shared with
   child processes. The size must be a multiple of the page size. If
   the allocation fails, exit with status -1. */
void *allocate_shared_mem (unsigned long size) {
  void *res = mmap (NULL, size, PROT_READ|PROT_WRITE,
                    MAP_SHARED|MAP_ANONYMOUS, -1, 0);
  if (res == NULL) {
    fprintf (stderr, "Buffer allocation failed, exiting\n");
    exit (-1);
  }

  return res;
}


/* Create a raw IP socket, in non-blocking mode with the IP_HDRINCL
   flag set, for communicating using the given protocol number. Return
   the socket number on success, or exit with status -1 on error. */
int create_raw_socket (int protocol) {
  int sock, res, opt_val = 1;

  /* Create the raw socket  */
#ifdef __MACH__
  sock = socket(AF_INET, SOCK_RAW, protocol);
  fcntl(sock, F_SETFL, fcntl(sock, F_GETFL) + O_NONBLOCK);
#else
  sock = socket (AF_INET, SOCK_RAW | SOCK_NONBLOCK, protocol);
#endif
  if (sock == -1) {
    fprintf (stderr, "Could not create raw socket for IP communication: %s\n",
             strerror (errno));
    exit (-1);
  }

  /* Set the IP_HDRINCL socket option, so we generate our own IP headers */
  printf ("Setting socket options\n");
  res = setsockopt (sock, IPPROTO_IP, IP_HDRINCL,
                    &opt_val, sizeof(opt_val));
  if (res == -1) {
    fprintf (stderr, "Could not set IP_HDRINCL option on raw IP socket: %s\n",
             strerror (errno));
    exit (-1);
  }

  return sock;
}


/* Read the next packet from the given raw socket, if there is one,
   and store it in the memory pointed to by buffer. buffer_size is a
   read/write parameter, which should have the size of buffer on input
   and will be updated to the number of bytes actually read on
   output. Return 1 if a packet was read, 0 if not. If there is an error,
   exit with status -1. */
int recv_ip_packet (int socket, struct kestrel_ip_packet *buffer, unsigned long buffer_size) {
  int res;
  struct in_addr source_addr;

  res = recv (socket, buffer, buffer_size, MSG_DONTWAIT);
  if (res == -1 && errno == EWOULDBLOCK) {
    return 0;
  } else if (res == -1) {
    fprintf (stderr, "Read error on raw socket: %s\n", strerror (errno));
    exit (-1);
  } else {
    source_addr.s_addr = buffer->hdr.iphdr.saddr;
//    printf ("Received IP packet for topic %u of total size %i from address %s with first long %li\n",
//            buffer->hdr.chan_id, res, inet_ntoa (source_addr),
//	    *(long*)&(buffer->data));
    return res;
  }
}

/* Send a packet on the given raw socket. Exit with status -1 if there
   is an error. */
void send_ip_packet (int socket, struct kestrel_ip_packet *packet) {
  int res;
  struct sockaddr_in daddr;

  /* Need to set up a destination address structure separately from
     the IP packet itself (why?) */
  daddr.sin_family = AF_INET;
  daddr.sin_port = 0;
  daddr.sin_addr.s_addr = packet->hdr.iphdr.daddr;
  //daddr.sin_zero = 0;

  do {
    /*
    printf ("Sending packet with size %u to address %s\n",
	    ntohs (packet->hdr.iphdr.tot_len),
	    inet_ntoa (daddr.sin_addr));
    */
    res = sendto (socket, packet, ntohs (packet->hdr.iphdr.tot_len), 0,
                  (struct sockaddr*)&daddr, sizeof (struct sockaddr_in));
  } while (res == -1 && errno == EWOULDBLOCK);
  if (res == -1) {
    fprintf (stderr, "sendto error in send_ip_packet for topic %u: %s\n",
             packet->hdr.chan_id, strerror (errno));
    //exit (-1);
  }

//  printf ("Sent IP packet for topic %u of total size %i to address %s with first long %li\n",
//          packet->hdr.chan_id, ntohs (packet->hdr.iphdr.tot_len),
//	  inet_ntoa (daddr.sin_addr), *(long*)&(packet->data));
}
