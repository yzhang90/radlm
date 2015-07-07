/* Header file for interfacing with the CertiKOS kernel module */

#include "hypercall_vmx.h"

#ifdef __MACH__
#include <sys/ioctl.h>
#else
#include <linux/ioctl.h>
#endif

/* The "major number" for the CertiKOS kernel module; we like the fine
   structure constant of the universe */
#define CERTIKOS_MAJOR_NUM 137

/* The ioctl number for getting IVC buffer status. The _IOR macro
   means this ioctl call reads info from kernel space to user space,
   the first two arguments uniquely identify this particular ioctl
   call, and the last argument gives the type of data returned back to
   user-space. */
#define CERTIKOS_IOCTL_STATUS _IOR(CERTIKOS_MAJOR_NUM,0,shared_mem_state*)
