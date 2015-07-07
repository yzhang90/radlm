/***
 *** Hepler functions called by Kestrel code that are *not* synthesized
 ***/

#include <netinet/ip.h>
#include "kestrel_comm.h"


#ifdef DEBUG
#define PACKET_ERROR(...) fprintf(stderr, __VA_ARGS__)
#else
#define PACKET_ERROR(...) {}
#endif

#define LOG(msg) \
  do { printf("[Kestrel] INFO: " msg "\n"); exit(-1); } while(0)
#define LOGF(fmt, ...) \
  do { printf("[Kestrel] INFO: " fmt "\n", __VA_ARGS__); exit(-1); } while(0)

#define ERROR(msg) \
  do { fprintf(stderr, "[Kestrel] ERROR: " msg "\n"); exit(-1); } while(0)
#define ERRORF(fmt, ...) \
  do { fprintf(stderr, "[Kestrel] ERROR: " fmt "\n", __VA_ARGS__); exit(-1); } while(0)


/* Macos specific compatible fixes. */
#ifdef __MACH__

/* To set the socket to non blocking, wee need fcntl. */
#include <fcntl.h>

/* To allow the use of inet_aton. */
#include <arpa/inet.h>

/* There is no struct iphdr defined in the ip.h header file. */
struct iphdr
  {
#if __BYTE_ORDER == __LITTLE_ENDIAN
    unsigned int ihl:4;
    unsigned int version:4;
#elif __BYTE_ORDER == __BIG_ENDIAN
    unsigned int version:4;
    unsigned int ihl:4;
#else
# error  "Please fix <bits/endian.h>"
#endif
    u_int8_t tos;
    u_int16_t tot_len;
    u_int16_t id;
    u_int16_t frag_off;
    u_int8_t ttl;
    u_int8_t protocol;
    u_int16_t check;
    in_addr_t saddr;
    in_addr_t daddr;
    /*The options start here. */
  };

#define MAP_ANONYMOUS MAP_ANON

#endif //__MACH__



/* The directory containing files used for IVC. This directory
   will contain files vm1, vm2, etc., for all VM numbers we wish to
   communicate with. */
const char *certikos_ivc_dir;

/* The directory containing files used for LynxSecure IVC */
const char *lynxsecure_ivc_dir;


/* The header of a Kestrel IP packet */
struct kestrel_ip_hdr {
  struct iphdr iphdr; /* IP packet header */
  radl_channel_id chan_id; /* Channel id: works like a port number */
};

/* The structure of a Kestrel IP packet */
struct kestrel_ip_packet {
  struct kestrel_ip_hdr hdr;
  char data[];
};


/* Get the page size */
unsigned long get_page_size ();

/* Ensure that the certikos_ivc_kmod is installed */
void ensure_certikos_ivc_kmod (unsigned long *buffer_sizes,
			       unsigned int my_vm_num, unsigned int num_vms);

/* Ensure that the lynxsecure_ivc_kmod is installed */
void ensure_lynxsecure_ivc_kmod ();

/* Mmap size bytes of filename, which represents an IVC buffer. */
void *map_ivc_buffer (char *filename, int size);

/* Map the Certikos IVC buffers into the address space. The array
   buffer_sizes, with size num_vms, gives the sizes of buffers for
   communicating with each other VM; specifically, buffer_sizes[i]
   gives the size of the buffer for communicating with VM i+1 (since
   VMs start with number 1). Buffer sizes should be a multiple of the
   page size. The my_vm_num argument gives the VM number we are
   running on, so no buffer is allocated for that one;
   buffer_sizes[my_vm_num-1] is ignored. Pointers to the
   newly-allocated buffers are stored in the buffers array, which
   should also have size num_vms on entry.
*/
void map_certikos_buffers (unsigned long *buffer_sizes, unsigned int num_vms,
                           unsigned int my_vm_num, void **buffers);

void sync_certikos_buffer (char *filename);

/* Allocate a memory buffer of size bytes that can be shared with
   child processes. The size must be a multiple of the page size. If
   the allocation fails, exit with status -1. */
void *allocate_shared_mem (unsigned long size);

/* Create a raw IP socket, in non-blocking mode with the IP_HDRINCL
   flag set, for communicating using the given protocol number. Return
   the socket number on success, or exit with status -1 on error. */
int create_raw_socket (int protocol);

/* Read the next packet from the given raw socket, if there is one,
   and store it in the memory pointed to by buffer, which should have
   size buffer_size. Return the size of the packet that was read, or 0
   if no packet was available. */
int recv_ip_packet (int socket, struct kestrel_ip_packet *buffer, unsigned long buffer_size);

/* Send a packet on the given raw socket. Exit with status -1 if there
   is an error. */
void send_ip_packet (int socket, struct kestrel_ip_packet *packet);
