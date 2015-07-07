#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************************/
/* SYNTHESIZED CODE unedited                                             */ 
/*************************************************************************/

/* Include files */

#include "stdio.h"

#include "stdlib.h"

#include "string.h"

#include "netinet/ip.h"

/* Verbatim Text */


#define ip_hdr_size                    sizeof (struct kestrel_ip_hdr)
#define get_packet_at                  (struct kestrel_ip_packet *)
                                       
#define mbuf_hdr_size                  sizeof (struct mbox_buffer_hdr)
                                       
#define Linux_IPC_MBoxInfo_size        sizeof (kestrel_linux_ipc_chan_info *)
#define linux_ipc_chan_info_size       sizeof (kestrel_linux_ipc_chan_info) 
                                       
#define IPC_MailBox_at                 (kestrel_linux_ipc_chan_info *)
#define IPC_MBoxInfoArr_at             (kestrel_linux_ipc_chan_info **)

#define IP_MailBox_at                  (kestrel_linux_ip_chan_info *)
#define IP_MBoxInfoArr_at              (kestrel_linux_ip_chan_info **)

#define at_index(arr,index)            arr[index]   

#define iphdr_saddr(iphdr)             (&(iphdr->saddr))
#define iphdr_daddr(iphdr)             (&(iphdr->daddr))

#define eq_in_addr_t(addr1,addr2)      (addr1 == addr2)
#define default_in_addr_t              0

typedef char *                         charstar     ;  
typedef const char *                   constcharstar;  
typedef void *                         voidstar     ;  
typedef int *                          intstar      ;   


/* Turn structs into types */
typedef struct kestrel_linux_ipc_chan_info kestrel_linux_ipc_chan_info;
typedef struct kestrel_certikosVM_ivc_chan_info kestrel_certikosVM_ivc_chan_info;
typedef struct kestrel_lynxsecureVM_ivc_chan_info kestrel_lynxsecureVM_ivc_chan_info;
typedef struct kestrel_certikos_ipc_chan_info kestrel_certikos_ipc_chan_info;
typedef struct kestrel_linux_ip_chan_info kestrel_linux_ip_chan_info;


/* Mailbox arrays */
typedef kestrel_linux_ipc_chan_info ** Linux_IPC_MBoxInfoArr;
typedef kestrel_linux_ip_chan_info **  Linux_IP_MBoxInfoArr ;

/* This needs to be global until we know how to generate the array typedef */

#define MAX_PACKET_SIZE 1500



/* Definitions */

#define KESTREL_PROTO 137
#define MAX_CERTIKOS_VMS 20
#define MAX_PACKET_READS 5
/* Structs/Unions/Types */

typedef struct kestrel_ip_packet* IPpacket;

typedef struct mbox_buffer* MBuf;

typedef uint32_t radl_channel_id;

typedef struct iphdr* IPhdr;

struct radl_channel {
  radl_channel_id id;
  uint32_t data_size;
  voidstar data_init_value;
};

struct radl_lynxsecureVM_ivc_channel {
  struct radl_channel gen_chan;
  constcharstar region_name;
};

struct kestrel_lynxsecureVM_ivc_chan_info {
  struct radl_lynxsecureVM_ivc_channel chan;
  MBuf buffer;
  uint64_t buffer_size;
  uint64_t region_offset;
};

typedef struct kestrel_lynxsecureVM_ivc_chan_info* LynxsecureVM_IVC_MBoxInfo;

struct radl_linux_ip_channel {
  struct radl_channel gen_chan;
  constcharstar my_addr;
  constcharstar other_addr;
};

struct kestrel_linux_ip_chan_info {
  struct radl_linux_ip_channel chan;
  MBuf buffer;
  uint64_t buffer_size;
};

typedef struct kestrel_linux_ip_chan_info* Linux_IP_MBoxInfo;

struct radl_linux_ipc_channel {
  struct radl_channel gen_chan;
};

struct kestrel_linux_ipc_chan_info {
  struct radl_linux_ipc_channel chan;
  MBuf buffer;
  uint64_t buffer_size;
};

typedef struct kestrel_linux_ipc_chan_info* Linux_IPC_MBoxInfo;

typedef uint32_t radl_vm_id;

struct radl_certikosVM_ivc_channel {
  struct radl_channel gen_chan;
  radl_vm_id my_vm;
  radl_vm_id other_vm;
};

struct kestrel_certikosVM_ivc_chan_info {
  struct radl_certikosVM_ivc_channel chan;
  MBuf buffer;
  uint64_t buffer_size;
  uint64_t region_offset;
};

typedef struct kestrel_certikosVM_ivc_chan_info* CertikosVM_IVC_MBoxInfo;

/* Variables */

extern uint32_t num_ip_chans;

extern Linux_IP_MBoxInfoArr ip_chans;

/* Functions */

int32_t lookup_id_in_ipchans_starting_at (radl_channel_id, int32_t);

void flush_incoming_IP_step (int, IPpacket);

void kestrel_linux_ip_start_init (void);

Linux_IP_MBoxInfo kestrel_linux_ip_init_chan (struct radl_linux_ip_channel);

void kestrel_linux_ip_finish_init (void);

void kestrel_lynxsecureVM_ivc_start_init (void);

LynxsecureVM_IVC_MBoxInfo kestrel_lynxsecureVM_ivc_init_chan 
                         (struct radl_lynxsecureVM_ivc_channel);

void kestrel_lynxsecureVM_ivc_finish_init (void);

void kestrel_certikosVM_ivc_start_init (void);

CertikosVM_IVC_MBoxInfo kestrel_certikosVM_ivc_init_chan 
                       (struct radl_certikosVM_ivc_channel);

void kestrel_certikosVM_ivc_finish_init (void);

void flush_incoming_IP (void);

constcharstar kestrel_linux_ipc_update (Linux_IPC_MBoxInfo, intstar);

charstar kestrel_linux_ipc_stall (Linux_IPC_MBoxInfo);

void kestrel_linux_ipc_release (Linux_IPC_MBoxInfo);

void kestrel_linux_ipc_commit (Linux_IPC_MBoxInfo);

constcharstar kestrel_linux_ip_update (Linux_IP_MBoxInfo, intstar);

charstar kestrel_linux_ip_stall (Linux_IP_MBoxInfo);

void kestrel_linux_ip_release (Linux_IP_MBoxInfo);

void kestrel_linux_ip_commit (Linux_IP_MBoxInfo);

constcharstar kestrel_lynxsecureVM_ivc_update 
             (LynxsecureVM_IVC_MBoxInfo, intstar);

charstar kestrel_lynxsecureVM_ivc_stall (LynxsecureVM_IVC_MBoxInfo);

void kestrel_lynxsecureVM_ivc_release (LynxsecureVM_IVC_MBoxInfo);

void kestrel_lynxsecureVM_ivc_commit (LynxsecureVM_IVC_MBoxInfo);

constcharstar kestrel_certikosVM_ivc_update (CertikosVM_IVC_MBoxInfo, intstar);

charstar kestrel_certikosVM_ivc_stall (CertikosVM_IVC_MBoxInfo);

void kestrel_certikosVM_ivc_release (CertikosVM_IVC_MBoxInfo);

void kestrel_certikosVM_ivc_commit (CertikosVM_IVC_MBoxInfo);




/*************************************************************************/
/* END SYNTHESIZED CODE unedited                                         */ 
/*************************************************************************/


/* FIXME: these declarations need to be here, but are no longer generated...? */
void kestrel_linux_ipc_start_init (void);
Linux_IPC_MBoxInfo kestrel_linux_ipc_init_chan (struct radl_linux_ipc_channel);
void kestrel_linux_ipc_finish_init (void);

#ifdef __cplusplus
}
#endif
