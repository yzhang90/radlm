
#ifndef _SYS_SYS_VIRT_HYPERCALL_H_
#define _SYS_SYS_VIRT_HYPERCALL_H_

typedef enum {
        HYPERCALL_BITAND,
        HYPERCALL_BITOR,
        HYPERCALL_BITXOR,
        HYPERCALL_BITNOT,
        HYPERCALL_FIFO_STATUS,
        HYPERCALL_FIFO_SEND,
        HYPERCALL_FIFO_RECV,
        HYPERCALL_MEM_SHARE_FIFO_SEND,
        HYPERCALL_MEM_SHARE_FIFO_RECV,
        HYPERCALL_OFFER_SHARED_MEMORY,
        HYPERCALL_SHARED_MEMORY_STATUS,
        HYPERCALL_VMNAME_TO_PID,
        HYPERCALL_NULL, /* XXX: should be the last one */
} hypercall_t;  

typedef enum{
	NONAME_VM,
	Landsharkvm1,
	Landsharkvm2,
	Landsharkvm3,
	Landsharkvm4,
} vm_name_t;


/* Inter-VM Channel Status*/
typedef enum {
	NOTEXISTCHAN =0,
	CHAN_EMPTY =1,
	CHAN_FULL =2,
	IVC_ERROR =3,
	IVC_RECV_BUSY =4,
	IVC_RECV_ERROR =5,
	IVC_RECV_SUCCESS =6,
	IVC_SEND_BUSY =7,
	IVC_SEND_ERROR =8,
	IVC_SEND_SUCCESS =9,
	IVC_ERROR_DATA_LARGER_THAN_BUFFER =10,
} ivc_status_t;

/* Inter-VM Channel Direction */
typedef enum{
	IVC_IN,
	IVC_OUT,
	IVC_EMPTY,
} ivc_direct_t;

#define HYPERCALL_BUFFER_SIZE 64*1024
#define MM_CP_BUFFER_SIZE 64*1024
#define SM_IVC_BUFFER_SIZE 1024*1024 //share memory inter-VM channel buffer size 1M

#endif /* !_SYS_SYS_VIRT_HYPERCALL_H_ */
