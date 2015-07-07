#include "hypercall.h"

#define gcc_inline      __inline __attribute__((always_inline))

/*
 * CertiKOS hypercall calling convention
 * - Input
 *    %eax/%rax: hypercall number (defined in hypercall_t)
 *    %ebx/%rbx: the 1st parameter a0
 *    %ecx/%rcx: the 2st parameter a1
 *    %edx/%rdx: the 3rd parameter a2
 *    %esi/%rsi: the 4th parameter a3
 * - Output
 *    %eax/%rax: the return value ret
 *    others   : unmodified
 */

/* helper macros*/
#define DEF_HYPERCALL_HEAD(name) static uint32_t gcc_inline hypercall_##name
#define DEF_HYPERCALL_BODY(nr)                  \
        {                                       \
        uint32_t c;                             \
        asm volatile("vmcall"                  \
        : "=a" (c)                              \
        : "a" (nr)
#define DEF_HYPERCALL_RET                       \
        : "cc", "memory");                      \
        return c;                               \
        }
#define DEF_HYPERCALL_0(name, nr)               \
        DEF_HYPERCALL_HEAD(name)(void)          \
        DEF_HYPERCALL_BODY(nr)                  \
        DEF_HYPERCALL_RET
#define DEF_HYPERCALL_1(name, nr)               \
        DEF_HYPERCALL_HEAD(name)(uint32_t a0)   \
        DEF_HYPERCALL_BODY(nr)                  \
        , "b" (a0)                              \
        DEF_HYPERCALL_RET
#define DEF_HYPERCALL_2(name, nr)                               \
        DEF_HYPERCALL_HEAD(name)(uint32_t a0, uint32_t a1)      \
        DEF_HYPERCALL_BODY(nr)                                  \
        , "b" (a0), "c" (a1)                                    \
        DEF_HYPERCALL_RET
#define DEF_HYPERCALL_3(name, nr)                                       \
        DEF_HYPERCALL_HEAD(name)(uint32_t a0, uint32_t a1, uint32_t a2) \
        DEF_HYPERCALL_BODY(nr)                                          \
        , "b" (a0), "c" (a1), "d" (a2)                                  \
        DEF_HYPERCALL_RET
#define DEF_HYPERCALL_4(name, nr)                                       \
        DEF_HYPERCALL_HEAD(name)(uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3) \
        DEF_HYPERCALL_BODY(nr)                                          \
        , "b" (a0), "c" (a1), "d" (a2), "S" (a3)                        \
        DEF_HYPERCALL_RET

#define DEF_HYPERCALL_2_RET2(name, nr)                               \
        DEF_HYPERCALL_HEAD(name)(uint32_t a0, uint32_t a1)      \
        DEF_HYPERCALL_BODY_2(nr)                                  \
        , "b" (a0), "c" (a1)                                    \
        DEF_HYPERCALL_RET
/*
 * Define hypercalls.
 *
 * DEF_HYPERCALL_#(name, nr) defines such a function
 * - it's for the hypercall nr (defined as hypercall_t)
 * - its name is hypercall_##name
 * - it requires # parameters
 */

//hypercalls for tests 
DEF_HYPERCALL_2(bitand, HYPERCALL_BITAND)
DEF_HYPERCALL_2(bitor, HYPERCALL_BITOR)
DEF_HYPERCALL_2(bitxor, HYPERCALL_BITXOR)
DEF_HYPERCALL_1(bitnot, HYPERCALL_BITNOT)

/*
 *hypercall_fifostatus(target_vm_id, ivc_direction)
 *target_vm_id, the ID of the target VM
 *ivc_direction, IVC channel direction IVC_IN or IVC_OUT
 */
DEF_HYPERCALL_2(fifostatus, HYPERCALL_FIFO_STATUS)

/*
 *hypercall_fifosend(target_vm_id, uint32_t data)
 *target_vm_id, the ID of the target VM
 *data, 4 bytes data for sending
 */
DEF_HYPERCALL_2(fifosend, HYPERCALL_FIFO_SEND)

/*
 *hypercall_fiforecv(target_vm_id, uint32_t * data)
 *target_vm_id, the ID of the target VM
 *data, received 4 bytes data 
 */
static uint32_t gcc_inline 
hypercall_fiforecv(uint32_t target_vm_id, uint32_t *data)
{                                       
	uint32_t c,a1 = 0;
        asm volatile("vmcall"                  
        : "=a" (c), "=c" (a1)                             
        : "a" (HYPERCALL_FIFO_RECV)
        , "b" (target_vm_id)
	, "c" (a1) 
        : "cc", "memory");                      
	*data = a1;
        return c;                               
}

/*
 *hypercall_memfifosend(target_vm_id, uint32_t* data, uint32_t size)
 *target_vm_id, the ID of the target VM
 *data, pointer to the data in the sending guest VM memory space
 *size, size of data for sending 
 */
DEF_HYPERCALL_3(memfifosend, HYPERCALL_MEM_SHARE_FIFO_SEND)


/*
 *hypercall_memfiforecv(target_vm_id, uint32_t *data, uint32_t *size)
 *target_vm_id, the ID of the target VM
 *data, pointer to the location for store the incoming data in the receiving guest VM memory space
 *size, return the size of data received
 */
static uint32_t gcc_inline 
hypercall_memfiforecv(uint32_t target_vm_id, uint32_t *data, uint32_t *size)
{                                       
	uint32_t c,a1 = 0;
        asm volatile("vmcall"                  
        : "=a" (c), "=d" (a1)                             
        : "a" (HYPERCALL_MEM_SHARE_FIFO_RECV)
        , "b" (target_vm_id)
	, "c" (data) 
	, "d" (a1) 
        : "cc", "memory");                      
	*size = a1;
        return c;                               
}

static pid_t gcc_inline 
hyp_vmname_to_pid(uint32_t vmname)
{                                       
        uint32_t c;
        asm volatile("vmcall"
        : "=a" (c)
        : "a" (HYPERCALL_VMNAME_TO_PID)
        , "b" (vmname)
        : "cc", "memory");
        return c;                             
}

/////////////////////////// Shared Memory Hypercalls ///////////////////////////

typedef enum {
        SHARED_MEM_READY = 0u,
        SHARED_MEM_PENDING = 1u,
        SHARED_MEM_DEAD = 2u,
} shared_mem_state;

typedef enum {
        RET_SHARED_MEM_READY = SHARED_MEM_READY,
        RET_SHARED_MEM_PENDING = SHARED_MEM_PENDING,
        RET_UNMATCHED_SIZES = 3u,
        RET_SHARED_MEM_ALLOC_FAILED = 4u,
        RET_SHARED_MEM_PAGE_MAP_1_FAILED = 5u,
        RET_SHARED_MEM_PAGE_MAP_2_FAILED = 6u,
        RET_SHARED_MEM_BAD_SIZE = 7u, // buffer size must be a multiple of PAGESIZE
        RET_SHARED_MEM_BAD_ALIGNMENT = 8u, // buffer must be aligned to a page boundary
        RET_SHARED_MEM_BAD_LOCATION = 9u, // certikos process's buffer must be in userspace
        RET_SHARED_MEM_OTHER_PROC_INVALID = 10u,
        RET_SHARED_MEM_INVALID_DEST_PID = 11u,
        RET_SHARED_MEM_SOURCE_CANNOT_EQUAL_DEST = 12u,
} shared_mem_ret_value;

static shared_mem_ret_value gcc_inline
hyp_offer_shared_memory(pid_t dest, volatile void * source_va,
        uintptr_t size)
{
        uint32_t c;
        asm volatile("vmcall"
        : "=a" (c)
        : "a" (HYPERCALL_OFFER_SHARED_MEMORY)
        , "b" (dest)
        , "c" (source_va)
        , "d" (size)
        : "cc", "memory");
        return c;
}

static shared_mem_state gcc_inline
hyp_shared_memory_status(pid_t dest)
{
        uint32_t c;
        asm volatile("vmcall"
        : "=a" (c)
        : "a" (HYPERCALL_SHARED_MEMORY_STATUS)
        , "b" (dest)
        : "cc", "memory");
        return c;
}

/*
 * Hypercalls from the previous version, redefined at best effort to their new
 * equivalent 
 */

// FIXME: hypercall_create_sm_ivc does an incorrect case from an
// unsigned 32-bit integer to a void*

/*
#define IVC_SM_IVC_NOT_EXIST    SHARED_MEM_DEAD
#define IVC_SM_IVC_NEED_ACK     SHARED_MEM_PENDING
#define IVC_SM_IVC_PENDING      SHARED_MEM_PENDING
#define IVC_SM_IVC_READY        SHARED_MEM_READY

static shared_mem_ret_value gcc_inline
hypercall_create_sm_ivc(vm_name_t target, uint32_t buffer, uint32_t size)
{
        return hyp_offer_shared_memory(hyp_vmname_to_pid(target),
                (volatile void *)buffer, size);
}

static shared_mem_state gcc_inline
hypercall_sm_ivc_status(vm_name_t target)
{
        return hyp_shared_memory_status(hyp_vmname_to_pid(target));
}
*/

/*
 * hypercall_rm_sm_ivc and hypercall_get_sm_ivc_buffer no longer have any
 * equivalent, and thus no longer exist.
 */

