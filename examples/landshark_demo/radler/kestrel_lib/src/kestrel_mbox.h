/* Include files */

#include "stdint.h"

/* Verbatim Text */


#define mbuf_hdr_size  sizeof (struct mbox_buffer_hdr)

typedef char *         charstar     ;
typedef const char *   constcharstar;
typedef int *          intstar      ;  

/***
 *** This file implements a generic type of buffer, called a "mailbox"
 *** buffer, useful for shared-memory communication. Mailbox buffers
 *** are implemented with two underlying buffers, one used by the
 *** writer and the other by the reader. When a write completes, the
 *** two are swapped, so the next reader sees the value most recently
 *** written.
 ***/



/*************** The mailbox buffers themselves *****************/

/* The header of a mailbox buffer */ 
struct mbox_buffer_hdr {                                      
  unsigned int unused:13,                                     
               stale0:1,    /* Set iff the first slot is stale */           
               stale1:1,    /* Set iff the second slot is stale */          
               read_slot:1; /* The slot number most recently written */  
};                                                            
                                                              
/* The structure of a mailbox buffer */                       
struct mbox_buffer {                                          
  struct mbox_buffer_hdr hdr;                                 
  char                   data[];        
};    



/***** Calls for reading from and writing to mailbox buffers  ***/
 
/* Structs/Unions/Types */

typedef struct mbox_buffer* MBuf;

/* Functions */

uint64_t fit_into_buffer (uint32_t);

uint64_t mbox_buffer_size (uint32_t);

constcharstar mbox_buffer_update (MBuf, intstar, uint32_t);

charstar mbox_buffer_stall (MBuf, uint32_t);

void mbox_buffer_release (MBuf);

void mbox_buffer_commit (MBuf);


