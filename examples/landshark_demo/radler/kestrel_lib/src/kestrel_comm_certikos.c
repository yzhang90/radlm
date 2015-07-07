/***
 *** This file implements the inter-node communication of RADL for
 *** RADL nodes running as CertiKOS processes.
 ***/

#include "kestrel_comm.h"
#include "kestrel_mbox.h"

/***
 *** Compile-time customizable flags
 ***/

/* Max number of total RADL channels used by a whole CertiKOS process;
   there is no theoretical limit, just need a max number at compile
   time */
#define MAX_RADL_CHANS 1024

/* Max number of distinct other CertiKOS processes that one CertiKOS
   process can communicate with; again, there is no theoretical limit,
   it just needs to be known at compile time */
#define MAX_OTHER_PROCS 128

/* The maximum number of bytes to be used for IPC communication
   buffer; again, there is no theoretical limit, it just needs to be
   known at compile time */
#define MAX_IPC_BUFFER_SPACE 16777216

/* Page size */
#define CERTIKOS_PAGE_SIZE 4096


/***
 *** Data structures to describe channels
 ***/

/* Channel info struct, describing a single channel */
struct kestrel_certikos_ipc_chan_info {
  struct radl_certikos_ipc_channel chan;
  struct mbox_buffer *buffer;
  unsigned long buffer_size;
  kestrel_certikos_ipc_chan_info *next_chan; /* For a linked list structure */
};

/* Structure describing a "region" of memory, all for communicating
   with the same other process */
typedef struct region_info {
  radl_certikos_proc_name other_proc_name;
  pid_t other_pid;
  char *region_ptr;
  unsigned long region_size;
  shared_mem_state status; /* The last-seen status of this region */
  kestrel_certikos_ipc_chan_info *channels;
} region_info;


/***
 *** Misc helper functions
 ***/

/* Round up len to a multiple of the page size */
unsigned long round_up_to_page_size (unsigned long len) {
  return (len == 0) ? 0 : (((len-1) / CERTIKOS_PAGE_SIZE) + 1) * CERTIKOS_PAGE_SIZE;
}


/***
 *** Allocators that use statically-allocated arrays, since CertiKOS
 *** does not have malloc
 ***/

/* Statically allocated pool of channel info structures */
kestrel_certikos_ipc_chan_info chan_info_struct_pool[MAX_RADL_CHANS];
unsigned int num_allocated_chans = 0;

/* malloc function for channel info structs */
kestrel_certikos_ipc_chan_info *malloc_chan_info_struct () {
  unsigned int i = num_allocated_chans++;
  if (i >= MAX_RADL_CHANS) {
    PANIC ("malloc_chan_info_struct: ran out of channels!\n");
  }

  return &chan_info_struct_pool[i];
}

/* Statically allocated pool of region_info structures */
region_info *region_info_struct_pool[MAX_OTHER_PROCS];
unsigned int num_allocated_regions = 0;

/* malloc function for region_info structs */
region_info *malloc_region_info_struct () {
  unsigned int i = num_allocated_regions++;
  if (i >= MAX_OTHER_PROCS) {
    PANIC ("malloc_region_info_struct: ran out of regions!\n");
  }

  return &region_info_struct_pool[i];
}

/* Statically allocated pool of unstructured memory */
char buffer_space [MAX_IPC_BUFFER_SPACE];
unsigned int buffer_space_free_ptr = 0;

/* malloc function for allocated buffers */
char *malloc_buffer (unsigned int size) {
  char *buffer = &buffer_space[buffer_space_free_ptr];
  buffer_space_free_ptr += size;

  if (buffer_space_free_ptr >= MAX_IPC_BUFFER_SPACE) {
    PANIC ("malloc_buffer: ran out of buffer space!\n");
  }
  return buffer;
}


/***
 *** Functions to manage regions
 ***/

/* Lookup the region_info struct for a given CertiKOS process name,
   allocating a new struct if one does not exist */
region_info *lookup_region_info (radl_certikos_proc_name proc_name) {
  int i;
  for (i = 0; i < num_allocated_regions; ++i) {
    if (region_info_struct_pool[i].other_proc_name == proc_name) {
      return &region_info_struct_pool[i];
    }
  }

  region_info *region = malloc_region_info_struct ();
  region->other_proc_name = proc_name;
  region->channels = NULL;
  return region;
}

/* Insert a channel info structure into a region, in RADL id order */
void insert_chan_info_struct (region_info *region, kestrel_certikos_ipc_chan_info *chan) {
  radl_channel_id chan_id = chan->chan.gen_chan.id;
  kestrel_certikos_ipc_chan_info **chan_ptr = &region->chanels;

  while (*chan_ptr != NULL && (*chan_ptr)->chan.gen_chan.id < chan_id) {
    chan_ptr = &(*chan_ptr)->next_chan;
  }

  chan->next_chan = *chan_ptr;
  *chan_ptr = chan;
}


/***
 *** Channel initialization
 ***/

/* Beginning the initialization process for CertiKOS IPC is a no-op */
void kestrel_certikos_ipc_start_init () {
  return;
}

/* Initialize a channel. This just allocates the channel info
   structure and puts it into its region; the actual shared-memory
   buffers are allocated and offered to the other processes by
   finish_init, below */
kestrel_certikos_ipc_chan_info *kestrel_certikos_ipc_init_chan (struct radl_certikos_ipc_channel chan) {
  kestrel_certikos_ipc_chan_info *info = malloc_chan_info_struct ();
  info->chan = chan;

  region_info *region = lookup_region_info (chan->other_proc_name);
  insert_chan_info_struct (region, info);

  return info;
}

/* To finish initialization, we allocate all the regions, offer them
   to the other processes, and wait for the other processes to accept
   them */
void kestrel_certikos_ipc_finish_init () {
  int i, offers_accepted = 0;

  /* Allocate each region and offer it to the other CertiKOS process */
  for (i = 0; i < num_allocated_regions; ++i) {
    region_info *region = &region_info_struct_pool[i];
    kestrel_certikos_ipc_chan_info *chan;

    /* First, compute the total size of the region, rounding it up to a page size */
    region->region_size = 0;
    for (chan = region->channels; chan != NULL; chan = chan->next_chan) {
      chan->buffer_size = chan->chan.gen_chan.data_size;
      region->region_size += chan->buffer_size;
    }
    region->region_size = round_up_to_page_size (region->region_size);

    /* Next, allocate the region, and assign portions of it to the channels */
    char *buffer_ptr = malloc_buffer (region->region_size);
    region->region_ptr = buffer_ptr;
    for (chan = region->channels; chan != NULL; chan = chan->next_chan) {
      chan->buffer = buffer_ptr;
      buffer_ptr += chan->buffer_size;
    }

    /* Now we offer the buffer to the other process */
    region->other_pid = process_name_to_id (region->other_proc_name);
    shared_mem_ret_value ret =
      sys_offer_shared_memory (region->other_pid, region->buffer_ptr, region->buffer_size);
    if (ret != RET_SHARED_MEM_READY && ret != RET_SHARED_MEM_PENDING) {
      PANIC ("kestrel_certikos_ipc_finish_init: error %i returned by sys_offer_shared_memory", ret);
    }
    region->status = ret;
  }

  /* Now that all of the shared-memory offers have been made, wait for
     all of them to become "ready" */
  while (!offers_accepted) {
    offers_accepted = 1;
    for (i = 0; i < num_allocated_regions; ++i) {
      region_info *region = &region_info_struct_pool[i];

      /* If we have not yet seen that the region has been accepted,
         check again */
      if (region->status == SHARED_MEM_PENDING) {
        region->status = sys_shared_memory_status (region->other_pid);
      }

      if (region->status == SHARED_MEM_DEAD) {
        /* If the region is now dead, panic */
        PANIC ("kestrel_certikos_ipc_finish_init: dead shared memory region!\n");
      } else if (region->status == SHARED_MEM_PENDING) {
        /* If the region has still not been accepted, loop again */
        offers_accepted = 0;
      }
    }

    /* Yield if we are still waiting for someone */
    if (!offers_accepted) {
      sys_yield ();
    }
  }
}
