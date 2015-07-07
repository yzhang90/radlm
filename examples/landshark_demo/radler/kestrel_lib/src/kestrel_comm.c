/* Include files */

#include "kestrel_comm.h"

#include "kestrel_mbox.h"

#include "kestrel_helpers.h"

/* Verbatim Text */



/* global variable: temporary buffer for reading incoming IP packets */
char ip_mbox_buffer [MAX_PACKET_SIZE];


/*************************************************************************/
/* ORIGINAL CODE for initialization (has not been synthesized yet)       */ 
/*               must come before synthesized code for send/receive      */ 
/*************************************************************************/



/***
 *** Variable-length arrays of pointers
 ***/

/* A variable-length pointer array is represented using three
   elements: the array itself; the length of the array; and the number
   of currently used slots in the array. */

/* Initialize a pointer array by allocating the array, setting its
   length, and setting the number of elements to 0. The tp argument
   gives the base type of the array. */
#define INIT_PTR_ARRAY(arr,len,num_elts,tp)	\
  do {						\
    len = 20;					\
    arr = (tp **)malloc (len * sizeof (tp *));	\
    num_elts = 0;				\
  } while (0)

/* Allocate a new pointer inside a pointer array, growing the array if
   necessary, and assign the result to var */
#define PTR_ARRAY_ALLOC(arr,len,num_elts,tp,var)		\
  do {								\
    if (num_elts >= len) {					\
      int i;							\
      len *= 2;							\
      tp **new_arr = (tp**) malloc (len * sizeof (tp*));	\
      if (new_arr == NULL) {					\
	fprintf (stderr, "malloc error!\n");			\
	exit (-1);						\
      }								\
      for (i = 0; i < num_elts; ++i)				\
	new_arr[i] = arr[i];					\
      free (arr);						\
      arr = new_arr;						\
    }								\
    var = (tp*) malloc (sizeof (tp));				\
    if (var == NULL) {						\
      fprintf (stderr, "malloc error!\n");			\
      exit (-1);						\
    }								\
    arr[num_elts++] = var;					\
  } while (0)


/***
 *** Misc helper functions
 ***/

/* Round up len to a multiple of the page size */
unsigned long round_up_to_page_size (unsigned long len) {
  unsigned long page_size = get_page_size ();
  return (len == 0) ? 0 : (((len-1) / page_size) + 1) * page_size;
}


/***
 *** Kestrel global, VM-level init calls; require root access in Linux
 ***/

/* For each protocol, there is start_*_init() that is called once
   before any channel initialization, init_*_chan that is called once
   per channel, and finish_*_init that is called after all channel
   initialization is completed. */


/* Variable-length pointer array to store all IPC channels allocated
   in the current Linux instance. */
kestrel_linux_ipc_chan_info **ipc_chans;
unsigned int ipc_chans_length;
unsigned int num_ipc_chans;

/* Look up a kestrel_linux_ipc_chan_info by id, returning NULL if none is found */
kestrel_linux_ipc_chan_info *lookup_ipc_chan (radl_channel_id id) {
  int i;
  for (i = 0; i < num_ipc_chans; ++i) {
    if (ipc_chans[i]->chan.gen_chan.id == id)
      return ipc_chans[i];
  }
  return NULL;
}

/* Initialize the ipc_chans array */
void kestrel_linux_ipc_start_init () {
  INIT_PTR_ARRAY(ipc_chans,ipc_chans_length,
		 num_ipc_chans,kestrel_linux_ipc_chan_info);
}


/* Allocates a shared-memory (intra-VM) mailbox buffer for the
   channel. This mailbox buffer needs to store two slots of size
   chan.gen_chan.data_size, as well as a struct mbox_buffer_hdr. */
kestrel_linux_ipc_chan_info *kestrel_linux_ipc_init_chan (struct radl_linux_ipc_channel chan) {
  kestrel_linux_ipc_chan_info *info = lookup_ipc_chan (chan.gen_chan.id);
  unsigned long buffer_size =
    round_up_to_page_size (mbox_buffer_size (chan.gen_chan.data_size));

  if (info == NULL) {
    PTR_ARRAY_ALLOC(ipc_chans,ipc_chans_length,num_ipc_chans,
		    kestrel_linux_ipc_chan_info,info);
    info->chan = chan;
    info->buffer_size = buffer_size;
    info->buffer = allocate_shared_mem (info->buffer_size);

    /* Initialize the buffer */
    char *buffer_data = mbox_buffer_stall (info->buffer, chan.gen_chan.data_size);
    memcpy (buffer_data, chan.gen_chan.data_init_value, chan.gen_chan.data_size);
    mbox_buffer_commit (info->buffer);

  } else {
    if (info->buffer_size != buffer_size) {
      fprintf (stderr, "kestrel_linux_ipc_init_chan: buffer size mismatch\n");
      exit (-1);
    }
  }

  return info;
}

/* Nothing is needed to finish IPC init */
void kestrel_linux_ipc_finish_init () {
  return;
}


/* Global variable to store the raw IP socket */
int ip_socket;

/* Variable-length pointer array for storing all IP channels allocated
   in the system. */
kestrel_linux_ip_chan_info **ip_chans;
unsigned int ip_chans_length;
unsigned int num_ip_chans;


/* Start IP init */
void kestrel_linux_ip_start_init () {
  /* Initialize pointer array */
  INIT_PTR_ARRAY (ip_chans,ip_chans_length,
		  num_ip_chans,kestrel_linux_ip_chan_info);

  /* Create ip_socket */
  ip_socket = create_raw_socket (KESTREL_PROTO);
}

/* Allocates a shared-memory (intra-VM) mailbox buffer for storing
   data coming in / doing out over the wire. This mailbox buffer needs
   to contain two slots, each of which contain the data_size amount of
   data plus space for the kestrel_ip_hdr header, along with the
   mbox_buffer_hdr. The returned kestrel_linux_ip_chan_info pointer is
   stored in the ip_chans structure. */
kestrel_linux_ip_chan_info *
kestrel_linux_ip_init_chan (struct radl_linux_ip_channel chan) {
  kestrel_linux_ip_chan_info *info;

  /* Allocate the info structure, storing it in ip_chans */
  PTR_ARRAY_ALLOC(ip_chans,ip_chans_length,num_ip_chans,
		  kestrel_linux_ip_chan_info,info);

  /* Set up the fields of the info structure, including allocating the
     buffer itself */
  info->chan = chan;
  info->buffer_size =
    round_up_to_page_size (mbox_buffer_size (chan.gen_chan.data_size + sizeof(struct kestrel_ip_hdr)));
  info->buffer = allocate_shared_mem (info->buffer_size);

  /* Initialize the buffer */
  struct kestrel_ip_packet *packet =
    (struct kestrel_ip_packet *) mbox_buffer_stall (info->buffer, chan.gen_chan.data_size);
  memcpy (&packet->data, chan.gen_chan.data_init_value, chan.gen_chan.data_size);
  mbox_buffer_commit (info->buffer);

  return info;
}

/* This is a no-op */
void kestrel_linux_ip_finish_init () {
  return;
}


/* The most complex initialization here is for CertiKOS shared-memory
   IVC. To do this, we perform the following steps:

   * First, store all requests for an IVC buffer, where a "request" is
   a call to kestrel_certikosVM_ivc_init_chan.

   * Next, we sort them by other VM number and channel id, and then go
   through the list in sorted order, assigning each buffer an offset
   into the global IVC buffer for its global VM.

   * Finally, we allocate the CertiKOS buffers in kernel space using
   the certikos_ivc_kmod kernel module, and then map those buffers
   into user space.
*/

/* A structure representing the entire memory region / buffer for
   CertiKOS IVC communication with a single other VM */
struct certikos_region {
  radl_vm_id vm_num; // The VM this buffer communicates with
  void *region_ptr; // Pointer to the buffer in my address space
  unsigned int region_size; // Size of region_ptr

  // The remaining fields define a variable-length pointer array to
  // all the channels using this buffer
  kestrel_certikosVM_ivc_chan_info **ivc_chans;
  unsigned int ivc_chans_length, num_ivc_chans;
};

/* Variable-length pointer array to store all CertiKOS IVC buffers
   that have been allocated so far. */
struct certikos_region **certikos_regions;
unsigned int certikos_regions_length, num_certikos_regions;

/* My VM number */
unsigned int my_vm_num;

/* Start up IVC init by allocating the certikosVM_ivc_chans array */
void kestrel_certikosVM_ivc_start_init () {
  INIT_PTR_ARRAY (certikos_regions, certikos_regions_length,
		  num_certikos_regions, struct certikos_region);
  my_vm_num = 0;
}

/* Look up a certikos_region by VM number */
struct certikos_region *lookup_certikos_region (radl_vm_id vm_num) {
  int i;
  for (i = 0; i < num_certikos_regions; ++i) {
    if (certikos_regions[i]->vm_num == vm_num)
      return certikos_regions[i];
  }
  return NULL;
}

/* Allocate a kestrel_certikosVM_ivc_chan_info and store it in its
   region, allocating the region if necessary */
kestrel_certikosVM_ivc_chan_info *kestrel_certikosVM_ivc_init_chan (struct radl_certikosVM_ivc_channel chan) {
  struct certikos_region *region;
  kestrel_certikosVM_ivc_chan_info *info;

  /* Lookup the buffer for the given VM, allocating one if
     necessary. NOTE: the region_ptr is NOT yet set */
  region = lookup_certikos_region (chan.other_vm);
  if (region == NULL) {
    PTR_ARRAY_ALLOC (certikos_regions, certikos_regions_length,
		     num_certikos_regions, struct certikos_region, region);
    region->vm_num = chan.other_vm;
    INIT_PTR_ARRAY (region->ivc_chans, region->ivc_chans_length,
		    region->num_ivc_chans, kestrel_certikosVM_ivc_chan_info);
  }

  /* Allocate a kestrel_certikosVM_ivc_chan_info in region */
  PTR_ARRAY_ALLOC (region->ivc_chans, region->ivc_chans_length,
		   region->num_ivc_chans, kestrel_certikosVM_ivc_chan_info,
		   info);
  info->chan = chan;
  info->buffer_size = round_up_to_page_size (mbox_buffer_size (chan.gen_chan.data_size));

  /* Update my_vm_num, if appropriate */
  if (my_vm_num == 0) {
    my_vm_num = chan.my_vm;
  } else {
    if (chan.my_vm != my_vm_num) {
      fprintf (stderr, "Disagreement on my VM number in initializing IVC buffers!\n");
      exit (-1);
    }
  }

  /* Finally, return the newly allocated info structure */
  return info;
}


/* Comparison function for sorting certikosVM_ivc_chans by id */
static int certikosVM_ivc_chan_compare (const void *arg1, const void *arg2) {
  kestrel_certikosVM_ivc_chan_info **pinfo1 = (kestrel_certikosVM_ivc_chan_info **)arg1;
  kestrel_certikosVM_ivc_chan_info **pinfo2 = (kestrel_certikosVM_ivc_chan_info **)arg2;
  radl_channel_id id1 = (*pinfo1)->chan.gen_chan.id;
  radl_channel_id id2 = (*pinfo2)->chan.gen_chan.id;
  return (id1 < id2) ? -1 : (id1 > id2);
}

/* Finish the IVC init process described above */
void kestrel_certikosVM_ivc_finish_init () {
  int i, j;
  unsigned int num_vms = 0;
  unsigned long buffer_sizes[MAX_CERTIKOS_VMS];

  printf ("Finishing IVC init...\n");

  /* Initialize the buffer_sizes array */
  for (i = 0; i < MAX_CERTIKOS_VMS; ++i) {
    buffer_sizes[i] = 0;
  }

  /* Assign all the channels in each region their respective slots in
     the region, thereby also computing the total region sizes */
  for (i = 0; i < num_certikos_regions; ++i) {
    struct certikos_region *region = certikos_regions[i];
    region->region_size = 0;

    /* First, sort the channels by id */
    qsort (region->ivc_chans, region->num_ivc_chans,
	   sizeof (kestrel_certikosVM_ivc_chan_info *),
	   &certikosVM_ivc_chan_compare);

    /* Now assign each channel an offset, tracking the total region
       size in region_size */
    for (j = 0; j < region->num_ivc_chans; ++j) {
      kestrel_certikosVM_ivc_chan_info *info = region->ivc_chans[j];
      info->region_offset = region->region_size;
      region->region_size += info->buffer_size;
    }

    /* Finally, update num_vms and buffer_sizes */
    if (region->vm_num > num_vms) {
      num_vms = region->vm_num;
    }
    if (region->vm_num >= MAX_CERTIKOS_VMS) {
      fprintf (stderr, "CertiKOS VM nubmer %u exceeds the compile-time max of %u; please update MAX_CERTIKOS_VMS",
	       region->vm_num, MAX_CERTIKOS_VMS);
      exit (-1);
    }

    /* README: VM numbers start with 1, while C arrays start with 0,
       so we have to subtract one here for the buffer_sizes index */
    buffer_sizes[region->vm_num-1] = region->region_size;
  }

  /* Ensure that the certikos_ivc_kmod is installed */
  ensure_certikos_ivc_kmod (buffer_sizes, my_vm_num, num_vms);

  /* Set up all the buffer pointers for all the channels, and wait for
     the buffers to be ready (i.e., for the other VM to have already
     requested the same buffer). */
  for (i = 0; i < num_certikos_regions; ++i) {
    struct certikos_region *region = certikos_regions[i];
    char ivc_filename[200];
    unsigned int buffer_size;

    /* Map region into our address space */
    sprintf (ivc_filename, "%s/vm%u", certikos_ivc_dir, region->vm_num);
    region->region_ptr = map_ivc_buffer (ivc_filename, region->region_size);

    /* Wait for the other VM to request a buffer to us */
    sync_certikos_buffer (ivc_filename);

    /* For each channel in region, set the buffer pointer to the
       region pointer plus the channel's region_offset, and initialize
       the buffer */
    for (j = 0; j < region->num_ivc_chans; ++j) {
      kestrel_certikosVM_ivc_chan_info *info = region->ivc_chans[j];
      info->buffer = region->region_ptr + info->region_offset;

      printf ("Mapped IVC buffer for channel %u at %lx\n",
	      info->chan.gen_chan.id, (unsigned long)info->buffer);

      /* Initialize the buffer */
      char *buffer_data = mbox_buffer_stall (info->buffer, info->chan.gen_chan.data_size);
      memcpy (buffer_data, info->chan.gen_chan.data_init_value, info->chan.gen_chan.data_size);
      mbox_buffer_commit (info->buffer);
    }
  }
}


/* Initialization for LynxSecure shared-memory IVC. Very similar to
   that for CertiKOS. */

/* A LynxSecure region for us is a region name along with a list of
   the channels using that region name */
struct lynxsecure_region {
  const char *region_name; // The filename for this region
  void *region_ptr; // Pointer to the buffer in my address space
  unsigned int region_size; // Size of region_ptr

  // A variable-length pointer array of the channels using this region
  kestrel_lynxsecureVM_ivc_chan_info **ivc_chans;
  unsigned int ivc_chans_length, num_ivc_chans;
};


/* Variable-length pointer array for the LynxSecure regions */
struct lynxsecure_region **lynxsecure_regions;
unsigned int lynxsecure_regions_length, num_lynxsecure_regions;

/* Start up IVC init by allocating the lynxsecureVM_ivc_chans array */
void kestrel_lynxsecureVM_ivc_start_init () {
  INIT_PTR_ARRAY (lynxsecure_regions, lynxsecure_regions_length,
		  num_lynxsecure_regions, struct lynxsecure_region);
}

/* Look up a LynxSecure region by name */
struct lynxsecure_region *lookup_lynxsecure_region (const char *name) {
  int i;
  for (i = 0; i < num_lynxsecure_regions; ++i) {
    struct lynxsecure_region *region = lynxsecure_regions[i];
    if (!strcmp (name, region->region_name)) {
      return region;
    }
  }
  return NULL;
}

/* Allocate a kestrel_lynxsecureVM_ivc_chan_info and store it in its
   region, allocating the region if necessary */
kestrel_lynxsecureVM_ivc_chan_info *kestrel_lynxsecureVM_ivc_init_chan (struct radl_lynxsecureVM_ivc_channel chan) {
  struct lynxsecure_region *region;
  kestrel_lynxsecureVM_ivc_chan_info *info;

  /* Lookup the buffer for the given VM, allocating one if
     necessary. NOTE: the region_ptr is NOT yet set */
  region = lookup_lynxsecure_region (chan.region_name);
  if (region == NULL) {
    PTR_ARRAY_ALLOC (lynxsecure_regions, lynxsecure_regions_length,
		     num_lynxsecure_regions, struct lynxsecure_region, region);
    region->region_name = chan.region_name;
    INIT_PTR_ARRAY (region->ivc_chans, region->ivc_chans_length,
		    region->num_ivc_chans, kestrel_lynxsecureVM_ivc_chan_info);
  }

  /* Allocate a kestrel_lynxsecureVM_ivc_chan_info in region */
  PTR_ARRAY_ALLOC (region->ivc_chans, region->ivc_chans_length,
		   region->num_ivc_chans, kestrel_lynxsecureVM_ivc_chan_info,
		   info);
  info->chan = chan;
  info->buffer_size = round_up_to_page_size (mbox_buffer_size (chan.gen_chan.data_size));

  /* Finally, return the newly allocated info structure */
  return info;
}


/* Comparison function for sorting lynxsecureVM_ivc_chans by id */
static int lynxsecureVM_ivc_chan_compare (const void *arg1, const void *arg2) {
  kestrel_lynxsecureVM_ivc_chan_info **pinfo1 = (kestrel_lynxsecureVM_ivc_chan_info **)arg1;
  kestrel_lynxsecureVM_ivc_chan_info **pinfo2 = (kestrel_lynxsecureVM_ivc_chan_info **)arg2;
  radl_channel_id id1 = (*pinfo1)->chan.gen_chan.id;
  radl_channel_id id2 = (*pinfo2)->chan.gen_chan.id;
  return (id1 < id2) ? -1 : (id1 > id2);
}

/* Finish the IVC init process described above */
void kestrel_lynxsecureVM_ivc_finish_init () {
  int i, j;

  /* Assign all the channels in each region their respective slots in
     the region, thereby also computing the total region sizes */
  for (i = 0; i < num_lynxsecure_regions; ++i) {
    struct lynxsecure_region *region = lynxsecure_regions[i];
    region->region_size = 0;

    /* First, sort the channels by id */
    qsort (region->ivc_chans, region->num_ivc_chans,
	   sizeof (kestrel_lynxsecureVM_ivc_chan_info *),
	   &lynxsecureVM_ivc_chan_compare);

    /* Now assign each channel an offset, tracking the total region
       size in region_size */
    for (j = 0; j < region->num_ivc_chans; ++j) {
      kestrel_lynxsecureVM_ivc_chan_info *info = region->ivc_chans[j];
      info->region_offset = region->region_size;
      region->region_size += info->buffer_size;
      printf ("Assigning channel %u to offset %llx in region %s\n",
              info->chan.gen_chan.id, info->region_offset, region->region_name);
    }
  }

  /* Ensure that the lynxsecure_ivc_kmod is installed */
  ensure_lynxsecure_ivc_kmod ();

  /* Finally, set up all the buffer pointers for all the channels */
  for (i = 0; i < num_lynxsecure_regions; ++i) {
    struct lynxsecure_region *region = lynxsecure_regions[i];
    char ivc_filename[200];

    /* Map region into our address space */
    sprintf (ivc_filename, "%s/%s", lynxsecure_ivc_dir, region->region_name);
    region->region_ptr = map_ivc_buffer (ivc_filename, region->region_size);

    /* For each channel in region, set the buffer pointer to be the
       region pointer plus the channel's region_offset, and initialize
       the buffer */
    for (j = 0; j < region->num_ivc_chans; ++j) {
      kestrel_lynxsecureVM_ivc_chan_info *info = region->ivc_chans[j];
      info->buffer = region->region_ptr + info->region_offset;

      printf ("Mapped IVC buffer for channel %u at %lx\n",
	      info->chan.gen_chan.id, (unsigned long)info->buffer);

      /* Initialize the buffer */
      char *buffer_data = mbox_buffer_stall (info->buffer, info->chan.gen_chan.data_size);
      memcpy (buffer_data, info->chan.gen_chan.data_init_value, info->chan.gen_chan.data_size);
      mbox_buffer_commit (info->buffer);
    }
  }
}





/* Function definitions */

int32_t lookup_id_in_ipchans_starting_at (radl_channel_id id, int32_t i) {
  if (num_ip_chans == i || (at_index(ip_chans, i))->chan.gen_chan.id == id) {
    return i;
  }
  return lookup_id_in_ipchans_starting_at(id, i + 1);
}

void flush_incoming_IP_step (int num_reads, IPpacket buf) {
  uint32_t packet_size;
  packet_size = recv_ip_packet(ip_socket, buf, MAX_PACKET_SIZE);
  if (num_reads < MAX_PACKET_READS && (!(packet_size == 0))) {
    struct in_addr* bin_saddr_ptr;
    struct in_addr _in_addr_;
    Linux_IP_MBoxInfo info;
    int32_t i;
    i = lookup_id_in_ipchans_starting_at(buf->hdr.chan_id, 0);
    info = at_index(ip_chans, i);
    _in_addr_.s_addr = default_in_addr_t;
    bin_saddr_ptr = &_in_addr_;
    if (i < num_ip_chans && 
        ip_hdr_size + info->chan.gen_chan.data_size == packet_size && 
        (!(inet_aton(info->chan.other_addr, bin_saddr_ptr) == 0)) && 
        eq_in_addr_t(bin_saddr_ptr->s_addr, (&(buf->hdr.iphdr))->saddr)) {
      memcpy(mbox_buffer_stall(info->buffer, packet_size), buf, packet_size);
      mbox_buffer_commit(info->buffer);
      flush_incoming_IP_step(num_reads + 1, buf);
      return;
    }
    flush_incoming_IP_step(num_reads + 1, buf);
    return;
  }
  return;
}

void flush_incoming_IP (void) {
  flush_incoming_IP_step(0, get_packet_at(ip_mbox_buffer));
  return;
}

constcharstar kestrel_linux_ipc_update (Linux_IPC_MBoxInfo ibox, intstar stale) {
  return mbox_buffer_update(ibox->buffer, stale, ibox->chan.gen_chan.data_size);
}

charstar kestrel_linux_ipc_stall (Linux_IPC_MBoxInfo obox) {
  return mbox_buffer_stall(obox->buffer, obox->chan.gen_chan.data_size);
}

void kestrel_linux_ipc_release (Linux_IPC_MBoxInfo ibox) {
  mbox_buffer_release(ibox->buffer);
  return;
}

void kestrel_linux_ipc_commit (Linux_IPC_MBoxInfo obox) {
  mbox_buffer_commit(obox->buffer);
  return;
}

constcharstar kestrel_linux_ip_update (Linux_IP_MBoxInfo ibox, intstar stale) {
  flush_incoming_IP();
  return mbox_buffer_update
         (ibox->buffer, stale, ibox->chan.gen_chan.data_size + ip_hdr_size) + 
         ip_hdr_size;
}

charstar kestrel_linux_ip_stall (Linux_IP_MBoxInfo obox) {
  return mbox_buffer_stall
         (obox->buffer, obox->chan.gen_chan.data_size + ip_hdr_size) + 
         ip_hdr_size;
}

void kestrel_linux_ip_release (Linux_IP_MBoxInfo ibox) {
  mbox_buffer_release(ibox->buffer);
  return;
}

void kestrel_linux_ip_commit (Linux_IP_MBoxInfo obox) {
  IPpacket packet;
  IPhdr iphdr;
  packet = 
  get_packet_at
  (mbox_buffer_stall(obox->buffer, obox->chan.gen_chan.data_size + ip_hdr_size));
  iphdr = (&(packet->hdr.iphdr));
  iphdr->version = 4;
  iphdr->ihl = 5;
  iphdr->tos = 0;
  iphdr->tot_len = htons(obox->chan.gen_chan.data_size + ip_hdr_size);
  iphdr->id = 0;
  iphdr->frag_off = 0;
  iphdr->ttl = 30;
  iphdr->protocol = KESTREL_PROTO;
  iphdr->check = 0;
  if (inet_aton(obox->chan.my_addr, iphdr_saddr(iphdr)) == 0) {
    exit(-1);
    return;
  }
  if (inet_aton(obox->chan.other_addr, iphdr_daddr(iphdr)) == 0) {
    exit(-1);
    return;
  }
  packet->hdr.chan_id = obox->chan.gen_chan.id;
  send_ip_packet(ip_socket, packet);
  mbox_buffer_commit(obox->buffer);
  return;
}

constcharstar kestrel_lynxsecureVM_ivc_update 
             (LynxsecureVM_IVC_MBoxInfo ibox, intstar stale) {
  return mbox_buffer_update(ibox->buffer, stale, ibox->chan.gen_chan.data_size);
}

charstar kestrel_lynxsecureVM_ivc_stall (LynxsecureVM_IVC_MBoxInfo obox) {
  return mbox_buffer_stall(obox->buffer, obox->chan.gen_chan.data_size);
}

void kestrel_lynxsecureVM_ivc_release (LynxsecureVM_IVC_MBoxInfo ibox) {
  mbox_buffer_release(ibox->buffer);
  return;
}

void kestrel_lynxsecureVM_ivc_commit (LynxsecureVM_IVC_MBoxInfo obox) {
  mbox_buffer_commit(obox->buffer);
  return;
}

constcharstar kestrel_certikosVM_ivc_update 
             (CertikosVM_IVC_MBoxInfo ibox, intstar stale) {
  return mbox_buffer_update(ibox->buffer, stale, ibox->chan.gen_chan.data_size);
}

charstar kestrel_certikosVM_ivc_stall (CertikosVM_IVC_MBoxInfo obox) {
  return mbox_buffer_stall(obox->buffer, obox->chan.gen_chan.data_size);
}

void kestrel_certikosVM_ivc_release (CertikosVM_IVC_MBoxInfo ibox) {
  mbox_buffer_release(ibox->buffer);
  return;
}

void kestrel_certikosVM_ivc_commit (CertikosVM_IVC_MBoxInfo obox) {
  mbox_buffer_commit(obox->buffer);
  return;
}


