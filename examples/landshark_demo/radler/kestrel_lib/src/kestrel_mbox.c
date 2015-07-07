/* Include files */

#include "kestrel_mbox.h"

/* Function definitions */

uint64_t fit_into_buffer (uint32_t size) {
  uint64_t page_size;
  page_size = get_page_size();
  if (size == 0) {
    return 0;
  }
  return size - 1 / page_size + 1 * page_size;
}

uint64_t mbox_buffer_size (uint32_t size) {
  return fit_into_buffer(2 * size + mbuf_hdr_size);
}

constcharstar mbox_buffer_update (MBuf buf, intstar stale, uint32_t size) {
  if (buf->hdr.read_slot == 0) {
    *stale = buf->hdr.stale0;
    buf->hdr.stale0 = 1;
    return buf->data;
  }
  *stale = buf->hdr.stale1;
  buf->hdr.stale1 = 1;
  return buf->data + size;
}

charstar mbox_buffer_stall (MBuf buf, uint32_t size) {
  if (buf->hdr.read_slot == 0) {
    return buf->data + size;
  }
  return buf->data;
}

void mbox_buffer_release (MBuf buf) {
  return;
}

void mbox_buffer_commit (MBuf buf) {
  if (buf->hdr.read_slot == 0) {
    buf->hdr.stale1 = 0;
    buf->hdr.read_slot = 1;
    return;
  }
  buf->hdr.stale0 = 0;
  buf->hdr.read_slot = 0;
  return;
}


