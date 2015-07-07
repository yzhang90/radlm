/*** Utility to get the status of a CertiKOS buffer via the ioctl
 *** interface defined in the certikos_ivc_kmod kernel module.
 ***/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "certikos_ivc_kmod.h"

/* Table to map from status numbers to strings */
const char *status_names[3] = {
  "READY", "PENDING", "DEAD"
};

int main (int argc, char **argv) {
  int fd, err;
  shared_mem_state state;

  if (argc != 2) {
    fprintf (stderr, "Usage: %s <filename>\n", argv[0]);
    exit (-1);
  }

  fd = open (argv[1], O_RDWR);
  if (fd == -1) {
    fprintf (stderr, "Error opening %s: %s\n",
             argv[1], strerror (errno));
  }

  err = ioctl (fd, CERTIKOS_IOCTL_STATUS, &state);
  if (err == -1) {
    fprintf (stderr, "ioctl error: %s\n", strerror (errno));
  }

  printf ("Status: %i (%s)\n", state, status_names[state]);
  return 0;
}
