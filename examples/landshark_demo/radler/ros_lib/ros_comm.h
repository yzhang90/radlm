/*
 * Created on Nov, 2014
 * @author: Léonard Gérard leonard.gerard@sri.com
 *
 * Ros communication handling under the generic radl comm API.
 *
 */

#pragma once

//TODO change the API to remove the useless '_init'

/*
 * ROS VM level protocol and channel setups.
 * Most of them aren't doing anything except complying to the generic API.
 * The start function are initializing a ros master local to the VM.
 */

typedef struct {
} ros_ipc_chan_info;

typedef struct {
} ros_ip_chan_info;

typedef struct {
} ros_ivc_chan_info;

void ros_vm_start_ipc_init();
ros_ipc_chan_info* ros_vm_init_ipc_chan(radl_ipc_chan_info *chan);
void ros_vm_finish_ipc_init();

void ros_vm_start_ip_init();
ros_ip_chan_info* ros_vm_init_ip_chan(radl_ip_chan_info *chan);
void ros_vm_finish_ip_init();

void ros_vm_start_ivc_init();
ros_ivc_chan_info* ros_vm_init_ivc_chan(radl_ivc_chan_info *chan);
void ros_vm_finish_ivc_init();

/*
 * ROS node level protocol and channel setups.
 * This is the usual level of action in ROS.
 */

typedef struct {
} ros_ipc_mbox_info;

typedef struct {
} ros_ip_mbox_info;

typedef struct {
} ros_ivc_mbox_info;

void ros_node_start_ipc_init();
ros_ipc_mbox_info* ros_node_init_ipc_chan(ros_ipc_chan_info *chan);
void ros_node_finish_ipc_init();

void ros_node_start_ip_init();
ros_ip_mbox_info* ros_node_init_ip_chan(ros_ip_chan_info *chan);
void ros_node_finish_ip_init();

void ros_node_start_ivc_init();
ros_ivc_mbox_info* ros_node_init_ivc_chan(ros_ivc_chan_info *chan);
void ros_node_finish_ivc_init();

/***
 *** Mailbox writing and reading
 ***/

/*
 * Update is used to get the most recent mailbox value.
 * It returns a generic pointer to it,
 * and ensure the value doesn't change until release.
 */
const char* ros_ipc_mbox_update(ros_ipc_mbox_info *mbox, int *stale);
const char* ros_ip_mbox_update(ros_ip_mbox_info *mbox, int *stale);
const char* ros_ivc_mbox_update(ros_ivc_mbox_info *mbox, int *stale);

/*
 * Stall is used to the buffer where to write the value to be sent.
 * It returns a generic pointer to an allocated memory,
 * and it ensures the memory won't be interfered until commit.
 */
char* ros_ipc_mbox_stall(ros_ipc_mbox_info *mbox);
char* ros_ip_mbox_stall(ros_ip_mbox_info *mbox);
char* ros_ivc_mbox_stall(ros_ivc_mbox_info *mbox);

/*
 * Release is used to give back ownership of the input mbox.
 */
void ros_ipc_mbox_release(ros_ipc_mbox_info *mbox);
void ros_ip_mbox_release(ros_ip_mbox_info *mbox);
void ros_ivc_mbox_release(ros_ivc_mbox_info *mbox);

/*
 * Commit gives back the output buffer and ask for its value to be sent.
 */
void ros_ipc_mbox_commit(ros_ipc_mbox_info *mbox);
void ros_ip_mbox_commit(ros_ip_mbox_info *mbox);
void ros_ivc_mbox_commit(ros_ivc_mbox_info *mbox);

