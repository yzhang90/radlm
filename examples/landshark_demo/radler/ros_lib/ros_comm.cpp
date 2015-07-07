/*
 * Created on Nov, 2014
 * @author: Léonard Gérard leonard.gerard@sri.com
 *
 * Ros communication handling under the generic radl comm API.
 *
 */

#include "ros_comm.h"
#include <ros/ros.h>

/*
 * One call to ros_vm_start per VM.
 *
 */


/*
 * ROS initialization is very basic,
 * there needs to be one and only one rosmaster per system.
 * The rosmaster is launched in the master VM.
 * In every VMs, the ROS_MASTER_URI needs to be set to the master system.
 */



/*****
 ***** VM Level
 *****/

typedef struct {
  bool master_vm;
  char* master_uri;
} ros_radl_vm_info;

typedef struct {
  int master_process_id;
} ros_vm_info;

ros_vm_info vm_info;


int launch_roscore() {
  // Returns the process Id of the launched roscore
  static bool launched = false;
  if (!launched) {
    //TODO launch roscore with correct user privilege
    //TODO if it can't be launched throw an error

  }
  return 42; //TODO return the correct process id
}

void ros_vm_start(const ros_radl_vm_info* i) {
  setenv("ROS_MASTER_URI", i->master_uri, 1);
  if (i->master_vm) {
    vm_info.master_process_id = launch_roscore();
  }
}

void ros_vm_stop(const ros_radl_vm_info* i) {
  if (i->master_vm) {
    kill(vm_info.master_process_id, SIGINT);
  }
}


/*****
 ***** Node Level
 *****/


/*
 * ROS node level protocol and channel setups.
 * This is the usual level of action in ROS.
 */

typedef struct {
  ros::Publisher pub;

} ros_pub_info;

typedef struct {
  ros::Subscriber sub;
} ros_sub_info;

void ros_node_start();
ros_mbox_info* ros_node_sub_init(ros_radl_sub_info *chan);
ros_pub_info* ros_node_pub_init(ros_radl_pub_info *chan);
void ros_node_finish();



/***
 *** Mailbox writing and reading
 ***/

/*
 * Update is used to get the most recent mailbox value.
 * It returns a generic pointer to it,
 * and ensure the value doesn't change until release.
 */
const char* ros_ipc_mbox_update (ros_ipc_mbox_info *mbox, int *stale);
const char* ros_ip_mbox_update (ros_ip_mbox_info *mbox, int *stale);
const char* ros_ivc_mbox_update (ros_ivc_mbox_info *mbox, int *stale);

/*
 * Stall is used to the buffer where to write the value to be sent.
 * It returns a generic pointer to an allocated memory,
 * and it ensures the memory won't be interfered until commit.
 */
char* ros_ipc_mbox_stall (ros_ipc_mbox_info *mbox);
char* ros_ip_mbox_stall (ros_ip_mbox_info *mbox);
char* ros_ivc_mbox_stall (ros_ivc_mbox_info *mbox);

/*
 * Release is used to give back ownership of the input mbox.
 */
void ros_ipc_mbox_release (ros_ipc_mbox_info *mbox);
void ros_ip_mbox_release (ros_ip_mbox_info *mbox);
void ros_ivc_mbox_release (ros_ivc_mbox_info *mbox);

/*
 * Commit gives back the output buffer and ask for its value to be sent.
 */
void ros_ipc_mbox_commit (ros_ipc_mbox_info *mbox);
void ros_ip_mbox_commit (ros_ip_mbox_info *mbox);
void ros_ivc_mbox_commit (ros_ivc_mbox_info *mbox);




