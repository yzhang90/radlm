#include "pubLandsharkGpsImu.h"
#include <stdio.h>
#include <math.h>

PubLandsharkGpsImu::PubLandsharkGpsImu() {
  this->cnt = 1;
  this->latitude_origin = 37.4571014863;
  this->longitude_origin = -122.173496991;
  this->gps_radius=0.0001000000;
  this->pi=3.14159265359;
  this->freq=100; 
  this->dt=1.0/freq;
}

void PubLandsharkGpsImu::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                  radl_out_t * out, radl_out_flags_t* outflags) {

  this->cnt++;

  out->landshark_gps_stub->latitude = \
  this->latitude_origin + this->gps_radius*cos(2.0*this->pi*this->cnt*this->dt);
  out->landshark_gps_stub->longitude = \
  this->longitude_origin + this->gps_radius*sin(2.0*this->pi*this->cnt*this->dt);
  double latitude = out->landshark_gps_stub->latitude;
  double longitude = out->landshark_gps_stub->longitude;
  double vel = 1.0;
  out->landshark_gps_vel_stub->x = vel; 

//try dummy values \ne 0.0 to see if fixes tf error in landshark_2dnav move_base
  double imu_orientation = 0.1;
  double imu_ang_vel = 0.1;
  double imu_lin_acc = 0.1;

  out->landshark_imu_stub->orientation.z = imu_orientation;
  out->landshark_imu_stub->orientation.w = imu_ang_vel;
  out->landshark_imu_stub->linear_acceleration.x = imu_lin_acc;
  
  printf("out->landshark_gps_stub->latitude = %15.10f\n",latitude);
  printf("out->landshark_gps_stub->longitude = %15.10f\n",longitude);
  printf("out->landshark_gps_vel_stub->vel = %15.10f\n",vel);
  printf("out->landshark_imu_stub->orientation.x = %15.10f\n",imu_orientation);

}
