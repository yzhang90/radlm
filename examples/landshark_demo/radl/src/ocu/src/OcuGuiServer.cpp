/*
 * Copyright (C) 2014, SRI International (R)
 *
 * The material contained in this release is copyrighted. It may not be copied,
 * reproduced, translated, reverse engineered, modified or reduced to any electronic
 * medium or machine-readable form without the prior written consent of
 * SRI International (R).
 *
 * Portions of files in this release may be unpublished work
 * containing SRI International (R) CONFIDENTIAL AND PROPRIETARY INFORMATION.
 * Disclosure, use, reverse engineering, modification, or reproduction without
 * written authorization of SRI International (R) is likewise prohibited.
 *
 * Author(s): Aravind Sundaresan (aravind@ai.sri.com)
 *
 */

#include "OcuGuiServer.h"
#include <iostream>
#include <sstream>
#include <cmath>

inline std::string flag_to_string( const uint8_t flag ) 
{
  std::stringstream ss;
  ss << "stale: v=" << radl_is_value_stale( flag );
  ss << ", m=" << radl_is_mbox_stale( flag );
  ss << ", timeout: v=" << radl_is_value_timeout( flag );
  ss << ", m=" << radl_is_mbox_timeout( flag );
  return ss.str();
}

OcuGui::OcuGui( ) 
  : SELECT_NONE( radlast_constants()->ControllerSelect->NONE )
  , SELECT_JOY( radlast_constants()->ControllerSelect->JOY )
  , SELECT_CCC( radlast_constants()->ControllerSelect->CCC )
  , SELECT_PP( radlast_constants()->ControllerSelect->PP )
  , STATUS_ESTOP( radlast_constants()->ControllerStatus->ESTOP )
  , STATUS_JOY( radlast_constants()->ControllerStatus->JOY )
  , STATUS_CCC( radlast_constants()->ControllerStatus->CCC )
  , STATUS_PP( radlast_constants()->ControllerStatus->PP )
  , BASE_NORMAL( radlast_constants()->BaseStatus->NORMAL )
  , BASE_ESTOP( radlast_constants()->BaseStatus->ESTOP )
  , BASE_FAILURE( radlast_constants()->BaseStatus->FAILURE )
  , ARRAY_SIZE( *RADL_THIS->array_size )
{
}

OcuGui::~OcuGui() 
{
}

int OcuGui::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
{
  // Process GUI commands
  server.receive();

  // CCC
  out->ccc_speed->data = server.ccc_speed;

  // controller select
  if ( server.select_joy ) {
    std::cout << "Selected JOY" << std::endl;
    out->select->data = SELECT_JOY;
  }
  else if ( server.select_ccc ) {
    std::cout << "Selected CCC" << std::endl;
    out->select->data = SELECT_CCC;
  }
  else if ( server.select_pp ) {
    std::cout << "Selected PP" << std::endl;
    out->select->data = SELECT_PP;
  }
  else {
    out->select->data = SELECT_NONE;
  }
  server.select_joy = server.select_ccc = server.select_pp = false;

  // PP 
  out->pp_goal->latitude = server.pp_goal_lat;
  out->pp_goal->longitude = server.pp_goal_lon;
  out->pp_goal->altitude = server.pp_goal_alt;

  out->pp_map->data = false;

  std::stringstream ss;
  if ( server.pp_map.data ) {
    ss << "Loaded map (" << server.pp_map.points.size() << " points)";
    if ( server.pp_map.points.size() == ARRAY_SIZE ) {
      for ( size_t i = 0; i < server.pp_map.points.size(); i++ ) {
        out->pp_map->points[i].x = server.pp_map.points[i].x;
        out->pp_map->points[i].y = server.pp_map.points[i].y;
        out->pp_map->points[i].z = server.pp_map.points[i].z;
      }
      out->pp_map->data = true;
      ss << ": OK";
    }
    else {
      ss << ": failure";
    }
    server.pp_map.data = false;
  }

  // Process system updates
  server.fsm_status = get_fsm_status( in, in_flags );
  server.fsm_actuator = get_fsm_actuator( in, in_flags );
  server.base_status = get_base_status( in, in_flags );
  server.actuator_status = get_actuator_status( in, in_flags );
  server.rse_status = get_rse_status( in, in_flags );
  server.ccc_status = get_ccc_status( in, in_flags );
  server.pp_status = get_pp_status( in, in_flags );
  server.monitor_status = get_monitor_status( in, in_flags );
  
  // Get sensor values
  server.gps = get_gps( in, in_flags );
  server.mag_front = get_magnetometer( in, in_flags, FRONT );
  server.mag_rear = get_magnetometer( in, in_flags, REAR );

  server.send();

  // set flags
  out_flags->select = 0;
  out_flags->pp_map = 0;
  out_flags->pp_goal = 0;
  out_flags->ccc_speed = 0;
}

std::string OcuGui::get_fsm_status( const radl_in_t *in, const radl_in_flags_t *in_flags ) 
{
  std::stringstream ss;
  ss << "FSM: " <<  flag_to_string( in_flags->controller_status ) << ", state: ";
  if ( in->controller_status->data == STATUS_JOY ) {
    ss << "JOY";
  }
  else if ( in->controller_status->data == STATUS_CCC ) {
    ss << "CCC";
  }
  else if ( in->controller_status->data == STATUS_PP ) {
    ss << "PP";
  }
  else if ( in->controller_status->data == STATUS_ESTOP ) {
    ss << "ESTOP";
  }
  else {
    ss << "UNKNOWN";
  }
  return ss.str();
}

std::string OcuGui::get_fsm_actuator( const radl_in_t *in, const radl_in_flags_t *in_flags ) 
{
  std::stringstream ss;
  ss.precision( 2 );
  ss << "FSM: " << flag_to_string( in_flags->base ) << ", ";
  ss << "linear= [" << in->base->linear.x << ", "
    << in->base->linear.y << ", " 
    << in->base->linear.z << "]  ";
  ss << "angular= [" << in->base->angular.x << ", "
    << in->base->angular.y << ", " 
    << in->base->angular.z << "]  ";
  return ss.str();
}

std::string OcuGui::get_base_status( const radl_in_t *in, const radl_in_flags_t *in_flags ) 
{
  std::stringstream ss;
  ss.precision( 2 );
  ss << "Base: " << flag_to_string( in->base_status->flags );
  ss << ", seq= " << in->base_status->seq << ", l= " << in->base_status->left << ", r= " << in->base_status->right;
  ss << ", status= ";
  if ( in->base_status->data == BASE_NORMAL ) {
    ss << "NORMAL";
  }
  else if ( in->base_status->data == BASE_ESTOP ) {
    ss << "ESTOP";
  }
  else if ( in->base_status->data == BASE_FAILURE) {
    ss << "FAILURE";
  }
  else {
    ss << "UNKNOWN";
  }
  return ss.str();
}

std::string OcuGui::get_actuator_status( const radl_in_t *in, const radl_in_flags_t *in_flags ) 
{
  std::stringstream ss;
  ss.precision( 2 );
  ss << "Actuator: seq= " << in->actuator_status->seq << ", l= " << in->actuator_status->left << ", r= " << in->actuator_status->right;
  ss << ", elapsed: boost= " << in->base_status->elapsed_boost_time;
  ss << "s, radl= " << in->base_status->elapsed_radl_time << "s";
  return ss.str();
}

std::string OcuGui::get_rse_status( const radl_in_t *in, const radl_in_flags_t *in_flags ) 
{
  std::stringstream ss;
  ss << "RSE: " << flag_to_string( in_flags->rse_status );
  ss << ", status= ";
  if ( in->rse_status->data ==  radlast_constants()->RSEStatus->INIT ) {
    ss << "Init" ;
  }
  else if ( in->rse_status->data ==  radlast_constants()->RSEStatus->GOOD ) {
    ss << "Good" ;
  }
  else if ( in->rse_status->data ==  radlast_constants()->RSEStatus->BAD ) {
    ss << "Bad" ;
  }
  else {
    ss << "Unknown" ;
  }
  return ss.str();
}

std::string OcuGui::get_pp_status( const radl_in_t *in, const radl_in_flags_t *in_flags ) 
{
  std::stringstream ss;
  ss << "PP: " << flag_to_string( in_flags->pp_status );
  ss << ", status= ";
  if ( in->pp_status->data ==  radlast_constants()->PPStatus->OFF ) {
    ss << "OFF" ;
  }
  else if ( in->pp_status->data ==  radlast_constants()->PPStatus->ON ) {
    ss << "ON" ;
  }
  else if ( in->pp_status->data ==  radlast_constants()->PPStatus->ENGAGED ) {
    ss << "ENGAGED" ;
  }
  else if ( in->pp_status->data ==  radlast_constants()->PPStatus->GOAL_SUCCESS ) {
    ss << "GOAL_SUCCESS" ;
  }
  else if ( in->pp_status->data ==  radlast_constants()->PPStatus->GOAL_FAIL ) {
    ss << "GOAL_FAIL" ;
  }
  else if ( in->pp_status->data ==  radlast_constants()->PPStatus->FAILURE ) {
    ss << "FAILURE" ;
  }
  else if ( in->pp_status->data ==  radlast_constants()->PPStatus->TRYING ) {
    ss << "TRYING" ;
  }
  else {
    ss << "Unknown" ;
  }
  return ss.str();
}

std::string OcuGui::get_monitor_status( const radl_in_t *in, const radl_in_flags_t *in_flags ) 
{
  std::stringstream ss;
  ss << "DW Monitor: " << flag_to_string( in_flags->monitor_estop );
  ss << ", value= ";
  if ( in->monitor_estop->data ==  radlast_constants()->EStop->NONE ) {
    ss << "None" ;
  }
  else if ( in->monitor_estop->data ==  radlast_constants()->EStop->SET ) {
    ss << "Set" ;
  }
  else if ( in->monitor_estop->data ==  radlast_constants()->EStop->RESET ) {
    ss << "Reset" ;
  }
  else {
    ss << "Unknown" ;
  }
  return ss.str();
}

std::string OcuGui::get_ccc_status( const radl_in_t *in, const radl_in_flags_t *in_flags ) 
{
  std::stringstream ss;
  ss << "CCC: " << flag_to_string( in_flags->ccc_status );
  ss << ", status= ";
  if ( in->ccc_status->data ==  radlast_constants()->CCCStatus->OFF ) {
    ss << "OFF" ;
  }
  else if ( in->ccc_status->data ==  radlast_constants()->CCCStatus->ON ) {
    ss << "ON" ;
  }
  else if ( in->ccc_status->data ==  radlast_constants()->CCCStatus->ENGAGED_GUARANTEE ) {
    ss << "ENGAGED_GUARANTEE" ;
  }
  else if ( in->ccc_status->data ==  radlast_constants()->CCCStatus->ENGAGED_NO_GUARANTEE ) {
    ss << "ENGAGED_NO_GUARANTEE" ;
  }
  else if ( in->ccc_status->data ==  radlast_constants()->CCCStatus->FAILURE ) {
    ss << "FAILURE" ;
  }
  else if ( in->ccc_status->data ==  radlast_constants()->CCCStatus->TRYING ) {
    ss << "TRYING" ;
  }
  else {
    ss << "Unknown" ;
  }
  return ss.str();
}

std::string OcuGui::get_gps( const radl_in_t *in, const radl_in_flags_t *in_flags ) 
{
  std::stringstream ss;
  double lat = in->gps_navsatfix->latitude;
  double lon = in->gps_navsatfix->longitude;
  double alt = in->gps_navsatfix->altitude;
  ss.setf( std::ios_base::fixed, std::ios_base::floatfield );
  ss.precision( 6 );
  ss << "GPS: " << fabs( lat ) << ( lat >= 0 ? " N, " : " S " );
  ss << fabs( lon ) << ( lon >= 0 && lon < 180 ? " E, " : " W " );
  ss.precision( 4 );
  ss << alt;
  ss.precision( 3 );
  ss << "m, linear= [" << in->gps_velocity->linear.x << ", "
    << in->gps_velocity->linear.y << ", " 
    << in->gps_velocity->linear.z;
  ss << "],  angular= [" << in->gps_velocity->angular.x << ", "
    << in->gps_velocity->angular.y << ", " 
    << in->gps_velocity->angular.z << "]  ";
  
  return ss.str();
}

/**
 * @brief convert magnetometer vector to direction (0 is N, 90 is E)
 */

double vector_to_north( double x, double y, double z ) {
  double val = atan2( y, x ) * 360 / M_PI;
  val = val < 0 ? val + 360 : val > 360 ? val - 360 : val;
  return val;
};

double OcuGui::get_magnetometer( const radl_in_t *in, const radl_in_flags_t *in_flags, const imu_t name ) {
  if ( name == FRONT ) {
    return vector_to_north( in->front_magnetometer->vector.x, in->front_magnetometer->vector.y, in->front_magnetometer->vector.z );
  }
  else {
    return vector_to_north( in->rear_magnetometer->vector.x, in->rear_magnetometer->vector.y, in->rear_magnetometer->vector.z );
  }
}

