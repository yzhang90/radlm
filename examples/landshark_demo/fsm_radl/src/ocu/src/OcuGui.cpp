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

#include "OcuGui.h"

OcuGui::OcuGui( ) 
  : size_( *RADL_THIS->array_size )
{
  data_.map_size = size_;
  data_.map.points.resize( size_ );
  std::cout << "OcuGui: Map size= " << size_ << std::endl;
  make_window( &data_ );
  thread_ = boost::thread( run );
}

OcuGui::~OcuGui() 
{
  std::cout << "waiting for thread" << std::endl;
  thread_.join();
}


int OcuGui::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
{
  boost::mutex::scoped_lock lock( data_.mutex );
  // set goal 
  out->goal->latitude = data_.latitude;
  out->goal->longitude = data_.longitude;
  out->goal->altitude = data_.altitude;

  // XXX We assume map is an fixed size array of points
  out->map->data = data_.map.data;
  if ( data_.map.data ) {
    assert( data_.map.points.size() == size_ );
    for ( size_t i = 0; i < data_.map.points.size(); i++ ) {
      out->map->points[i].x = data_.map.points[i].x;
      out->map->points[i].y = data_.map.points[i].y;
      out->map->points[i].z = data_.map.points[i].z;
    }
    data_.map.data = false;
  }

  // send out planner start / stop signals
  out->pp->data = radlast_constants()->PathPlanner->NONE;
  if ( data_.pp_start ) {
    out->pp->data = radlast_constants()->PathPlanner->START;
    data_.pp_start = false;
    std::cout << "PP start: Goal= (" << data_.latitude << ",";
    std::cout << data_.longitude << "," << data_.altitude << ")" << std::endl;

  }
  else if ( data_.pp_stop ) {
    out->pp->data = radlast_constants()->PathPlanner->STOP;
    data_.pp_stop = false;
  }
}

