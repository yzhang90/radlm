/*
 * Copyright (C) 2012, SRI International (R)
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

#ifndef __LANDSHARK_OCU_H__
#define __LANDSHARK_OCU_H__

#include RADL_HEADER
#include <zmq.hpp>
#include <string>
#include <unistd.h>
#include "ocu_server.h"

typedef enum {
  FRONT = 0,
  REAR
} imu_t;

class OcuGui 
{
  public: 
    OcuGui( );
    ~OcuGui( );
    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, 
        radl_out_t *out, radl_out_flags_t *out_flags );

  private:
    const uint8_t SELECT_NONE;
    const uint8_t SELECT_JOY;
    const uint8_t SELECT_CCC;
    const uint8_t SELECT_PP;

    const uint8_t STATUS_ESTOP;
    const uint8_t STATUS_JOY;
    const uint8_t STATUS_CCC;
    const uint8_t STATUS_PP;

    const uint8_t BASE_NORMAL;
    const uint8_t BASE_ESTOP;
    const uint8_t BASE_FAILURE;
    const size_t ARRAY_SIZE;

    ocu_server server;

  private:
    std::string get_fsm_status( const radl_in_t *in, const radl_in_flags_t *in_flags );
    std::string get_fsm_actuator( const radl_in_t *in, const radl_in_flags_t *in_flags );
    std::string get_base_status( const radl_in_t *in, const radl_in_flags_t *in_flags );
    std::string get_actuator_status( const radl_in_t *in, const radl_in_flags_t *in_flags );
    std::string get_rse_status( const radl_in_t *in, const radl_in_flags_t *in_flags );
    std::string get_ccc_status( const radl_in_t *in, const radl_in_flags_t *in_flags );
    std::string get_pp_status( const radl_in_t *in, const radl_in_flags_t *in_flags );
    std::string get_monitor_status( const radl_in_t *in, const radl_in_flags_t *in_flags );
    std::string get_gps( const radl_in_t *in, const radl_in_flags_t *in_flags );
    double get_magnetometer( const radl_in_t *in, const radl_in_flags_t *in_flags, const imu_t name );
};

#endif // __LANDSHARK_OCU_H__
