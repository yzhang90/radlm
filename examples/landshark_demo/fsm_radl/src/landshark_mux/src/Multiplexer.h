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

#ifndef __MULTIPLEXER_H__
#define __MULTIPLEXER_H__

#include <ros/ros.h>

#include RADL_HEADER

#define NO_ACC

enum select_t {
  SELECT_NONE = 0,
  SELECT_JOY = 1,
  SELECT_CCC = 2,
  SELECT_ACC = 3,
  SELECT_PATHPLANNER = 4
};

class Multiplexer
{
  private:
    uint8_t select_;

  public:
    Multiplexer(); 

    // @brief multiplexes based on in->select field.
    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );
};
#endif // __MULTIPLEXER_H__

