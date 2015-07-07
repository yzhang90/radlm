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
 * Author(s): Thomas de Candia (thomasd@ai.sri.com)
 *            Aravind Sundaresan (aravind@ai.sri.com)
 *
 */

#ifndef __LANDSHARK_JOYSTICK_H__
#define __LANDSHARK_JOYSTICK_H__

#include "Joystick.h"
#include <ros/ros.h>

#include RADL_HEADER

class LandsharkJoystick : public Joystick 
{
  private:
    const double max_linear_;
    const double max_angular_;
    double heartbeat_timeout_ms_;
    size_t nbuttons_;
    size_t naxes_;
    uint8_t jstype_;
    const uint8_t XBOX_WIRED;
    const uint8_t XBOX_WIRELESS;
  public: 
    LandsharkJoystick( );
    virtual void translate( std::vector<double>& axes, std::vector<int>& buttons, radl_out_t *out, bool zero );
    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );
};

#endif // __LANDSHARK_JOYSTICK_H__


