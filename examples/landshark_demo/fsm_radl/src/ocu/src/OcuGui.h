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


#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "flgui.h"
#include RADL_HEADER

class OcuGui 
{
  public: 
    OcuGui( );
    ~OcuGui( );
    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );

  private:
    boost::thread thread_;
    FlData data_;
    const size_t size_;
};

#endif // __LANDSHARK_OCU_H__
