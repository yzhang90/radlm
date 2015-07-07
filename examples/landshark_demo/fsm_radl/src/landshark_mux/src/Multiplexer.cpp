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

#include "Multiplexer.h"

Multiplexer::Multiplexer() 
  : select_( SELECT_JOY )
{
}

int Multiplexer::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
{
#ifdef __DEBUG__
  std::cout << "[mux] step()" << std::flush;
#endif // __DEBUG__
  switch ( in->select->data ) {
    case SELECT_JOY:
    case SELECT_CCC:
    case SELECT_ACC:
    case SELECT_PATHPLANNER:
      select_ = in->select->data;
#ifdef __DEBUG__
      std::cout << "[muxer] " << (int) select_;
#endif // __DEBUG__
      break;
    case SELECT_NONE:
      break;
    default: 
      std::cerr << "Multiplexer received unknown code!" << std::endl;
  }
  switch ( select_ ) {
    case SELECT_JOY:
      out->base->angular = in->base_joy->angular;
      out->base->linear = in->base_joy->linear;
      break;
    case SELECT_CCC:
      out->base->angular = in->base_ccc->angular;
      out->base->linear = in->base_ccc->linear;
      break;
#ifndef NO_ACC
    case SELECT_ACC:
      out->base->angular = in->base_acc->angular;
      out->base->linear = in->base_acc->linear;
      break;
#endif // NO_ACC
    case SELECT_PATHPLANNER:
      out->base->angular = in->base_pp->angular;
      out->base->linear = in->base_pp->linear;
      break;
    default: 
      std::cerr << "Error!" << std::endl;
  }
  out->status->data = select_;
#ifdef __DEBUG__
  std::cout << "...done" << std::endl;
#endif // __DEBUG__
  return 0;
}


