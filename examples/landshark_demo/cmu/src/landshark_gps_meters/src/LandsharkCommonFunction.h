/*
 * Copyright (C) 2012, SRI International
 *
 * The material contained in this release is copyrighted. It may not be copied,
 * reproduced, translated, reverse engineered, modified or reduced to any
 * electronic medium or machine-readable form without the prior written consent
 * of REARM team .
 *
 * Portions of files in this release may be unpublished work
 * containing REARM team CONFIDENTIAL AND PROPRIETARY INFORMATION.
 * Disclosure, use, reverse engineering, modification, or reproduction without
 * written authorization of REARM team is likewise prohibited.
 *
 * Author(s): Thomas de Candia (thomasd@ai.sri.com)
 *
 * $Id$
 *
 */

#ifndef LANDSHARKCOMMONFUNCTION_H
#define LANDSHARKCOMMONFUNCTION_H

#include <assert.h>
#include <math.h>

namespace landshark
{
  template <typename T>
  bool IsWithinRange(T variable, T min, T max)
  {
    assert(max > min);

    if ((variable >= min) && (variable <= max))
    {
      return true;
    }

    return false;
  }

  template <typename T>
  T ClampMinMax(T variable, T min, T max)
  {
    assert(max > min);

    if (variable < min)
    {
      return min;
    }

    if (variable > max)
    {
      return max;
    }

    return variable;
  }

  template <typename T>
  bool HaveSameSign(T argA, T argB)
  {
    if ((argA >= 0) && (argB >= 0))
    {
      return true;
    }

    if ((argA <= 0) && (argB <= 0))
    {
      return true;
    }

    return false;
  }

  template <typename T>
  T Sign(T arg)
  {
    if (arg >= 0.0)
    {
      return 1.0;
    }

    return -1.0;
  }

  template <typename T>
  T Round(T arg)
  {
    if (arg >= 0.0)
    {
      return floor(arg + 0.5);
    }

    return ceil(arg - 0.5);
  }
} // namespace

#endif
