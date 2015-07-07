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

#ifndef __FLGUI_H__
#define __FLGUI_H__

#include <iostream>
#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Counter.H>
#include <FL/Fl_File_Chooser.H>
#include <cstdlib>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

struct point_t {
  double x;
  double y;
  double z;
};

struct map_t {
  bool data;
  std::vector<point_t> points;
};

struct FlData {
  boost::mutex mutex;
  double speed;
  bool ccc_start;
  bool ccc_stop;
  bool pp_start;
  bool pp_stop;
  map_t map;
  size_t map_size;
  double latitude;
  double longitude;
  double altitude;
};

void make_window( FlData* );
int run();

#endif // __FLGUI_H__


