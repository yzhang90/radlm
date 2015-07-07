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

#include "flgui.h"
//#include RADL_HEADER

void speed_cb( Fl_Widget*, void* );  
void ccc_start_cb( Fl_Widget*, void* );  
void ccc_stop_cb( Fl_Widget*, void* );  
void quit_cb( Fl_Widget*, void* ); 
void open_cb( Fl_Widget*, void* ); 
void lat_cb( Fl_Widget*, void* ); 
void lon_cb( Fl_Widget*, void* ); 
void alt_cb( Fl_Widget*, void* ); 
void pp_start_cb( Fl_Widget*, void* ); 
void pp_stop_cb( Fl_Widget*, void* ); 

void read_map( const std::string& , const size_t, map_t& );

const int PAD( 10 );

void make_window( FlData *data )
{
   Fl_Window* win= new Fl_Window( 480, 360, "Ocu GUI");

   win->begin();		
     // --
     Fl_Counter*	speed = new Fl_Counter( PAD, PAD, 120, 30, "Speed (m/s)" );		 //child 0
     Fl_Button*  ccc_start = new Fl_Button( 160, PAD,  90, 30, "S&tart CCC" ); //child 1
     Fl_Button*  ccc_stop = new Fl_Button( 280, PAD,  90, 30, "S&top CCC" ); //child 2
     // --
     Fl_Output*	out = new Fl_Output( 40, 120, 330, 30, "Map" );	//child 3
     Fl_Button*	map = new Fl_Button( 400, 120, 70, 30, "Select" );		 //child 4
     // --
     Fl_Input*	lat = new Fl_Input( 40, 180, 70, 30, "Lat." );		 //child 5
     Fl_Input*	lon = new Fl_Input( 150, 180, 70, 30, "Lon." );		 //child 6
     Fl_Input*	alt = new Fl_Input( 290, 180, 70, 30, "Alt (m)" );		 //child 7
     Fl_Button*	pp_start = new Fl_Button( 370, 180, 40, 30, "Go" );		 //child 8
     Fl_Button*	pp_stop = new Fl_Button( 420, 180, 50, 30, "Stop" );		 //child 9
     // --
     Fl_Button* quit = new Fl_Button( PAD, 240,  70, 30, "&Quit" );   //child 9
   win->end();

   speed->callback( (Fl_Callback*) speed_cb, data );
   ccc_start->callback( (Fl_Callback*) ccc_start_cb, data );
   ccc_stop->callback( (Fl_Callback*) ccc_stop_cb, data );
   quit->callback( (Fl_Callback*) quit_cb);
   map->callback( (Fl_Callback*) open_cb, data);
   lat->callback( (Fl_Callback*) lat_cb, data);
   lon->callback( (Fl_Callback*) lon_cb, data);
   alt->callback( (Fl_Callback*) alt_cb, data);
   pp_start->callback( (Fl_Callback*) pp_start_cb, data);
   pp_stop->callback( (Fl_Callback*) pp_stop_cb, data);
   win->show();
}

void lat_cb( Fl_Widget* obj , void* d )
{
  FlData *data = (FlData *) d;
  Fl_Input* counter = ( Fl_Input*)obj;
  boost::mutex::scoped_lock( data->mutex );
  data->latitude = atof( counter->value() );
}

void lon_cb( Fl_Widget* obj , void* d )
{
  FlData *data = (FlData *) d;
  Fl_Input* counter = ( Fl_Input*)obj;
  boost::mutex::scoped_lock( data->mutex );
  data->longitude = atof( counter->value() );
}

void alt_cb( Fl_Widget* obj , void* d )
{
  FlData *data = (FlData *) d;
  Fl_Input* counter = ( Fl_Input*)obj;
  boost::mutex::scoped_lock( data->mutex );
  data->altitude = atof( counter->value() );
}

void pp_start_cb( Fl_Widget* obj , void* d )
{
  FlData *data = (FlData *) d;
  Fl_Button* button=(Fl_Button*)obj;
  boost::mutex::scoped_lock( data->mutex );
  data->pp_start = true;
}

void pp_stop_cb( Fl_Widget* obj , void* d )
{
  FlData *data = (FlData *) d;
  Fl_Button* button=(Fl_Button*)obj;
  boost::mutex::scoped_lock( data->mutex );
  data->pp_stop = true;
}

void speed_cb( Fl_Widget* obj , void* d )
{
  FlData *data = (FlData *) d;
  Fl_Counter* counter = ( Fl_Counter*)obj;
  data->speed = counter->value();
}

void ccc_start_cb( Fl_Widget* obj , void* d )
{
  FlData *data = (FlData *) d;
  Fl_Button* button=(Fl_Button*)obj;
  boost::mutex::scoped_lock( data->mutex );
  data->ccc_start = true;
}

void ccc_stop_cb( Fl_Widget* obj , void* d )
{
  FlData *data = (FlData *) d;
  Fl_Button* button=(Fl_Button*)obj;
  boost::mutex::scoped_lock( data->mutex );
  data->ccc_stop = true;
}

void open_cb(Fl_Widget* obj, void* d) {
  FlData *data = (FlData *) d;
  Fl_Button* button=(Fl_Button*)obj;

  // Create the file chooser, and show it
  Fl_File_Chooser chooser(".",                        // directory
      "*",                        // filter
      Fl_File_Chooser::MULTI,     // chooser type
      "Title Of Chooser");        // title
  chooser.show();

  // Block until user picks something.
  while(chooser.shown()) {
    Fl::wait(); 
  }

  // User hit cancel?
  if ( chooser.value() == NULL ) { 
    std::cout << "(User hit 'Cancel')\n" << std::endl;
    return;
  }

  std::string file;
  // Multiple files? Show all of them
  if ( chooser.count() > 1 ) {
    std::cerr << "You should select only one file (using first)!" << std::endl;
    file = std::string( chooser.value(0) );
  }
  else {
    file = std::string( chooser.value() );
  }
  std::cout << "User selected: " << file << std::endl;
  {
    boost::mutex::scoped_lock( data->mutex );
    read_map( file, data->map_size, data->map );
  }
  if ( file.size() > 30 ) {
    file.replace( 0, file.size() - 27, "..." );
  }

  (  (Fl_Output*)(button->parent()->child(3))  )->value( file.c_str() );

}

void quit_cb( Fl_Widget* obj, void*)
{
  exit(0);
}

/**
 *
 */

int run() {
  return Fl::run();
}

/**
 * Read map from user specified file and set data equal to true
 */

void read_map( const std::string& file, const size_t map_size, map_t& map ) {
  std::cout << "Reading map from " << file << std::endl;
  // this function should read the file and populate the map structure
  std::cout << "Map reading function has not been implemented!" << std::endl;
  map.data = true;
}


