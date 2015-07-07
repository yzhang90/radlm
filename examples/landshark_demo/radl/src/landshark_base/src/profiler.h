#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>

#include RADL_HEADER

/**
 *
 */

class Profiler {
  private:
    std::vector<int64_t> values;
    std::vector<std::string> labels;
    uint64_t index;
    uint64_t frames;
    radl_duration_t t0;
    radl_duration_t printtime;
    boost::posix_time::ptime printtime_boost; 
    int64_t printperiod_ms;
    std::string filename;
    std::ofstream file;

    inline int64_t get_time_us( ) 
    {
      radl_duration_t t1( radl_gettime() );
      int64_t elapsed = ( t1 - t0 ) / 1000;
      t0 = t1;
      return elapsed;
    }

  public:
    inline Profiler( ) 
      : frames( 0 )
      , t0( radl_gettime() )
      , printtime( t0 )
      , printtime_boost( )
      , printperiod_ms( 1000 )
      , filename( "/home/hacms/base_profiler.log" )
    {
      std::cout << "Writing profiler log to " << filename << std::endl;
      file.open( filename.c_str() );
      file.precision( 4 );
    }


    inline ~Profiler() 
    {
      print_summary();
      std::cout << "Closing file " << filename << std::endl;
      file.close();
    }


    inline void init( ) 
    {
      frames++;
      index = 0;
      get_time_us();
    }


    inline void add( const std::string label ) 
    {
      add( label, get_time_us() );
    }

    inline void add( const std::string label, const int64_t value ) 
    {
      if ( index >= values.size() ) {
        values.push_back( value );
        labels.push_back( label );
      }
      else {
        values[index] = std::max( value, values[index] );
      }
      index++;
    }

    /**
     *
     */

    inline void print_header( ) {
      file << "loops, interval (s), boost interval (s)";
      for ( size_t i = 0; i < labels.size(); i++ ) {
        file << ", " << labels[i] << " (us)";
      }
      file << std::endl;
    }

    inline void clear( ) {
      frames = 0;
      for ( size_t i = 0; i < values.size(); i++ ) {
        values[i] = 0;
      }
      get_time_us();
    }

    inline void print_summary( ) {

      static bool first_time( true );
      if ( first_time ) {
        print_header();
        first_time = false;
      }
      radl_duration_t t1( radl_gettime() );
      boost::posix_time::ptime t1_boost = boost::posix_time::microsec_clock::local_time();
      int64_t elapsed_ms = ( t1 - printtime ) / 1e6;
      if ( elapsed_ms > printperiod_ms ) {
        int64_t elapsed_boost_ms = ( t1_boost - printtime_boost ).total_milliseconds();
        printtime_boost = t1_boost;
        printtime = t1;
        file << frames << ", " << elapsed_ms << ", " << elapsed_boost_ms; 
        for ( size_t i = 0; i < values.size(); i++ ) {
          file << ", " << values[i];
        }
        file << std::endl << std::flush;
        clear();
      }
    }

};
