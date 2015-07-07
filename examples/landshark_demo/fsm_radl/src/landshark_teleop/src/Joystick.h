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

#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#define __DEBUG_JOYSTICK__ 0

enum joystick_status_t {
  JOYSTICK_OPEN=0,       // Normal state, joystick is connected and sending messages
  JOYSTICK_CLOSED,
  JOYSTICK_ERROR_SELECT,
  JOYSTICK_ERROR_FD_ISSET
};

class Joystick
{
  private:
    int event_count_;
    struct timeval tv_;
    double scale_;
    joystick_status_t status_;

  protected:
    std::string joy_dev_;
    double deadzone_;
    double unscaled_deadzone_; 

    std::vector<double> axes_;
    std::vector<int> buttons_;
    boost::posix_time::ptime event_time_;
    bool fresh_;
    bool init_;
    boost::mutex data_mutex_;

    inline long msec_since_last_event( ) {
      boost::mutex::scoped_lock lock( data_mutex_ );
      using namespace boost::posix_time;
      return ( microsec_clock::local_time() - event_time_ ).total_milliseconds();
    }

    void set_run( bool val ) 
    {
      boost::mutex::scoped_lock lock( run_mutex_ );
      run_flag_ = val;
    }

    bool get_run( ) 
    {
      boost::mutex::scoped_lock lock( run_mutex_ );
      return run_flag_;
    }

  private:
    bool run_flag_;
    boost::mutex run_mutex_;
    boost::thread thread_;

  public:
    Joystick( ) 
      : fresh_( false )
      , init_( false )
      , run_flag_( false )
      , deadzone_( 0.2 )
    {
    }

    void init( ) {
      unscaled_deadzone_ = 32767. * deadzone_;
      scale_ = -1. / (1. - deadzone_) / 32767.;
      event_count_ = 0;

      // set timeout interval 
      tv_.tv_sec = 0;
      tv_.tv_usec = 10000;

      // start thread
      set_run( true );
      thread_ = boost::thread( &Joystick::read_joy, this );
    }

    ~Joystick( ) 
    {
      set_run( false );
      thread_.join();
    }


    /**
     * @brief Read joystick device 
     * @returns 1 if a value is read
     *          0 if no value available (timeout)
     *          -1 select() returns -1
     *          -2 select() returns 1, but FD_ISSET( fd, &set ) returns false
     *          -3 select() returns 1, but read error happens
     *
     * We first use the select() function to check if there is data to be read. 
     * select() will return 1 if there is a value to be read, otherwise it returns 0.
     * A timeout will occur after a time period set in tv_.
     */

    int read_joy()
    {
      bool opened = false;
      bool first_attempt( true );
      fd_set set;
      js_event event;
      int joy_fd;
      status_ = JOYSTICK_CLOSED;

      using namespace boost::posix_time;
      event_time_ = microsec_clock::local_time();

      while ( get_run() ) {
        // If not opened, open it 
        if ( !opened ) {
          // There seems to be a bug in the driver or something where the
          // initial events that are to define the initial state of the
          // joystick are not the values of the joystick when it was opened
          // but rather the values of the joystick when it was last closed.
          // Opening then closing and opening again is a hack to get more
          // accurate initial state data.
          joy_fd = open(joy_dev_.c_str(), O_RDONLY);
          if (joy_fd != -1) {
            close(joy_fd);
            joy_fd = open(joy_dev_.c_str(), O_RDONLY);
            if (joy_fd != -1) {
              status_ = JOYSTICK_OPEN;
              opened = true;
              std::cout << "[joy] Opened joystick: " << joy_dev_ << std::endl;
            }
          }
        }
        // If still not opened, sleep and try again 
        if ( opened ) {
          first_attempt = true;
        }
        else { 
          if ( first_attempt )  {
            std::cout << "...disconnected! trying to connect..." << std::endl;
            first_attempt = false;
          }
          usleep( 1e6 ); 
          continue;
        }
        // If opened, read values until joystick is closed or there is an error
        while ( opened && get_run() ) {
          // check what these functions do
          FD_ZERO( &set );
          FD_SET( joy_fd, &set );
          // check if there is a value to be read
          int select_ret = select( joy_fd+1, &set, NULL, NULL, &tv_ );

#if __DEBUG_JOYSTICK__
          if ( select_ret != 0 ) {
            std::cout << "(select_ret= " << select_ret << " ) ";
          }
#endif // __DEBUG_JOYSTICK__

          if ( select_ret > 0 ) {
            event_time_ = microsec_clock::local_time();
            if ( FD_ISSET(joy_fd, &set)  ) {
              int readval = read(joy_fd, &event, sizeof(js_event));
              if ( readval == -1 ) { 
                std::cout << "[joy] read error: returned " << readval << ", errno = " << errno;
                if ( errno != EAGAIN) {
                  std::cout << " (disconnected)" << std::endl;
                  status_ = JOYSTICK_CLOSED;
                  usleep( 1e5 );
                  break;
                }
                else {
                  std::cout << " (unknown error)" << std::endl;
                }
              }
              else {
                int type_val = event.type & JS_EVENT_INIT;
                bool init = type_val > 0 ? true : false;
                event_count_++;
#if __DEBUG_JOYSTICK__
                std::cout << "event: count= " << event_count_ << ", event.type= " << (int) event.type;
                std::cout << ", event.number= " << (int) event.number << ", val= " << event.value;
                std::cout << ", init = " << init << ", select_ret= " << select_ret;
                std::cout << std::endl;
#endif // __DEBUG_JOYSTICK__
                boost::mutex::scoped_lock lock( data_mutex_ );
                fresh_ = true;
                switch( event.type ) {
                  case JS_EVENT_BUTTON | JS_EVENT_INIT:
                    if (event.number >= buttons_.size()) {
                      int old_size = buttons_.size();
                      buttons_.resize(event.number+1);
                    }
                    std::cout << "[joy] init buttons[" << (int) event.number << "] = " << event.value << std::endl;
                    init_ = true;

                  case JS_EVENT_BUTTON:
                    if ( buttons_.size() > (int) event.number ) {
                      buttons_[event.number] = (event.value ? 1 : 0);
                    }
                    else {
                      std::cerr << "Buttons number is too high!" << std::endl;
                    }
                    break;

                  case JS_EVENT_AXIS | JS_EVENT_INIT:
                    if (event.number >= axes_.size()) {
                      int old_size = axes_.size();
                      axes_.resize(event.number+1);
                    }
                    init_ = true;
                    std::cout << "[joy] init axes[" << (int) event.number << "] = " << event.value << std::endl;

                  case JS_EVENT_AXIS:
                    if ( axes_.size() > event.number ) {
                      double val = event.value;
                      // Allows deadzone to be "smooth"
                      if (val > unscaled_deadzone_) {
                        val -= unscaled_deadzone_;
                      }
                      else if (val < -unscaled_deadzone_) {
                        val += unscaled_deadzone_;
                      }
                      else {
                        val = 0;
                      }
                      axes_[event.number] = val * scale_;
                    }
                    else {
                      std::cerr << "[joy] Axes number is too high!" << std::endl;
                    }
                    break;

                  default:
                    std::cerr << "[joy] Unknown event: type= " << event.type << std::endl;
                }
              }
            }
            else {
              // If val > 0, then FD_ISSET( 0, &) should be true according to manpage of select
              // maybe, the joystick should be reset 
              std::cout << "unknown error!" << std::endl;
              status_ = JOYSTICK_ERROR_FD_ISSET;
            }
          }
          else if ( select_ret == 0 ) {
          }
          else { // if (val < 0 ) 
            perror( "[joy] error in select()" );
            usleep( 10000 );
            status_ = JOYSTICK_ERROR_SELECT;
          }
          usleep( 5000 );
        }
        std::cout << "[joy] Quit loop" << std::endl;
        if ( opened ) {
          std::cout << "[joy] Closing joystick..." << std::endl;
          close(joy_fd);
          opened = false;
        }
      }
    }
};

#endif // __JOYSTICK_H__
