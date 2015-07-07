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

#include "LandsharkJoystick.h"

/**
 *
 */

LandsharkJoystick::LandsharkJoystick( )
  : Joystick( )
  , max_linear_( 2 )
  , max_angular_( M_PI )
  , heartbeat_timeout_ms_( 1000 )
  , jstype_( 0 )
  , XBOX_WIRED( *RADL_THIS->XBOX_WIRED )
  , XBOX_WIRELESS( *RADL_THIS->XBOX_WIRELESS )
{
  joy_dev_ = *RADL_THIS->device;
  deadzone_ = *RADL_THIS->deadzone;
  heartbeat_timeout_ms_ = radl_to_nsec( *RADL_THIS->heartbeat_timeout ) * 1e-6;
  jstype_ = *RADL_THIS->jstype;

  std::cout << "[joy] device " << joy_dev_;
  std::cout << " [PENN STEP MOD]";

  if ( jstype_ == XBOX_WIRED ) {
    naxes_ = 8;
    nbuttons_ = 11;
    heartbeat_timeout_ms_ = 0;
    std:: cout << ", jstype = WIRED" << std::endl;
  }
  else if ( jstype_ == XBOX_WIRELESS ) {
    naxes_ = 6;
    nbuttons_ = 15;
    std:: cout << ", jstype = WIRELESS" << std::endl;
  }
  else {
    jstype_ = 0;
    std:: cout << ", jstype = UNKNOWN" << std::endl;
  }

  init( );
}

void LandsharkJoystick::translate( std::vector<double>& axes, std::vector<int>& buttons, radl_out_t *out, bool zero )
{
  double base_rot( 0 );
  double base_tra( 0 );
  bool deadman( false );
  bool over_ride( false );
  double turret_pan( 0 );
  double turret_tilt( 0 );
  double moog_pan( 0 );
  double moog_tilt( 0 );
  uint8_t trigger( 0 );
  uint8_t estop( 0 );

  static bool first_joy_error( true );
  static float step_value( 0 );

  if ( !zero ) {
    bool correct_size( true );
    if ( axes.size() != naxes_ || buttons.size() != nbuttons_ ) {
      correct_size = false;
      if ( first_joy_error ) {
        std::cerr << "[joy] Incorrect input size: axes.size = " << axes.size();
        std::cerr << " (should be " << naxes_ << "), buttons.size = " << buttons.size();
        std::cerr << "(should be " << nbuttons_ << ") " << std::endl;
        first_joy_error = false;
      }
    }

    if ( jstype_ > 0 && correct_size  ) {
      deadman = axes[5] < 0.5 ? true : false;
      double deadman_value = axes[5];
      first_joy_error = true;
      if ( deadman ) {
        if ( jstype_ == XBOX_WIRED ) {
          moog_pan = axes[6];
          moog_tilt = axes[7];
        }
        else if ( jstype_ == XBOX_WIRELESS ) {
          moog_pan = ( buttons[11] - buttons[12] );
          moog_tilt = ( buttons[13] - buttons[14] );
        }
        turret_pan = axes[0];
        turret_tilt = axes[1];
        trigger = axes[2] < 0 ? radlast_constants()->PaintballTrigger->SINGLE_SHOT : 0;
        base_rot = axes[3];
        base_tra = axes[4];
        over_ride = fabs( base_tra ) > 0.25 || fabs( base_rot ) > 0.25 || deadman_value < -0.9;
        over_ride = over_ride || buttons[8] > 0;
        step_value = 0;
      }
      else {
        if ( buttons[5] ) {
          // UPenn step function
          // If deadman is not pressed, this will be executed.
          // if deadman is pressed, output will be reset to 0
          // else, a fixed value will be sent based on the button pressed
          if ( jstype_ == XBOX_WIRED ) {
            if ( buttons[0] ) {
              step_value = 0.1;
              std::cout << step_value << std::endl;
            }
            else if ( buttons[1] ) {
              step_value = 0.2;
              std::cout << step_value << std::endl;
            }
            else if ( buttons[2] ) {
              step_value = 0.3;
              std::cout << step_value << std::endl;
            }
            else if ( buttons[3] ) {
              step_value = 0.4;
              std::cout << step_value << std::endl;
            }
          }
          base_tra = step_value / max_linear_;
          base_rot = 0;
        }
        else {
          step_value = 0;
        }
      }
      // Check ESTOP (LB and RB buttons)
      if ( jstype_ == XBOX_WIRED ) {
        estop = buttons[4] && buttons[5];
      }
    }
  }
  out->base->stamp = radl_gettime();

  out->estop->data = estop;
  out->over_ride->data = over_ride;
  out->base->linear.x = max_linear_ * base_tra;
  out->base->linear.y = 0;
  out->base->linear.z = 0;
  out->base->angular.x = 0;
  out->base->angular.y = 0;
  out->base->angular.z = -max_angular_ * base_rot;
  out->turret_pan->data = -turret_pan;
  out->turret_tilt->data = turret_tilt;
  out->moog_pan->data = moog_pan;
  out->moog_tilt->data = moog_tilt;
  out->trigger->data = trigger;
}

int LandsharkJoystick::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags ) 
{
  static bool first_time = true;
  timeval t;
  gettimeofday( &t, NULL );

  // zero if time elapsed since last "heartbeat" exceeds timeout
  // if heartbeat_timeout <= 0, then this feature is disabled, ie timeouts are ignored
  bool zero = ( heartbeat_timeout_ms_ > 0 ) ? msec_since_last_event() > heartbeat_timeout_ms_ : false;
  boost::mutex::scoped_lock lock( data_mutex_ );
  translate( axes_, buttons_, out, zero );
  if ( ( fresh_ != true && zero == true ) || ( get_status() != JOYSTICK_OPEN )  ) {
    radl_turn_on( radl_STALE, &out_flags->base );
    radl_turn_on( radl_STALE, &out_flags->over_ride );
    radl_turn_on( radl_STALE, &out_flags->turret_pan );
    radl_turn_on( radl_STALE, &out_flags->turret_tilt );
    radl_turn_on( radl_STALE, &out_flags->moog_pan );
    radl_turn_on( radl_STALE, &out_flags->moog_tilt );
    radl_turn_on( radl_STALE, &out_flags->trigger );
  }
  if ( zero ) { 
    if ( first_time == true ) {
      std::cout << "[joy] Zeroing joystick output (no heartbeat)" << std::endl;
      first_time = false;
    }
  }
  else {
    first_time = true;
  }
  fresh_ = false;
  return 0;
}


