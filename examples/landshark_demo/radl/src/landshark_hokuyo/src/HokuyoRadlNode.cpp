#include "HokuyoRadlNode.h"

#ifdef PUBLISH_SENSOR_MSGS
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#endif // PUBLISH_SENSOR_MSGS

RADL_NODE_NAME::RADL_NODE_NAME()
  : driver_()
{
  driver_.port_ = *RADL_THIS->device;
  driver_.min_ang_ = *RADL_THIS->min_ang;
  driver_.max_ang_ = *RADL_THIS->max_ang;
  driver_.cluster_ = *RADL_THIS->cluster;
  driver_.skip_ = *RADL_THIS->skip;
  driver_.calibrate_time_ = *RADL_THIS->calibrate_time;
  array_size_ = *RADL_THIS->array_size;
  driver_.doOpen();
  driver_.doStart();
}

RADL_NODE_NAME::~RADL_NODE_NAME()
{
  driver_.doStop();
}

int RADL_NODE_NAME::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
{
  static int last_ranges_size( -1 );
  static int last_intensities_size( -1 );
  if ( !driver_.getFirstScan() )
  {
    boost::mutex::scoped_lock lock( driver_.scan_mutex_ );
    out->scan->angle_min = driver_.scan_.config.min_angle;
    out->scan->angle_max = driver_.scan_.config.max_angle;
    out->scan->angle_increment = driver_.scan_.config.ang_increment;
    out->scan->time_increment = driver_.scan_.config.time_increment;
    out->scan->scan_time = driver_.scan_.config.scan_time;
    out->scan->range_min = driver_.scan_.config.min_range;
    out->scan->range_max = driver_.scan_.config.max_range;

    if ( array_size_ == driver_.scan_.ranges.size() ) {
      for ( size_t i = 0; i < array_size_; i++ ) {
        out->scan->ranges[i] = driver_.scan_.ranges[i];
      }
    }
    else {
      if ( driver_.scan_.ranges.size() != last_ranges_size ) {
        std::cout << "[hokuyo] Warning! ranges.size() = " << driver_.scan_.ranges.size();
        std::cout << ". It should be " << array_size_ << std::endl;
      }
    }
    if ( driver_.scan_.ranges.size() != last_ranges_size ) {
      std::cout << "[hokuyo] ranges.size() = " << driver_.scan_.ranges.size() << std::endl;
      last_ranges_size = driver_.scan_.ranges.size();
    }

    if ( array_size_ == driver_.scan_.intensities.size() ) {
      for ( size_t i = 0; i < array_size_; i++ ) {
        out->scan->intensities[i] = driver_.scan_.intensities[i];
      }
    }
    else {
      if ( driver_.scan_.intensities.size() != last_intensities_size ) {
        std::cout << "[hokuyo] Warning! intensities.size() = " << driver_.scan_.intensities.size();
        std::cout << ". It should be " << array_size_ << std::endl;
      }
    }
    if ( driver_.scan_.intensities.size() != last_intensities_size ) {
      std::cout << "[hokuyo] intensities.size() = " << driver_.scan_.intensities.size() << std::endl;
      last_intensities_size = driver_.scan_.intensities.size();
    }
    out->scan->stamp = driver_.stamp_;
    if ( !driver_.fresh_ ) {
      radl_turn_on( radl_STALE, &out_flags->scan );
    }
    driver_.fresh_ = false;
  }
  else {
    boost::mutex::scoped_lock lock( driver_.scan_mutex_ );
    out->scan->angle_min = driver_.scan_.config.min_angle;
    out->scan->angle_max = driver_.scan_.config.max_angle;
    out->scan->angle_increment = driver_.scan_.config.ang_increment;
    out->scan->time_increment = driver_.scan_.config.time_increment;
    out->scan->range_min = driver_.scan_.config.min_range;
    out->scan->range_max = driver_.scan_.config.max_range;
    radl_turn_on( radl_STALE, &out_flags->scan );
  }

#ifdef PUBLISH_SENSOR_MSGS
  static ros::NodeHandle nh( "~" );
  static ros::Publisher p1 = nh.advertise<sensor_msgs::LaserScan>("scan", 100);
  static sensor_msgs::LaserScan scan_msg;

  {
    boost::mutex::scoped_lock lock( driver_.scan_mutex_ );
    scan_msg.angle_min = driver_.scan_.config.min_angle;
    scan_msg.angle_max = driver_.scan_.config.max_angle;
    scan_msg.angle_increment = driver_.scan_.config.ang_increment;
    scan_msg.time_increment = driver_.scan_.config.time_increment;
    scan_msg.scan_time = driver_.scan_.config.scan_time;
    scan_msg.range_min = driver_.scan_.config.min_range;
    scan_msg.range_max = driver_.scan_.config.max_range;
    scan_msg.ranges = driver_.scan_.ranges;
    scan_msg.intensities = driver_.scan_.intensities;
    scan_msg.header.stamp = ros::Time().fromNSec((uint64_t)driver_.scan_.system_time_stamp);
    scan_msg.header.frame_id = "hokuyo";
  }
  p1.publish( scan_msg );

#endif // PUBLISH_SENSOR_MSGS
}
