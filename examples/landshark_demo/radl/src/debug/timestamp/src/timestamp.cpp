#include "timestamp.h"

#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"

inline ros::Time radl_to_ros_time( radl_duration_t t ) {
  ros::Time stamp;
  int64_t sec;
  uint32_t nsec;
  radl_to_secnsec( t, &sec, &nsec );

  stamp.sec = sec;
  stamp.nsec = nsec;
  return stamp;
};

timestamp::timestamp() 
{
}

timestamp::~timestamp() 
{
}

int timestamp::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
{
  static ros::NodeHandle nh( "~" );
  static ros::Publisher p0 = nh.advertise<std_msgs::Float32>("/timestamp/reference", 10);
  static ros::Publisher p1 = nh.advertise<geometry_msgs::Vector3Stamped>("/timestamp/actuator", 10);
  static ros::Publisher p2 = nh.advertise<geometry_msgs::Vector3Stamped>("/timestamp/encoder", 10);
  static ros::Publisher p3 = nh.advertise<geometry_msgs::Vector3Stamped>("/timestamp/base", 10);
  static ros::Time start( ros::Time::now() );
  static bool first_time( true );


  std_msgs::Float32 r;
  geometry_msgs::Vector3Stamped m;
  ros::Time now( ros::Time::now() );
  radl_duration_t radlnow = radl_gettime();

  r.data = ( radl_to_ros_time( radlnow ) - now ).toSec();
  p0.publish( r );

  // actuator
  {
    ros::Time t = radl_to_ros_time( in->actuator->stamp );
    float f = in_flags->actuator > 0 ? 1.0 : 0;
    m.header.stamp = now;
    m.vector.x = ( t - start ).toSec();
    m.vector.y = ( t - now ).toSec();
    m.vector.z = f;
    p1.publish( m );
  }

  // encoder
  {
    ros::Time t = radl_to_ros_time( in->encoder->stamp );
    float f = in_flags->encoder > 0 ? 1.0 : 0;
    m.header.stamp = now;
    m.vector.x = ( t - start ).toSec();
    m.vector.y = ( t - now ).toSec();
    m.vector.z = f;
    p2.publish( m );
  }

  // base
  {
    ros::Time t = radl_to_ros_time( in->base->stamp );
    float f = in_flags->base > 0 ? 1.0 : 0;
    m.header.stamp = now;
    m.vector.x = ( t - start ).toSec();
    m.vector.y = ( t - now ).toSec();
    m.vector.z = f;
    p3.publish( m );
  }

  if ( first_time ) {
    ros::Time t2 = radl_to_ros_time( radlnow );
    double sec = radl_to_nsec( radlnow ) *1e-9;
    std::cout << "ROS:  sec=" << now.sec << ", nsec= " << now.nsec << std::endl;
    std::cout << "RADL: sec=" << std::fixed << sec << std::endl;
    std::cout << "RADL: sec=" << (int) sec << ", nsec= " << (int ) ( (sec - floor( sec ) ) * 1e9 ) << std::endl;
    std::cout << "ROS:  sec=" << t2.sec << ", nsec= " << t2.nsec << std::endl;
    first_time = false;
  }
}


