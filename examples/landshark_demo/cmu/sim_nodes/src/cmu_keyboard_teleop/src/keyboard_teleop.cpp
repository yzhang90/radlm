#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
//#include "landshark_msgs/BoolStamped.h"
#include <std_msgs/Bool.h>
#include <signal.h>
#include <termios.h>
#include <pthread.h>
//#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_SPACEBAR 0x20
#define KEY_Q 0x71

pthread_mutex_t publishLock = PTHREAD_MUTEX_INITIALIZER;
int kfd = 0;
struct termios cooked, raw;
int quit_flag = 0;

class TeleopLandshark
{
public:
  TeleopLandshark( ros::NodeHandle& nh )
    : nh_( nh )
    , linear_(0)
    , angular_(0)
    , m_BaseTranScale(1.0)
    , m_BaseRotScale(1.0)
  {
    nh_.getParam( "base_translation_scale", m_BaseTranScale );
    nh_.getParam( "base_rotation_scale", m_BaseRotScale );
    m_ControlTopicCmux = "/landshark_control_cmux/";
    m_ControlTopic = "/landshark_2dnav/";
    m_BaseVelocityControlTopic = m_ControlTopic + "base_velocity"; 
    m_BaseDeadmanControlTopic = m_ControlTopic + "deadman";
    bool latch = true;
    m_BaseVelocityControlPub = nh_.advertise<geometry_msgs::TwistStamped>(m_BaseVelocityControlTopic, 1,latch);
    m_BaseDeadmanControlPub = nh_.advertise<std_msgs::Bool>(m_BaseDeadmanControlTopic, 1,latch);
  }

  void keyLoop()
  {
    char c;
    bool pressed=false;

    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the landshark!!!");

    ros::Time last_pub_time = ros::Time::now();
    linear_ = 0;
    angular_ = 0;
    while( ros::ok() && quit_flag == 0 ) {
      // get the next event from the keyboard  
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }


//      ROS_INFO("value: 0x%02X\n", c);

      ROS_INFO("%c",c);

      switch(c)
      {
      
        case KEYCODE_L:
          ROS_INFO("Base left");
          angular_ = 0.3;
          pressed = true;
          break;
        case KEYCODE_R:
          ROS_INFO("Base right");
          angular_ = -0.3;
          pressed = true;
          break;
        case KEYCODE_U:
          ROS_INFO("Base forward");
          linear_ = 0.4;
          pressed = true;
          break;
        case KEYCODE_D:
          ROS_INFO("Base back");
          linear_ = -0.4;
          pressed = true;
          break;
        case KEYCODE_SPACEBAR:
          ROS_INFO("Stop");
          linear_ = 0.0;
          angular_ = 0.0;
          pressed = true;
          break;  
        case KEY_Q:
          ROS_INFO("Quit");
          quit_flag = 1;
          break;
        default:
          ROS_WARN( "Key: 0x%02x", c );
      }

      ros::Time stamp = ros::Time::now();
      if ( ( stamp - last_pub_time ).toSec() > 0.1 ) {
        if (pressed ==true)
        {
          last_pub_time = stamp;
          /// Base velocity command 
          m_BaseControlMsg.header.stamp = stamp;
          m_BaseControlMsg.twist.linear.x = m_BaseTranScale * linear_;
          m_BaseControlMsg.twist.angular.z = m_BaseRotScale * angular_;

          m_BaseVelocityControlPub.publish( m_BaseControlMsg );
          
          m_BaseControlDeadmanMsg.data = true;
          
          m_BaseDeadmanControlPub.publish( m_BaseControlDeadmanMsg );
          
          pressed=false;
          pressed = false;
          linear_ = 0;
          angular_ = 0;
        }
      }
    }


    return;
  }

private:
  ros::NodeHandle nh_;
  double linear_, angular_, m_BaseRotScale, m_BaseTranScale;
  ros::Publisher m_BaseVelocityControlPub;
  ros::Publisher m_BaseDeadmanControlPub;
  std::string m_ControlTopic;
  std::string m_ControlTopicCmux;
  std::string m_BaseVelocityControlTopic;
  std::string m_BaseDeadmanControlTopic;
  geometry_msgs::TwistStamped m_BaseControlMsg;
  std_msgs::Bool m_BaseControlDeadmanMsg;
};


void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  quit_flag = 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmu_keyboard_teleop");
  ros::NodeHandle node_handle;
  TeleopLandshark teleop_landshark( node_handle );

  signal(SIGINT,quit);
  teleop_landshark.keyLoop();
  
  return(0);
}

#if 0

void* run(void* v){
  geometry_msgs::TwistStamped baseControlMsg;
  ros::Publisher* p = (ros::Publisher*) v;

  baseControlMsg.twist.linear.x = 0.0d;
  baseControlMsg.twist.linear.y = 0.0d;
  baseControlMsg.twist.linear.z = 0.0d;
  baseControlMsg.twist.angular.x = 0.0d;
  baseControlMsg.twist.angular.y = 0.0d;
  baseControlMsg.twist.angular.z = 0.0d;

  for(;;){ 
    usleep(100000);
    //sleep(10);
    ros::Time stamp = ros::Time::now();
    /// Base velocity command 
    
    baseControlMsg.header.stamp = stamp;

    pthread_mutex_lock(&publishLock);
    p->publish( baseControlMsg );
    pthread_mutex_unlock(&publishLock);
   }
}

void TeleopLandshark::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the landshark!!!");

  pthread_t* runner = new pthread_t();
  
  pthread_create(runner,NULL,&run,(void*) &m_BaseVelocityControlPub );

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 2.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -2.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.5;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.5;
        dirty = true;
        break;
    }
   
    ros::Time stamp = ros::Time::now();
    /// Base velocity command 
    m_BaseControlMsg.header.stamp = stamp;
    m_BaseControlMsg.twist.linear.x = m_BaseTranScale * linear_;
    m_BaseControlMsg.twist.angular.z = m_BaseRotScale * angular_;

    if(dirty ==true)
    {
     pthread_mutex_lock(&publishLock);
     m_BaseVelocityControlPub.publish( m_BaseControlMsg );
     pthread_mutex_unlock(&publishLock);
      dirty=false;
    }
  }


  return;
}
#endif 


