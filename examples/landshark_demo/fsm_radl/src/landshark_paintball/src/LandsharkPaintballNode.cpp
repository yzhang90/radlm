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
 *
 * $Id$
 */

#include "LandsharkPaintball.h"

#include <ros/ros.h>
#include <landshark_msgs/PaintballTrigger.h>
#include <msg_auth/msg_auth.h>

/**
 * This class provides a ros interface to communicate with the Landshark paintball.
 */
class LandsharkPaintballRosWrapper
{
public:
  /**
   * Constructor
   */
  LandsharkPaintballRosWrapper();

  /**
   * Destructor
   */
  ~LandsharkPaintballRosWrapper();

  /**
   * Initialize the communication process
   */
  void Initialize();

private:
  void ProcessPaintballCommand(const landshark_msgs::PaintballTriggerConstPtr& rpCommand);

  landshark::LandsharkPaintball m_LandsharkPaintball;
  ros::NodeHandle m_NodeHandle;
  ros::Subscriber m_PaintballCommandSubscriber;

  msg_auth::MsgAuth m_MsgAuth;
  bool m_UseMsgAuth;
  msg_auth::Subscriber m_PaintballCommandAuthSubscriber;

};

LandsharkPaintballRosWrapper::LandsharkPaintballRosWrapper()
  : m_NodeHandle("~")
  , m_MsgAuth( m_NodeHandle )
  , m_UseMsgAuth( true )
{}

LandsharkPaintballRosWrapper::~LandsharkPaintballRosWrapper()
{}

void LandsharkPaintballRosWrapper::Initialize()
{
  std::string paintballCommandTopic, controllerIp;

  double processRate(5.0); // in Hz

  m_NodeHandle.getParam("paintball_command_topic", paintballCommandTopic);
  m_NodeHandle.getParam("controller_ip", controllerIp);
  m_NodeHandle.getParam("process_rate", processRate);
  m_NodeHandle.getParam("use_msg_auth", m_UseMsgAuth);

  try
  {
    m_LandsharkPaintball.Initialize(controllerIp);
  }
  catch (std::exception& exception)
  {
    ROS_ERROR_STREAM( "Failed to initialize the Landshark paintball with ip: " << controllerIp <<
                      ", exception: " << exception.what() << ". Will shut down." );
    return;
  }

  if ( m_UseMsgAuth ) {
    ROS_WARN_STREAM( "Subscribing to authenticated topic for " << paintballCommandTopic );
    m_PaintballCommandAuthSubscriber = m_MsgAuth.subscribe<landshark_msgs::PaintballTrigger>(paintballCommandTopic, 1, &LandsharkPaintballRosWrapper::ProcessPaintballCommand, this);
  }
  else {
    ROS_WARN_STREAM( "Subscribing to plain topic for " << paintballCommandTopic );
  m_PaintballCommandSubscriber = m_NodeHandle.subscribe<landshark_msgs::PaintballTrigger>(paintballCommandTopic, 1, &LandsharkPaintballRosWrapper::ProcessPaintballCommand, this);
  }

  ros::Rate rate(processRate);

  while (m_NodeHandle.ok())
  {
    ros::spinOnce();

    rate.sleep();
  }
}

void LandsharkPaintballRosWrapper::ProcessPaintballCommand(const landshark_msgs::PaintballTriggerConstPtr& rpCommand)
{
  switch (rpCommand->mode)
  {
    case landshark_msgs::PaintballTrigger::SINGLE_SHOT:
    {
      m_LandsharkPaintball.Shoot();
      break;
    }
    case landshark_msgs::PaintballTrigger::TRIPLE_SHOT:
    {
      m_LandsharkPaintball.TripleShoot();
      break;
    }
    case landshark_msgs::PaintballTrigger::BURST_SHOT:
    {
      m_LandsharkPaintball.Burst();
      break;
    }
    default:
    {
      ROS_WARN_STREAM( "Unknow control mode received: " << rpCommand->mode );
      break;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LandsharkPaintballNode");

  LandsharkPaintballRosWrapper landsharkPaintball;
  landsharkPaintball.Initialize();

  return 0;
}
