
/*!
 * \file rovio_teleop.cpp
 * \brief Allows for control of the Rovio with a joystick.
 *
 * rovio_teleop creates a ROS node that allows the control of a Rovio with a joystick.
 * This node listens to a joy topic and sends messages to the cmd_vel topic in the rovio_move node and head_ctrl service in the rovio_head node.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 6, 2014
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <rovio_shared/head_ctrl.h>
#include <rovio_ctrl/rovio_teleop.h>
#include <rovio_shared/man_drv.h>
#include <rovio_shared/wav_play.h>
#include <sensor_msgs/Joy.h>
#include <string>

using namespace std;

teleop_controller::teleop_controller()
{
  // check for all the correct parameters
  if (!node.getParam(ROVIO_WAV, rovio_wav))
  {
    ROS_ERROR("Parameter %s not found.", ROVIO_WAV);
    exit(-1);
  }

  // create the published topic and client
  cmd_vel = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  head_ctrl = node.serviceClient<rovio_shared::head_ctrl>("head_ctrl");
  wav_play = node.serviceClient<rovio_shared::wav_play>("wav_play");

  //subscribe to the joystick
  joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 10, &teleop_controller::joy_cback, this);

  ROS_INFO("Rovio Teleop Started");
}

void teleop_controller::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // create the message for a speed message and request for the head
  rovio_shared::man_drv drv;
  rovio_shared::head_ctrl head;
  rovio_shared::wav_play wav;

  // check for any head control buttons
  head.request.head_pos = -1;
  if (joy->buttons.at(0) == 1)
    head.request.head_pos = rovio_shared::head_ctrl::Request::HEAD_DOWN;
  else if (joy->buttons.at(1) == 1)
    head.request.head_pos = rovio_shared::head_ctrl::Request::HEAD_MIDDLE;
  else if (joy->buttons.at(2) == 1)
    head.request.head_pos = rovio_shared::head_ctrl::Request::HEAD_UP;

  // check if a head request was made
  if (head.request.head_pos != -1)
    // send the request
    head_ctrl.call(head);

  // check for any audio buttons
  wav.request.f = "";
  wav.request.f.append(rovio_wav);
  if (joy->buttons.at(4) == 1)
    wav.request.f.append("/G11.wav");
  else if (joy->buttons.at(5) == 1)
    wav.request.f.append("/G03.wav");
  else if (joy->buttons.at(6) == 1)
    wav.request.f.append("/G27a.wav");
  else if (joy->buttons.at(7) == 1)
    wav.request.f.append("/G34.wav");
  else
    wav.request.f = "";

  // check if a audio request was made
  if (wav.request.f.size() > 0)
    // send the request
    wav_play.call(wav);

  // create the twist message
  geometry_msgs::Twist twist;
  // left joystick controls the linear movement
  twist.linear.x = joy->axes.at(1) * THROTTLE;
  twist.linear.y = joy->axes.at(0) * THROTTLE;
  twist.linear.z = 0;
  // right joystick controls the angular movement
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = joy->axes.at(2) * THROTTLE;
  // send the twist command
  cmd_vel.publish(twist);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rovio_teleop");

  // initialize the Rovio controller
  teleop_controller controller;

  // continue until a ctrl-c has occurred
  ros::spin();
}
