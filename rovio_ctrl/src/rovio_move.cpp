/*!
 * \file rovio_move.cpp
 * \brief Communication node to the Rovio's motors.
 *
 * The rovio_head creates a ROS node that allows messages to control the motors of the Rovio.
 * The motors can be controlled by providing either a rovio_shared/man_drv message (relating to motor commands defined by the Rovio's API) or geometry_msgs/Twist messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 6, 2014
 */

#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/man_drv.h>
#include <rovio_shared/rovio_http.h>
#include <rovio_ctrl/rovio_move.h>
#include <sstream>
#include <string>

using namespace std;

move_controller::move_controller()
{
  string user;
  string pass;

  // check for all the correct parameters
  if (!node.getParam(USER, user))
  {
    ROS_ERROR("Parameter %s not found.", USER);
    exit(-1);
  }
  if (!node.getParam(PASS, pass))
  {
    ROS_ERROR("Parameter %s not found.", PASS);
    exit(-1);
  }
  if (!node.getParam(HOST, host))
  {
    ROS_ERROR("Parameter %s not found.", HOST);
    exit(-1);
  }

  // create the communication object to talk to Rovio
  rovio = new rovio_http(user, pass);

  // add subscriptions that will control the Rovio
  man_drv = node.subscribe<rovio_shared::man_drv>("man_drv", 10, &move_controller::man_drv_callback, this);
  cmd_vel = node.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &move_controller::cmd_vel_callback, this);

  drive = 0;
  speed = 0;
  rotate = 0;

  ROS_INFO("Rovio Move Controller Initialized");
}

move_controller::~move_controller()
{
  // free up the rovio_http object
  delete rovio;
}

void move_controller::update()
{
  // check for a drive command
  if (drive > 0)
  {
    // build the URL command and send it
    stringstream ss;
    ss << "http://" << host.c_str() << "/rev.cgi?Cmd=nav&action=18&drive=" << drive << "&speed=" << speed;
    rovio_response *buf = rovio->send(ss.str().c_str());
    rovio_response_clean(buf);
  }
  // check for a rotation command
  if (rotate > 0)
  {
    // build the URL command and send it
    stringstream ss;
    ss << "http://" << host.c_str() << "/rev.cgi?Cmd=nav&action=18&drive=6&speed=" << rotate;
    rovio_response *buf = rovio->send(ss.str().c_str());
    rovio_response_clean(buf);
  }
  else if (rotate < 0)
  {
    // build the URL command and send it
    stringstream ss;
    ss << "http://" << host.c_str() << "/rev.cgi?Cmd=nav&action=18&drive=5&speed=" << -rotate;
    rovio_response *buf = rovio->send(ss.str().c_str());
    rovio_response_clean(buf);
  }
}

void move_controller::man_drv_callback(const rovio_shared::man_drv::ConstPtr &msg)
{
  // check to see if all the requests are valid
  if (msg->drive < rovio_shared::man_drv::MIN_DRIVE_VAL || msg->drive > rovio_shared::man_drv::MAX_DRIVE_VAL)
  {
    ROS_ERROR(
        "Manual Drive 'drive' value of %i out of range [%i,%i].", msg->drive, rovio_shared::man_drv::MIN_DRIVE_VAL, rovio_shared::man_drv::MAX_DRIVE_VAL);
    return;
  }
  if (msg->speed < rovio_shared::man_drv::MIN_SPEED_VAL || msg->speed > rovio_shared::man_drv::MAX_SPEED_VAL)
  {
    ROS_ERROR(
        "Manual Drive 'speed' value of %i out of range [%i,%i].", msg->speed, rovio_shared::man_drv::MIN_SPEED_VAL, rovio_shared::man_drv::MAX_SPEED_VAL);
    return;
  }

  // set the values
  drive = msg->drive;
  speed = msg->speed;
}

void move_controller::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
  // check if we are moving left or right
  if (msg->linear.x == 0 && msg->linear.y > 0)
  {
    // drive left
    drive = 3;
  }
  else if (msg->linear.x == 0 && msg->linear.y < 0)
  {
    // drive right
    drive = 4;
  }
  else
  {
    // calculate the angle
    double angle = atan2(msg->linear.x, msg->linear.y);
    // bound the movements to one of the discrete Rovio drive commands
    if (msg->linear.x > 0 && angle <= 5 * M_PI / 8 && angle >= 3 * M_PI / 8)
    {
      // drive forwards
      drive = 1;
    }
    else if (msg->linear.x > 0 && msg->linear.y > 0 && angle < 3 * M_PI / 8 && angle >= M_PI / 8)
    {
      // drive forwards and right
      drive = 8;
    }
    else if (msg->linear.y > 0 && (angle >= -1 * M_PI / 8 || angle >= 3 * M_PI / 8))
    {
      // drive right
      drive = 4;
    }
    else if (msg->linear.x < 0 && msg->linear.y > 0 && angle < -1 * M_PI / 8 && angle >= -3 * M_PI / 8)
    {
      // drive backwards and right
      drive = 10;
    }
    else if (msg->linear.x < 0 && angle <= -3 * M_PI / 8 && angle >= -5 * M_PI / 8)
    {
      // drive backwards
      drive = 2;
    }
    else if (msg->linear.x < 0 && msg->linear.y < 0 && angle <= -5 * M_PI / 8 && angle >= -7 * M_PI / 8)
    {
      // drive backwards and left
      drive = 9;
    }
    else if (msg->linear.y < 0 && (angle < -7 * M_PI / 8 || angle >= 7 * M_PI / 8))
    {
      // drive left
      drive = 3;
    }
    else if (msg->linear.x > 0 && msg->linear.y < 0 && angle < 7 * M_PI / 8 && angle >= 5 * M_PI / 8)
    {
      // drive forwards and left
      drive = 7;
    }
    else
    {
      // no movement
      drive = 0;
    }
  }

  // get the rotational speed
  rotate = -msg->angular.z * 10.0;

  // get the linear speed (10 being the fastest)
  speed = min(10, (int)(sqrt(pow(msg->linear.x, 2.0) + pow(msg->linear.y, 2.0)) * 10));
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rovio_move");

  // initialize the Rovio controller
  move_controller controller;

  // update at 5 Hz
  ros::Rate loop_rate(5);
  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    ros::spinOnce();
    // update the motors
    controller.update();
    loop_rate.sleep();
  }
}
