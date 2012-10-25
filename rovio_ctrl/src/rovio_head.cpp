/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Worcester Polytechnic Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Worcester Polytechnic Institute nor the
 *     names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*!
 * \file rovio_head.cpp
 * \brief Communication node to the Rovio's head motors.
 *
 * rovio_head creates a ROS node that allows service calls to change the head position and publishes head position data.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date November 22, 2011
 */

#include <ros/ros.h>
#include <rovio_shared/head_ctrl.h>
#include <rovio_ctrl/rovio_head.h>
#include <rovio_shared/rovio_http.h>
#include <sstream>
#include <std_msgs/String.h>
#include <string>

using namespace std;

head_controller::head_controller()
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

  // add services and published topics
  head_ctrl = node.advertiseService("head_ctrl", &head_controller::head_ctrl_callback, this);
  head_sensor = node.advertise<std_msgs::String>("head_sensor", 8);

  ROS_INFO("Rovio Head Controller Initialized");
}

head_controller::~head_controller()
{
  // free up the rovio_http object
  delete rovio;
}

bool head_controller::head_ctrl_callback(rovio_shared::head_ctrl::Request &req, rovio_shared::head_ctrl::Response &resp)
{

  // make sure the head position value is valid
  if (req.head_pos != rovio_shared::head_ctrl::Request::HEAD_UP
      && req.head_pos != rovio_shared::head_ctrl::Request::HEAD_DOWN
      && req.head_pos != rovio_shared::head_ctrl::Request::HEAD_MIDDLE)
  {
    ROS_ERROR("Head position 'head_pos' value of %i is not valid.", req.head_pos);
    return false;
  }

  // build the URL command and send it
  stringstream ss;
  ss << "http://" << host.c_str() << "/rev.cgi?Cmd=nav&action=18&drive=" << (int)req.head_pos;
  rovio_response *buf = rovio->send(ss.str().c_str());

  // parse out the response
  int resp_code = -1;
  sscanf(strstr(buf->data, "responses = "), "responses = %i", &resp_code);
  resp.response = resp_code;

  rovio_response_clean(buf);
  return true;
}

void head_controller::pub_head_sensor()
{
  // build the URL command and send it
  stringstream ss;
  ss << "http://" << host.c_str() << "/rev.cgi?Cmd=nav&action=1";
  rovio_response *buf = rovio->send(ss.str().c_str());

  // parse out the response
  int resp_code = -1;
  sscanf(strstr(buf->data, "head_position="), "head_position=%i", &resp_code);

  // decide which head position the Rovio is in
  ss.str("");
  if (resp_code > 140)
  {
    ss << "HEAD_DOWN";
  }
  else if (resp_code < 135)
  {
    ss << "HEAD_UP";
  }
  else
  {
    ss << "HEAD_MIDDLE";
  }

  std_msgs::String msg;
  msg.data = ss.str();
  head_sensor.publish(msg);

  rovio_response_clean(buf);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rovio_head");

  // initialize the Rovio controller
  head_controller controller;

  // update at 5 Hz
  ros::Rate loop_rate(5);
  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    ros::spinOnce();
    //publish the topics
    controller.pub_head_sensor();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
