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
 * \file rovio_move.h
 * \brief Communication node to the Rovio's motors.
 *
 * rovio_move creates a ROS node that allows messages to control the motors of the Rovio.
 * The motors can be controlled by providing either a rovio_shared/man_drv message (relating to motor commands defined by the Rovio's API) or geometry_msgs/Twist messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date November 22, 2011
 */

#ifndef ROVIO_MOVE_H_
#define ROVIO_MOVE_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <rovio_shared/man_drv.h>
#include <rovio_shared/rovio_http.h>
#include <string>

/*!
 * \class move_controller
 * \brief Provides direct communication to the Rovio to control the motors.
 *
 * The move_controller handles communication to the Rovio's motor devices. ROS nodes and topics are created and maintained within this object.
 */
class move_controller
{
public:
  /*!
   * \brief Creates a move_controller using ROS parameters.
   *
   * Creates a move_controller object that can be used control the Rovio's motors. A valid username, password, and host must be set as ROS parameters.
   */
  move_controller();

  /*!
   * \brief Cleans up any resources and connections to the Rovio.
   *
   * Uses the deconstructor from the rovio_http class to clean up any resources and connections to the Rovio.
   */
  ~move_controller();

  /*!
   * \brief Sends commands to the Rovio to change the wheel speeds. head sensor information.
   *
   * Based on the most recent message received by either the man_drive or cmd_vel topic, one of Rovio's motor commands is sent to the robot.
   */
  void update();

private:
  /*!
   * \brief man_drv topic callback function.
   *
   * Process the manual drive command and set the appropriate fields.
   *
   * \param msg the message for the man_drv topic
   */
  void man_drv_callback(const rovio_shared::man_drv::ConstPtr &msg);

  /*!
   * \brief cmd_vel topic callback function.
   *
   * Process the twist message and set the appropriate fields. This function is adapted from http://www.ros.org/wiki/rovio_controller.
   * A positive angular-z value corresponds to a clockwise rotation. Movement along linear-y corresponds to moving left/right and movement along linear-x corresponds to moving forwards/backwards.
   *
   * \param msg the message for the man_drv topic
   */
  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);

  std::string host; /*!< host of the Rovio */
  rovio_http *rovio; /*!< communicates with the Rovio */
  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Subscriber man_drv, cmd_vel; /*!< the man_drv and cmd_vel topics */

  int drive, speed, rotate; /*!< used to move the Rovio in the update function */
};

/*!
 * Creates and runs the rovio_move node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
