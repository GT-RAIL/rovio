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
 * \file rovio_head.h
 * \brief Communication node to the Rovio's head motors.
 *
 * rovio_head creates a ROS node that allows service calls to change the head position and publishes head position data.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date November 22, 2011
 */

#ifndef ROVIO_HEAD_H_
#define ROVIO_HEAD_H_

#include <ros/ros.h>
#include <rovio_shared/head_ctrl.h>
#include <rovio_shared/rovio_http.h>
#include <string>

/*!
 * \class head_controller
 * \brief Provides direct communication to the Rovio to control the head.
 *
 * The head_controller handles communication to the Rovio's head motor devices. ROS nodes, services, and publishers are created and maintained within this object.
 */
class head_controller
{
public:
  /*!
   * \brief Creates a head_controller using ROS parameters.
   *
   * Creates a head_controller object that can be used control and query the Rovio's head motors. A valid username, password, and host must be set as ROS parameters.
   */
  head_controller();

  /*!
   * \brief Cleans up any resources and connections to the Rovio.
   *
   * Uses the deconstructor from the rovio_http class to clean up any resources and connections to the Rovio.
   */
  ~head_controller();

  /*!
   * \brief Publishes head sensor information.
   *
   * Queries the Rovio for its current head position and publishes a string representation of the data (HEAD_UP, HEAD_DOWN, or HEAD_MIDDLE).
   */
  void pub_head_sensor();
private:
  /*!
   * \brief head_ctrl service callback function.
   *
   * Process the service request and attempt to move the Rovio's head.
   *
   * \param req the request for the head_ctrl service
   * \param resp the response for the head_ctrl service to be filled
   * \return if the request was successfully sent to the Rovio
   */
  bool head_ctrl_callback(rovio_shared::head_ctrl::Request &req, rovio_shared::head_ctrl::Response &resp);

  std::string host; /*!< host of the Rovio */
  rovio_http *rovio; /*!< communicates with the Rovio */
  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::ServiceServer head_ctrl; /*!< the head_ctrl service */
  ros::Publisher head_sensor; /*!< the head_sensor publisher */
};

/*!
 * Creates and runs the rovio_head node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
