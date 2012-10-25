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
 * \file rovio_http.h
 * \brief Communication library to the Rovio's HTTP server.
 *
 * rovio_http allows direct communication to the Rovio's HTTP server. This library uses cURL to transmit messages to and from the Rovio.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date November 22, 2011
 */

#ifndef ROVIO_HTTP_H_
#define ROVIO_HTTP_H_

#include <curl/curl.h>
#include <ros/ros.h>
#include <semaphore.h>
#include <string>

/*!
 * \def USER
 * The username ROS parameter name
 */
#define USER "/rovio_shared/user"
/*!
 * \def PASS
 * The password ROS parameter name
 */
#define PASS "/rovio_shared/pass"
/*!
 * \def HOST
 * The hostname ROS parameter name
 */
#define HOST "/rovio_shared/host"

/*!
 * \struct rovio_response
 * A rovio_response contains the data returned from the Rovio's HTTP server.
 */
typedef struct
{
  char *data; /*!< the data returned from the server */
  size_t size; /*!< the size of the data */
} rovio_response;

/*!
 * \class rovio_http
 * \brief Provides direct communication to the Rovio to via cURL.
 *
 * The rovio_http handles communication to the Rovio's HTTP server using cURL.
 */
class rovio_http
{
public:
  /*!
   * Create a rovio_http object that can be used to send HTTP commands to the Rovio. A valid username and password must be provide at construction.
   *
   * \param user the username used to authenticate with the Rovio
   * \param pass the password used to authenticate with the Rovio
   */
  rovio_http(std::string user, std::string pass);

  /*!
   * Cleanup any resources from the rovio_http object and from cURL.
   */
  virtual ~rovio_http();

  /*!
   * Send the given full URL command to the Rovio. A buffer containing the response is returned.
   *
   * \param url the full URL command to send to the Rovio
   * \return a buffer containing the response from the Rovio
   */
  rovio_response *send(const char *url);

private:
  CURL *curl; /*!< used to communicate with the Rovio */
  sem_t sem; /*!< used to ensure only one call to cURL occurs */
};

/*!
 * Cleanup any resources used by a rovio_response struct.
 *
 * \param resp the rovio_response struct to cleanup
 */
void rovio_response_clean(rovio_response *resp);

/*!
 * The callback function used by cURL to store the response from the Rovio. This function should only be used by cURL internally by the rovio_http.
 */
size_t write_data(char *ptr, size_t size, size_t nmemb, rovio_response *buf);

#endif
