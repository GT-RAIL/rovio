/*!
 * \file rovio_http.cpp
 * \brief Communication library to the Rovio's HTTP server.
 *
 * rovio_http allows direct communication to the Rovio's HTTP server. This library uses CURL to transmit messages to and from the Rovio.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 6, 2014
 */

#include <rovio_shared/rovio_http.h>

using namespace std;

rovio_http::rovio_http(string user, string pass)
{
  // create the CURL handle
  curl = curl_easy_init();
  if (curl == NULL)
  {
    ROS_ERROR("Curl was unable to initialize.");
    exit(-1);
  }

  // set the username and password
  curl_easy_setopt(curl, CURLOPT_USERPWD, user.append(":").append(pass).c_str());
  // set the pointer to the function which handles the responses
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &write_data);

  // create the semaphore
  sem_init(&sem, 0, 1);
}

rovio_http::~rovio_http()
{
  // cleanup anything left by Curl
  curl_easy_cleanup(curl);
  // destroy the semaphore
  sem_destroy(&sem);
}

rovio_response *rovio_http::send(const char *url)
{
  // wait for the curl handle to be free
  sem_wait(&sem);

  // create the response for the Rovio
  rovio_response *resp = (rovio_response *)malloc(sizeof(rovio_response));
  resp->size = 0;
  resp->data = NULL;
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, resp);

  //send the command to the Rovio
  curl_easy_setopt(curl, CURLOPT_URL, url);
  curl_easy_perform(curl);

  sem_post(&sem);

  return resp;
}

size_t write_data(char *ptr, size_t size, size_t nmemb, rovio_response *buf)
{
  // actual size of the new data
  size_t new_data = size * nmemb;

  // see if there is any data
  if (new_data > 0)
  {

    // check if the buffer already has data
    if (buf->data)
      // resize the buffer
      buf->data = (char *)realloc(buf->data, buf->size + new_data + 1);
    else
      // allocate the initial memory
      buf->data = (char *)malloc(new_data + 1);

    // add the data to the buffer
    memcpy(&(buf->data[buf->size]), ptr, new_data);
    //update the size
    buf->size += new_data;
    // null terminate
    buf->data[buf->size] = '\0';
  }

  return new_data;
}

void rovio_response_clean(rovio_response *resp)
{
  // check if the response is null
  if (resp)
  {
    // free the data (if any)
    if (resp->data)
      free(resp->data);
    // free the struct
    free(resp);
  }
}
