#ifndef _ROS_sensor_msgs_CameraInfo_h
#define _ROS_sensor_msgs_CameraInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/RegionOfInterest.h"

namespace sensor_msgs
{

  class CameraInfo : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t height;
      uint32_t width;
      const char* distortion_model;
      uint8_t D_length;
      float st_D;
      float * D;
      float K[9];
      float R[9];
      float P[12];
      uint32_t binning_x;
      uint32_t binning_y;
      sensor_msgs::RegionOfInterest roi;

    CameraInfo():
      header(),
      height(0),
      width(0),
      distortion_model(""),
      D_length(0), D(NULL),
