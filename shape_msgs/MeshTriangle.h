#ifndef _ROS_shape_msgs_MeshTriangle_h
#define _ROS_shape_msgs_MeshTriangle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace shape_msgs
{

  class MeshTriangle : public ros::Msg
  {
    public:
      uint32_t vertex_indices[3];

    MeshTriangle():
