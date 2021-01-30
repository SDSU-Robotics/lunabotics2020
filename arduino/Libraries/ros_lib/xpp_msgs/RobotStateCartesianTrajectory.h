#ifndef _ROS_xpp_msgs_RobotStateCartesianTrajectory_h
#define _ROS_xpp_msgs_RobotStateCartesianTrajectory_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "xpp_msgs/RobotStateCartesian.h"

namespace xpp_msgs
{

  class RobotStateCartesianTrajectory : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t points_length;
      typedef xpp_msgs::RobotStateCartesian _points_type;
      _points_type st_points;
      _points_type * points;

    RobotStateCartesianTrajectory():
      header(),
      points_length(0), st_points(), points(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->points_length);
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->points_length);
      if(points_lengthT > points_length)
        this->points = (xpp_msgs::RobotStateCartesian*)realloc(this->points, points_lengthT * sizeof(xpp_msgs::RobotStateCartesian));
      points_length = points_lengthT;
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(xpp_msgs::RobotStateCartesian));
      }
     return offset;
    }

    virtual const char * getType() override { return "xpp_msgs/RobotStateCartesianTrajectory"; };
    virtual const char * getMD5() override { return "cf81e80e883d7cf1f8652d3cf7985437"; };

  };

}
#endif
