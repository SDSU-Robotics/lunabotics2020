#ifndef _ROS_xpp_msgs_StateLin3d_h
#define _ROS_xpp_msgs_StateLin3d_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

namespace xpp_msgs
{

  class StateLin3d : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _pos_type;
      _pos_type pos;
      typedef geometry_msgs::Vector3 _vel_type;
      _vel_type vel;
      typedef geometry_msgs::Vector3 _acc_type;
      _acc_type acc;

    StateLin3d():
      pos(),
      vel(),
      acc()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pos.serialize(outbuffer + offset);
      offset += this->vel.serialize(outbuffer + offset);
      offset += this->acc.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pos.deserialize(inbuffer + offset);
      offset += this->vel.deserialize(inbuffer + offset);
      offset += this->acc.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "xpp_msgs/StateLin3d"; };
    virtual const char * getMD5() override { return "c4069b8f5d3058377f8685efad96ae30"; };

  };

}
#endif
