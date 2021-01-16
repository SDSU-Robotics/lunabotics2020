#ifndef _ROS_xpp_msgs_State6d_h
#define _ROS_xpp_msgs_State6d_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"

namespace xpp_msgs
{

  class State6d : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Twist _twist_type;
      _twist_type twist;
      typedef geometry_msgs::Accel _accel_type;
      _accel_type accel;

    State6d():
      pose(),
      twist(),
      accel()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      offset += this->accel.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
      offset += this->accel.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "xpp_msgs/State6d"; };
    virtual const char * getMD5() override { return "12a3981be6e2e29bdfd02e1da364b8ff"; };

  };

}
#endif
