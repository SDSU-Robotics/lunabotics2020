#ifndef _ROS_xpp_msgs_RobotParameters_h
#define _ROS_xpp_msgs_RobotParameters_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

namespace xpp_msgs
{

  class RobotParameters : public ros::Msg
  {
    public:
      uint32_t ee_names_length;
      typedef char* _ee_names_type;
      _ee_names_type st_ee_names;
      _ee_names_type * ee_names;
      uint32_t nominal_ee_pos_length;
      typedef geometry_msgs::Point _nominal_ee_pos_type;
      _nominal_ee_pos_type st_nominal_ee_pos;
      _nominal_ee_pos_type * nominal_ee_pos;
      typedef geometry_msgs::Vector3 _ee_max_dev_type;
      _ee_max_dev_type ee_max_dev;
      typedef float _base_mass_type;
      _base_mass_type base_mass;

    RobotParameters():
      ee_names_length(0), st_ee_names(), ee_names(nullptr),
      nominal_ee_pos_length(0), st_nominal_ee_pos(), nominal_ee_pos(nullptr),
      ee_max_dev(),
      base_mass(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ee_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ee_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ee_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ee_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ee_names_length);
      for( uint32_t i = 0; i < ee_names_length; i++){
      uint32_t length_ee_namesi = strlen(this->ee_names[i]);
      varToArr(outbuffer + offset, length_ee_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->ee_names[i], length_ee_namesi);
      offset += length_ee_namesi;
      }
      *(outbuffer + offset + 0) = (this->nominal_ee_pos_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nominal_ee_pos_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nominal_ee_pos_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nominal_ee_pos_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nominal_ee_pos_length);
      for( uint32_t i = 0; i < nominal_ee_pos_length; i++){
      offset += this->nominal_ee_pos[i].serialize(outbuffer + offset);
      }
      offset += this->ee_max_dev.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->base_mass);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t ee_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ee_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ee_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ee_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ee_names_length);
      if(ee_names_lengthT > ee_names_length)
        this->ee_names = (char**)realloc(this->ee_names, ee_names_lengthT * sizeof(char*));
      ee_names_length = ee_names_lengthT;
      for( uint32_t i = 0; i < ee_names_length; i++){
      uint32_t length_st_ee_names;
      arrToVar(length_st_ee_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_ee_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_ee_names-1]=0;
      this->st_ee_names = (char *)(inbuffer + offset-1);
      offset += length_st_ee_names;
        memcpy( &(this->ee_names[i]), &(this->st_ee_names), sizeof(char*));
      }
      uint32_t nominal_ee_pos_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nominal_ee_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nominal_ee_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nominal_ee_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nominal_ee_pos_length);
      if(nominal_ee_pos_lengthT > nominal_ee_pos_length)
        this->nominal_ee_pos = (geometry_msgs::Point*)realloc(this->nominal_ee_pos, nominal_ee_pos_lengthT * sizeof(geometry_msgs::Point));
      nominal_ee_pos_length = nominal_ee_pos_lengthT;
      for( uint32_t i = 0; i < nominal_ee_pos_length; i++){
      offset += this->st_nominal_ee_pos.deserialize(inbuffer + offset);
        memcpy( &(this->nominal_ee_pos[i]), &(this->st_nominal_ee_pos), sizeof(geometry_msgs::Point));
      }
      offset += this->ee_max_dev.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->base_mass));
     return offset;
    }

    virtual const char * getType() override { return "xpp_msgs/RobotParameters"; };
    virtual const char * getMD5() override { return "93bb9137a8bf2b168102f89fd6a86853"; };

  };

}
#endif
