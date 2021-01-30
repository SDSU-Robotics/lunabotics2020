#ifndef _ROS_xpp_msgs_TerrainInfo_h
#define _ROS_xpp_msgs_TerrainInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace xpp_msgs
{

  class TerrainInfo : public ros::Msg
  {
    public:
      uint32_t surface_normals_length;
      typedef geometry_msgs::Vector3 _surface_normals_type;
      _surface_normals_type st_surface_normals;
      _surface_normals_type * surface_normals;
      typedef float _friction_coeff_type;
      _friction_coeff_type friction_coeff;

    TerrainInfo():
      surface_normals_length(0), st_surface_normals(), surface_normals(nullptr),
      friction_coeff(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->surface_normals_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->surface_normals_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->surface_normals_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->surface_normals_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->surface_normals_length);
      for( uint32_t i = 0; i < surface_normals_length; i++){
      offset += this->surface_normals[i].serialize(outbuffer + offset);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->friction_coeff);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t surface_normals_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      surface_normals_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      surface_normals_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      surface_normals_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->surface_normals_length);
      if(surface_normals_lengthT > surface_normals_length)
        this->surface_normals = (geometry_msgs::Vector3*)realloc(this->surface_normals, surface_normals_lengthT * sizeof(geometry_msgs::Vector3));
      surface_normals_length = surface_normals_lengthT;
      for( uint32_t i = 0; i < surface_normals_length; i++){
      offset += this->st_surface_normals.deserialize(inbuffer + offset);
        memcpy( &(this->surface_normals[i]), &(this->st_surface_normals), sizeof(geometry_msgs::Vector3));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->friction_coeff));
     return offset;
    }

    virtual const char * getType() override { return "xpp_msgs/TerrainInfo"; };
    virtual const char * getMD5() override { return "58f8d0d19c0428c00252cd1c16c74dcf"; };

  };

}
#endif
