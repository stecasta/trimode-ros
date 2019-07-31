#ifndef _ROS_SERVICE_Overwrite_h
#define _ROS_SERVICE_Overwrite_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace traversability_msgs
{

static const char OVERWRITE[] = "traversability_msgs/Overwrite";

  class OverwriteRequest : public ros::Msg
  {
    public:
      typedef bool _enable_type;
      _enable_type enable;

    OverwriteRequest():
      enable(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
     return offset;
    }

    const char * getType(){ return OVERWRITE; };
    const char * getMD5(){ return "8c1211af706069c994c06e00eb59ac9e"; };

  };

  class OverwriteResponse : public ros::Msg
  {
    public:

    OverwriteResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return OVERWRITE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Overwrite {
    public:
    typedef OverwriteRequest Request;
    typedef OverwriteResponse Response;
  };

}
#endif
