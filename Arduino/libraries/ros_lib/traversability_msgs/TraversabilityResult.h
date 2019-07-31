#ifndef _ROS_traversability_msgs_TraversabilityResult_h
#define _ROS_traversability_msgs_TraversabilityResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace traversability_msgs
{

  class TraversabilityResult : public ros::Msg
  {
    public:
      typedef bool _is_safe_type;
      _is_safe_type is_safe;
      typedef float _traversability_type;
      _traversability_type traversability;
      typedef float _area_type;
      _area_type area;

    TraversabilityResult():
      is_safe(0),
      traversability(0),
      area(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_safe;
      u_is_safe.real = this->is_safe;
      *(outbuffer + offset + 0) = (u_is_safe.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_safe);
      offset += serializeAvrFloat64(outbuffer + offset, this->traversability);
      offset += serializeAvrFloat64(outbuffer + offset, this->area);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_safe;
      u_is_safe.base = 0;
      u_is_safe.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_safe = u_is_safe.real;
      offset += sizeof(this->is_safe);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->traversability));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->area));
     return offset;
    }

    const char * getType(){ return "traversability_msgs/TraversabilityResult"; };
    const char * getMD5(){ return "14ffe3323c91cd823bef7a313714954e"; };

  };

}
#endif