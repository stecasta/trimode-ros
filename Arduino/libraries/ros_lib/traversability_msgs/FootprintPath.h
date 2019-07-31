#ifndef _ROS_traversability_msgs_FootprintPath_h
#define _ROS_traversability_msgs_FootprintPath_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PolygonStamped.h"

namespace traversability_msgs
{

  class FootprintPath : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseArray _poses_type;
      _poses_type poses;
      typedef float _radius_type;
      _radius_type radius;
      typedef geometry_msgs::PolygonStamped _footprint_type;
      _footprint_type footprint;
      typedef bool _conservative_type;
      _conservative_type conservative;

    FootprintPath():
      poses(),
      radius(0),
      footprint(),
      conservative(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->poses.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->radius);
      offset += this->footprint.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_conservative;
      u_conservative.real = this->conservative;
      *(outbuffer + offset + 0) = (u_conservative.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->conservative);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->poses.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->radius));
      offset += this->footprint.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_conservative;
      u_conservative.base = 0;
      u_conservative.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->conservative = u_conservative.real;
      offset += sizeof(this->conservative);
     return offset;
    }

    const char * getType(){ return "traversability_msgs/FootprintPath"; };
    const char * getMD5(){ return "d43dfe3c00d64e1bee2d24de57335cf2"; };

  };

}
#endif