#ifndef _ROS_kindr_msgs_VectorAtPosition_h
#define _ROS_kindr_msgs_VectorAtPosition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"

namespace kindr_msgs
{

  class VectorAtPosition : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _type_type;
      _type_type type;
      typedef const char* _name_type;
      _name_type name;
      typedef geometry_msgs::Vector3 _vector_type;
      _vector_type vector;
      typedef geometry_msgs::Point _position_type;
      _position_type position;
      typedef const char* _position_frame_id_type;
      _position_frame_id_type position_frame_id;
      enum { TYPE_TYPELESS = 0 };
      enum { TYPE_JERK = 6 };
      enum { TYPE_ACCELERATION = 7 };
      enum { TYPE_VELOCITY = 8 };
      enum { TYPE_POSITION = 9 };
      enum { TYPE_FORCE = 10 };
      enum { TYPE_MOMEMTUM = 11 };
      enum { TYPE_ANGULAR_JERK = 12 };
      enum { TYPE_ANGULAR_ACCELERATION = 13 };
      enum { TYPE_ANGULAR_VELOCITY = 14 };
      enum { TYPE_TORQUE = 16 };
      enum { TYPE_ANGULAR_MOMEMTUM = 17 };

    VectorAtPosition():
      header(),
      type(0),
      name(""),
      vector(),
      position(),
      position_frame_id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += this->vector.serialize(outbuffer + offset);
      offset += this->position.serialize(outbuffer + offset);
      uint32_t length_position_frame_id = strlen(this->position_frame_id);
      varToArr(outbuffer + offset, length_position_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->position_frame_id, length_position_frame_id);
      offset += length_position_frame_id;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += this->vector.deserialize(inbuffer + offset);
      offset += this->position.deserialize(inbuffer + offset);
      uint32_t length_position_frame_id;
      arrToVar(length_position_frame_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_position_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_position_frame_id-1]=0;
      this->position_frame_id = (char *)(inbuffer + offset-1);
      offset += length_position_frame_id;
     return offset;
    }

    const char * getType(){ return "kindr_msgs/VectorAtPosition"; };
    const char * getMD5(){ return "fcf32a1df9f6d53ef1926f20ce6b66e0"; };

  };

}
#endif