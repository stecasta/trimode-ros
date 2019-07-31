#ifndef _ROS_SERVICE_CheckFootprintPath_h
#define _ROS_SERVICE_CheckFootprintPath_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "traversability_msgs/TraversabilityResult.h"
#include "traversability_msgs/FootprintPath.h"

namespace traversability_msgs
{

static const char CHECKFOOTPRINTPATH[] = "traversability_msgs/CheckFootprintPath";

  class CheckFootprintPathRequest : public ros::Msg
  {
    public:
      uint32_t path_length;
      typedef traversability_msgs::FootprintPath _path_type;
      _path_type st_path;
      _path_type * path;

    CheckFootprintPathRequest():
      path_length(0), path(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->path_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->path_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->path_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->path_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_length);
      for( uint32_t i = 0; i < path_length; i++){
      offset += this->path[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t path_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->path_length);
      if(path_lengthT > path_length)
        this->path = (traversability_msgs::FootprintPath*)realloc(this->path, path_lengthT * sizeof(traversability_msgs::FootprintPath));
      path_length = path_lengthT;
      for( uint32_t i = 0; i < path_length; i++){
      offset += this->st_path.deserialize(inbuffer + offset);
        memcpy( &(this->path[i]), &(this->st_path), sizeof(traversability_msgs::FootprintPath));
      }
     return offset;
    }

    const char * getType(){ return CHECKFOOTPRINTPATH; };
    const char * getMD5(){ return "1ca32ddc180224c61d2318492df7ea61"; };

  };

  class CheckFootprintPathResponse : public ros::Msg
  {
    public:
      uint32_t result_length;
      typedef traversability_msgs::TraversabilityResult _result_type;
      _result_type st_result;
      _result_type * result;

    CheckFootprintPathResponse():
      result_length(0), result(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->result_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->result_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->result_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->result_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->result_length);
      for( uint32_t i = 0; i < result_length; i++){
      offset += this->result[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t result_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      result_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      result_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      result_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->result_length);
      if(result_lengthT > result_length)
        this->result = (traversability_msgs::TraversabilityResult*)realloc(this->result, result_lengthT * sizeof(traversability_msgs::TraversabilityResult));
      result_length = result_lengthT;
      for( uint32_t i = 0; i < result_length; i++){
      offset += this->st_result.deserialize(inbuffer + offset);
        memcpy( &(this->result[i]), &(this->st_result), sizeof(traversability_msgs::TraversabilityResult));
      }
     return offset;
    }

    const char * getType(){ return CHECKFOOTPRINTPATH; };
    const char * getMD5(){ return "403c136f58a3e688827b9a267b872101"; };

  };

  class CheckFootprintPath {
    public:
    typedef CheckFootprintPathRequest Request;
    typedef CheckFootprintPathResponse Response;
  };

}
#endif
