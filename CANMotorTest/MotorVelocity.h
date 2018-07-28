#ifndef _ROS_titan_base_MotorVelocity_h
#define _ROS_titan_base_MotorVelocity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace titan_base
{

  class MotorVelocity : public ros::Msg
  {
    public:
      typedef float _left_angular_vel_type;
      _left_angular_vel_type left_angular_vel;
      typedef float _right_angular_vel_type;
      _right_angular_vel_type right_angular_vel;

    MotorVelocity():
      left_angular_vel(0),
      right_angular_vel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_angular_vel;
      u_left_angular_vel.real = this->left_angular_vel;
      *(outbuffer + offset + 0) = (u_left_angular_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_angular_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_angular_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_angular_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_angular_vel);
      union {
        float real;
        uint32_t base;
      } u_right_angular_vel;
      u_right_angular_vel.real = this->right_angular_vel;
      *(outbuffer + offset + 0) = (u_right_angular_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_angular_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_angular_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_angular_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_angular_vel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_angular_vel;
      u_left_angular_vel.base = 0;
      u_left_angular_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_angular_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_angular_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_angular_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_angular_vel = u_left_angular_vel.real;
      offset += sizeof(this->left_angular_vel);
      union {
        float real;
        uint32_t base;
      } u_right_angular_vel;
      u_right_angular_vel.base = 0;
      u_right_angular_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_angular_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_angular_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_angular_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_angular_vel = u_right_angular_vel.real;
      offset += sizeof(this->right_angular_vel);
     return offset;
    }

    const char * getType(){ return "titan_base/MotorVelocity"; };
    const char * getMD5(){ return "823c424390c2ee045ee83831121ee873"; };

  };

}
#endif