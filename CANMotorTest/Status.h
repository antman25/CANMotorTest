#ifndef _ROS_titan_base_Status_h
#define _ROS_titan_base_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace titan_base
{

  class Status : public ros::Msg
  {
    public:
      typedef int8_t _DeviceId_type;
      _DeviceId_type DeviceId;
      typedef bool _Fault_OverTemp_type;
      _Fault_OverTemp_type Fault_OverTemp;
      typedef bool _Fault_UnderVoltage_type;
      _Fault_UnderVoltage_type Fault_UnderVoltage;
      typedef bool _Fault_ForLim_type;
      _Fault_ForLim_type Fault_ForLim;
      typedef bool _Fault_RevLim_type;
      _Fault_RevLim_type Fault_RevLim;
      typedef bool _Fault_HardwareFailure_type;
      _Fault_HardwareFailure_type Fault_HardwareFailure;
      typedef bool _Fault_ForSoftLim_type;
      _Fault_ForSoftLim_type Fault_ForSoftLim;
      typedef bool _Fault_RevSoftLim_type;
      _Fault_RevSoftLim_type Fault_RevSoftLim;
      typedef bool _StckyFault_OverTemp_type;
      _StckyFault_OverTemp_type StckyFault_OverTemp;
      typedef bool _StckyFault_UnderVoltage_type;
      _StckyFault_UnderVoltage_type StckyFault_UnderVoltage;
      typedef bool _StckyFault_ForLim_type;
      _StckyFault_ForLim_type StckyFault_ForLim;
      typedef bool _StckyFault_RevLim_type;
      _StckyFault_RevLim_type StckyFault_RevLim;
      typedef bool _StckyFault_ForSoftLim_type;
      _StckyFault_ForSoftLim_type StckyFault_ForSoftLim;
      typedef bool _StckyFault_RevSoftLim_type;
      _StckyFault_RevSoftLim_type StckyFault_RevSoftLim;
      typedef int16_t _AppliedThrottle_type;
      _AppliedThrottle_type AppliedThrottle;
      typedef int32_t _CloseLoopErr_type;
      _CloseLoopErr_type CloseLoopErr;
      typedef int32_t _SensorPosition_type;
      _SensorPosition_type SensorPosition;
      typedef int32_t _SensorVelocity_type;
      _SensorVelocity_type SensorVelocity;
      typedef float _Current_type;
      _Current_type Current;
      typedef bool _BrakeIsEnabled_type;
      _BrakeIsEnabled_type BrakeIsEnabled;
      typedef int32_t _EncPosition_type;
      _EncPosition_type EncPosition;
      typedef int32_t _EncVel_type;
      _EncVel_type EncVel;
      typedef float _Temp_type;
      _Temp_type Temp;
      typedef float _BatteryV_type;
      _BatteryV_type BatteryV;
      typedef int32_t _ResetCount_type;
      _ResetCount_type ResetCount;
      typedef int32_t _ResetFlags_type;
      _ResetFlags_type ResetFlags;

    Status():
      DeviceId(0),
      Fault_OverTemp(0),
      Fault_UnderVoltage(0),
      Fault_ForLim(0),
      Fault_RevLim(0),
      Fault_HardwareFailure(0),
      Fault_ForSoftLim(0),
      Fault_RevSoftLim(0),
      StckyFault_OverTemp(0),
      StckyFault_UnderVoltage(0),
      StckyFault_ForLim(0),
      StckyFault_RevLim(0),
      StckyFault_ForSoftLim(0),
      StckyFault_RevSoftLim(0),
      AppliedThrottle(0),
      CloseLoopErr(0),
      SensorPosition(0),
      SensorVelocity(0),
      Current(0),
      BrakeIsEnabled(0),
      EncPosition(0),
      EncVel(0),
      Temp(0),
      BatteryV(0),
      ResetCount(0),
      ResetFlags(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_DeviceId;
      u_DeviceId.real = this->DeviceId;
      *(outbuffer + offset + 0) = (u_DeviceId.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->DeviceId);
      union {
        bool real;
        uint8_t base;
      } u_Fault_OverTemp;
      u_Fault_OverTemp.real = this->Fault_OverTemp;
      *(outbuffer + offset + 0) = (u_Fault_OverTemp.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Fault_OverTemp);
      union {
        bool real;
        uint8_t base;
      } u_Fault_UnderVoltage;
      u_Fault_UnderVoltage.real = this->Fault_UnderVoltage;
      *(outbuffer + offset + 0) = (u_Fault_UnderVoltage.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Fault_UnderVoltage);
      union {
        bool real;
        uint8_t base;
      } u_Fault_ForLim;
      u_Fault_ForLim.real = this->Fault_ForLim;
      *(outbuffer + offset + 0) = (u_Fault_ForLim.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Fault_ForLim);
      union {
        bool real;
        uint8_t base;
      } u_Fault_RevLim;
      u_Fault_RevLim.real = this->Fault_RevLim;
      *(outbuffer + offset + 0) = (u_Fault_RevLim.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Fault_RevLim);
      union {
        bool real;
        uint8_t base;
      } u_Fault_HardwareFailure;
      u_Fault_HardwareFailure.real = this->Fault_HardwareFailure;
      *(outbuffer + offset + 0) = (u_Fault_HardwareFailure.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Fault_HardwareFailure);
      union {
        bool real;
        uint8_t base;
      } u_Fault_ForSoftLim;
      u_Fault_ForSoftLim.real = this->Fault_ForSoftLim;
      *(outbuffer + offset + 0) = (u_Fault_ForSoftLim.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Fault_ForSoftLim);
      union {
        bool real;
        uint8_t base;
      } u_Fault_RevSoftLim;
      u_Fault_RevSoftLim.real = this->Fault_RevSoftLim;
      *(outbuffer + offset + 0) = (u_Fault_RevSoftLim.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Fault_RevSoftLim);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_OverTemp;
      u_StckyFault_OverTemp.real = this->StckyFault_OverTemp;
      *(outbuffer + offset + 0) = (u_StckyFault_OverTemp.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->StckyFault_OverTemp);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_UnderVoltage;
      u_StckyFault_UnderVoltage.real = this->StckyFault_UnderVoltage;
      *(outbuffer + offset + 0) = (u_StckyFault_UnderVoltage.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->StckyFault_UnderVoltage);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_ForLim;
      u_StckyFault_ForLim.real = this->StckyFault_ForLim;
      *(outbuffer + offset + 0) = (u_StckyFault_ForLim.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->StckyFault_ForLim);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_RevLim;
      u_StckyFault_RevLim.real = this->StckyFault_RevLim;
      *(outbuffer + offset + 0) = (u_StckyFault_RevLim.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->StckyFault_RevLim);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_ForSoftLim;
      u_StckyFault_ForSoftLim.real = this->StckyFault_ForSoftLim;
      *(outbuffer + offset + 0) = (u_StckyFault_ForSoftLim.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->StckyFault_ForSoftLim);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_RevSoftLim;
      u_StckyFault_RevSoftLim.real = this->StckyFault_RevSoftLim;
      *(outbuffer + offset + 0) = (u_StckyFault_RevSoftLim.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->StckyFault_RevSoftLim);
      union {
        int16_t real;
        uint16_t base;
      } u_AppliedThrottle;
      u_AppliedThrottle.real = this->AppliedThrottle;
      *(outbuffer + offset + 0) = (u_AppliedThrottle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AppliedThrottle.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->AppliedThrottle);
      union {
        int32_t real;
        uint32_t base;
      } u_CloseLoopErr;
      u_CloseLoopErr.real = this->CloseLoopErr;
      *(outbuffer + offset + 0) = (u_CloseLoopErr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_CloseLoopErr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_CloseLoopErr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_CloseLoopErr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->CloseLoopErr);
      union {
        int32_t real;
        uint32_t base;
      } u_SensorPosition;
      u_SensorPosition.real = this->SensorPosition;
      *(outbuffer + offset + 0) = (u_SensorPosition.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_SensorPosition.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_SensorPosition.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_SensorPosition.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->SensorPosition);
      union {
        int32_t real;
        uint32_t base;
      } u_SensorVelocity;
      u_SensorVelocity.real = this->SensorVelocity;
      *(outbuffer + offset + 0) = (u_SensorVelocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_SensorVelocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_SensorVelocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_SensorVelocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->SensorVelocity);
      union {
        float real;
        uint32_t base;
      } u_Current;
      u_Current.real = this->Current;
      *(outbuffer + offset + 0) = (u_Current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Current);
      union {
        bool real;
        uint8_t base;
      } u_BrakeIsEnabled;
      u_BrakeIsEnabled.real = this->BrakeIsEnabled;
      *(outbuffer + offset + 0) = (u_BrakeIsEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->BrakeIsEnabled);
      union {
        int32_t real;
        uint32_t base;
      } u_EncPosition;
      u_EncPosition.real = this->EncPosition;
      *(outbuffer + offset + 0) = (u_EncPosition.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_EncPosition.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_EncPosition.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_EncPosition.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->EncPosition);
      union {
        int32_t real;
        uint32_t base;
      } u_EncVel;
      u_EncVel.real = this->EncVel;
      *(outbuffer + offset + 0) = (u_EncVel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_EncVel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_EncVel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_EncVel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->EncVel);
      union {
        float real;
        uint32_t base;
      } u_Temp;
      u_Temp.real = this->Temp;
      *(outbuffer + offset + 0) = (u_Temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Temp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Temp);
      union {
        float real;
        uint32_t base;
      } u_BatteryV;
      u_BatteryV.real = this->BatteryV;
      *(outbuffer + offset + 0) = (u_BatteryV.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_BatteryV.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_BatteryV.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_BatteryV.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->BatteryV);
      union {
        int32_t real;
        uint32_t base;
      } u_ResetCount;
      u_ResetCount.real = this->ResetCount;
      *(outbuffer + offset + 0) = (u_ResetCount.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ResetCount.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ResetCount.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ResetCount.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ResetCount);
      union {
        int32_t real;
        uint32_t base;
      } u_ResetFlags;
      u_ResetFlags.real = this->ResetFlags;
      *(outbuffer + offset + 0) = (u_ResetFlags.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ResetFlags.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ResetFlags.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ResetFlags.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ResetFlags);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_DeviceId;
      u_DeviceId.base = 0;
      u_DeviceId.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->DeviceId = u_DeviceId.real;
      offset += sizeof(this->DeviceId);
      union {
        bool real;
        uint8_t base;
      } u_Fault_OverTemp;
      u_Fault_OverTemp.base = 0;
      u_Fault_OverTemp.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Fault_OverTemp = u_Fault_OverTemp.real;
      offset += sizeof(this->Fault_OverTemp);
      union {
        bool real;
        uint8_t base;
      } u_Fault_UnderVoltage;
      u_Fault_UnderVoltage.base = 0;
      u_Fault_UnderVoltage.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Fault_UnderVoltage = u_Fault_UnderVoltage.real;
      offset += sizeof(this->Fault_UnderVoltage);
      union {
        bool real;
        uint8_t base;
      } u_Fault_ForLim;
      u_Fault_ForLim.base = 0;
      u_Fault_ForLim.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Fault_ForLim = u_Fault_ForLim.real;
      offset += sizeof(this->Fault_ForLim);
      union {
        bool real;
        uint8_t base;
      } u_Fault_RevLim;
      u_Fault_RevLim.base = 0;
      u_Fault_RevLim.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Fault_RevLim = u_Fault_RevLim.real;
      offset += sizeof(this->Fault_RevLim);
      union {
        bool real;
        uint8_t base;
      } u_Fault_HardwareFailure;
      u_Fault_HardwareFailure.base = 0;
      u_Fault_HardwareFailure.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Fault_HardwareFailure = u_Fault_HardwareFailure.real;
      offset += sizeof(this->Fault_HardwareFailure);
      union {
        bool real;
        uint8_t base;
      } u_Fault_ForSoftLim;
      u_Fault_ForSoftLim.base = 0;
      u_Fault_ForSoftLim.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Fault_ForSoftLim = u_Fault_ForSoftLim.real;
      offset += sizeof(this->Fault_ForSoftLim);
      union {
        bool real;
        uint8_t base;
      } u_Fault_RevSoftLim;
      u_Fault_RevSoftLim.base = 0;
      u_Fault_RevSoftLim.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Fault_RevSoftLim = u_Fault_RevSoftLim.real;
      offset += sizeof(this->Fault_RevSoftLim);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_OverTemp;
      u_StckyFault_OverTemp.base = 0;
      u_StckyFault_OverTemp.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->StckyFault_OverTemp = u_StckyFault_OverTemp.real;
      offset += sizeof(this->StckyFault_OverTemp);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_UnderVoltage;
      u_StckyFault_UnderVoltage.base = 0;
      u_StckyFault_UnderVoltage.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->StckyFault_UnderVoltage = u_StckyFault_UnderVoltage.real;
      offset += sizeof(this->StckyFault_UnderVoltage);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_ForLim;
      u_StckyFault_ForLim.base = 0;
      u_StckyFault_ForLim.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->StckyFault_ForLim = u_StckyFault_ForLim.real;
      offset += sizeof(this->StckyFault_ForLim);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_RevLim;
      u_StckyFault_RevLim.base = 0;
      u_StckyFault_RevLim.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->StckyFault_RevLim = u_StckyFault_RevLim.real;
      offset += sizeof(this->StckyFault_RevLim);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_ForSoftLim;
      u_StckyFault_ForSoftLim.base = 0;
      u_StckyFault_ForSoftLim.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->StckyFault_ForSoftLim = u_StckyFault_ForSoftLim.real;
      offset += sizeof(this->StckyFault_ForSoftLim);
      union {
        bool real;
        uint8_t base;
      } u_StckyFault_RevSoftLim;
      u_StckyFault_RevSoftLim.base = 0;
      u_StckyFault_RevSoftLim.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->StckyFault_RevSoftLim = u_StckyFault_RevSoftLim.real;
      offset += sizeof(this->StckyFault_RevSoftLim);
      union {
        int16_t real;
        uint16_t base;
      } u_AppliedThrottle;
      u_AppliedThrottle.base = 0;
      u_AppliedThrottle.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AppliedThrottle.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->AppliedThrottle = u_AppliedThrottle.real;
      offset += sizeof(this->AppliedThrottle);
      union {
        int32_t real;
        uint32_t base;
      } u_CloseLoopErr;
      u_CloseLoopErr.base = 0;
      u_CloseLoopErr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_CloseLoopErr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_CloseLoopErr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_CloseLoopErr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->CloseLoopErr = u_CloseLoopErr.real;
      offset += sizeof(this->CloseLoopErr);
      union {
        int32_t real;
        uint32_t base;
      } u_SensorPosition;
      u_SensorPosition.base = 0;
      u_SensorPosition.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_SensorPosition.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_SensorPosition.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_SensorPosition.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->SensorPosition = u_SensorPosition.real;
      offset += sizeof(this->SensorPosition);
      union {
        int32_t real;
        uint32_t base;
      } u_SensorVelocity;
      u_SensorVelocity.base = 0;
      u_SensorVelocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_SensorVelocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_SensorVelocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_SensorVelocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->SensorVelocity = u_SensorVelocity.real;
      offset += sizeof(this->SensorVelocity);
      union {
        float real;
        uint32_t base;
      } u_Current;
      u_Current.base = 0;
      u_Current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Current = u_Current.real;
      offset += sizeof(this->Current);
      union {
        bool real;
        uint8_t base;
      } u_BrakeIsEnabled;
      u_BrakeIsEnabled.base = 0;
      u_BrakeIsEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->BrakeIsEnabled = u_BrakeIsEnabled.real;
      offset += sizeof(this->BrakeIsEnabled);
      union {
        int32_t real;
        uint32_t base;
      } u_EncPosition;
      u_EncPosition.base = 0;
      u_EncPosition.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_EncPosition.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_EncPosition.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_EncPosition.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->EncPosition = u_EncPosition.real;
      offset += sizeof(this->EncPosition);
      union {
        int32_t real;
        uint32_t base;
      } u_EncVel;
      u_EncVel.base = 0;
      u_EncVel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_EncVel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_EncVel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_EncVel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->EncVel = u_EncVel.real;
      offset += sizeof(this->EncVel);
      union {
        float real;
        uint32_t base;
      } u_Temp;
      u_Temp.base = 0;
      u_Temp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Temp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Temp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Temp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Temp = u_Temp.real;
      offset += sizeof(this->Temp);
      union {
        float real;
        uint32_t base;
      } u_BatteryV;
      u_BatteryV.base = 0;
      u_BatteryV.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_BatteryV.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_BatteryV.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_BatteryV.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->BatteryV = u_BatteryV.real;
      offset += sizeof(this->BatteryV);
      union {
        int32_t real;
        uint32_t base;
      } u_ResetCount;
      u_ResetCount.base = 0;
      u_ResetCount.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ResetCount.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ResetCount.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ResetCount.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ResetCount = u_ResetCount.real;
      offset += sizeof(this->ResetCount);
      union {
        int32_t real;
        uint32_t base;
      } u_ResetFlags;
      u_ResetFlags.base = 0;
      u_ResetFlags.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ResetFlags.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ResetFlags.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ResetFlags.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ResetFlags = u_ResetFlags.real;
      offset += sizeof(this->ResetFlags);
     return offset;
    }

    const char * getType(){ return "titan_base/Status"; };
    const char * getMD5(){ return "ecb0a2154305887f61cce4cb81170ab6"; };

  };

}
#endif