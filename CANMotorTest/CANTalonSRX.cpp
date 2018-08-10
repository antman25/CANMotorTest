#include "Arduino.h"
#include "CANTalonSRX.h"


#define TALONSRX_PREFIX 0x02040000
#define STATUS_1        0x02041400
#define STATUS_2        0x02041440
#define STATUS_3        0x02041480
#define STATUS_4        0x020414C0
#define STATUS_5        0x02041500
#define STATUS_6        0x02041540
#define STATUS_7        0x02041580
#define STATUS_8        0x020415C0
#define STATUS_9        0x02041600
#define STATUS_10       0x02041640
#define STATUS_11       0x02041680
#define STATUS_12       0x020416C0
#define STATUS_13       0x02041700
#define STATUS_14       0x02041740
#define STATUS_15       0x02041780

#define CONTROL_1       0x02040000
#define CONTROL_2       0x02040040
#define CONTROL_3       0x02040080
#define CONTROL_5       0x02040100
#define CONTROL_6       0x02040140

#define MOTOR_CONTROL   0x0401BF



#define EXPECTED_RESPONSE_TIMEOUT_MS  (200)

#define PARAM_REQUEST     0x02041800
#define PARAM_RESPONSE    0x02041840
#define PARAM_SET         0x02041880

#define ID_REQUEST        0x0204F800

const int kParamArbIdValue =  PARAM_RESPONSE;
const int kParamArbIdMask =   0xFFFFFFFF;

const double FLOAT_TO_FXP_10_22 = (double)0x400000;
const double FXP_TO_FLOAT_10_22 = 0.0000002384185791015625;

const double FLOAT_TO_FXP_0_8 = (double)0x100;
const double FXP_TO_FLOAT_0_8 = 0.00390625;

/* encoder/decoders */






 
void printCAN(CAN_message_t msg)
{
  Serial.print(millis());
  Serial.print(" --- ");
  Serial.print("ID: ");
  Serial.print(msg.id,HEX);
  Serial.print(" Ext: ");
  Serial.print(msg.ext,HEX);
  Serial.print(" Len: ");
  Serial.print(msg.len);
  Serial.print(" Data: ");
  for (int i=0;i<msg.len;i++)
  {
    Serial.print(msg.buf[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void CANTalonSRX::printParams()
{
  for (int i = 0;i<450;i++)
  {
    if (param_timestamp[i] > 0)
    {
      Serial.print(param_timestamp[i]);
      Serial.print(" -- Param: ");
      Serial.print(i);
      Serial.print(" = ");
      Serial.print(param[i]);
      
      float bit8 = FXP_TO_FLOAT_0_8 * param[i];
      float bit10 = FXP_TO_FLOAT_10_22 * param[i];
      Serial.print(" - 0_8: " );
      Serial.print(bit8);
      Serial.print(" - 10_22: " );
      Serial.println(bit10);
    }
  }
}

CANTalonSRX::CANTalonSRX(uint8_t deviceNumber,int controlPeriodMs)
{
  CANTalonSRX::deviceNumber = deviceNumber;
  CANTalonSRX::controlPeriodMs = controlPeriodMs;
  
}

void CANTalonSRX::begin(FlexCAN *CANDev)
{
  CANbus0 = *CANDev;

  for (byte i =0;i<255;i++)
  {
    param[i] = 0;
    param_timestamp[i] = 0;
  }
  memset(&status1,0,sizeof(status1));
  memset(&status1,0,sizeof(status2));
  memset(&status1,0,sizeof(status3));
  memset(&status1,0,sizeof(status4));
  memset(&status1,0,sizeof(status5));
  memset(&status1,0,sizeof(status6));
  memset(&status1,0,sizeof(status7));
  memset(&status1,0,sizeof(status8));
  memset(&status1,0,sizeof(control1));
  memset(&param_response,0,sizeof(param_response));
  memset(&demand,0,sizeof(demand));
}

void CANTalonSRX::sendMotorEnable(bool motor_enable)
{
  CAN_message_t msg;
  msg.id = MOTOR_CONTROL;
  for (byte i=0;i<8;i++)
  {
    msg.buf[i] = 0;
  }
  if (motor_enable == true)
  {
    msg.buf[0] = 0x01;
  }
  msg.len = 8;
  CANbus0.write(msg);
}

void CANTalonSRX::GetParamRaw(param_t paramEnum, int32_t value, int8_t subValue, int8_t ordinal)
{
  TALON_Param_Request_t frame;
  memset(&frame,0,sizeof(frame));

  //Serial.print("Generating Request for Param: ");
  //Serial.println((int)paramEnum);

  byte paramEnum_h8 = (byte)((int)paramEnum >> 4);
  byte paramEnum_l4 = (byte)((int)paramEnum & 0xF);

  /*Serial.print("enum_h8: ");
  Serial.println(paramEnum_h8, HEX);
  Serial.print("enum_l4: ");
  Serial.println(paramEnum_l4, HEX);*/

  //Serial.print("Generating Request for Param: ");
  //Serial.println((int)paramEnum);
  
  frame.ParamEnumH = paramEnum_h8;
  frame.ParamEnumL = paramEnum_l4;
  
  
  frame.ParamValueH = value >> 0x18;
  frame.ParamValueMH = value >> 0x10;
  frame.ParamValueML = value >> 0x08;
  frame.ParamValueL = value;

  frame.SubValue = subValue;
  //int32_t status = 0;
  //FRC_NetworkCommunication_CANSessionMux_sendMessage(PARAM_SET | GetDeviceNumber(), (const uint8_t*)&frame, 5, 0, &status);
  CAN_message_t msg;
  msg.id = PARAM_REQUEST | deviceNumber;
  msg.ext = 1;
  memcpy(msg.buf,&frame,sizeof(frame));
  msg.len = 8;
  //Serial.println("Request Message:");
  //printCAN(msg);
  CANbus0.write(msg);
  delay(10);
}

void CANTalonSRX::SetParamRaw(unsigned paramEnum, int rawBits, byte subValue, byte ordinal)
{
  TALON_Param_Request_t frame;
  memset(&frame,0,sizeof(frame));
  
  byte paramEnum_h8 = (byte)((int)paramEnum >> 4);
  byte paramEnum_l4 = (byte)((int)paramEnum & 0xF);
  
  frame.ParamEnumH = paramEnum_h8;
  frame.ParamEnumL = paramEnum_l4;
  
  frame.ParamValueH = rawBits >> 0x18;
  frame.ParamValueMH = rawBits >> 0x10;
  frame.ParamValueML = rawBits >> 0x08;
  frame.ParamValueL = rawBits;

  frame.SubValue = subValue;
  //int32_t status = 0;
  //FRC_NetworkCommunication_CANSessionMux_sendMessage(PARAM_SET | GetDeviceNumber(), (const uint8_t*)&frame, 5, 0, &status);
  CAN_message_t msg;
  msg.id = PARAM_SET | deviceNumber;
  msg.ext = 1;
  memcpy(msg.buf,&frame,sizeof(frame));
  msg.len = 8;
  //Serial.println("Set Param Raw:");
  //printCAN(msg);
  CANbus0.write(msg);
  delay(10);
}

void CANTalonSRX::SetParam(param_t paramEnum, double value)
{
  int32_t rawbits = 0;
  switch(paramEnum){
    case eProfileParamSlot_P:/* unsigned 10.22 fixed pt value */
    case eProfileParamSlot_I:
    case eProfileParamSlot_D:
    {
      uint32_t urawbits;
      value = min(value,1023.0); /* bounds check doubles that are outside u10.22 */
      value = max(value,0.0);
      urawbits = value * FLOAT_TO_FXP_10_22; /* perform unsign arithmetic */
      
      rawbits = urawbits; /* copy bits over.  SetParamRaw just stuffs into CAN frame with no sense of signedness */
    } break;
    case eProfileParamSlot_F:  /* signed 10.22 fixed pt value */
      value = min(value, 512.0); /* bounds check doubles that are outside s10.22 */
      value = max(value,-512.0);
      rawbits = value * FLOAT_TO_FXP_10_22;
      //Serial.print("Raw: ");
      //Serial.println(rawbits);
      break;
    default: /* everything else is integral */
      rawbits = (int32_t)value;
      break;
  }
  SetParamRaw(paramEnum,rawbits,0,0);
}

void CANTalonSRX::SetDemand(int mode, int demand0)
{ 
  byte d0_h = (byte)(demand0 >> 16);
  byte d0_m = (byte)(demand0 >> 8);
  byte d0_l = (byte)(demand0 >> 0);

  byte ctrl_mode = (byte)(mode & 0x0F);
  //memcpy(demand,0,sizeof(demand));
  demand.Demand0H = d0_h;
  demand.Demand0M = d0_m;
  demand.Demand0L = d0_l;
  demand.Mode_4b = ctrl_mode;
  demand.SensorPhase = sensorPhase;
  demand.Inverted = outputInvert;
  demand.NeutralMode = neutralMode;

  CAN_message_t msg;
  msg.id = CONTROL_3 | deviceNumber;
  msg.ext = 1;
  memcpy(msg.buf,&demand,sizeof(demand));
  msg.len = 8;
  //printCAN(msg);
  CANbus0.write(msg);
  delay(1);
}

void CANTalonSRX::Set(int mode, double demand0)
{
  /*if(value > 1)
    value = 1;
  else if(value < -1)
    value = -1;
  SetDemand(kMode_DutyCycle, 1023*value);*/
  switch (mode)
  {
    case kMode_CurrentCloseLoop:
    case kMode_PositionCloseLoop:
    case kMode_VelocityCloseLoop:
      setPoint = (int)demand0;
      break;
    case kMode_DutyCycle:
    case kMode_SlaveFollower:
    case kMode_NoDrive:
      /* there is no target */
      break;
  }

  switch (mode)
  {
    case kMode_DutyCycle:
      if (demand0 > +1) { demand0 = +1; }
      if (demand0 < -1) { demand0 = -1; }
      /* scale [-1,+1] => [-1023,+1023] */
      demand0 *= 1023;
      break;
    case kMode_PositionCloseLoop: /* if a mode is missing, compiler warning will catch it */
    case kMode_VelocityCloseLoop:
    case kMode_CurrentCloseLoop:
    case kMode_SlaveFollower:  
    case kMode_NoDrive:
      break;
  }

  int idemand0 = (int)demand0;
  //int idemand1 = (int)demand1;
  /* unpack */
  /*byte d0_h8 = (byte)(idemand0 >> 0x10);
  byte d0_m8 = (byte)(idemand0 >> 0x08);
  byte d0_l8 = (byte)(idemand0);*/
  
  //int mode_4b = (int)mode & 0xf;
  
  /*demand.Demand0H = d0_h8;
  demand.Demand0M = d0_m8;
  demand.Demand0L = d0_l8;

  demand.Mode_4b = mode_4b;*/

  SetDemand(mode,idemand0);
}

void CANTalonSRX::SetProfileSlotSelect(int param)
{
  byte slot = param & 0x03;
  demand.SlotProfile = slot;
}

void CANTalonSRX::SetSensorPhase(bool param)
{
  //sensorPhase = param ? 1 : 0;
  if (param == true)
  {
    sensorPhase = 1;
  }
  else
  {
    sensorPhase = 0;
  }
}

void CANTalonSRX::SetMotorInvert(bool param)
{
  if (param == true)
  {
    outputInvert = 1;
  }
  else
  {
    outputInvert = 0;
  }
}

void CANTalonSRX::SetRevFeedbackSensor(int param)
{
  

}

/*public void SetSensorPhase(bool PhaseSensor)
{
  byte aBit = PhaseSensor ? 1 : 0;
  SetClrSmallVal(aBit, 1, 7, 7, CONTROL_3);
}*/

void CANTalonSRX::SetPgain(unsigned slotIdx,double gain)
{
  if(slotIdx == 0)
    return SetParam(eProfileParamSlot_P, gain);
  return SetParam(eProfileParamSlot_P, gain);
}
void CANTalonSRX::SetIgain(unsigned slotIdx,double gain)
{
  if(slotIdx == 0)
    return SetParam(eProfileParamSlot_I, gain);
  return SetParam(eProfileParamSlot_I, gain);
}
void CANTalonSRX::SetDgain(unsigned slotIdx,double gain)
{
  if(slotIdx == 0)
    return SetParam(eProfileParamSlot_D, gain);
  return SetParam(eProfileParamSlot_D, gain);
}
void CANTalonSRX::SetFgain(unsigned slotIdx,double gain)
{
  if(slotIdx == 0)
    return SetParam(eProfileParamSlot_F, gain);
  return SetParam(eProfileParamSlot_F, gain);
}
void CANTalonSRX::SetIzone(unsigned slotIdx,int zone)
{
  if(slotIdx == 0)
    return SetParam(eProfileParamSlot_IZone, zone);
  return SetParam(eProfileParamSlot_IZone, zone);
}
void CANTalonSRX::SetCloseLoopRampRate(unsigned slotIdx,int closeLoopRampRate)
{
  if(slotIdx == 0)
    return SetParam(eClosedloopRamp, closeLoopRampRate);
  return SetParam(eClosedloopRamp, closeLoopRampRate);
}

void CANTalonSRX::SetFeedbackCoeff(float coeff)
{
  int val = (int)(65536.0 * coeff);
  SetParam(eSelectedSensorCoefficient,val);
}

void CANTalonSRX::SetNeutralMode(byte val)
{
  neutralMode = val & 0x03;
}

int32_t CANTalonSRX::GetSensorPos()
{
  int32_t raw = 0;
  raw |= status2.SensorPositionH;
  raw <<= 8;
  raw |= status2.SensorPositionM;
  raw <<= 8;
  raw |= status2.SensorPositionL;
  raw <<= (32-24); /* sign extend */
  raw >>= (32-24); /* sign extend */
  if(status2.PosDiv8)
    raw *= 8;
  return(int)raw;
}

int32_t CANTalonSRX::GetSensorVel()
{
  int32_t raw = 0;
  raw |= status2.SensorVelocityH;
  raw <<= 8;
  raw |= status2.SensorVelocityL;
  raw <<= (32-16); /* sign extend */
  raw >>= (32-16); /* sign extend */
  if(status2.VelDiv4)
    raw *= 4;
  return (int)raw;
}

int32_t CANTalonSRX::GetEncoderPos()
{
  int32_t raw = 0;
  raw |= status3.EncPositionH;
  raw <<= 8;
  raw |= status3.EncPositionM;
  raw <<= 8;
  raw |= status3.EncPositionL;
  raw <<= (32-24); /* sign extend */
  raw >>= (32-24); /* sign extend */
  if(status3.PosDiv8)
    raw *= 8;
  return(int)raw;
}

int32_t CANTalonSRX::GetEncoderVel()
{
  //GET_STATUS3();
  int32_t raw = 0;
  raw |= status3.EncVelH;
  raw <<= 8;
  raw |= status3.EncVelL;
  raw <<= (32-16); /* sign extend */
  raw >>= (32-16); /* sign extend */
  if(status3.VelDiv4)
    raw *= 4;
  return (int)raw;
}

int32_t CANTalonSRX::GetAppliedThrottle()
{
  int32_t raw = 0;
  raw |= status1.AppliedThrottle_h3;
  raw <<= 8;
  raw |= status1.AppliedThrottle_l8;
  raw <<= (32-11); /* sign extend */
  raw >>= (32-11); /* sign extend */
  return (int)raw;
}

int32_t CANTalonSRX::GetCloseLoopErr()
{
  int32_t raw = 0;
  raw |= status1.CloseLoopErrH;
  raw <<= 16 - 8;
  raw |= status1.CloseLoopErrM;
  raw <<= 8;
  raw |= status1.CloseLoopErrL;

  /*Serial.print("H: ");
  Serial.print(status1.CloseLoopErrH, HEX);
  Serial.print(" M: ");
  Serial.print(status1.CloseLoopErrM, HEX);
  Serial.print(" L: ");
  Serial.println(status1.CloseLoopErrL, HEX);*/
  
  raw <<= (32-24); /* sign extend */
  raw >>= (32-24); /* sign extend */
  return (int)raw;
}

int CANTalonSRX::getSetPoint()
{
  return setPoint;
}

int32_t CANTalonSRX::GetFeedbackDeviceSelect()
{
  return status1.FeedbackDeviceSelect;
}

int32_t CANTalonSRX::GetModeSelect()
{
  uint32_t raw = 0;
  raw |= status1.ModeSelect_h1;
  raw <<= 3;
  raw |= status1.ModeSelect_b3;
  return (int)raw;
}

double CANTalonSRX::GetCurrent()
{
  //GET_STATUS2();
  uint32_t raw = 0;
  raw |= status2.Current_h8;
  raw <<= 2;
  raw |= status2.Current_l2;
  return (double)raw * 0.125 + 0;
}

double CANTalonSRX::GetTemp()
{
  uint32_t raw = status4.Temp;
  return (double)raw;
}

double CANTalonSRX::GetBatteryV()
{
  uint32_t raw = status4.BatteryV;
  return (double)raw * 0.05 + 4;
}

bool CANTalonSRX::GetFaultRevSoftLimit()
{
  return status1.Fault_RevSoftLim ? true : false;
}

bool CANTalonSRX::GetFaultForSoftLimit()
{
  return status1.Fault_ForSoftLim ? true : false;
}

bool CANTalonSRX::GetFaultHardwarFailure()
{
  return status1.Fault_HardwareFailure ? true : false;
}

bool CANTalonSRX::GetFaultRevLimit()
{
  return status1.Fault_RevLim ? true : false;
}

bool CANTalonSRX::GetFaultForLimit()
{
  return status1.Fault_ForLim ? true : false;
}

bool CANTalonSRX::GetFaultUnderVoltage()
{
  return status1.Fault_UnderVoltage ? true : false;
}

bool CANTalonSRX::GetFaultOverTemp()
{
  return status1.Fault_OverTemp ? true : false;
}

bool CANTalonSRX::GetStickyFaultRevSoftLimit()
{
  return status2.StckyFault_RevSoftLim ? true : false;
}

bool CANTalonSRX::GetStickyFaultForSoftLimit()
{
  return status2.StckyFault_ForSoftLim ? true : false;
}

bool CANTalonSRX::GetStickyFaultRevLimit()
{
  return status2.StckyFault_RevLim ? true : false;
}

bool CANTalonSRX::GetStickyFaultForLimit()
{
  return status2.StckyFault_ForLim ? true : false;
}

bool CANTalonSRX::GetStickyFaultUnderVoltage()
{
  return status2.StckyFault_UnderVoltage ? true : false;
}

bool CANTalonSRX::GetStickyFaultOverTemp()
{
  return status2.StckyFault_OverTemp ? true : false;
}

int16_t CANTalonSRX::GetResetCount()
{
  int16_t raw = status5.ResetCountH;
  raw <<= 8;
  raw |= status5.ResetCountL;
  return raw;
}

int16_t CANTalonSRX::GetResetFlags()
{
  int16_t raw = status5.ResetFlagsH;
  raw <<= 8;
  raw |= status5.ResetFlagsL;
  return raw;
}

void CANTalonSRX::update(CAN_message_t *inmsg)
{
  
    //if ((msg.id & TALONSRX_PREFIX) != TALONSRX_PREFIX)
    //  return;
    CAN_message_t msg = *inmsg;

    uint32_t messageMask = msg.id & 0xFFFF000F;
    uint32_t talonMask = TALONSRX_PREFIX | deviceNumber;
    
    /*Serial.print("MsgID: ");
    Serial.println(msg.id,HEX);
    Serial.print("MsgMask: ");
    Serial.println(messageMask,HEX);
    Serial.print("TalonMask: ");
    Serial.println(talonMask,HEX);*/
    
    if (messageMask != talonMask)
      return;
    
    if (msg.id == (STATUS_1 | deviceNumber))
    {
      memcpy(&status1, msg.buf, msg.len);
      //printCAN(msg);
      status1_period = millis() - status1_timestamp;
      status1_timestamp = millis();
    } 
    else if (msg.id == (STATUS_2 | deviceNumber))
    {
      memcpy(&status2, msg.buf, msg.len);
      status2_period = millis() - status2_timestamp;
      status2_timestamp = millis();
      //if (status2.PosDiv8 == 0x01 || status2.VelDiv4 == 0x01)
      if (status2.VelDiv4 == 0x01)
      //if (status2.PosDiv8 == 0x01)
        digitalWrite(LED_BUILTIN, HIGH);
      //printCAN(msg);
    }
    else if (msg.id == (STATUS_3 | deviceNumber))
    {
      memcpy(&status3, msg.buf, msg.len);
      status3_period = millis() - status3_timestamp;
      status3_timestamp = millis();
      /*Serial.print("A: ");
      Serial.print(status3.QuadApin);
      Serial.print(" B: ");
      Serial.println(status3.QuadBpin);*/
    }
    else if (msg.id == (STATUS_4 | deviceNumber))
    {
      memcpy(&status4, msg.buf, msg.len);
      status4_period = millis() - status4_timestamp;
      status4_timestamp = millis();
    }
    else if (msg.id == (STATUS_5 | deviceNumber))
    {
      memcpy(&status5, msg.buf, msg.len);
      status5_period = millis() - status5_timestamp;
      status5_timestamp = millis();
    }
    else if (msg.id == (STATUS_6 | deviceNumber))
    {
      //Serial.println("Status 6");
      memcpy(&status6, msg.buf, msg.len);
      status3_period = millis() - status3_timestamp;
      status3_timestamp = millis();
    }
    else if (msg.id == (STATUS_7 | deviceNumber))
    {
      //Serial.println("Status 7");
      memcpy(&status7, msg.buf, msg.len);
    }
    else if (msg.id == (STATUS_8 | deviceNumber))
    {
      //Serial.println("Status 8");
    }
    else if (msg.id == (STATUS_9 | deviceNumber))
    {
      //Serial.println("Status 9");
    }
    else if (msg.id == (STATUS_10 | deviceNumber))
    {
      //Serial.println("Status 10");
    }
    else if (msg.id == (STATUS_11 | deviceNumber))
    {
      //Serial.println("Status 11");
    }
    else if (msg.id == (STATUS_12 | deviceNumber))
    {
      //Serial.println("Status 12");
    }
    else if (msg.id == (STATUS_13 | deviceNumber))
    {
      //Serial.println("Status 13");
    }
    else if (msg.id == (STATUS_14 | deviceNumber))
    {
      //Serial.println("Status 14");
    }
    else if (msg.id == (STATUS_15 | deviceNumber))
    {
      //Serial.println("Status 15");
    }
    else if (msg.id == (CONTROL_1 | deviceNumber))
    {
      Serial.println("Control 1");
    }
    else if (msg.id == (CONTROL_2 | deviceNumber))
    {
      //Serial.println("Control 2");
    }
    else if (msg.id == (CONTROL_3 | deviceNumber))
    {
      //Serial.println("Control 3");
    }
    else if (msg.id == (CONTROL_5 | deviceNumber))
    {
      //Serial.println("Control 5");
    }
    else if (msg.id == (CONTROL_6 | deviceNumber))
    {
      //Serial.println("Control 6");
    }
    else if (msg.id == MOTOR_CONTROL)
    {
      //Serial.println("Motor Enable");
    }
    else if (msg.id == (PARAM_SET | deviceNumber))
    {
      //Serial.println("Param Set");
    }
    else if (msg.id == (PARAM_REQUEST | deviceNumber))
    {
      //Serial.println("Param Request");
    }
    else if (msg.id == (PARAM_RESPONSE | deviceNumber))
    {
      //Serial.println("Param Response:");
      memcpy(&param_response, msg.buf, msg.len);
      //printCAN(msg);
      int32_t ParamEnum = param_response.ParamEnumH;
      ParamEnum <<= 4;
      ParamEnum |= param_response.ParamEnumL;
      
      int32_t val = param_response.ParamValueH;
      val <<= 8;
      val |=  param_response.ParamValueMH;
      val <<= 8;
      val |=  param_response.ParamValueML;
      val <<= 8;
      val |=  param_response.ParamValueL;

      param_timestamp[ParamEnum] = millis();
      param[ParamEnum] = val;
      printParams();
    }
    else if (msg.id == (ID_REQUEST | deviceNumber))
    {
      
    }
    else
    {
      Serial.print("Unknown Frame(");
      Serial.print(deviceNumber);
      Serial.println(")");
      printCAN(msg);
    }
  
}

CANTalonSRX::~CANTalonSRX()
{
  
}

