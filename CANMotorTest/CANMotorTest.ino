#include "CANTalonSRX.h"

#include <ros.h>
#include <ros/time.h>

#include <titan_base/Status.h>
#include <titan_base/MotorVelocity.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

#include <MPU9250.h>
#include <Wire.h>



#define LEFT_MASTER_ID        1
#define LEFT_SLAVE_1_ID       2
#define LEFT_SLAVE_2_ID       3

#define RIGHT_MASTER_ID       4
#define RIGHT_SLAVE_1_ID      5
#define RIGHT_SLAVE_2_ID      6

#define TOTAL_MOTORS          6

#define TIMEOUT_MOTOR_CMD     300
#define TIMEOUT_MOTOR_STATUS  50
#define TIMEOUT_IMU           50

CANTalonSRX motor[TOTAL_MOTORS] = { 
  CANTalonSRX(LEFT_MASTER_ID),
  CANTalonSRX(LEFT_SLAVE_1_ID),
  CANTalonSRX(LEFT_SLAVE_2_ID),
  CANTalonSRX(RIGHT_MASTER_ID),
  CANTalonSRX(RIGHT_SLAVE_1_ID),
  CANTalonSRX(RIGHT_SLAVE_2_ID)
};

titan_base::Status motor_status;
titan_base::MotorVelocity vel_cmd;

sensor_msgs::Imu imuRawMsg;
sensor_msgs::MagneticField magMsg;
sensor_msgs::Temperature tempMsg;

ros::NodeHandle nh;

ros::Publisher pubMotorStatus("motor_status", &motor_status);
ros::Publisher pubIMURaw("imu/data_raw", &imuRawMsg);
ros::Publisher pubMag("imu/mag", &magMsg);
ros::Publisher pubTemp("imu/temp", &tempMsg);


float left_motor_sp = 0.0f;
float right_motor_sp = 0.0f;


byte debug = 1;
FlexCAN CANbus0;
MPU9250 IMU(Wire,0x68);

float F_gain = 1.0;//10.0 / 1024.0;
float P_gain = 1.4;
float I_gain = 0.01;
float D_gain = 20.0;

long timerMotorTimeout = millis();
long timerMotorStatus = millis();
long timerIMU = millis();

long seq = 0;

byte getIndex(byte id)
{
  return id - 1;
}

void cbMotorVelocity( const titan_base::MotorVelocity &msg)
{
  left_motor_sp = (float)msg.left_angular_vel;
  right_motor_sp = (float)msg.right_angular_vel;
  timerMotorTimeout = millis();

  float left_rev_s = left_motor_sp;
  int left_sp = 2.0 * 1024* left_rev_s / 10;

  float right_rev_s = right_motor_sp;
  int right_sp = 2.0 * 1024* right_rev_s / 10;

  motor[getIndex(LEFT_MASTER_ID)].sendMotorEnable(true);
  
  motor[getIndex(LEFT_MASTER_ID)].Set(CANTalonSRX::kMode_VelocityCloseLoop,left_sp);
  motor[getIndex(LEFT_SLAVE_1_ID)].SetDemand(CANTalonSRX::kMode_SlaveFollower, LEFT_MASTER_ID);
  motor[getIndex(LEFT_SLAVE_2_ID)].SetDemand(CANTalonSRX::kMode_SlaveFollower, LEFT_MASTER_ID);
  
  motor[getIndex(RIGHT_MASTER_ID)].Set(CANTalonSRX::kMode_VelocityCloseLoop,right_sp);
  motor[getIndex(RIGHT_SLAVE_1_ID)].SetDemand(CANTalonSRX::kMode_SlaveFollower, RIGHT_MASTER_ID);
  motor[getIndex(RIGHT_SLAVE_2_ID)].SetDemand(CANTalonSRX::kMode_SlaveFollower, RIGHT_MASTER_ID);

  
}

ros::Subscriber<titan_base::MotorVelocity> subMotorVelocity("motor_velocity", cbMotorVelocity);

void publishMotorStatus()
{
  for (byte i = 0;i < TOTAL_MOTORS;i++)
  {
    motor_status.DeviceId = i + 1;
    motor_status.StckyFault_OverTemp = motor[i].GetFaultOverTemp();
    motor_status.Fault_UnderVoltage = motor[i].GetFaultUnderVoltage();
    motor_status.Fault_ForLim = motor[i].GetFaultForLimit();
    motor_status.Fault_RevLim = motor[i].GetFaultRevLimit();
    motor_status.Fault_HardwareFailure = motor[i].GetFaultHardwarFailure();
    motor_status.Fault_ForSoftLim = motor[i].GetFaultRevSoftLimit();
    motor_status.Fault_RevSoftLim = motor[i].GetFaultForSoftLimit();
    
    motor_status.StckyFault_OverTemp = motor[i].GetStickyFaultOverTemp();;
    motor_status.StckyFault_ForLim = motor[i].GetStickyFaultUnderVoltage();
    motor_status.StckyFault_RevLim = motor[i].GetStickyFaultForLimit();
    motor_status.StckyFault_ForSoftLim = motor[i].GetStickyFaultRevSoftLimit();
    motor_status.StckyFault_RevSoftLim = motor[i].GetStickyFaultForSoftLimit();
    
    motor_status.AppliedThrottle = motor[i].GetAppliedThrottle();
    motor_status.CloseLoopErr = motor[i].GetCloseLoopErr();
    motor_status.SensorPosition = motor[i].GetSensorPos();
    motor_status.SensorVelocity = motor[i].GetSensorVel();
    motor_status.Current = motor[i].GetCurrent();

    motor_status.BrakeIsEnabled = false;
    motor_status.Temp = motor[i].GetTemp();
    motor_status.BatteryV = motor[i].GetBatteryV();
    
    motor_status.ResetCount = motor[i].GetResetCount();
    motor_status.ResetFlags = motor[i].GetResetFlags();

    pubMotorStatus.publish(&motor_status);
  }
}

void publishIMU()
{
  IMU.readSensor();
  
  char id[] = "imu_link";
  imuRawMsg.header.frame_id = id;
  imuRawMsg.header.stamp=nh.now();
  imuRawMsg.header.seq = seq;
  seq = seq + 1;

  imuRawMsg.orientation.x = 0;
  imuRawMsg.orientation.y = 0;
  imuRawMsg.orientation.z = 0;
  imuRawMsg.orientation.w = 0;  
  imuRawMsg.orientation_covariance[0] = -1;

  imuRawMsg.linear_acceleration.x = IMU.getAccelY_mss();
  imuRawMsg.linear_acceleration.y = IMU.getAccelX_mss();
  imuRawMsg.linear_acceleration.z = -IMU.getAccelZ_mss();
  imuRawMsg.linear_acceleration_covariance[0] = -1;
 
  imuRawMsg.angular_velocity.x = IMU.getGyroY_rads();
  imuRawMsg.angular_velocity.y = IMU.getGyroX_rads();
  imuRawMsg.angular_velocity.z = -IMU.getGyroZ_rads();
  imuRawMsg.angular_velocity_covariance[0] = -1;
  
  pubIMURaw.publish(&imuRawMsg);

  magMsg.header.frame_id = id;
  magMsg.header.stamp=nh.now();
  magMsg.header.seq = seq;
  seq = seq + 1;
  magMsg.magnetic_field.x = IMU.getMagY_uT() / 1000000.0;
  magMsg.magnetic_field.y = IMU.getMagX_uT() / 1000000.0;
  magMsg.magnetic_field.z = IMU.getMagZ_uT()/ 1000000.0;

  pubMag.publish(&magMsg);


  tempMsg.header.frame_id = id;
  tempMsg.header.stamp=nh.now();
  tempMsg.header.seq = seq;
  seq = seq + 1;
  tempMsg.temperature = IMU.getTemperature_C();
  
  pubTemp.publish(&tempMsg);
}


void checkTimers()
{
  if (millis() - timerMotorTimeout > TIMEOUT_MOTOR_CMD)
  {
    left_motor_sp = 0.0f;
    right_motor_sp = 0.0f;
  } 

  /*if (millis() - timerIMU > TIMEOUT_IMU)
  {
    timerIMU = millis();
    publishIMU();
  }*/

  if (millis() - timerMotorStatus > TIMEOUT_MOTOR_STATUS)
  {
   timerMotorStatus = millis();
   
   publishMotorStatus();
  }

  
}

void setupPIDF()
{
  motor[getIndex(LEFT_MASTER_ID)].SetFgain(0, F_gain);
  motor[getIndex(LEFT_SLAVE_1_ID)].SetFgain(0, F_gain);
  motor[getIndex(LEFT_SLAVE_2_ID)].SetFgain(0, F_gain);
  
  motor[getIndex(RIGHT_MASTER_ID)].SetFgain(0, F_gain);
  motor[getIndex(RIGHT_SLAVE_1_ID)].SetFgain(0, F_gain);
  motor[getIndex(RIGHT_SLAVE_2_ID)].SetFgain(0, F_gain);

  motor[getIndex(LEFT_MASTER_ID)].SetPgain(0, P_gain);
  motor[getIndex(LEFT_MASTER_ID)].SetIgain(0, I_gain);
  motor[getIndex(LEFT_MASTER_ID)].SetDgain(0, D_gain);

  motor[getIndex(RIGHT_MASTER_ID)].SetPgain(0, P_gain);
  motor[getIndex(RIGHT_MASTER_ID)].SetIgain(0, I_gain);
  motor[getIndex(RIGHT_MASTER_ID)].SetDgain(0, D_gain); 
}

void setupMotors()
{
  motor[getIndex(LEFT_MASTER_ID)].begin(&CANbus0);
  motor[getIndex(LEFT_SLAVE_1_ID)].begin(&CANbus0);
  motor[getIndex(LEFT_SLAVE_2_ID)].begin(&CANbus0);
  
  motor[getIndex(RIGHT_MASTER_ID)].begin(&CANbus0);
  motor[getIndex(RIGHT_SLAVE_1_ID)].begin(&CANbus0);
  motor[getIndex(RIGHT_SLAVE_2_ID)].begin(&CANbus0);
  
  
  motor[getIndex(RIGHT_MASTER_ID)].SetSensorPhase(true);
  
  motor[getIndex(LEFT_MASTER_ID)].SetSensorPhase(true);
  motor[getIndex(LEFT_MASTER_ID)].SetMotorInvert(true);
  motor[getIndex(LEFT_SLAVE_1_ID)].SetMotorInvert(true);
  motor[getIndex(LEFT_SLAVE_2_ID)].SetMotorInvert(true);

  
  
  motor[getIndex(LEFT_MASTER_ID)].SetParam(CANTalonSRX::eProfileParamSlot_PeakOutput, 250);
  motor[getIndex(RIGHT_MASTER_ID)].SetParam(CANTalonSRX::eProfileParamSlot_PeakOutput, 250);
}

void setupIMU()
{
    int status = IMU.begin();
    if (status < 0) {
      while(1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
      }
    }
}

void resetEncoderCounts()
{
  motor[getIndex(LEFT_MASTER_ID)].SetParam(CANTalonSRX::eSelectedSensorPosition, (int)0);
  motor[getIndex(RIGHT_MASTER_ID)].SetParam(CANTalonSRX::eSelectedSensorPosition, (int)0);
}

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  
  CANbus0 = FlexCAN(1000000,0,0,0);
  CANbus0.begin();
  
  setupMotors();
  setupPIDF();
  resetEncoderCounts();

  setupIMU();
  
  nh.initNode();
  nh.advertise(pubMotorStatus);
  nh.advertise(pubIMURaw);
  nh.advertise(pubMag);
  nh.advertise(pubTemp);
  
  nh.subscribe(subMotorVelocity);
  
  
}

void loop(void)
{
  
  if(CANbus0.available()) 
  {
    CAN_message_t msg;
    
    CANbus0.read(msg);

    for (byte i = 0;i<TOTAL_MOTORS;i++)
    {
      motor[i].update(&msg);
    }
  }

  checkTimers();
  nh.spinOnce();
}
