#include "CANTalonSRX.h"
#include <ros.h>
#include <titan_base/Status.h>
#include <titan_base/MotorVelocity.h>


#define LEFT_MASTER_ID        1
#define LEFT_SLAVE_1_ID       2
#define LEFT_SLAVE_2_ID       3

#define RIGHT_MASTER_ID       4
#define RIGHT_SLAVE_1_ID      5
#define RIGHT_SLAVE_2_ID      6

#define TOTAL_MOTORS          6

#define TIMEOUT_MOTOR_CMD     300
#define TIMEOUT_MOTOR_STATUS  50

/*CANTalonSRX motorLeft(LEFT_MASTER_ID,10);
CANTalonSRX motorLeft_Slave1(LEFT_SLAVE_1_ID,10);
CANTalonSRX motorLeft_Slave2(LEFT_SLAVE_2_ID,10);

CANTalonSRX motorRight(RIGHT_MASTER_ID,10);
CANTalonSRX motorRight_Slave1(RIGHT_SLAVE_1_ID,10);
CANTalonSRX motorRight_Slave2(RIGHT_SLAVE_2_ID,10);*/

CANTalonSRX motor[6];

titan_base::Status motor_status;
titan_base::MotorVelocity vel_cmd;

ros::Publisher pubMotorStatus("motor_status", &motor_status);


float left_motor_sp = 0.0f;
float right_motor_sp = 0.0f;


byte debug = 1;
FlexCAN CANbus0;

float F_gain = 1.0;//10.0 / 1024.0;
float P_gain = 1.4;
float I_gain = 0.01;
float D_gain = 20.0;

long timerMotorTimeout = millis();
long timerMotorStatus = millis();

long seq = 0;


void cbMotorVelocity( const titan_base::MotorVelocity &msg)
{
  left_motor_sp = (float)msg.left_angular_vel;
  right_motor_sp = (float)msg.right_angular_vel;
  timerMotorTimeout = millis();
}

ros::Subscriber<titan_base::MotorVelocity> subMotorVelocity("motor_velocity", cbMotorVelocity);

void publishMotorStatus()
{
  
}

void checkTimers()
{
  if (millis() - timerMotorTimeout > TIMEOUT_MOTOR_CMD)
  {
    left_motor_sp = 0.0f;
    right_motor_sp = 0.0f;
  }

  if (millis() - timerMotorStatus > TIMEOUT_MOTOR_STATUS)
  {
   timerMotorStatus = millis();
   
   publishMotorStatus();
  }
}

void configurePID()
{
  motorLeft.SetFgain(0, F_gain);
  motorLeft_Slave1.SetFgain(0, F_gain);
  motorLeft_Slave2.SetFgain(0, F_gain);
  
  motorRight.SetFgain(0, F_gain);
  motorRight_Slave1.SetFgain(0, F_gain);
  motorRight_Slave2.SetFgain(0, F_gain);

  motorLeft.SetPgain(0, P_gain);
  motorLeft.SetIgain(0, I_gain);
  motorLeft.SetDgain(0, D_gain);

  motorRight.SetPgain(0, P_gain);
  motorRight.SetIgain(0, I_gain);
  motorRight.SetDgain(0, D_gain); 
}

void configureMotors()
{
  for (byte i=0;i<TOTAL_MOTORS;i++)
  {
    motor[i] = CANTalonSRX(i+1,10);
  }
  
  motorRight.SetSensorPhase(true);
  
  motorLeft.SetSensorPhase(true);
  
  motorLeft.SetMotorInvert(true);
  motorLeft_Slave1.SetMotorInvert(true);
  motorLeft_Slave2.SetMotorInvert(true);
}

void setup(void)
{
  CANbus0 = FlexCAN(1000000,0,0,0);
  CANbus0.begin();
  
  motorLeft.begin(&CANbus0);
  motorLeft_Slave1.begin(&CANbus0);
  motorLeft_Slave2.begin(&CANbus0);
  
  motorRight.begin(&CANbus0);
  motorRight_Slave1.begin(&CANbus0);
  motorRight_Slave2.begin(&CANbus0);
  
  motorLeft.SetParam(CANTalonSRX::eSelectedSensorPosition, 0);
  motorRight.SetParam(CANTalonSRX::eSelectedSensorPosition, 0);

  motorLeft.SetParam(CANTalonSRX::eProfileParamSlot_PeakOutput, 150);
  motorRight.SetParam(CANTalonSRX::eProfileParamSlot_PeakOutput, 150);
  
  
  configurePID();
  
  configureMotors();

  
  
}

void loop(void)
{

  if(CANbus0.available()) 
  {
    CAN_message_t msg;
    
    CANbus0.read(msg);
  
    motorLeft.update(&msg);
    motorLeft_Slave1.update(&msg);
    motorLeft_Slave2.update(&msg);
    
    motorRight.update(&msg);
    motorRight_Slave1.update(&msg);
    motorRight_Slave2.update(&msg);
  }
  
  

  float left_rev_s = 0.25;
  int left_sp = 2.0 * 1024* left_rev_s / 10;

  float right_rev_s = 0.25;
  int right_sp = 2.0 * 1024* right_rev_s / 10;

  //motorLeft.SetDemand(CANTalonSRX::kMode_DutyCycle, 0.05);
  motorLeft.Set(CANTalonSRX::kMode_VelocityCloseLoop,left_sp);
  motorLeft_Slave1.SetDemand(CANTalonSRX::kMode_SlaveFollower, LEFT_MASTER_ID);
  motorLeft_Slave2.SetDemand(CANTalonSRX::kMode_SlaveFollower, LEFT_MASTER_ID);
  
  motorRight.Set(CANTalonSRX::kMode_VelocityCloseLoop,right_sp);
  //motorRight.SetDemand(CANTalonSRX::kMode_DutyCycle, 0.05);
  motorRight_Slave1.SetDemand(CANTalonSRX::kMode_SlaveFollower, RIGHT_MASTER_ID);
  motorRight_Slave2.SetDemand(CANTalonSRX::kMode_SlaveFollower, RIGHT_MASTER_ID);


    /*int32_t right_vel = motorRight.GetSensorVel();
    int32_t left_vel = motorLeft.GetSensorVel();
    // counts / s
    // 100 ms
    
    float right_rev_s = (right_vel * 10.0) / 1024.0;
    float right_rad_s = right_rev_s * 3.1415 * 2;
    float right_wheel_rev_s = right_rev_s / 2.0;
    float right_wheel_rad_s = right_rad_s / 2.0;

    float left_rev_s = (left_vel * 10.0) / 1024.0;
    float left_rad_s = left_rev_s * 3.1415 * 2;
    float left_wheel_rev_s = left_rev_s / 2.0;
    float left_wheel_rad_s = left_rad_s / 2.0;*/

  checkTimers();
  //nh.spinOnce();
}
