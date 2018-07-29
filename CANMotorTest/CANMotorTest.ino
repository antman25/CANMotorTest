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

CANTalonSRX motor[6];

titan_base::Status motor_status;
titan_base::MotorVelocity vel_cmd;

ros::NodeHandle nh;

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

  float left_rev_s = left_motor_sp;
  int left_sp = 2.0 * 1024* left_rev_s / 10;

  float right_rev_s = right_motor_sp;
  int right_sp = 2.0 * 1024* right_rev_s / 10;


  motor[LEFT_MASTER_ID].Set(CANTalonSRX::kMode_VelocityCloseLoop,left_sp);
  motor[LEFT_SLAVE_1_ID].SetDemand(CANTalonSRX::kMode_SlaveFollower, LEFT_MASTER_ID);
  motor[LEFT_SLAVE_2_ID].SetDemand(CANTalonSRX::kMode_SlaveFollower, LEFT_MASTER_ID);
  
  motor[RIGHT_MASTER_ID].Set(CANTalonSRX::kMode_VelocityCloseLoop,right_sp);
  motor[RIGHT_SLAVE_1_ID].SetDemand(CANTalonSRX::kMode_SlaveFollower, RIGHT_MASTER_ID);
  motor[RIGHT_SLAVE_2_ID].SetDemand(CANTalonSRX::kMode_SlaveFollower, RIGHT_MASTER_ID);
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
  motor[LEFT_MASTER_ID].SetFgain(0, F_gain);
  motor[LEFT_SLAVE_1_ID].SetFgain(0, F_gain);
  motor[LEFT_SLAVE_2_ID].SetFgain(0, F_gain);
  
  motor[RIGHT_MASTER_ID].SetFgain(0, F_gain);
  motor[RIGHT_SLAVE_1_ID].SetFgain(0, F_gain);
  motor[RIGHT_SLAVE_2_ID].SetFgain(0, F_gain);

  motor[LEFT_MASTER_ID].SetPgain(0, P_gain);
  motor[LEFT_MASTER_ID].SetIgain(0, I_gain);
  motor[LEFT_MASTER_ID].SetDgain(0, D_gain);

  motor[RIGHT_MASTER_ID].SetPgain(0, P_gain);
  motor[RIGHT_MASTER_ID].SetIgain(0, I_gain);
  motor[RIGHT_MASTER_ID].SetDgain(0, D_gain); 
}

void configureMotors()
{
  for (byte i=0;i<TOTAL_MOTORS;i++)
  {
    motor[i] = CANTalonSRX(i+1,10);
  }

  motor[LEFT_MASTER_ID].begin(&CANbus0);
  motor[LEFT_SLAVE_1_ID].begin(&CANbus0);
  motor[LEFT_SLAVE_2_ID].begin(&CANbus0);
  
  motor[RIGHT_MASTER_ID].begin(&CANbus0);
  motor[RIGHT_SLAVE_1_ID].begin(&CANbus0);
  motor[RIGHT_SLAVE_2_ID].begin(&CANbus0);
  
  
  motor[RIGHT_MASTER_ID].SetSensorPhase(true);
  
  motor[LEFT_MASTER_ID].SetSensorPhase(true);
  motor[LEFT_MASTER_ID].SetMotorInvert(true);
  motor[LEFT_SLAVE_1_ID].SetMotorInvert(true);
  motor[LEFT_SLAVE_2_ID].SetMotorInvert(true);

  

  motor[LEFT_MASTER_ID].SetParam(CANTalonSRX::eProfileParamSlot_PeakOutput, 150);
  motor[RIGHT_MASTER_ID].SetParam(CANTalonSRX::eProfileParamSlot_PeakOutput, 150);
}

void resetEncoderCounts()
{
  motor[LEFT_MASTER_ID].SetParam(CANTalonSRX::eSelectedSensorPosition, 0);
  motor[RIGHT_MASTER_ID].SetParam(CANTalonSRX::eSelectedSensorPosition, 0);
}

void setup(void)
{
  CANbus0 = FlexCAN(1000000,0,0,0);
  CANbus0.begin();

  configureMotors();
  configurePID();

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
