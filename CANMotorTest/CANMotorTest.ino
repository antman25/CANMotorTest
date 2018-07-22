#include "CANTalonSRX.h"

#define LEFT_MASTER_ID    1
#define LEFT_SLAVE_1_ID   2
#define LEFT_SLAVE_2_ID   3

#define RIGHT_MASTER_ID    4
#define RIGHT_SLAVE_1_ID   5
#define RIGHT_SLAVE_2_ID   6

CANTalonSRX motorLeft(LEFT_MASTER_ID,10);
CANTalonSRX motorLeft_Slave1(LEFT_SLAVE_1_ID,10);
CANTalonSRX motorLeft_Slave2(LEFT_SLAVE_2_ID,10);

CANTalonSRX motorRight(RIGHT_MASTER_ID,10);
CANTalonSRX motorRight_Slave1(RIGHT_SLAVE_1_ID,10);
CANTalonSRX motorRight_Slave2(RIGHT_SLAVE_2_ID,10);

uint32_t prev_time = millis();
byte debug = 1;
FlexCAN CANbus0;

float F_gain = 1.0;//10.0 / 1024.0;
float P_gain = 1.4;
float I_gain = 0.01;
float D_gain = 20.0;
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
  
  //motorLeft.GetParamRaw(CANTalonSRX::eProfileParamSlot_F, 0x00,0x00,0x00);
  //motorRight.GetParamRaw(CANTalonSRX::eProfileParamSlot_F, 0x00,0x00,0x00);
  delay(3000);

  
  motorRight.SetSensorPhase(true);
  
  motorLeft.SetSensorPhase(true);
  motorLeft.SetMotorInvert(true);
  motorLeft_Slave1.SetMotorInvert(true);
  motorLeft_Slave2.SetMotorInvert(true);
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
  
  /*if(CANbus0.available()) 
  {
    uint32_t delta = millis() - prev_time;
    CANbus0.read(msg);
    Serial.print(millis());
    Serial.print(" --- ");
    Serial.print("dt: ");
    Serial.print(delta);
    Serial.print(" --- ");
    printCAN(msg);
    //Serial.print("CAN bus 0: "); hexDump(8, msg.buf);
    prev_time = millis();
  }*/
  motorLeft.sendMotorEnable(true);
  motorRight.sendMotorEnable(true);

  float left_rev_s = 0.25;
  int left_sp = 
  2.0 * 1024* left_rev_s / 10;

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

  
  //motor1.motorTest();
  
  if (millis() - prev_time > 50 && debug == 1)
  {
    int32_t right_vel = motorRight.GetSensorVel();
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
    float left_wheel_rad_s = left_rad_s / 2.0;

    Serial.print("RIGHT Sensor Pos: ");
    Serial.print(motorRight.GetSensorPos());
    Serial.print(" Sensor Vel: ");
    Serial.println(right_vel);

    Serial.print("RIGHT Wheel Vel Right: ");
    Serial.print(right_wheel_rad_s);
    Serial.print(" rad/s --- ");
    Serial.print(right_wheel_rev_s);
    Serial.println(" rev/s");

    Serial.print("RIGHT Target : ");
    Serial.println(motorRight.getSetPoint());
    Serial.print("RIGHT Throttle : ");
    Serial.println(motorRight.GetAppliedThrottle());

    Serial.print("Right Current: ");
    Serial.print(motorRight.GetCurrent());

    Serial.print(" -- ");
    Serial.print(motorRight_Slave1.GetCurrent());

    Serial.print(" -- ");
    Serial.println(motorRight_Slave2.GetCurrent());


    Serial.print("LEFT Sensor Pos: ");
    Serial.print(motorLeft.GetSensorPos());
    Serial.print(" Sensor Vel: ");
    Serial.println(left_vel);

    Serial.print("LEFT Wheel Vel LEFT: ");
    Serial.print(left_wheel_rad_s);
    Serial.print(" rad/s --- ");
    Serial.print(left_wheel_rev_s);
    Serial.println(" rev/s");

    Serial.print("LEFT Target : ");
    Serial.println(motorLeft.getSetPoint());
    Serial.print("LEFT Throttle : ");
    Serial.println(motorLeft.GetAppliedThrottle());

    Serial.print("Left Current: ");
    Serial.print(motorLeft.GetCurrent());

    Serial.print(" -- ");
    Serial.print(motorLeft_Slave1.GetCurrent());

    Serial.print(" -- ");
    Serial.println(motorLeft_Slave2.GetCurrent());
    

    motorLeft.printParams();
    Serial.println("--------------------------------------");
    prev_time = millis();
  }
  delay(1);
}
