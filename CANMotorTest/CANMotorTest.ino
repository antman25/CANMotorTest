#include "CANTalonSRX.h"

#include <ros.h>
#include <ros/time.h>

#include <titan_msgs/Status.h>
#include <titan_msgs/MotorVelocity.h>
#include <titan_msgs/PIDF.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <MPU9250.h>
#include <Wire.h>



#define STATUS_1        0x00001400
#define STATUS_2        0x00001440
#define STATUS_13       0x00001700
#define CONTROL_3       0x00040080

#define LEFT_MASTER_ID        1
#define LEFT_SLAVE_1_ID       2
#define LEFT_SLAVE_2_ID       3

#define RIGHT_MASTER_ID       4
#define RIGHT_SLAVE_1_ID      5
#define RIGHT_SLAVE_2_ID      6

#define TOTAL_MOTORS          6

#define TIMEOUT_MOTOR_CMD     200
#define TIMEOUT_MOTOR_STATUS  40
#define TIMEOUT_IMU           50
#define TIMEOUT_MOTOR_UPDATE  30
#define TIMEOUT_DEBUG         200

#define GEAR_RATIO            2.0
#define TICKS_PER_REV         1024.0

//#define NEUTRAL_MODE        CANTalonSRX::kBrakeOverride_OverrideBrake
#define NEUTRAL_MODE          CANTalonSRX::kBrakeOverride_OverrideCoast

#define MAX_THROTTLE          512.0

#define VELOCITY_PERIOD       20
#define VELOCITY_NUM_SAMPLE   16


#define STATUS_1_PERIOD       10
#define STATUS_2_PERIOD       20
#define STATUS_13_PERIOD      10
#define CONTROL_3_PERIOD      200

#define PIN_PUMP              2


CANTalonSRX motor[TOTAL_MOTORS] = {
  CANTalonSRX(LEFT_MASTER_ID),
  CANTalonSRX(LEFT_SLAVE_1_ID),
  CANTalonSRX(LEFT_SLAVE_2_ID),
  CANTalonSRX(RIGHT_MASTER_ID),
  CANTalonSRX(RIGHT_SLAVE_1_ID),
  CANTalonSRX(RIGHT_SLAVE_2_ID)
};

titan_msgs::Status motor_status;
titan_msgs::MotorVelocity vel_cmd;

sensor_msgs::Imu imuRawMsg;
sensor_msgs::MagneticField magMsg;
sensor_msgs::Temperature tempMsg;

std_msgs::String debug;

ros::NodeHandle nh;

ros::Publisher pubMotorStatus("motor_status", &motor_status);
ros::Publisher pubIMURaw("imu/data_raw", &imuRawMsg);
ros::Publisher pubMag("imu/mag", &magMsg);
ros::Publisher pubTemp("imu/temp", &tempMsg);
ros::Publisher pubDebug("debug", &debug);

float left_motor_sp = 0.0f;
float right_motor_sp = 0.0f;
bool valid_sp = false;


FlexCAN CANbus0;
MPU9250 IMU(Wire, 0x68);


float P_gain = 3.0f;//3.0F;
float I_gain = 0.0F;
float D_gain = 0.0F;
float F_gain = 3.0f;
float FeedbackCoeff = 1000.0f / (GEAR_RATIO * TICKS_PER_REV);
int CloseLoopRampRate = 0;

long timerMotorTimeout = millis();
long timerMotorStatus = millis();
long timerIMU = millis();
long timerMotorUpdate = millis();
long timerDebug = millis();

uint32_t timerTestStart = millis();
uint32_t timerTestEnd = millis();

uint32_t count = 0;
long seq = 0;

void setupPIDF();

byte getIndex(byte id)
{
  return id - 1;
}

void updateMotors()
{
  //int left_sp = (left_motor_sp * (1.0/(2.0*PI)) * TICKS_PER_REV * GEAR_RATIO) / 10.0;
  //int right_sp = (right_motor_sp * (1.0/(2.0*PI)) * TICKS_PER_REV * GEAR_RATIO) / 10.0;

  float left_sp = left_motor_sp * (1.0 / (2.0 * PI)) * (1000.0 / 10.0);
  float right_sp = right_motor_sp * (1.0 / (2.0 * PI)) * (1000.0 / 10.0);

  motor[getIndex(LEFT_MASTER_ID)].sendMotorEnable(valid_sp);


  motor[getIndex(LEFT_MASTER_ID)].Set(CANTalonSRX::kMode_VelocityCloseLoop, left_sp);
  //motor[getIndex(LEFT_MASTER_ID)].Set(CANTalonSRX::kMode_DutyCycle,left_motor_sp);
  motor[getIndex(LEFT_SLAVE_1_ID)].SetDemand(CANTalonSRX::kMode_SlaveFollower, LEFT_MASTER_ID);
  motor[getIndex(LEFT_SLAVE_2_ID)].SetDemand(CANTalonSRX::kMode_SlaveFollower, LEFT_MASTER_ID);


  motor[getIndex(RIGHT_MASTER_ID)].Set(CANTalonSRX::kMode_VelocityCloseLoop, right_sp);
  //motor[getIndex(RIGHT_MASTER_ID)].Set(CANTalonSRX::kMode_DutyCycle,right_motor_sp);
  motor[getIndex(RIGHT_SLAVE_1_ID)].SetDemand(CANTalonSRX::kMode_SlaveFollower, RIGHT_MASTER_ID);
  motor[getIndex(RIGHT_SLAVE_2_ID)].SetDemand(CANTalonSRX::kMode_SlaveFollower, RIGHT_MASTER_ID);


  /*char output[120];

    sprintf(output,"Left SP: %f rad/s -- %f ticks/100ms -- Right SP: %f rad/s -- %f ticks/100ms",left_motor_sp,left_sp,right_motor_sp,right_sp);
    debug.data = output;
    pubDebug.publish( &debug );*/
}

void cbMotorVelocity( const titan_msgs::MotorVelocity &msg)
{
  //digitalWrite(LED_BUILTIN, HIGH);
  left_motor_sp = (float)msg.left_angular_vel;
  right_motor_sp = (float)msg.right_angular_vel;
  valid_sp = true;
  timerMotorTimeout = millis();
  //digitalWrite(LED_BUILTIN, LOW);
}

void cbSetPIDFParam ( const titan_msgs::PIDF &msg)
{
  P_gain = msg.P_Gain;
  I_gain = msg.I_Gain;
  D_gain = msg.D_Gain;
  F_gain = msg.F_Gain;
  FeedbackCoeff = msg.FeedbackCoeff;
  CloseLoopRampRate = msg.RampRate;

  setupPIDF();

}

void cbSetPump ( const std_msgs::Float32 &msg)
{
  if (msg.data > 0)
  {
    digitalWrite(PIN_PUMP, HIGH);
  }
  else
  {
    digitalWrite(PIN_PUMP, LOW);
  }
}


ros::Subscriber<titan_msgs::MotorVelocity> subMotorVelocity("motor_velocity", cbMotorVelocity);
ros::Subscriber<titan_msgs::PIDF> subSetPIDFParam("set_pidf_param", cbSetPIDFParam);
ros::Subscriber<std_msgs::Float32> subSetPump("set_pump", cbSetPump);

void publishDebug()
{
  char output[120];

  sprintf(output, "Long cycles: %i ", (int)count);
  debug.data = output;
  pubDebug.publish( &debug );

  /*sprintf(output,"Left pos: %i  -- Right pos: %i",(int)motor[getIndex(LEFT_MASTER_ID)].GetSensorPos(),(int)motor[getIndex(RIGHT_MASTER_ID)].GetSensorPos());
    debug.data = output;
    pubDebug.publish( &debug );

    sprintf(output,"Left Vel: %i  -- Right Vel: %i",(int)motor[getIndex(LEFT_MASTER_ID)].GetSensorVel(),(int)motor[getIndex(RIGHT_MASTER_ID)].GetSensorVel());
    debug.data = output;
    pubDebug.publish( &debug );

    float left_sp = left_motor_sp * (1000.0/(2.0 * PI * 10.0));
    float right_sp = right_motor_sp * (1000.0/(2.0 * PI * 10.0));
    sprintf(output,"Left SP: %f rad/s -- %f ticks/100ms -- Right SP: %f rad/s -- %f ticks/100ms",left_motor_sp,left_sp,right_motor_sp,right_sp);
    debug.data = output;
    pubDebug.publish( &debug );*/
}

void publishMotorPIDF()
{
  for (int8_t i = 0; i < TOTAL_MOTORS; i++)
  {
    //float p =
  }
}

void publishMotorStatus()
{

  for (byte i = 0; i < TOTAL_MOTORS; i++)
  {
    //if (i != 0 && i != 3)
    //  continue;
    char id[] = "";
    motor_status.header.frame_id = id;
    motor_status.header.stamp = nh.now();
    motor_status.header.seq = seq;
    seq = seq + 1;

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
  imuRawMsg.header.stamp = nh.now();
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
  magMsg.header.stamp = nh.now();
  magMsg.header.seq = seq;
  seq = seq + 1;
  magMsg.magnetic_field.x = IMU.getMagY_uT() / 1000000.0;
  magMsg.magnetic_field.y = IMU.getMagX_uT() / 1000000.0;
  magMsg.magnetic_field.z = IMU.getMagZ_uT() / 1000000.0;

  pubMag.publish(&magMsg);


  tempMsg.header.frame_id = id;
  tempMsg.header.stamp = nh.now();
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
    valid_sp = false;
    //digitalWrite(LED_BUILTIN, HIGH);
  }

  if (millis() - timerMotorUpdate > TIMEOUT_MOTOR_UPDATE)
  {
    timerMotorUpdate = millis(); 
    updateMotors();

  }

  if (millis() - timerIMU > TIMEOUT_IMU)
  {
    timerIMU = millis();
    //publishIMU();
  }

  if (millis() - timerMotorStatus > TIMEOUT_MOTOR_STATUS)
  {
    timerMotorStatus = millis();
    publishMotorStatus();
  }

  if (millis() - timerDebug > TIMEOUT_DEBUG)
  {
    timerDebug = millis();
    publishDebug();
  }
}

void setupPIDF()
{
  for (int8_t i = 0; i < TOTAL_MOTORS; i++)
  {
    motor[i].SetPgain(0, P_gain);
    motor[i].SetIgain(0, I_gain);
    motor[i].SetDgain(0, D_gain);
    motor[i].SetFgain(0, F_gain);
    motor[i].SetFeedbackCoeff(FeedbackCoeff);
    motor[i].SetCloseLoopRampRate(0, CloseLoopRampRate);
  }
}

void setupPhase()
{
  motor[getIndex(LEFT_MASTER_ID)].SetSensorPhase(true);

  motor[getIndex(LEFT_MASTER_ID)].SetMotorInvert(true);
  motor[getIndex(LEFT_SLAVE_1_ID)].SetMotorInvert(true);
  motor[getIndex(LEFT_SLAVE_2_ID)].SetMotorInvert(true);

  motor[getIndex(RIGHT_MASTER_ID)].SetSensorPhase(true);
}

void setupNeutralMode()
{
  motor[getIndex(LEFT_MASTER_ID)].SetNeutralMode(NEUTRAL_MODE);
  motor[getIndex(LEFT_SLAVE_1_ID)].SetNeutralMode(NEUTRAL_MODE);
  motor[getIndex(LEFT_SLAVE_2_ID)].SetNeutralMode(NEUTRAL_MODE);

  motor[getIndex(RIGHT_MASTER_ID)].SetNeutralMode(NEUTRAL_MODE);
  motor[getIndex(RIGHT_SLAVE_1_ID)].SetNeutralMode(NEUTRAL_MODE);
  motor[getIndex(RIGHT_SLAVE_2_ID)].SetNeutralMode(NEUTRAL_MODE);
}

void setupThrottleLimit()
{
  motor[getIndex(LEFT_MASTER_ID)].SetParam(CANTalonSRX::ePeakPosOutput, MAX_THROTTLE);
  motor[getIndex(LEFT_MASTER_ID)].SetParam(CANTalonSRX::ePeakNegOutput, -MAX_THROTTLE);

  motor[getIndex(RIGHT_MASTER_ID)].SetParam(CANTalonSRX::ePeakPosOutput, MAX_THROTTLE);
  motor[getIndex(RIGHT_MASTER_ID)].SetParam(CANTalonSRX::ePeakNegOutput, -MAX_THROTTLE);

  motor[getIndex(LEFT_MASTER_ID)].SetParam(CANTalonSRX::eProfileParamSlot_PeakOutput, MAX_THROTTLE);
  motor[getIndex(RIGHT_MASTER_ID)].SetParam(CANTalonSRX::eProfileParamSlot_PeakOutput, MAX_THROTTLE);
}

void resetEncoderCounts()
{
  motor[getIndex(LEFT_MASTER_ID)].SetParam(CANTalonSRX::eSelectedSensorPosition, 0);
  motor[getIndex(RIGHT_MASTER_ID)].SetParam(CANTalonSRX::eSelectedSensorPosition, 0);
}

void setupVelocityCalc()
{
  motor[getIndex(LEFT_MASTER_ID)].SetParam(CANTalonSRX::eSampleVelocityPeriod, VELOCITY_PERIOD);
  motor[getIndex(RIGHT_MASTER_ID)].SetParam(CANTalonSRX::eSampleVelocityPeriod, VELOCITY_PERIOD);

  motor[getIndex(LEFT_MASTER_ID)].SetParam(CANTalonSRX::eSampleVelocityWindow, VELOCITY_NUM_SAMPLE);
  motor[getIndex(RIGHT_MASTER_ID)].SetParam(CANTalonSRX::eSampleVelocityWindow, VELOCITY_NUM_SAMPLE);
}

void setupMessageRates()
{

  //motor[getIndex(LEFT_MASTER_ID)].SetStatusFramePeriod(STATUS_1,STATUS_1_PERIOD);
  //motor[getIndex(RIGHT_MASTER_ID)].SetStatusFramePeriod(STATUS_1,STATUS_1_PERIOD);

  //motor[getIndex(LEFT_MASTER_ID)].SetStatusFramePeriod(STATUS_2, STATUS_2_PERIOD);
  //motor[getIndex(RIGHT_MASTER_ID)].SetStatusFramePeriod(STATUS_2, STATUS_2_PERIOD);

  //motor[getIndex(LEFT_MASTER_ID)].SetStatusFramePeriod(STATUS_13,STATUS_13_PERIOD);
  //motor[getIndex(RIGHT_MASTER_ID)].SetStatusFramePeriod(STATUS_13,STATUS_13_PERIOD);

  //motor[getIndex(LEFT_MASTER_ID)].SetStatusFramePeriod(CONTROL_3,CONTROL_3_PERIOD);
  //motor[getIndex(RIGHT_MASTER_ID)].SetStatusFramePeriod(CONTROL_3,CONTROL_3_PERIOD);
}

void setupMotors()
{
  motor[getIndex(LEFT_MASTER_ID)].begin(&CANbus0);
  motor[getIndex(LEFT_SLAVE_1_ID)].begin(&CANbus0);
  motor[getIndex(LEFT_SLAVE_2_ID)].begin(&CANbus0);

  motor[getIndex(RIGHT_MASTER_ID)].begin(&CANbus0);
  motor[getIndex(RIGHT_SLAVE_1_ID)].begin(&CANbus0);
  motor[getIndex(RIGHT_SLAVE_2_ID)].begin(&CANbus0);



  setupPhase();
  setupNeutralMode();
  setupThrottleLimit();
  setupVelocityCalc();
  //setupMessageRates();
  resetEncoderCounts();


}

void setupIMU()
{
  int status = IMU.begin();
  if (status < 0) {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }
}



void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);

  CANbus0 = FlexCAN(1000000, 0, 0, 0);
  CANbus0.begin();

  setupMotors();
  setupPIDF();


  //setupIMU();

  nh.initNode();
  nh.advertise(pubMotorStatus);
  nh.advertise(pubIMURaw);
  nh.advertise(pubMag);
  nh.advertise(pubTemp);
  nh.advertise(pubDebug);

  nh.subscribe(subMotorVelocity);
  nh.subscribe(subSetPIDFParam);
  nh.subscribe(subSetPump);

  //Serial.begin(115200);

}

void loop(void)
{
  timerTestStart = millis();
  nh.spinOnce();
  if (CANbus0.available())
  {
    CAN_message_t msg;

    CANbus0.read(msg);

    for (byte i = 0; i < TOTAL_MOTORS; i++)
    {
      motor[i].update(&msg);
    }
  }
  //delay(5);
  checkTimers();
  timerTestEnd = millis() - timerTestStart;
  if (timerTestEnd > 40)
  {
    count++;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
