#ifndef CANTALONSRX_H
#define CANTALONSRX_H

#include <FlexCAN.h>
#include "ctre_frames.h"

class CANTalonSRX
{
  
    
    
    
  public:
    static const int kDefaultControlPeriodMs = 10; //!< default control update rate is 10ms.
    CANTalonSRX(uint8_t deviceNumber = 0,int controlPeriodMs = kDefaultControlPeriodMs);
    ~CANTalonSRX();
    void update(CAN_message_t *msg);
    void begin(FlexCAN *CANDev);

    
    /* mode select enumerations */
    static const int kMode_DutyCycle = 0; //!< Demand is 11bit signed duty cycle [-1023,1023].
    static const int kMode_PositionCloseLoop = 1; //!< Position PIDF.
    static const int kMode_VelocityCloseLoop = 2; //!< Velocity PIDF.
    static const int kMode_CurrentCloseLoop = 3; //!< Current close loop - not done.
    static const int kMode_VoltCompen = 4; //!< Voltage Compensation Mode - not done.  Demand is fixed pt target 8.8 volts.
    static const int kMode_SlaveFollower = 5; //!< Demand is the 6 bit Device ID of the 'master' TALON SRX.
    static const int kMode_NoDrive = 15; //!< Zero the output (honors brake/coast) regardless of demand.  Might be useful if we need to change modes but can't atomically change all the signals we want in between.
    /* limit switch enumerations */
    static const int kLimitSwitchOverride_UseDefaultsFromFlash = 1;
    static const int kLimitSwitchOverride_DisableFwd_DisableRev = 4;
    static const int kLimitSwitchOverride_DisableFwd_EnableRev = 5;
    static const int kLimitSwitchOverride_EnableFwd_DisableRev = 6;
    static const int kLimitSwitchOverride_EnableFwd_EnableRev = 7;
    /* brake override enumerations */
    static const int kBrakeOverride_UseDefaultsFromFlash = 0;
    static const int kBrakeOverride_OverrideCoast = 1;
    static const int kBrakeOverride_OverrideBrake = 2;
    /* feedback device enumerations */
    static const int kFeedbackDev_DigitalQuadEnc=0;
    static const int kFeedbackDev_AnalogPot=2;
    static const int kFeedbackDev_AnalogEncoder=3;
    static const int kFeedbackDev_CountEveryRisingEdge=4;
    static const int kFeedbackDev_CountEveryFallingEdge=5;
    static const int kFeedbackDev_PosIsPulseWidth=8;
    /* ProfileSlotSelect enumerations*/
    static const int kProfileSlotSelect_Slot0 = 0;
    static const int kProfileSlotSelect_Slot1 = 1;
    /* status frame rate types */
    static const int kStatusFrame_General = 0;
    static const int kStatusFrame_Feedback = 1;
    static const int kStatusFrame_Encoder = 2;
    static const int kStatusFrame_AnalogTempVbat = 3;
    static const int kStatusFrame_PulseWidthMeas = 4;
    
    typedef enum _param_t{
      eOnBoot_BrakeMode = 31,
        eQuadFilterEn = 91,
        eQuadIdxPolarity = 108,
        eClearPositionOnIdx = 100,
        eMotionProfileHasUnderrunErr = 119,
        eMotionProfileTrajectoryPointDurationMs = 120,
        eClearPosOnLimitF = 144,
        eClearPosOnLimitR = 145,

        eStatusFramePeriod = 300,
        eOpenloopRamp = 301,
        eClosedloopRamp = 302,
        eNeutralDeadband = 303,

        ePeakPosOutput = 305,
        eNominalPosOutput = 306,
        ePeakNegOutput = 307,
        eNominalNegOutput = 308,

        eProfileParamSlot_P = 310,
        eProfileParamSlot_I = 311,
        eProfileParamSlot_D = 312,
        eProfileParamSlot_F = 313,
        eProfileParamSlot_IZone = 314,
        eProfileParamSlot_AllowableErr = 315,
        eProfileParamSlot_MaxIAccum = 316,
        eProfileParamSlot_PeakOutput = 317,

        eClearPositionOnLimitF = 320,
        eClearPositionOnLimitR = 321,
        eClearPositionOnQuadIdx = 322,

        eSampleVelocityPeriod = 325,
        eSampleVelocityWindow = 326,

        eFeedbackSensorType = 330, // feedbackDevice_t
        eSelectedSensorPosition = 331,
        eFeedbackNotContinuous = 332,
        eRemoteSensorSource = 333, // RemoteSensorSource_t
        eRemoteSensorDeviceID = 334, // [0,62] DeviceID
        eSensorTerm = 335, // feedbackDevice_t (ordinal is the register)
        eRemoteSensorClosedLoopDisableNeutralOnLOS = 336,
        ePIDLoopPolarity = 337,
        ePIDLoopPeriod = 338,
        eSelectedSensorCoefficient = 339,

        eForwardSoftLimitThreshold = 340,
        eReverseSoftLimitThreshold = 341,
        eForwardSoftLimitEnable = 342,
        eReverseSoftLimitEnable = 343,

        eNominalBatteryVoltage = 350,
        eBatteryVoltageFilterSize = 351,

        eContinuousCurrentLimitAmps = 360,
        ePeakCurrentLimitMs = 361,
        ePeakCurrentLimitAmps = 362,

        eClosedLoopIAccum = 370,

        eCustomParam = 380,

        eStickyFaults = 390,

        eAnalogPosition = 400,
        eQuadraturePosition = 401,
        ePulseWidthPosition = 402,

        eMotMag_Accel = 410,
        eMotMag_VelCruise = 411,

        eLimitSwitchSource = 421, // ordinal (fwd=0,reverse=1), @see LimitSwitchSource_t
        eLimitSwitchNormClosedAndDis = 422, // ordinal (fwd=0,reverse=1). @see LimitSwitchNormClosedAndDis_t
        eLimitSwitchDisableNeutralOnLOS = 423,
        eLimitSwitchRemoteDevID = 424,
        eSoftLimitDisableNeutralOnLOS = 425,

        ePulseWidthPeriod_EdgesPerRot = 430,
        ePulseWidthPeriod_FilterWindowSz = 431,

        eYawOffset = 160,
        eCompassOffset = 161,
        eBetaGain = 162,
        eEnableCompassFusion = 163,
        eGyroNoMotionCal = 164,
        eEnterCalibration = 165,
        eFusedHeadingOffset = 166,
        eStatusFrameRate = 169,
        eAccumZ = 170,
        eTempCompDisable = 171,
        eMotionMeas_tap_threshX = 172,
        eMotionMeas_tap_threshY = 173,
        eMotionMeas_tap_threshZ = 174,
        eMotionMeas_tap_count = 175,
        eMotionMeas_tap_time = 176,
        eMotionMeas_tap_time_multi = 177,
        eMotionMeas_shake_reject_thresh = 178,
        eMotionMeas_shake_reject_time = 179,
        eMotionMeas_shake_reject_timeout = 180,
    }param_t;

    

    int32_t GetSensorPos();
    int32_t GetSensorVel();
    int32_t GetEncoderPos();
    int32_t GetEncoderVel();
    int32_t GetAppliedThrottle();
    int32_t GetCloseLoopErr();
    int32_t GetFeedbackDeviceSelect();
    int32_t GetModeSelect();
    double GetCurrent();
    double GetTemp();
    double GetBatteryV();
    int getSetPoint();
    
    bool GetFaultRevSoftLimit();
    bool GetFaultForSoftLimit();
    bool GetFaultHardwarFailure();
    bool GetFaultRevLimit();
    bool GetFaultForLimit();
    bool GetFaultUnderVoltage();
    bool GetFaultOverTemp();

    bool GetStickyFaultRevSoftLimit();
    bool GetStickyFaultForSoftLimit();
    bool GetStickyFaultRevLimit();
    bool GetStickyFaultForLimit();
    bool GetStickyFaultUnderVoltage();
    bool GetStickyFaultOverTemp();
    
    int16_t GetResetCount();
    int16_t GetResetFlags();
    

    

    void sendMotorEnable(bool motor_enable);
    void Set(int mode, double demand0);
    void GetParamRaw(param_t paramEnum, int32_t value, int8_t subValue, int8_t ordinal);
    void SetParam(param_t paramEnum, double value);
    void SetParamRaw(unsigned paramEnum, int rawBits, byte subValue, byte ordinal);
    void SetDemand(int mode, int demand0);
    //void SetDemand(int param);
    //void SetModeSelect(int modeSelect,int demand);
    //void SetModeSelect(int param);
    void SetProfileSlotSelect(int param);
    void SetRevFeedbackSensor(int param);
    void SetPgain(unsigned slotIdx,double gain);
    void SetIgain(unsigned slotIdx,double gain);
    void SetDgain(unsigned slotIdx,double gain);
    void SetFgain(unsigned slotIdx,double gain);
    void SetIzone(unsigned slotIdx,int zone);
    void SetStatusFramePeriod(int32_t arbId, int8_t timeMs);
    void SetSensorPhase(bool val);
    void SetMotorInvert(bool val);
    void SetNeutralMode(byte val);
    void SetCloseLoopRampRate(unsigned slotIdx,int closeLoopRampRate);
    void SetFeedbackCoeff(float coeff);
    void printParams();

    
private:
    uint8_t sensorPhase = 0;
    uint8_t outputInvert = 0;
    uint8_t neutralMode = 0;
    uint8_t deviceNumber = 0;
    int controlPeriodMs = 0;
    FlexCAN CANbus0;
    TALON_Status_1_General_10ms_t status1;
    TALON_Status_2_Feedback_20ms_t status2;
    TALON_Status_3_Enc_100ms_t status3;
    TALON_Status_4_AinTempVbat_100ms_t status4;
    TALON_Status_5_Startup_OneShot_t status5;
    TALON_Status_6_Eol_t status6;
    TALON_Status_7_Debug_200ms_t status7;
    TALON_Status_8_PulseWid_100ms_t status8;
    TALON_Status_13_CloseLoop_100ms_t status13;
    TALON_Control_1_General_10ms_t control1;
    TALON_Param_Request_t param_response;
    TALON_Demand_t demand;
    
    unsigned long timer=millis();
    
    unsigned long status1_timestamp = millis();
    unsigned long status2_timestamp = millis();
    unsigned long status3_timestamp = millis();
    unsigned long status4_timestamp = millis();
    unsigned long status5_timestamp = millis();
    unsigned long status6_timestamp = millis();
    unsigned long status7_timestamp = millis();
    unsigned long status8_timestamp = millis();
    unsigned long status13_timestamp = millis();

    int32_t status1_period = 0;
    int32_t status2_period = 0;
    int32_t status3_period = 0;
    int32_t status4_period = 0;
    int32_t status5_period = 0;
    int32_t status6_period = 0;
    int32_t status7_period = 0;
    int32_t status8_period = 0;
    int32_t status13_period = 0;
    int32_t control1_period = 0;

    unsigned long timer_control1 = millis();
    
    unsigned long param_timestamp[450];
    int32_t param[450];

    int setPoint = 0;
};

#endif
