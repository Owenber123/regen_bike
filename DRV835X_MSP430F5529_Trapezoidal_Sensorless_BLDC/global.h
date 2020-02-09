/* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**************************************************************************
 * @file        global.h
 * @author      MDBU Software Team
 * @brief       SPI API file for SPI Module.
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 * @date		May 2016
 ******************************************************************************/
#ifndef global_H_
#define global_H_

#include "SPI_API.h"
#include "Sensorless_Trap_Parameters_Setup.h"
#include "Init.h"
#include "mdbu_global.h"
#include "dataTypeDefinition.h"
#include "msp430.h"
#include "mdbuserial.h"
#include "mdbuserial_protocol.h"
#include "mdbuserial.h"
#include "usb.h"
#include "IQmathLib.h"
#include "QmathLib.h"
#include "USB_config/descriptors.h"
#include "USB_app/usbConstructs.h"
#include "mdbuserial.h"
#define drv83xx_REGISTER_WIDTH (2) // Defines the No of bytes used in 1 SPI trasaction , because we use 16 bit SPI register width is 2
/*
 * The system parameters are listed below
 */
#define PWM_PERIOD 					(25000 / PWM_FREQUENCY)    //PWM Period time = 25Mhz/[this number =  100uS - 10kHz
#define DRV_COUNT_PER_VOLTS     (4095.0F / FULL_SCALE_VOLTAGE)
#define COUNTER_1_MSECOND                       0x19    //25*40.96us = 1ms
#define COUNTER_10_MSECONDS                     244     //250 * 40.96us = 10ms
#define COUNTER_100_MSECONDS            		2442    //2500 * 40.96us = 100ms
#define COUNTER_200_MSECONDS            		4883    //5000 * 40.96us = 200ms
#define COUNTER_1_SECOND                        24414   //24414 * 40.96us = 2s
#define COUNTER_2_SECONDS                       48828   //50000 * 40.96us = 2s
#define ACCEL_30_DEGREES                        10417
#define ACCEL_60_DEGREES                        20834
#define NUMCONFIG_IDRIVE_VALUES                 16       // Defines the number of configurable Idrive values for the DRv83xxS Devices
#define NUMCONFIG_TDRIVE_VALUES                 4       // Defines the number of configurable Idrive values for the DRv83xxS Devices
// Motor Parameters
#define MTR_PARAM_ELEC_SPEED 0
#define MTR_PARAM_FAULT_STATE 1
#define MTR_PARAM_DEVICE_ID 2
#define MTR_PARAM_ISC_MIN_BEMF 3
#define MTR_PARAM_ISC_BRAKE_TIME 4
#define MTR_PARAM_IPD_BRAKE_TIME 5
#define MTR_PARAM_IPD_PULSE_TIME 6
#define MTR_PARAM_IPD_DECAY_CONSTANT 7
#define MTR_PARAM_ALIGN_SECTOR 8
#define MTR_PARAM_ALIGN_WAIT_TIME 9
#define MTR_PARAM_ACCEL_RATE 10
#define MTR_PARAM_ACCEL_STOP 11
#define MTR_PARAM_ACCEL_VEL_INIT 12
#define MTR_PARAM_BEMF_THRESHOLD 13
#define MTR_PARAM_RAMP_RATE_DELAY 14
#define MTR_PARAM_DIR_REV_DELAY 15
#define MTR_PARAM_COMM_BLANK_TIME 16
#define MTR_PARAM_PWM_BLANK_COUNTS 17
#define MTR_PARAM_MAX_DUTY_CYCLE 18
#define MTR_PARAM_MIN_OFF_DUTY_CYCLE 19
#define MTR_PARAM_MIN_ON_DUTY_CYCLE 20
#define MTR_PARAM_PWM_FREQ 22
#define MTR_PARAM_UNDER_VOL_LIM 23
#define MTR_PARAM_OVER_VOL_LIM 24
#define MTR_PARAM_STALL_DETECT_REV 25
#define MTR_PARAM_STALL_DETECT_TIME 26
#define MTR_PARAM_MOTOR_PHASE_CURR_LIM 27
#define MTR_PARAM_AUTO_FAULT_RECOVERY_TIME 28
#define MTR_PARAM_ALIGN_IPD 29
#define MTR_PARAM_DIR  30
#define MTR_PARAM_SPEED 31
#define MTR_START_STOP_MOTOR 32
#define MTR_START_BEMF_CALCULATION 39
#define MTR_UNDER_RATING 40
#define MTR_RATED_VOLTAGE 41
#define MTR_RATED_SPEED 42
#define MTR_POLES 43
#define DEVICE_FULL_SCALE_VOLTAGE 44

#define DRV835X_DEVICE							1      // Device variant used for DRV835x
#define DRV832X_DEVICE							0      // Device variant used for DRV832x
#define DRV83XX_DEVICE							2      // Device variant used for DRV832x

void EnableGateDrivers(void);
void DisableGateDrivers(void);
void sensorlessTrapController_Init(void);
void SetPWMDutyCycle(uint16_t PWMDutyCycle);
void PWM_SetCommutation(uint8_t commState);
void IPD_SetState(uint8_t commState);
void UpdateNextCommutation(void);
void SetMotorSpeed(void);
void UpdateBEMFADC(void);
void FastReadBEMF(void);
void ReadVCC(void);
void ReadCurrentShunt(void);
void ISCReadPhaseVoltage(void);
void HostControllerInit(void);
void ReadSPDFDBK(void);
void BrakeMotor(void);
inline void CommutationStateDetect();
inline void calculateBEMFThreshold();
inline void CatchInitialSpeed();
inline void CalDutyPerSpeed();
/* Host Controller Specifics*/

// GPIO Port 1 Definitions
#define EN_DRV	    BIT6    // P1.6
// GPIO Port 2 Definitions
#define nFAULT	    BIT7    // P2.7
void drv83xx_regRestoreFromCache();
void drv83xx_setGPIO(unsigned char gpioPort, unsigned char gpioNum, unsigned char gpioVal);
unsigned char drv83xx_getGPIO(unsigned char gpioPort, unsigned char gpioNum);
void drv83xx_setMtrParam(unsigned char num, unsigned long value);
unsigned long drv83xx_getMtrParam(unsigned char num);
void drv83xx_StartMotor();
void drv83xx_StopMotor();
unsigned int drv83xx_registerRead(unsigned char address);
void drv83xx_registerWrite(unsigned char address, unsigned int value);
void drv83xx_setCtrlTypeParam(unsigned char value);
unsigned char drv83xx_getCtrlTypeParam(void);
typedef enum
{
    SYSTEM_INIT = 0,
	SYSTEM_IDLE =1,
    MOTOR_IDLE = 2,
    MOTOR_ISC = 3,
    MOTOR_ALIGN = 4,
    MOTOR_IPD = 5,
    MOTOR_START = 6,
    MOTOR_RUN = 7,
    MOTOR_STOP = 8,
    FAULT = 9
}STATE_MACHINE;

typedef enum
{
    HOST_EN = 0,
    HOST_IDLE = 1,
    HOST_ACTIVE = 2
} HOSTCONTROL_STATUS;

typedef enum
{
    START = 0,
    TIMER_INTERRUPT = 1,
    ADC_READ = 2,
    BRAKE = 3,
    DONE = 4
}IPD_STATE;

typedef enum
{
    READ_BEMF = 0,
    RUN_IPD = 1
}ISC_STATE;

typedef enum
{
    NOFAULT = 0,
	VOLTAGE = 1,
	OVERCURRENT = 2,
	OVERTEMPERATURE = 3,
    MOTOR_STALL = 4,
    GATE_DRIVER = 5,
	POWER_SUPPLY = 6,
    UNKNOWN = 7,
	ACCELERATION = 8

}FAULTS;

extern uint16_t Register_Counter;
extern uint16_t DRV835xS_IdriveP_RegData[NUMCONFIG_IDRIVE_VALUES];
extern uint16_t DRV835xS_IdriveN_RegData[NUMCONFIG_IDRIVE_VALUES];
extern uint16_t DRV832xS_IdriveP_RegData[NUMCONFIG_IDRIVE_VALUES];
extern uint16_t DRV832xS_IdriveN_RegData[NUMCONFIG_IDRIVE_VALUES];
extern uint16_t DRV835xS_Tdrive_RegData[NUMCONFIG_TDRIVE_VALUES];
extern uint16_t DRV832xS_Tdrive_RegData[NUMCONFIG_TDRIVE_VALUES];
typedef struct APPLICATION_STATUS
{
    STATE_MACHINE currentstate;
    STATE_MACHINE previousstate;
    FAULTS fault;
} APPLICATION_STATUS;

// Host Controller
typedef struct HOST_CONTROLLER_Obj
{
	uint8_t EnabledGateDrivers;
	uint8_t Start_Stop_Motor;        //If Motor is in Stop_Mode = 1, If Motor is in Start_mode = 0
	uint8_t Calibrate_Motor;
} HOST_CONTROLLER_Obj;

typedef struct SENSORLESS_TRAP_Obj
{
    //Initial Speed Control Variables
	IQMATH32  IQMATH_TEMP_A;
	IQMATH32  IQMATH_TEMP_B;
	IQMATH32  IQMATH_TEMP_C;

	QMATH16  QMATH_TEMP_A;
	QMATH16  QMATH_TEMP_B;
	QMATH16  QMATH_TEMP_C;
    uint16_t LineAB_BEMF;
    uint16_t LineBC_BEMF;
    uint16_t LineCA_BEMF;
    uint16_t PhaseA_BEMF;
    uint16_t Phase_Min_Diff;
    uint16_t ISC_BEMF_Clamped;
    uint16_t ISC_Current_BEMF;
    uint16_t ISC_Prev_BEMF;
    uint16_t ISC_BEMF_Diff[8];
    uint16_t ISC_Motor_Speed[8];
    uint16_t ISC_Threshold_Speed_Count;
    uint16_t ISC_Threshold_MAX_Speed_Count;
    uint16_t ISC_Current_State;
    uint16_t ISC_Next_State;
    uint16_t ISC_Comm_Match_State;
    uint16_t ISC_Prev_State;
    uint16_t  ISC_Phase_Match;
    BOOL    ISC_Phase_Match_Flag;
    uint8_t ISC_Counter;
    uint16_t PhaseB_BEMF;
    uint16_t PhaseC_BEMF;
    uint16_t BEMF_Counter;

    ISC_STATE ISCStatus;

    //IPD Variables
    uint16_t IPDCoastTime;
    uint16_t IPDCount;
    uint16_t IPDCurrent;
    BOOL IPDDone;
    uint16_t IPDMaxCRV;
    SINT32 IPDCurrentRiseValue[7];
    BOOL IPDStart;
    uint16_t IPDState;
    IPD_STATE IPDStatus;

    //Align Variables
    BOOL AlignComplete;
    uint16_t AlignWaitCounter;
    BOOL StartAlign;

    //Open Loop Acceleration Variables
    uint16_t AccelCounter;
    uint16_t AccelDistance;
    BOOL AccelDone;
    uint32_t AccelVelocityInit;    /* This variable holds the initial value of open loop acceleration */
    uint16_t Counter_1M_Second;    /* This variable holds the counter value for 1 milli second based on PWM frequency */
    uint16_t Counter_1_Second;    /* This variable holds the counter value for 1  second based on PWM frequency */



    //Closed Loop Variables
    BOOL ADCchange;
    uint16_t ADCcnt;
    uint16_t ADCdelay;
    BOOL ADCready;
    BOOL ADCswitch;
    uint16_t BEMFtrigger;
    uint8_t CommStateDetect;
    uint16_t CTvoltage;
    uint16_t GetBEMF;
    BOOL SpeedChange;
    uint16_t SpeedDivider;
    uint16_t SumBEMF;
    BOOL Calibrate_Motor_run;
    BOOL Calibrate_Motor_Start;
    BOOL Calibrate_Motor_Direction;
    uint16_t Motor_Under_Rating;
    uint16_t Motor_Electrical_Speed;
    uint16_t Motor_Poles;
    uint16_t Motor_Rated_Speed;
    uint16_t Motor_Rated_Voltage;
    float_t Hertz_Per_Volt_Counts;
    float_t Hertz_Per_Unit_Volt_Counts;
    float_t Duty_Per_Unit_Speed;
    float_t Duty_Per_Speed;
    // ISC Variables

    //System Variables
    uint16_t IdriveP_Min_Value;
    uint16_t IdriveN_Min_Value;
    uint16_t Tdrive_Max_Value;
    uint16_t IdriveP_Setting;
    uint16_t IdriveN_Setting;
    uint16_t Tdrive_Setting;
    BOOL GateToggleFaultDetect;      // For DRV83xxH devices Upon Enabling gate drivers, reporting of faults is disabled for first 5 ms, reporting is enabled after 10ms
    uint16_t GateToggleCounter;          // This counter is used to count for first 5 ms after changing gate drive status to toggle Gate drive fault detect
    uint8_t CurrentCommState;
    uint16_t CurrentDutyCycle;
    BOOL Direction;
    BOOL Direction_flag;
    uint16_t faultreg;
    uint16_t OClimit;
    uint16_t SystemCount;
    uint16_t TargetDutyCycle;
    uint16_t VCCvoltage;
    uint16_t deviceIDADC;
    uint16_t MotorPhaseCurrent;
    uint16_t RestartDelay;
    uint16_t RestartDelayCounter;
    uint16_t RotationCount;
    uint16_t StallDetectCounter;
    uint16_t StallDetectDelay;
    BOOL TimerOverflowFlag;  /* Interrupt counter to ensure the speed measurement with 18bit resolution*/
    BOOL AccelSpeedLowFlag;  /* Interrupt counter to ensure the speed measurement with 18bit resolution*/
    uint16_t CaibBEMFThreshold[2];
    //MDBU Serial Variables
    uint16_t AccelSPDFdbk;  /* This array holds the  Motor Spin frequency in timer counts for each commutation state with 16bit resolution */
    uint16_t AccelPrevSPDFdbk;
    uint16_t AccelAvgSPDFdbk;
    uint32_t AccelSPDAvg;
    uint16_t SPDFdbk;          /* This variable holds the Motor parameter (0) Motor Spin frequency in timer counts with 18bit resolution */
    uint16_t DeviceID;         /* This variable holds the Motor parameter (2) Device ID to set the appropriate motor control page for a connected device */
    uint16_t IPDBrakeTime;     /* This variable holds the Motor parameter (5) IPD brake time to set the amount of time to brake before applying another pulse during IPD */
    uint16_t IPDPulseTime;     /* This variable holds the Motor parameter (6) IPD Pulse time to set the amount of time current pulse is given during IPD */
    uint16_t IPDDecayConstant; /* This variable holds the Motor parameter (7) IPD Decay Constant time to set the amount of time current pulse is allowed to decay during IPD */
    uint16_t AlignSector;      /* This variable holds the Motor parameter (8) Align secor to set the commutation state to be aligned at the motor start up */
    uint16_t AlignWaitTime;    /* This variable holds the Motor parameter (9) Align wait time to set the amount of time for which voltage pulses are given during Aligning the rotor at start up */
    uint16_t AccelRate;        /* This variable holds the Motor parameter (10) Accel rate defines the Open loop Blind  acceleration rate */
    uint32_t AccelStop;        /* This variable holds the Motor parameter (11) Accel stop defines the open loop to closed loop hand off velocity */
    uint32_t AccelVelocity;    /* This variable holds the Motor parameter (11) Accel stop defines the open loop initial velocity */
    uint16_t BEMFThreshold;    /* This variable holds the Motor parameter (13) BEMF_threshold to set the BEMF integration threshold value */
    uint16_t RampRateDelay;    /* This variable holds the Motor parameter (14) Ramp rate delay to set the acceleration/ desceleration of the motor */
    uint16_t SetMotorSpeed;    /* This Variable holds the Motor parameter (31) speed input from the GUI through MDBU serial */

    uint16_t CommutationBlankTime; /* This Variable holds the Motor parameter (16) to set the number of PWM cycles to before which BEMF is sampled */
    uint16_t PWMBlankCounts;   /* This Variable holds the Motor parameter (17) to set the number of PWM cycles to after which BEMF is sampled after a commutation is taken place */
    uint16_t MaxDutyCycle;     /* This Variable holds the Motor parameter (18) to set the PWM Maximum duty cycle */
    uint16_t MinOffDutyCycle;  /* This Variable holds the Motor parameter (19) to set the PWM Minimum off duty cycle to spin the motor */
    uint16_t MinOnDutyCycle;   /* This Variable holds the Motor parameter (20) to set the PWM Minimum on duty cycle to start the motor */
    uint16_t StartupDutyCycle; /* This Variable holds the Motor parameter (21) to set the PWM duty cycle at strtup*/
    uint16_t PWMPeriod;          /* This Variable holds the Motor parameter (22) to set the PWM switching frequency*/
    uint16_t MinPowerSupply;      /* This Variable holds the Motor parameter (23) to set the supply undervoltage limit */
    uint16_t UnderVolLim;      /* This Variable holds the Motor parameter (23) to set the supply undervoltage limit */
    uint16_t OverVolLim;       /* This Variable holds the Motor parameter  (24) to set the supply overvoltage limit */
    uint16_t StallDetectRev;    /* This Variable holds the Motor parameter  (25) to set the minimum revolutions to detect for a stall fault */
    uint16_t StallDetecttime;    /* This Variable holds the Motor parameter  (26) to set the time after which stall fault is triggered */
    uint16_t MotorPhaseCurrentLimit; /* This Variable holds the Motor parameter (27) to set the motor Phase current limit */
	uint16_t AutoFaultRecoveryTime; /* This Variable holds the Motor parameter (28) to set the time limit after which faults are automatically recovered */
	uint16_t Align_IPD;           /* This Variable holds the Motor parameter (29) to set the rotor or to find the current position of rotor */
	uint16_t PWM_Mode;			  /* This Variable holds the Ctrl type : 0 for 6PWM mode , 1 for 1 PWM mode*/
    uint16_t DeviceVariant;    /* This variable holds the Device variant, Device variant is set to 1 for DRV835x devices and set to zero for drv83xx Variants */
    uint16_t ShuntVariant;     /* This variable holds the Shunt variant, Shunt variant is set to 1 for DRV83x3x devices and set to zero for DRV83x0 Variants */
    uint16_t SPIVariant;       /* This variable holds the SPI variant, SPI variant is set to 1 for drv83xxS devices and set to zero for drv83xxH Variants */
} SENSORLESS_TRAP_Obj;

typedef struct FLT_STAT_REG0_Obj
{
    uint8_t REG0_FAULT;      // bit 10
    uint8_t REG0_VDS_OCP;    // bit 9
    uint8_t REG0_GDF;        // bit 8
    uint8_t REG0_UVLO;       // bit 7
    uint8_t REG0_OTSD;       // bit 6
    uint8_t REG0_VDS_HA;     // bit 5
    uint8_t REG0_VDS_LA;     // bit 4
    uint8_t REG0_VDS_HB;     // bit 3
    uint8_t REG0_VDS_LB;     // bit 2
    uint8_t REG0_VDS_HC;     // bit 1
    uint8_t REG0_VDS_LC;     // bit 0
} FLT_STAT_REG0_Obj;

typedef struct VGS_STAT_REG1_Obj
{
    uint8_t REG1_SA_OC;      // bit 10
    uint8_t REG1_SB_OC;      // bit 9
    uint8_t REG1_SC_OC;      // bit 8
    uint8_t REG1_OTW;        // bit 7
    uint8_t REG1_CPUV;       // bit 6
    uint8_t REG1_VGS_HA;     // bit 5
    uint8_t REG1_VGS_LA;     // bit 4
    uint8_t REG1_VGS_HB;     // bit 3
    uint8_t REG1_VGS_LB;     // bit 2
    uint8_t REG1_VGS_HC;     // bit 1
    uint8_t REG1_VGS_LC;     // bit 0
} VGS_STAT_REG1_Obj;

typedef struct DRV_CTRL_REG2_Obj
{
    uint8_t REG2_OCP_ACT;   // bit 10
    uint8_t REG2_DIS_CPUV;  // bit 9
    uint8_t REG2_DIS_GDF;   // bit 8
    uint8_t REG2_OTW_REP;   // bit 7
    uint8_t REG2_PWM_MODE;  // bit 6:5
    uint8_t REG2_PWM_COM;   // bit 4
    uint8_t REG2_PWM_DIR;   // bit 3
    uint8_t REG2_COAST;     // bit 2
    uint8_t REG2_BRAKE;     // bit 1
    uint8_t REG2_CLR_FLT;   // bit 0
} DRV_CTRL_REG2_Obj;

typedef struct GATE_DRV_HS_REG3_Obj
{
    uint8_t REG3_LOCK;          // bit 10:8
    uint8_t REG3_IDRIVEP_HS;    // bit 7:4
    uint8_t REG3_IDRIVEN_HS;    // bit 3:0
} GATE_DRV_HS_REG3_Obj;

typedef struct GATE_DRV_LS_REG4_Obj
{
    uint8_t REG4_CBC;           // bit 10
    uint8_t REG4_TDRIVE;        // bit 9:8
    uint8_t REG4_IDRIVEP_LS;    // bit 7:4
    uint8_t REG4_IDRIVEN_LS;    // bit 3:0
} GATE_DRV_LS_REG4_Obj;

typedef struct OCP_CTRL_REG5_Obj
{
    uint8_t REG5_TRETRY;        // bit 10
    uint8_t REG5_DEAD_TIME;     // bit 9:8
    uint8_t REG5_OCP_MODE;      // bit 7:6
    uint8_t REG5_OCP_DEG;       // bit 5:4
    uint8_t REG5_VDS_LVL;     // bit 3:0
} OCP_CTRL_REG5_Obj;

typedef struct CSA_CTRL_REG6_Obj
{
    uint8_t REG6_CSA_FET;       // bit 10
    uint8_t REG6_VREF_DIV;      // bit 9
    uint8_t REG6_LS_REF;        // bit 8
    uint8_t REG6_CSA_GAIN;      // bit 7:6
    uint8_t REG6_DIS_SEN;       // bit 5
    uint8_t REG6_CSA_CAL_A;     // bit 4
    uint8_t REG6_CSA_CAL_B;     // bit 3
    uint8_t REG6_CSA_CAL_C;     // bit 2
    uint8_t REG6_SEN_LVL;       // bit 1:0
} CSA_CTRL_REG6_Obj;

typedef struct DRV_CONFIG_REG7_Obj
{
    uint8_t REG7_RSVD;          // bit 10:1
    uint8_t REG7_CAL_MODE;       // bit 0

} DRV_CONFIG_REG7_Obj;

typedef struct REG_MAP_Obj
{
    uint16_t Fault_Status_Reg0;
    uint16_t VGS_Status_Reg1;
    uint16_t Driver_Control_Reg2;
    uint16_t Gate_Drive_HS_Reg3;
    uint16_t Gate_Drive_LS_Reg4;
    uint16_t OCP_Control_Reg5;
    uint16_t CSA_Control_Reg6;
    uint16_t DRV_Config_Reg7;
} REG_MAP_Obj;


#endif
