/**************************************************************************
 * @file        global.c
 * @author      MDBU Software Team
 * @brief       This file defines all the functions used by application during runtime
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 ******************************************************************************/

#ifndef global_H_
#define global_H_

#include "Init.h"
#include "dataTypeDefinition.h"
#include "msp430.h"
#include "SPI_API.h"
#include "TrapSensored_Parameters_Setup.h"
#include "mdbuserial.h"
#include "usb.h"
#include "mdbu_global.h"
#include "USB_config/descriptors.h"
#include "USB_app/usbConstructs.h"
#include "mdbuserial.h"
#include "usci_a_uart.h"
#include "bike.h"

// Motor Parameter Numbers
#define MTR_PARAM_DIR	(14)
#define MTR_PARAM_SPEED	(15)
#define MTR_PARAM_DEVICE_ID		(2)
#define MTR_MIN_DUTY_CYCLE		(4)
#define MTR_MAX_DUTY_CYCLE		(5)
#define MTR_RAMP_RATE_DELAY		(6)
#define MTR_UNDER_VOLTAGE_LIMIT		(7)
#define MTR_OVER_VOLTAGE_LIMIT		(8)
#define MTR_MIN_STALLDETECT_DUTY	(9)
#define MTR_STALLDETECT_REV_THRESHOLD	(10)
#define MTR_STALLDETECT_TIMER_THRESHOLD	(11)
#define MTR_AUTO_FAULT_RECOVERY_TIME	(12)
#define MTR_MOTOR_PHASE_CURRENT_LIMIT	(13)
#define MTR_FAULT_STATUS				(1)
#define MTR_PARAM_ELEC_SPEED 			(0)
#define MTR_PARAM_PWM_FREQ				(3)
#define MTR_START_STOP_MOTOR			(16)
#define MTR_HALL_CALIBRATE  			(17)
#define MTR_CALIBRATE_DUTY_CYCLE		(18)
#define MTR_CALIBRATION_CYCLES		    (19)
#define MTR_AUTO_CALIBRATION            (20)
#define DEVICE_FULL_SCALE_VOLTAGE       (21)

#define EN_DRV	    BIT6    // P1.6
#define nFAULT	    BIT7    // P2.7
#define NUMCONFIG_IDRIVE_VALUES                 16       // Defines the number of configurable Idrive values for the DRv83xxS Devices
#define NUMCONFIG_TDRIVE_VALUES                 4       // Defines the number of configurable Idrive values for the DRv83xxS Devices

#define DRV835X_DEVICE							1      // Device variant used for DRV835x
#define DRV832X_DEVICE							0      // Device variant used for DRV832x
#define DRV83XX_DEVICE							2      // Device variant used for DRV832x

#define HALL_SHIFT_FOR_PIN_A 					0
#define HALL_SHIFT_FOR_PIN_B 					1
#define HALL_SHIFT_FOR_PIN_C 					4

/* Device register width in number of bytes */
#define drv83xx_REGISTER_WIDTH (2)

void EnableGateDrivers();
void DisableGateDrivers();
void SetPWMDutyCycle(uint16_t PWMDutyCycle);
void UpdateNextCommutation(void);
void Hall_AlignSetState(uint8_t hallState);
void PWM_SetCommutation(uint8_t hallState);
void ReadPotiSpeed(void);
void ReadVCC(void);
void ReadCurrentShunt(void);
void ReadSPDFDBK(void);

void drv83xx_regRestoreFromCache();
void drv83xx_setGPIO(unsigned char gpioPort, unsigned char gpioNum, unsigned char gpioVal);
unsigned char drv83xx_getGPIO(unsigned char gpioPort, unsigned char gpioNum);
void drv83xx_StartMotor();
void drv83xx_StopMotor();
unsigned int drv83xx_registerRead(unsigned char address);
void drv83xx_registerWrite(unsigned char address, unsigned int value);
unsigned long drv83xx_getMtrParam(unsigned char num);
void drv83xx_setMtrParam(unsigned char num, unsigned long value);
void drv83xx_setCtrlType(unsigned char value);
unsigned char drv83xx_getCtrlType(void);

extern uint16_t Register_Counter;
extern uint16_t DRV835xS_IdriveP_RegData[NUMCONFIG_IDRIVE_VALUES];
extern uint16_t DRV835xS_IdriveN_RegData[NUMCONFIG_IDRIVE_VALUES];
extern uint16_t DRV832xS_IdriveP_RegData[NUMCONFIG_IDRIVE_VALUES];
extern uint16_t DRV832xS_IdriveN_RegData[NUMCONFIG_IDRIVE_VALUES];
extern uint16_t DRV835xS_Tdrive_RegData[NUMCONFIG_TDRIVE_VALUES];
extern uint16_t DRV832xS_Tdrive_RegData[NUMCONFIG_TDRIVE_VALUES];

typedef enum
{
    HOST_EN = 0,
    HOST_IDLE = 1,
    HOST_ACTIVE = 2
} HOSTCONTROL_STATUS;

typedef struct HOST_CONTROLLER_Obj
{
	uint8_t EnabledGateDrivers;		/* 0 = Disabled, 1 = Enabled */
	uint8_t StartStopMotor;			/* 0 = Start, 1 = Stop */
	uint8_t CalibrateHall;          /* 1 = Start, 0 = Stop */
} HOST_CONTROLLER_Obj;

typedef enum
{
    SYSTEM_INIT = 0,
    SYSTEM_IDLE = 1,
    MOTOR_IDLE = 2,
    MOTOR_RAMP_UP = 3,
    MOTOR_RUN = 4,
    MOTOR_RAMP_DOWN = 5,
    MOTOR_STOP = 6,
    FAULT = 7,
    MOTOR_DIRECTION = 8,
	HALL_ALIGN = 9
}STATE_MACHINE;

typedef enum
{
    NOFAULT = 0,
	VOLTAGE = 1,
	OVERCURRENT = 2,
	OVERTEMPERATURE = 3,
	MOTOR_STALL = 4,
	HALLSENSORINVALID = 5,
	GATE_DRIVER = 6,
	POWER_SUPPLY = 7,
	UNKNOWN = 8,
	HALL_CALIBRATION =9

} FAULTS;

typedef enum
{
    BIKE_INIT = 0,
    BIKE_IDLE = 1,
    SWITCHING = 2,
    ACCELERATING = 3,
    REGENERATING = 4
} BIKE_STATE;

typedef struct BIKE_STATUS
{
    BIKE_STATE current;
    BIKE_STATE previous;
} BIKE_STATUS;

typedef struct APPLICATION_STATUS
{
    STATE_MACHINE currentstate;
    STATE_MACHINE previousstate;
    FAULTS fault;
} APPLICATION_STATUS;

typedef struct BIKE_CONTROLLER
{
    uint8_t accelerating;       // 1 = enabled, 0 = disabled
    uint8_t regenerating;       // 1 = enabled, 0 = disabled
    uint8_t idling;             // 1 = enabled, 0 = disabled
    uint8_t speed;              // Controlled by throttle
    uint8_t regen_level;        // Controlled by brake signal or buttons

}BIKE_CONTROLLER;

typedef struct SENSORED_TRAP_Obj
{
    BOOL HallAlignStartFlag;     /* Flag to indicate the start aligning the rotor to Hall identification vector */
    BOOL HallCalibComplete;      /* Flag to indicate the end of calibration of Hall Signals */
    BOOL AutoHallCalib;			 /* Flag to indicate the auto calibration of Hall signals */
    BOOL HallAlignDoneFlag;      /* Flag to indicate the end of aligning the rotor to Hall identification vector */
    BOOL HallAlignInterruptFlag; /* Falg to indicate for verification of Hall alignment */
    BOOL TimerOverflowFlag;  		 /* Interrupt counter to ensure the speed measurement with 16bit resolution*/
    BOOL GateToggleFaultDetect;      /* For DRV83xxH devices Upon Enabling gate drivers, reporting of faults is disabled for first 5 ms, reporting is enabled after 10ms */

    uint8_t HallAlignState;		/* Indicates the Hall align state */
    uint8_t currentHallstate;	/* Indicates the Current Hall align state */
    uint8_t ShiftForPINA;       /* Indicates the Shift required for HallA read status proper positioning according to MCU */
    uint8_t ShiftForPINB;		/* Indicates the Shift required for HallB read status proper positioning according to MCU */
    uint8_t ShiftForPINC;		/* Indicates the Shift required for HallC read status proper positioning according to MCU */
    uint8_t ADCReadState;      /* specifies in which state ADC read has been initaiated */
    uint8_t Direction;         /* specifies the current direction of rotation */
    uint8_t Direction_flag;    /* flag to monitor if direction change is required */

    uint16_t IdriveP_Min_Value;  /* Indicates the IdriveP minimum value */
    uint16_t IdriveN_Min_Value;  /* Indicates the IdriveN minimum value */
    uint16_t Tdrive_Max_Value;   /* Indicates the IdriveP maximum value */
    uint16_t IdriveP_Setting;    /* Indicates the IdriveN maximum value */
    uint16_t IdriveN_Setting;    /* Indicates the IdriveP Setting value */
    uint16_t Tdrive_Setting;     /* Indicates the IdriveP Setting value */

    uint16_t Accelerationcounter;     /*Acceleration variables Initialize a counter to compare with Acceleration Divider.*/
    uint16_t HallAligncounter;        /*Initialize a counter to compare with HallAlignWaitTime */
    uint16_t HallAlignValue[4];
    uint16_t Accelerationdivider;	    /*How many PWM cycles before increasing duty cycle. */
    uint16_t HallCalibrationCycles;		/* Defines number of PWM cycles Motor Hall Calibration is Executed */
    uint16_t MINDutyCycle;              /* Low speed setting depending on application*/
    uint16_t MAXDutyCycle;              /* High speed setting depending on application*/
    uint16_t CALDutyCycle;              /* Calibration duty cycle applied to motor during Hall Calibration */
    uint16_t RotationCount;				/*This keeps track of how many rotations (electrical) of the motor is completed since the last startup.*/
    uint16_t TargetDutyCycle;			/*Duty cycle that should be reached for the selected potentiometer command setting.*/

    uint16_t CurrentDutyCycle;			    /*Duty cycle that is going out on the PWM at this moment.*/
    uint16_t RampRate;                          /*How fast increasing and decreasing duty cycle*/

    /* Fault handling variables */
    uint16_t StallDetectCounter;        /*This is a counter used in Timer4 to check for the motor stall.*/
    uint16_t RestartDelay;              /*This is the variable that is used to count the time from when a fault is cleared*/
    uint16_t VCCvoltage;                /*How much voltage applied for EVM*/
    uint16_t MotorPhaseCurrent;         /* How much current flowing through motor phase */
    uint16_t underVoltageLimit;         /*Configurable Under Voltage Limit from HOST*/
    uint16_t overVoltageLimit;          /*Configurable Over Voltage Limit from HOST*/
    uint16_t minStallDetectDuty; 	    /* Minimum Duty Cycle above which stall will be detected */
    uint16_t stallDetectRevThreshold;   /* Number of revolutions below which stall fault will be flagged */
    uint16_t stallDetectTimerThreshold; /* Time in milli seconds above which if motor doesnt spin min revolutions specified above(stallDetectRevThreshold) a stall fault is triggered */
    uint16_t autoFaultRecoveryTime; 	/* Time in milli seconds after which system reinitialises itself if fault gets cleared */
    uint16_t readVccCounter;			/* Counter to read VCC every READ_VCC_PERIOD interval */
    uint16_t SPDFdbk;           		/* This variable holds the timer counts with 16bit resolution */
    uint16_t PWM_Mode;			  		/* This Variable holds the Ctrl type : 0 for 6PWM mode , 1 for 1 PWM mode*/
    uint16_t PWMPeriod;          		/* This Variable holds the Motor parameter to set the PWM switching frequency*/
    uint16_t DeviceID;         			/* This variable holds the Motor parameter (2) Device ID to set the appropriate motor control page for a connected device */
    uint16_t DeviceVariant;    			/* This variable holds the Device variant, Device variant is set to 1 for DRV835x devices and set to zero for drv83xx Variants */
    uint16_t deviceIDADC;
    uint16_t ShuntVariant;     			/* This variable holds the Shunt variant, Shunt variant is set to 1 for DRV83x3x devices and set to zero for DRV83x0 Variants */
    uint16_t SPIVariant;       			/* This variable holds the SPI variant, SPI variant is set to 1 for drv83xxS devices and set to zero for drv83xxH Variants */
    uint16_t MinPowerSupply;   			/* This variable holds the Digital equivalent of 5v */
    uint16_t GateToggleCounter;         /* This counter is used to count for first 5 ms after changing gate drive status to toggle Gate drive fault detect */
    uint16_t Counter_1M_Second;

    float_t MotorPhaseCurrentLimit;     /* Defines Maximum allowed Motor phase current  */

} SENSORED_TRAP_Obj;

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
    uint8_t REG2_OCP_ACT;      // bit 10
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

/////////////////////////
#endif /*global_H_*/
