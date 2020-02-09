/**************************************************************************
 * @file        TrapSensored_Parameters_Setup.h
 * @author      MDBU Software Team
 * @brief       Parameters to be set up by the User.
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 ******************************************************************************/
// USER DEFINES
#define ALGO_ID						1       // 2 Indicates sensorless , 1 indicates sensored algorithms
#define PROD_ID                     1       // 1 Indicated AMD PL devices
#define FW_VER_MAJ                  0       // FW version major
#define FW_VER_MIN                  8       // FW version minor
#define FW_VER_PATCH                0       // FW version patch

/*************** Idrive & Tdrive Configuration Settings ****************/
#define GATE_TO_DRAIN_CHARGE        8.7      // Set the gate to drain charge of the FET used on the EVM in uC
#define RISE_TIME                   100     // Select the Maximum Rise time desired for the FET in nano seconds
#define FALL_TIME                   50     // Select the Maximum Fall time desired for the FET in nano Seconds

/* System parameter setup */
#define SPEED_INPUT_SAMP_INTERVAL (127)         /* The number of PWM cycles after which change in Speed input through ADC is Sampled */
#define PWM_FACTOR (0)	          				/* 12 bit ADC result registers will be scaled to PWM period which is 10 bit  */
#define PWM_PERIOD (1000)						/* PWM Period time , With a 25Mhz clock , PWM will be generated at 25Khz*/
#define READ_VCC_PERIOD (100)					/* TIME INTERVAL AFTER WHICH VCC IS MONITORED (100ms) */
#define TIME_COUNT_1MS (25000)            		/* 1 ms timer count */
#define MIN_DUTY_CYCLE	(15)             	    /* Minimal duty cycle applied to motor */
#define MAX_DUTY_CYCLE	(1000)              	/* Maximal duty cycle applied to motor */
#define CAL_DUTY_CYCLE	(30)              	    /* Calibration duty cycle applied to motor during Hall Calibration */
#define RAMP_RATE (1)							/* Ramp rate for acceleration and deceleration of the motor speed*/
#define RAMP_RATE_DELAY (100)		        	/* How many PWM periods are between a change of the speed */
#define HALL_CALIB_CYCLES (4000)                /* Defines number of PWM cycles Motor Hall Calibration is Executed */
/* Fault handling setup */						/*ADC Max ref voltage is 3.3v ,  VCC is scaled by 0.074 internally (5/62 Ohms bridge) so that VCC ref input to ADC never cross 3.3v.*/
#define UNDER_VOLTAGE_LIMIT (10)			    /* Under Voltage set for below the specified volts - digital values are there by cal as (Limit*4095)/Full scale voltage  */
#define OVER_VOLTAGE_LIMIT	(60)     			/* Over Voltage set for above the specified volts - digital values are there by cal as (Limit*4095)/Full scale voltage */
#define FULL_SCALE_VOLTAGE  (115)				/*As BEMF for above full scale volts may give ADC ref volts exceeds nominal value 3.3V , MAX VCC is limited*/
#define MIN_POWER_SUPPLY    (5)                 /* Minimum power supply required to access the SPI , when supply voltage is below this limit gate drivers are disabled */
#define MIN_STALLDETECT_DUTY		(125)		/* Minimum Duty Cycle above which stall will be detected */
#define STALLDETECT_REV_THRESHOLD	(1)			/* Number of revolutions below which stall fault will be flagged */
#define STALLDETECT_TIMER_THRESHOLD (500)		/* Time in milli seconds above which if motor doesnt spin min revolutions specified above(STALLDETECT_REV_THRESHOLD) a stall fault is triggered */
#define AUTO_FAULT_RECOVERY_TIME (6000)      	/* Time in milli seconds after which system reinitialises itself if fault gets cleared */
#define STALL_DETECTOIN_FLAG (1)            	/* Enable motor stall fault detection*/
#define VCC_MONITOR_FLAG (1)                	/* Enable VCC Supply Voltage monitoring for fault detection */
#define MOTOR_PHASE_CURRENT_LIMIT (900)         /* Defines the max allowed motor phase current in digital counts. Motor phase current is monitored every electrical cycle , and when ever current limit is reached an OC fault is Triggered */
/* DRV832xS SPI REGISTER SETTINGS*/
/* SPI_REG_02 : DRIVER CONTROL */
#define DRV832xS_RSVD        (0x00)         /*  */
#define DRV832xS_DIS_CPUV    (0x00)         /*  */
#define DRV832xS_DIS_GDF     (0x00)         /*  */
#define DRV832xS_OTW_REP     (0x00)         /*  */
#define DRV832xS_PWM_MODE    (0x00)         /*  */
#define DRV832xS_PWM_COM     (0x00)         /*  */
#define DRV832xS_PWM_DIR     (0x00)         /*  */
#define DRV832xS_COAST_BIT   (0x00)         /*  */
#define DRV832xS_BRAKE_BIT   (0x00)         /*  */
#define DRV832xS_CLR_FLT     (0x00)         /*  */

/* SPI_REG_03 : GATE DRIVE HS */
#define DRV832xS_LOCK_BIT    (0x03)         /*  */
#define DRV832xS_IDRIVEP_HS  (0x0F)         /*  */
#define DRV832xS_IDRIVEN_HS  (0x0F)         /*  */

/* SPI_REG_04 : GATE DRIVE LS */
#define DRV832xS_CBC         (0x01)         /*  */
#define DRV832xS_TDRIVE      (0x03)         /*  */
#define DRV832xS_IDRIVEP_LS  (0x0F)         /*  */
#define DRV832xS_IDRIVEN_LS  (0x0F)         /*  */

/* SPI_REG_05 : OCP CONTROL */
#define DRV832xS_TRETRY      (0x00)         /*  */
#define DRV832xS_DEAD_TIME   (0x00)         /*  */
#define DRV832xS_OCP_MODE    (0x01)         /*  */
#define DRV832xS_OCP_DEG     (0x01)         /*  */
#define DRV832xS_VDS_LVL     (0x09)         /*  */

/* SPI_REG_06 : CSA CONTROL */
#define DRV832xS_CSA_FET     (0x00)         /*  */
#define DRV832xS_VREF_DIV    (0x01)         /*  */
#define DRV832xS_LS_REF      (0x00)         /*  */
#define DRV832xS_CSA_GAIN    (0x02)         /*  */           // By default Shunt Amplifier gain is set to 20V/V
#define DRV832xS_DIS_SEN     (0x00)         /*  */
#define DRV832xS_CSA_CAL_A   (0x00)         /*  */
#define DRV832xS_CSA_CAL_B   (0x00)         /*  */
#define DRV832xS_CSA_CAL_C   (0x00)         /*  */
#define DRV832xS_SEN_LVL     (0x03)         /*  */

/* DRV832xS IdriveP_HS/LS Config Values in mA */
#define DRV832xS_IdriveP_MODE0		10
#define DRV832xS_IdriveP_MODE1		30
#define DRV832xS_IdriveP_MODE2		60
#define DRV832xS_IdriveP_MODE3		80
#define DRV832xS_IdriveP_MODE4		120
#define DRV832xS_IdriveP_MODE5		140
#define DRV832xS_IdriveP_MODE6		170
#define DRV832xS_IdriveP_MODE7		190
#define DRV832xS_IdriveP_MODE8		260
#define DRV832xS_IdriveP_MODE9		330
#define DRV832xS_IdriveP_MODE10		370
#define DRV832xS_IdriveP_MODE11		440
#define DRV832xS_IdriveP_MODE12		570
#define DRV832xS_IdriveP_MODE13		680
#define DRV832xS_IdriveP_MODE14		820
#define DRV832xS_IdriveP_MODE15		1000


/* DRV832xS IdriveN_HS/LS Config Values in mA */
#define DRV832xS_IdriveN_MODE0		20
#define DRV832xS_IdriveN_MODE1		60
#define DRV832xS_IdriveN_MODE2		120
#define DRV832xS_IdriveN_MODE3		160
#define DRV832xS_IdriveN_MODE4		240
#define DRV832xS_IdriveN_MODE5		280
#define DRV832xS_IdriveN_MODE6		340
#define DRV832xS_IdriveN_MODE7		380
#define DRV832xS_IdriveN_MODE8		520
#define DRV832xS_IdriveN_MODE9		660
#define DRV832xS_IdriveN_MODE10		740
#define DRV832xS_IdriveN_MODE11		880
#define DRV832xS_IdriveN_MODE12		1140
#define DRV832xS_IdriveN_MODE13		1360
#define DRV832xS_IdriveN_MODE14		1640
#define DRV832xS_IdriveN_MODE15		2000

/* DRV832xS Tdrive_HS/LS Config Values in nS */
#define DRV832xS_Tdrive_MODE0		500
#define DRV832xS_Tdrive_MODE1		1000
#define DRV832xS_Tdrive_MODE2		2000
#define DRV832xS_Tdrive_MODE3		4000
/* DRV835xS SPI REGISTER SETTINGS*/
/* SPI_REG_02 : DRIVER CONTROL */
#define DRV835xS_OCP_ACT     (0x00)         /*  */
#define DRV835xS_DIS_CPUV    (0x00)         /*  */
#define DRV835xS_DIS_GDF     (0x00)         /*  */
#define DRV835xS_OTW_REP     (0x00)         /*  */
#define DRV835xS_PWM_MODE    (0x00)         /*  */
#define DRV835xS_PWM_COM     (0x00)         /*  */
#define DRV835xS_PWM_DIR     (0x00)         /*  */
#define DRV835xS_COAST_BIT   (0x00)         /*  */
#define DRV835xS_BRAKE_BIT   (0x00)         /*  */
#define DRV835xS_CLR_FLT     (0x00)         /*  */

/* SPI_REG_03 : GATE DRIVE HS */
#define DRV835xS_LOCK_BIT    (0x03)         /*  */
#define DRV835xS_IDRIVEP_HS  (0x0F)         /*  */
#define DRV835xS_IDRIVEN_HS  (0x0F)         /*  */

/* SPI_REG_04 : GATE DRIVE LS */
#define DRV835xS_CBC         (0x01)         /*  */
#define DRV835xS_TDRIVE      (0x03)         /*  */
#define DRV835xS_IDRIVEP_LS  (0x0F)         /*  */
#define DRV835xS_IDRIVEN_LS  (0x0F)         /*  */

/* SPI_REG_05 : OCP CONTROL */
#define DRV835xS_TRETRY      (0x00)         /*  */
#define DRV835xS_DEAD_TIME   (0x00)         /*  */
#define DRV835xS_OCP_MODE    (0x01)         /*  */
#define DRV835xS_OCP_DEG     (0x01)         /*  */
#define DRV835xS_VDS_LVL     (0x09)         /*  */

/* SPI_REG_06 : CSA CONTROL */
#define DRV835xS_CSA_FET     (0x00)         /*  */
#define DRV835xS_VREF_DIV    (0x01)         /*  */
#define DRV835xS_LS_REF      (0x00)         /*  */
#define DRV835xS_CSA_GAIN    (0x02)         /*  */            // By default Shunt Amplifier gain is set to 20V/V
#define DRV835xS_DIS_SEN     (0x00)         /*  */
#define DRV835xS_CSA_CAL_A   (0x00)         /*  */
#define DRV835xS_CSA_CAL_B   (0x00)         /*  */
#define DRV835xS_CSA_CAL_C   (0x00)         /*  */
#define DRV835xS_SEN_LVL     (0x03)         /*  */

/* SPI_REG_07 : CSA CONTROL */
#define DRV835xS_RSVD        (0x00)         /*  */
#define DRV835xS_CAL_MODE    (0x00)         /*  */

/* DRV835xS IdriveP_HS/LS Config Values in mA */

#define DRV835xS_IdriveP_MODE0		(50)
#define DRV835xS_IdriveP_MODE1		(50)
#define DRV835xS_IdriveP_MODE2		(100)
#define DRV835xS_IdriveP_MODE3		(150)
#define DRV835xS_IdriveP_MODE4		(300)
#define DRV835xS_IdriveP_MODE5		(350)
#define DRV835xS_IdriveP_MODE6		(400)
#define DRV835xS_IdriveP_MODE7		(450)
#define DRV835xS_IdriveP_MODE8		(550)
#define DRV835xS_IdriveP_MODE9		((600))
#define DRV835xS_IdriveP_MODE10		(650)
#define DRV835xS_IdriveP_MODE11		(700)
#define DRV835xS_IdriveP_MODE12		(850)
#define DRV835xS_IdriveP_MODE13		(900)
#define DRV835xS_IdriveP_MODE14		(950)
#define DRV835xS_IdriveP_MODE15		(1000)


/* DRV835xS IdriveN_HS/LS Config Values in mA */
#define DRV835xS_IdriveN_MODE0		100
#define DRV835xS_IdriveN_MODE1		100
#define DRV835xS_IdriveN_MODE2		200
#define DRV835xS_IdriveN_MODE3		300
#define DRV835xS_IdriveN_MODE4		600
#define DRV835xS_IdriveN_MODE5		700
#define DRV835xS_IdriveN_MODE6		800
#define DRV835xS_IdriveN_MODE7		900
#define DRV835xS_IdriveN_MODE8		1100
#define DRV835xS_IdriveN_MODE9		1200
#define DRV835xS_IdriveN_MODE10		1300
#define DRV835xS_IdriveN_MODE11		1400
#define DRV835xS_IdriveN_MODE12		1700
#define DRV835xS_IdriveN_MODE13		1800
#define DRV835xS_IdriveN_MODE14		1900
#define DRV835xS_IdriveN_MODE15		2000

/* DRV835xS Tdrive_HS/LS Config Values in nS */
#define DRV835xS_Tdrive_MODE0		500
#define DRV835xS_Tdrive_MODE1		1000
#define DRV835xS_Tdrive_MODE2		2000
#define DRV835xS_Tdrive_MODE3		4000
