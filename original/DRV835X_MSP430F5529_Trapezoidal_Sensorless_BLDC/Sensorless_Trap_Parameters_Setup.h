/* Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
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
 * @file        Sensorless_Trap_Parameters_Setup.h
 * @author      MDBU Software Team
 * @brief       SPI API file for SPI Module.
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 * @date		May 2016
 ******************************************************************************/

// USER DEFINES
#define ALGO_ID						2       // 2 Indicates sensorless , 1 indicates sensored algorithms
#define PROD_ID                     1       // 1 Indicated AMD PL devices
#define FW_VER_MAJ                  0       // FW version major
#define FW_VER_MIN                  9       // FW version minor
#define FW_VER_PATCH                0       // FW version patch

//SYSTEM Parameters
#define PWM_FREQUENCY				25U   		// PWM switching frequency in KHz
#define MOTOR_RATED_VOLTAGE			48.0F
#define MOTOR_RATED_SPEED 			3200.0F       // 3200 RPM Annaheim motor ,
#define MOTOR_POLES					8         // 8 pole for Annaheim motor
#define MOTOR_UNDER_RATING			90         // percentage of rated voltage at which motor spins at rated speed at no load

/*************** Idrive & Tdrive Configuration Settings ****************/
#define GATE_TO_DRAIN_CHARGE        8.7      // Set the gate to drain charge of the FET used on the EVM in uC
#define RISE_TIME                   100     // Select the Maximum Rise time desired for the FET in nano Seconds
#define FALL_TIME                   50     // Select the Maximum Fall time desired for the FET in nano Seconds

#define ALIGN_OR_IPD				0       //  0 indicates Six pulse method , 1 indicates rotor align
//ISC User Parameters
#define ISC_THRESHOLD_MAX_SPEED 	100		// Maximim Speed above which ISC routine
#define ISC_THRESHOLD_SPEED 		50		// Minimum Speed below which ISC routine skips to IPD after brake time set by ISC BRAKE TIME in ms to disscipate the residual BEMF( 1.4v *4095)/57.5 = 100
#define ISC_MIN_LINE_BEMF           20      // Minimum Line BEMF below which motor is assumed to be stand still
#define ISC_ZEROTH_PHASE_MATCH      12      // After two electrical cycles or 12 phase crossings ( for 1 electrical cycles, 3 phases cross each other 6 times) of free wheeling Start sampling the BEMF for better accuracy
#define ISC_FIRST_PHASE_MATCH       13      // After two electrical cycles or 13 phase crossings ( for 1 electrical cycles, 3 phases cross each other 6 times) of free wheeling sample the first BEMF match or phase crossing
#define ISC_SECOND_PHASE_MATCH      14      // After two electrical cycles or 14 phase crossings ( for 1 electrical cycles, 3 phases cross each other 6 times) of free wheeling sample the second BEMF match or phase crossing

//IPD User Parameters
#define IPD_ADD_BRAKE	 			30		// the (IPDRiseTime/512)+this number  IPDRiseTime is measured using timerA at 12.5MHz so 400us = 5000
#define IPD_PULSE_TIME	 			3000		// set no of clock cycles (1 clock cycles = 40nS) a voltage pulse is applied on a phase for Initial Position detection by Six pulse method
#define	IPD_DECAY_CONSTANT 			3		// how many times longer to coast the motor and wait until next pulse after braking in IPD

//Align User Parameters
#define ALIGN_SECTOR				1		// Align commutation sequence (1-6)
#define ALIGN_WAIT_TIME				500		// Number of PWM cycles to Wait during align align time seconds = [this number * 41us]

//Open Loop Acceleration User Parameters
#define ACCEL_RATE 					10		// {Hz/s}
#define ACCEL_STOP 					45	// {Hz}
#define ACCEL_VELOCITY_INIT			1   // {Hz}

//Closed Loop User Parameters
#define BEMF_THRESHOLD 				3000    // BEMF Integration threshold according to calculations of BEMF waveform
#define RAMP_RATE_DELAY		 		100		// This number controls the acceleration,  duty cycle is updated after ( RAMP_RATE_DELAY * 1000) clock cycles
#define RAMP_RATE					1		// This is the change in dutycycle (increment/decrement) for every update
#define COMMUTATION_BLANK_TIME 		5		// How many PWM cycles to blank before sampling the BEMF
#define PWM_BLANK_COUNTS 			5		// How many Clock cycles before the center of PWM  the BEMF is sampled

#define MAX_DUTY_CYCLE 				1000		// relative to PWM_PERIOD
#define MIN_OFF_DUTY 				240			// relative to PWM_PERIOD
#define MIN_ON_DUTY 				250			// relative to PWM_PERIOD
#define PWM_FACTOR					0       	//  ADC 12 bit to PWM width ratio , by default 0 represents 12 bit scaling

/* Fault handling setup */						/*ADC Max ref voltage is 3.3v ,  VCC is scaled by 0.0573 internally so that VCC ref input to ADC never cross 3.3v The maximum supply voltage is 115V.*/
#define UNDER_VOLTAGE_LIMIT (10)			    /* Under Voltage set for below the specified volts - digital values are there by cal as (Limit*4095)/Full scale voltage  */
#define OVER_VOLTAGE_LIMIT	(60)     			/* Over Voltage set for above the specified volts - digital values are there by cal as (Limit*4095)/Full scale voltage */
#define FULL_SCALE_VOLTAGE  (115)				/*As BEMF for above full scale volts may give ADC ref volts exceeds nominal value 3.3V , MAX VCC is limited*/
#define MIN_POWER_SUPPLY    (5)                 /* Minimum power supply required to access the SPI , when supply voltage is below this limit gate drivers are disabled */
#define STALLDETECT_REV_THRESHOLD	(1)			/* Number of revolutions below which stall fault will be flagged */
#define STALLDETECT_TIMER_THRESHOLD (200)		/* Time in milli seconds above which if motor doesnt spin min revolutions specified above(STALLDETECT_REV_THRESHOLD) a stall fault is triggered */
#define MOTOR_PHASE_CURRENT_LIMIT (1800)        /* Defines the max allowed motor phase current in digital counts . Motor phase current is monitored every electrical cycle , and when ever current limit is reached an OC fault is Triggered */
#define AUTO_FAULT_RECOVERY_TIME (6000)      	/*  Delay in milli Seconds after which system reinitialises itself if fault gets cleared */

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
#define DRV835xS_IDRIVEP_HS  (0x02)         /*  */
#define DRV835xS_IDRIVEN_HS  (0x02)         /*  */

/* SPI_REG_04 : GATE DRIVE LS */
#define DRV835xS_CBC         (0x01)         /*  */
#define DRV835xS_TDRIVE      (0x00)         /*  */
#define DRV835xS_IDRIVEP_LS  (0x02)         /*  */
#define DRV835xS_IDRIVEN_LS  (0x02)         /*  */

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
