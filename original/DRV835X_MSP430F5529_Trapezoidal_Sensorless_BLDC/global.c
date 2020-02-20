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
 * @file        global.c
 * @author      MDBU Software Team
 * @brief       SPI API file for SPI Module.
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 * @date		May 2016
 ******************************************************************************/
#include "global.h"

// Controller
extern SENSORLESS_TRAP_Obj SensorlessTrapController;
extern APPLICATION_STATUS ApplicationStatus;
extern HOSTCONTROL_STATUS HostControl_Status;
extern mdbuSerial_RxPacket mdbuSerial_RxPkt;
extern HOST_CONTROLLER_Obj HostController;
// Registers
extern FLT_STAT_REG0_Obj Fault_Status_Reg;
extern VGS_STAT_REG1_Obj VGS_Status_Reg;
extern DRV_CTRL_REG2_Obj Driver_Control_Reg;
extern GATE_DRV_HS_REG3_Obj Gate_Drive_HS_Reg;
extern GATE_DRV_LS_REG4_Obj Gate_Drive_LS_Reg;
extern OCP_CTRL_REG5_Obj OCP_Control_Reg;
extern CSA_CTRL_REG6_Obj CSA_Control_Reg;
extern DRV_CONFIG_REG7_Obj DRV_CONFIG_Reg;
extern REG_MAP_Obj Reg_Map_Cache;

uint16_t Register_Counter;
uint16_t DRV835xS_IdriveP_RegData[NUMCONFIG_IDRIVE_VALUES] = 		{	DRV835xS_IdriveP_MODE0, DRV835xS_IdriveP_MODE1, DRV835xS_IdriveP_MODE2, DRV835xS_IdriveP_MODE3,
															DRV835xS_IdriveP_MODE4, DRV835xS_IdriveP_MODE5, DRV835xS_IdriveP_MODE6, DRV835xS_IdriveP_MODE7,
															DRV835xS_IdriveP_MODE8, DRV835xS_IdriveP_MODE9, DRV835xS_IdriveP_MODE10, DRV835xS_IdriveP_MODE11,
															DRV835xS_IdriveP_MODE12, DRV835xS_IdriveP_MODE13, DRV835xS_IdriveP_MODE14, DRV835xS_IdriveP_MODE15,	};

uint16_t DRV835xS_IdriveN_RegData[NUMCONFIG_IDRIVE_VALUES] = 		{	DRV835xS_IdriveN_MODE0, DRV835xS_IdriveN_MODE1, DRV835xS_IdriveN_MODE2, DRV835xS_IdriveN_MODE3,
															DRV835xS_IdriveN_MODE4, DRV835xS_IdriveN_MODE5, DRV835xS_IdriveN_MODE6, DRV835xS_IdriveN_MODE7,
															DRV835xS_IdriveN_MODE8, DRV835xS_IdriveN_MODE9, DRV835xS_IdriveN_MODE10, DRV835xS_IdriveN_MODE11,
															DRV835xS_IdriveN_MODE12, DRV835xS_IdriveN_MODE13, DRV835xS_IdriveN_MODE14, DRV835xS_IdriveN_MODE15,	};

uint16_t DRV832xS_IdriveP_RegData[NUMCONFIG_IDRIVE_VALUES] = 		{	DRV832xS_IdriveP_MODE0, DRV832xS_IdriveP_MODE1, DRV832xS_IdriveP_MODE2, DRV832xS_IdriveP_MODE3,
															DRV832xS_IdriveP_MODE4, DRV832xS_IdriveP_MODE5, DRV832xS_IdriveP_MODE6, DRV832xS_IdriveP_MODE7,
															DRV832xS_IdriveP_MODE8, DRV832xS_IdriveP_MODE9, DRV832xS_IdriveP_MODE10, DRV832xS_IdriveP_MODE11,
															DRV832xS_IdriveP_MODE12, DRV832xS_IdriveP_MODE13, DRV832xS_IdriveP_MODE14, DRV832xS_IdriveP_MODE15,	};

uint16_t DRV832xS_IdriveN_RegData[NUMCONFIG_IDRIVE_VALUES] = 		{ DRV832xS_IdriveN_MODE0, DRV832xS_IdriveN_MODE1, DRV832xS_IdriveN_MODE2, DRV832xS_IdriveN_MODE3,
															DRV832xS_IdriveN_MODE4, DRV832xS_IdriveN_MODE5, DRV832xS_IdriveN_MODE6, DRV832xS_IdriveN_MODE7,
															DRV832xS_IdriveN_MODE8, DRV832xS_IdriveN_MODE9, DRV832xS_IdriveN_MODE10, DRV832xS_IdriveN_MODE11,
															DRV832xS_IdriveN_MODE12, DRV832xS_IdriveN_MODE13, DRV832xS_IdriveN_MODE14, DRV832xS_IdriveN_MODE15,	};

uint16_t DRV835xS_Tdrive_RegData[NUMCONFIG_TDRIVE_VALUES] = {DRV835xS_Tdrive_MODE0, DRV835xS_Tdrive_MODE1, DRV835xS_Tdrive_MODE2, DRV835xS_Tdrive_MODE3};
uint16_t DRV832xS_Tdrive_RegData[NUMCONFIG_TDRIVE_VALUES] = {DRV832xS_Tdrive_MODE0, DRV832xS_Tdrive_MODE1, DRV832xS_Tdrive_MODE2, DRV832xS_Tdrive_MODE3};

void EnableGateDrivers()
{
    /* Gate Drive Enable using Port 1.6 */
	//P1OUT |= BIT6;          // Enable Gate Drivers
}
/* switch on the low side switches of all three phases */
void BrakeMotor()
{
	SensorlessTrapController.CurrentDutyCycle = 0;
	SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);
	if(SensorlessTrapController.PWM_Mode == 0)  // If six PWM mode
	{
		P2SEL &= ~(BIT4 | BIT5);                                                                    /* Select P2.4 , P2.5 as I/O Function for Phase A*/
		P1SEL &= ~(BIT4 | BIT5);                                                                    /* Select P1.4 , P1.5 as I/O Function for Phase B*/
		P1SEL &= ~(BIT2 | BIT3);                                                                    /* Select P1.2 , P1.3 as I/O Function for Phase C*/

		P2OUT &= ~BIT5;                                     // Turn High Side U phase Off
		P1OUT &= ~BIT5;                                     // Turn High Side V phase Off
		P1OUT &= ~BIT3;                                     // Turn High Side W phase Off
		P2OUT |= BIT4;                                     // Turn Low Side U phase On
		P1OUT |= BIT4;                                     // Turn Low Side V phase On
		P1OUT |= BIT2;                                    // Turn Low Side W phase On
	}
	else
	{
		P1OUT &= ~BIT2  ; //  Make INLC as GPIO low to apply the brakes
	}
}
void DisableGateDrivers()
{
	SensorlessTrapController.CurrentDutyCycle = 0;
	SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);
	if(SensorlessTrapController.PWM_Mode == 0)  // If six PWM mode
	{
		P2OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P2.4 , P2.5  */
		P2SEL &= ~(BIT4 | BIT5);                                                                    /* Select P2.4 , P2.5 as I/O Function for Phase A*/

		/* Reset switches for phase B (LOW-HIGH)*/
		P1OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P1.4 , P1.5  */
		P1SEL &= ~(BIT4 | BIT5);                                                                    /* Select P1.4 , P1.5 as I/O Function for Phase B*/

		/* Reset switches for phase C (LOW-HIGH)*/
		P1OUT &= ~(BIT2 | BIT3);                                    /* Reset bits P1.2 , P1.3  */
		P1SEL &= ~(BIT2 | BIT3);                                                                    /* Select P1.2 , P1.3 as I/O Function for Phase C*/
	}
	else
	{
		P2SEL &= ~(BIT4 | BIT5);
		P1SEL &= ~(BIT4 | BIT5);        /* Select P1.4 , P1.5 as I/O Function for Phase B*/
		P1SEL &= ~(BIT2 | BIT3);        /* Select P1.2 , P1.3 as I/O Function for Phase C*/                                                                    /* Select P2.4  I/O Function for Phase A*/
		P2OUT &= ~BIT5;  // Set 0% duty cycle to turn off all switches
		P2OUT &= ~BIT4 ; // Make INLA as GPIO low to switch of phase A switches
		P1OUT &= ~BIT5 ; //  Make INHB as GPIO low to switch of phase B switches
		P1OUT &= ~BIT4 ; //  Make INLB as GPIO low to switch of phase C switches
		P1OUT |= BIT2  ; //  Make INLC as GPIO low to remove the brakes

	}
}

/*function
 * drv83xx_regRestoreFromCache()
 * Restores the device register values by rewriting them with the cached values.
 * */
void drv83xx_regRestoreFromCache()
{
    if(SensorlessTrapController.SPIVariant == TRUE)                  // Initialize the SPI variables and settings only if the device is "S" Variant
    {
		/* Write all the cached register values to the device */
		SPI_Write(SPI_REG_DRV_CTRL, Reg_Map_Cache.Driver_Control_Reg2);
		SPI_Write(SPI_REG_GATE_DRV_HS, Reg_Map_Cache.Gate_Drive_HS_Reg3);
		SPI_Write(SPI_REG_GATE_DRV_LS, Reg_Map_Cache.Gate_Drive_LS_Reg4);
		SPI_Write(SPI_REG_OCP_CTRL, Reg_Map_Cache.OCP_Control_Reg5);

		/* This register exists only in DRV83xxS */
		if (SensorlessTrapController.ShuntVariant == TRUE)
		{
			SPI_Write(SPI_REG_CSA_CTRL, Reg_Map_Cache.CSA_Control_Reg6);
		}
		if(SensorlessTrapController.DeviceVariant == DRV835X_DEVICE)
		{
			SPI_Write(SPI_REG_DRV_CONFIG, Reg_Map_Cache.DRV_Config_Reg7);
		}
    }
}

/*function
 * drv83xx_regToCache(unsigned char address)
 * Caches the device register value in the firmware
 * */
void drv83xx_regToCache(unsigned char address, unsigned int regValue)
{
    if(SensorlessTrapController.SPIVariant == TRUE)                  // Initialize the SPI variables and settings only if the device is "S" Variant
    {

		switch (address)
		{
			case 0:
				Reg_Map_Cache.Fault_Status_Reg0 = regValue;
				break;

			case 1:
				Reg_Map_Cache.VGS_Status_Reg1 = regValue;
				break;

			case 2:
				Reg_Map_Cache.Driver_Control_Reg2 = regValue;
				break;

			case 3:
				Reg_Map_Cache.Gate_Drive_HS_Reg3 = regValue;
				break;

			case 4:
				Reg_Map_Cache.Gate_Drive_LS_Reg4 = regValue;
				break;

			case 5:
				Reg_Map_Cache.OCP_Control_Reg5 = regValue;
				break;

			case 6:
				/* This register exists only in DRV83xxS */
				if (SensorlessTrapController.ShuntVariant == TRUE)
				{
					Reg_Map_Cache.CSA_Control_Reg6 = regValue;
				}
				break;
			case 7:
				/*This register exist only in DRV835xS*/
				if(SensorlessTrapController.DeviceVariant == DRV835X_DEVICE)
				{
					Reg_Map_Cache.DRV_Config_Reg7 = regValue;
				}
				break;
			default:
				break;
		}
    }
}

/*function
 * drv83xx_registerRead(unsigned char address)
 * Device specific register read funtion
 * */
unsigned int drv83xx_registerRead(unsigned char address)
{
    unsigned int regValue;

    /* Read the value from the device*/
    regValue = SPI_Read(address);

    /* Cache the value in the firmware */
    if (ApplicationStatus.fault == NOFAULT)
    {
        drv83xx_regToCache(address, regValue);
    }

    return regValue;
}

/*function
 * drv83xx_registerWrite()
 * Device specific register write funtion
 * */
void drv83xx_registerWrite(unsigned char address, unsigned int value)
{
     if((HostController.Start_Stop_Motor == 0) && (address == 0x02))  // Allow SPI write function to motor control modes address 0x02 only when the motor is in stop state
     {
		;
     }
     else
     {
        /* Cache the value in the firmware */
        if (ApplicationStatus.fault == NOFAULT)
        {
            drv83xx_regToCache(address, value);
        }
        
        /* Write the value to the device */       
        SPI_Write(address, value);
     }
}

void drv83xx_StartMotor()
{
	HostController.Start_Stop_Motor = 0;
}

void drv83xx_StopMotor()
{
	HostController.Start_Stop_Motor = 1;
}
void drv83xx_setCtrlTypeParam(unsigned char value)
{
	SensorlessTrapController.PWM_Mode = value;
	DisableGateDrivers();
    if(SensorlessTrapController.PWM_Mode == 0) // If 6x PWM mode configure the Mode pin to pull down
    {
        P4SEL &= ~BIT3;  // Select the GPIO functionality of the pin
    	P4DIR |= BIT3;   // Set the pin as Output
        P4OUT &= ~BIT3;  // Set the pin logic low
    }
    else // If 1x PWM mode configure mode pin to High Impedence state
    {
        P4SEL &= ~BIT3;	 // Select the GPIO functionality of the pin
    	P4DIR &= ~BIT3;  // Set the pin as Input
    	P4REN &= ~BIT3;  // Diable the the Pull up/ pull down set the pin to high impedence
    }
	if(SensorlessTrapController.SPIVariant == TRUE)                  // Write the PWM mode to registers only if the device is "S" Variant
	{
			unsigned int regValue;

			regValue = Reg_Map_Cache.Driver_Control_Reg2;

			/* 1XPWM */
			if (value)
			{
				regValue &= ~PWM_MODE_MASK;
				regValue |= PWM_MODE_1X;
			}
			else
			{
				regValue &= ~PWM_MODE_MASK;
			}

			Reg_Map_Cache.Driver_Control_Reg2 = regValue;
			SPI_Write(SPI_REG_DRV_CTRL, Reg_Map_Cache.Driver_Control_Reg2);
	}
}
unsigned char drv83xx_getCtrlTypeParam(void)
{
	return(SensorlessTrapController.PWM_Mode);
}
void drv83xx_setMtrParam(unsigned char num, unsigned long value)
{
	switch(num)
	{
		case MTR_PARAM_IPD_BRAKE_TIME:
			SensorlessTrapController.IPDBrakeTime = *((uint16_t*)&value);
			break;
		case MTR_PARAM_IPD_PULSE_TIME:
			SensorlessTrapController.IPDPulseTime = *((uint16_t*)&value);
			break;
		case MTR_PARAM_IPD_DECAY_CONSTANT:
			SensorlessTrapController.IPDDecayConstant = *((uint16_t*)&value);
			break;
		case MTR_PARAM_ALIGN_SECTOR:
			SensorlessTrapController.AlignSector = *((uint16_t*)&value);
			break;
		case MTR_PARAM_ALIGN_WAIT_TIME:
			SensorlessTrapController.AlignWaitTime = *((uint16_t*)&value);
			break;
		case MTR_PARAM_ACCEL_RATE:
			SensorlessTrapController.AccelRate = *((uint16_t*)&value);
			break;
		case MTR_PARAM_ACCEL_STOP:
			SensorlessTrapController.AccelStop = *((uint16_t*)&value);
			SensorlessTrapController.AccelStop *= 1000;
			break;
		case MTR_PARAM_ACCEL_VEL_INIT:
			SensorlessTrapController.AccelVelocity = *((uint16_t*)&value);
			SensorlessTrapController.AccelVelocity *= 1000;
			break;
		case MTR_PARAM_BEMF_THRESHOLD:
			SensorlessTrapController.BEMFThreshold = *((uint16_t*)&value);
			break;
		case MTR_PARAM_RAMP_RATE_DELAY:
			SensorlessTrapController.RampRateDelay = *((uint16_t*)&value);
			break;
		case MTR_PARAM_COMM_BLANK_TIME:
			SensorlessTrapController.CommutationBlankTime = *((uint16_t*)&value);
			break;
		case MTR_PARAM_PWM_BLANK_COUNTS:
			SensorlessTrapController.PWMBlankCounts = *((uint16_t*)&value);
			break;
		case MTR_PARAM_MAX_DUTY_CYCLE:
			SensorlessTrapController.MaxDutyCycle = *((uint16_t*)&value);
			break;
		case MTR_PARAM_MIN_OFF_DUTY_CYCLE:
			SensorlessTrapController.MinOffDutyCycle = *((uint16_t*)&value);
			break;
		case MTR_PARAM_MIN_ON_DUTY_CYCLE:
			SensorlessTrapController.MinOnDutyCycle = *((uint16_t*)&value);
			break;
		case MTR_PARAM_PWM_FREQ:
			SensorlessTrapController.PWMPeriod = *((uint16_t*)&value);
			SensorlessTrapController.Counter_1M_Second = (25000 / SensorlessTrapController.PWMPeriod);
			SensorlessTrapController.Counter_1_Second = SensorlessTrapController.Counter_1M_Second * 1000;
			TimerB0_Init();				// Timer B0 initialization
			TimerA0_Init();             // Timer A0 Initialization to generate 4 PWM's for switches
			TimerA2_Init();             // Timer A2 Initialization to generate 2 PWM's for switches
			break;
		case MTR_PARAM_UNDER_VOL_LIM:
			SensorlessTrapController.UnderVolLim = *((uint16_t*)&value);
			break;
		case MTR_PARAM_OVER_VOL_LIM:
			SensorlessTrapController.OverVolLim = *((uint16_t*)&value);
			break;
		case MTR_PARAM_STALL_DETECT_REV:
			SensorlessTrapController.StallDetectRev = *((uint16_t*)&value);
			break;
		case MTR_PARAM_STALL_DETECT_TIME:
			SensorlessTrapController.StallDetecttime = *((uint16_t*)&value);
			break;
		case MTR_PARAM_MOTOR_PHASE_CURR_LIM:
			SensorlessTrapController.MotorPhaseCurrentLimit = *((uint16_t*)&value);
			break;
		case MTR_PARAM_AUTO_FAULT_RECOVERY_TIME:
			SensorlessTrapController.AutoFaultRecoveryTime = *((uint16_t*)&value);
			break;
		case MTR_PARAM_ALIGN_IPD:
			SensorlessTrapController.Align_IPD = *((uint16_t*)&value);
			break;
		case MTR_PARAM_DIR:
			if((HostController.Start_Stop_Motor)&&(ApplicationStatus.currentstate != MOTOR_RUN))
			{
				SensorlessTrapController.Direction = !SensorlessTrapController.Direction;
			}
			else
			{
				SensorlessTrapController.Direction_flag =  !SensorlessTrapController.Direction_flag;
			}
			break;
		case MTR_PARAM_SPEED:
			SensorlessTrapController.SetMotorSpeed = *((uint16_t*)&value);
	        break;
		case MTR_START_BEMF_CALCULATION:
			HostController.Calibrate_Motor = *((uint16_t*)&value);
			break;
		case MTR_UNDER_RATING:
			SensorlessTrapController.Motor_Under_Rating = *((uint16_t*)&value);
			break;
		case MTR_RATED_VOLTAGE:
			SensorlessTrapController.Motor_Rated_Voltage = *((uint16_t*)&value);
			break;
		case MTR_RATED_SPEED:
			SensorlessTrapController.Motor_Rated_Speed = *((uint16_t*)&value);
			break;
		case MTR_POLES:
			SensorlessTrapController.Motor_Poles = *((uint16_t*)&value);
			break;
		default:
	        break;
	}
}
unsigned long drv83xx_getMtrParam(unsigned char num)
{
	switch(num)
	{
		case MTR_PARAM_ELEC_SPEED:
			return(SensorlessTrapController.SPDFdbk);
		case MTR_PARAM_FAULT_STATE:
			return(ApplicationStatus.fault);
		case MTR_PARAM_DEVICE_ID:
			return(SensorlessTrapController.DeviceID);
		case MTR_PARAM_IPD_BRAKE_TIME:
			return(SensorlessTrapController.IPDBrakeTime);
		case MTR_PARAM_IPD_PULSE_TIME:
			return(SensorlessTrapController.IPDPulseTime);
		case MTR_PARAM_IPD_DECAY_CONSTANT:
			return(SensorlessTrapController.IPDDecayConstant);
		case MTR_PARAM_ALIGN_SECTOR:
			return(SensorlessTrapController.AlignSector);
		case MTR_PARAM_ALIGN_WAIT_TIME:
			return(SensorlessTrapController.AlignWaitTime);
		case MTR_PARAM_ACCEL_RATE:
			return(SensorlessTrapController.AccelRate);
		case MTR_PARAM_ACCEL_STOP:
			return(ACCEL_STOP);
		case MTR_PARAM_ACCEL_VEL_INIT:
			return(ACCEL_VELOCITY_INIT);
		case MTR_PARAM_BEMF_THRESHOLD:
			return(SensorlessTrapController.BEMFThreshold);
		case MTR_PARAM_RAMP_RATE_DELAY:
			return(SensorlessTrapController.RampRateDelay);
		case MTR_PARAM_COMM_BLANK_TIME:
			return(SensorlessTrapController.CommutationBlankTime);
		case MTR_PARAM_PWM_BLANK_COUNTS:
			return(SensorlessTrapController.PWMBlankCounts);
		case MTR_PARAM_MAX_DUTY_CYCLE:
			return(SensorlessTrapController.MaxDutyCycle);
		case MTR_PARAM_MIN_OFF_DUTY_CYCLE:
			return(SensorlessTrapController.MinOffDutyCycle);
		case MTR_PARAM_MIN_ON_DUTY_CYCLE:
			return(SensorlessTrapController.MinOnDutyCycle);
		case MTR_PARAM_PWM_FREQ:
			return(SensorlessTrapController.PWMPeriod);
		case MTR_PARAM_UNDER_VOL_LIM:
			return(SensorlessTrapController.UnderVolLim);
		case MTR_PARAM_OVER_VOL_LIM:
			return(SensorlessTrapController.OverVolLim);
		case MTR_PARAM_STALL_DETECT_REV:
			return(SensorlessTrapController.StallDetectRev);
		case MTR_PARAM_STALL_DETECT_TIME:
			return(SensorlessTrapController.StallDetecttime);
		case MTR_PARAM_MOTOR_PHASE_CURR_LIM:
			return(SensorlessTrapController.MotorPhaseCurrentLimit);
		case MTR_PARAM_AUTO_FAULT_RECOVERY_TIME:
			return(SensorlessTrapController.AutoFaultRecoveryTime);
		case MTR_PARAM_ALIGN_IPD:
			return(SensorlessTrapController.Align_IPD);
		case MTR_PARAM_DIR:
			return(SensorlessTrapController.Direction);
		case MTR_PARAM_SPEED:
			return(SensorlessTrapController.SetMotorSpeed);
		case MTR_START_STOP_MOTOR:
			return(HostController.Start_Stop_Motor);
		case MTR_START_BEMF_CALCULATION:
			return(HostController.Calibrate_Motor);
		case MTR_UNDER_RATING:
			return(SensorlessTrapController.Motor_Under_Rating);
		case MTR_RATED_VOLTAGE:
			return(SensorlessTrapController.Motor_Rated_Voltage);
		case MTR_RATED_SPEED:
			return(SensorlessTrapController.Motor_Rated_Speed);
		case MTR_POLES:
			return(SensorlessTrapController.Motor_Poles);
        case DEVICE_FULL_SCALE_VOLTAGE:
        	return FULL_SCALE_VOLTAGE;
		default:
	        return(0);
	}
}


unsigned char drv83xx_getGPIO(unsigned char gpioPort, unsigned char gpioNum)
{
    if (gpioPort == 0x01)
    {
        if ((gpioNum & P1IN) == 0)
        {
        	HostController.EnabledGateDrivers = 0;
            return 0;
        }
        else
        {
        	HostController.EnabledGateDrivers = 1;
            return 1;
        }
    }
    else
    {
        // do nothing
        return 0;
    }
}
void drv83xx_setGPIO(unsigned char gpioPort, unsigned char gpioNum, unsigned char gpioVal)
{
	    if (gpioPort == 0x01)
	    {
	        if(gpioVal == 0)
	        {
	            P1OUT &= ~gpioNum;
	            HostController.EnabledGateDrivers = 0;
				SensorlessTrapController.GateToggleFaultDetect = TRUE;     // Set the Gate toggle fault detect for 10ms
				SensorlessTrapController.GateToggleCounter = 0;
	        }
	        else
	        {
	        	P1OUT |= gpioNum;
	            HostController.EnabledGateDrivers = 1;
				drv83xx_regRestoreFromCache();                      // Default values are read by GUI after enabling the pre drivers
				SensorlessTrapController.GateToggleFaultDetect = TRUE;     // Set the Gate toggle fault detect for 10ms
				SensorlessTrapController.GateToggleCounter = 0;
	        }
	    }
	    else
	    {
	        // do nothing
	    }

}


/*function
 * IPD_SetState(uint8_t hallState)
 * Set IPD State
 * INPUT: commutation  state
 * */
void IPD_SetState(uint8_t commState)
{
	if(SensorlessTrapController.PWM_Mode == 0)  // If six PWM mode
	{
		P2OUT &= ~BIT5;         // Turn High Side A phase Off
		P1OUT &= ~BIT5;         // Turn High Side B phase Off
		P1OUT &= ~BIT3;         // Turn High Side C phase Off
		P2OUT &= ~BIT4;         // Turn Low Side A phase Off
		P1OUT &= ~BIT4;         // Turn Low Side B phase Off
		P1OUT &= ~BIT2;         // Turn Low Side C phase Off
		switch(commState)
		{
		case 1:        /* B-C */

			P1OUT |= BIT5;                                                   /* Set High side of B phase*/
			P1OUT |= BIT2;                                                                          /* Set Low side of C phase */

			break;

		case 2:       /* C-A */

			P1OUT |= BIT3;                                                                          /* Set High side of C phase*/
			P2OUT |= BIT4;                                                                          /* Set Low side of A phase */

			break;

		case 3:        /* A-B */

			P2OUT |= BIT5;                                                                          /* Set High side of A phase*/
			P1OUT |= BIT4;                                                                          /* Set Low side of B phase */

			break;
		case 4:            /* B-A */

			P1OUT |= BIT5;                                                   /* Set High side of B phase*/
			P2OUT |= BIT4;                                                                          /* Set Low side of A phase */
			break;

		case 5:       /* A-C */

			P2OUT |= BIT5;                                                                          /* Set High side of A phase*/
			P1OUT |= BIT2;                                                                          /* Set Low side of C phase */

			break;

		case 6:       /* C-B */

			P1OUT |= BIT3;                                                  /* Set High side of C phase*/
			P1OUT |= BIT4;                                                                          /* Set Low side of B phase */

			break;
		default:
			break;
		}
	}
	else  // If 1x PWM mode
	{
		P2SEL &= ~(BIT4 | BIT5);
		P1SEL &= ~(BIT4 | BIT5);        /* Select P1.4 , P1.5 as I/O Function for Phase B*/
		P1SEL &= ~(BIT2 | BIT3);        /* Select P1.2 , P1.3 as I/O Function for Phase C*/                                                                    /* Select P2.4  I/O Function for Phase A*/
		P2OUT |= BIT5;   // Set 100% duty cycle to turn on switches
		P2OUT &= ~BIT4 ; // Make INLA as GPIO low to switch of phase A switches
		P1OUT &= ~BIT5 ; //  Make INHB as GPIO low to switch of phase B switches
		P1OUT &= ~BIT4 ; //  Make INLB as GPIO low to switch of phase C switches
		P1OUT |= BIT2  ; //  Make INLC as GPIO high to remove the brakes

		switch(commState)
	    {
	    case 1:        /* B-C */

	    	P2OUT |= BIT4;
	        P1OUT |= BIT5;
	        break;

	    case 2:        /* C-A */
	    	P1OUT |= BIT4;
	    	P1OUT |= BIT5;

	        break;

	    case 3:        /* A-B */
	    	P2OUT |= BIT4;
	    	P1OUT |= BIT4;
	        break;

	    case 4:            /* B-A */
	    	P1OUT |= BIT5;

	        break;

	    case 5:       /* A-C */
	    	P2OUT |= BIT4;

	        break;

	    case 6:         /* C-B */
	    	P1OUT |= BIT4;
	        break;
	    default:
	    	break;
	    }

	}
}

/*function
 * PWM_SetCommutation(uint8_t hallState)
 * Set PWM commutation
 * INPUT: Hall state
 * */
void PWM_SetCommutation(uint8_t commState)
{

	if(SensorlessTrapController.PWM_Mode == 0)
	{
	    /* Implementing Synchronous PWM i.e. to Toggle between High side and low side of a Phase with Dead Band*/

	    switch(commState)
	    {
	    case 1:        /* B-C */

	    	/* Reset switches for phase A (LOW-HIGH) */
		    P2OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P2.4 , P2.5  */
		    P2SEL &= ~(BIT4 | BIT5);                                                                    /* Select P2.4 , P2.5 as I/O Function for Phase A*/

		    /* Reset switches for phase C (HIGH)*/
		    P1OUT &= ~( BIT3);                                    /* Reset bits P1.2 , P1.3  */
		    P1SEL &= ~( BIT3);                                                                    /* Select P1.2 , P1.3 as I/O Function for Phase C*/


	        P1SEL |= BIT4 | BIT5;                                   /* Select Synchronous PWM for B phase*/
	        P1OUT |= BIT2;                                                                          /* Set Low side of C phase */
	        SensorlessTrapController.RotationCount++;
	        break;

	    case 2:        /* A-C */

	    	/* Reset switches for phase B (LOW-HIGH)*/
		    P1OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P1.4 , P1.5  */
		    P1SEL &= ~(BIT4 | BIT5);                                                                    /* Select P1.4 , P1.5 as I/O Function for Phase B*/

		    /* Reset switches for phase C (HIGH)*/
		    P1OUT &= ~(BIT3);                                    /* Reset bits P1.2 , P1.3  */
		    P1SEL &= ~(BIT3);                                                                    /* Select P1.2 , P1.3 as I/O Function for Phase C*/

	    	P2SEL |= BIT4 | BIT5;                                                           /* Select Synchronous PWM for of A phase*/
	        P1OUT |= BIT2;                                                                          /* Set Low side of C phase */
	        break;

	    case 3:        /* A-B */

	    	/* Reset switches for phase B (HIGH)*/
		    P1OUT &= ~(BIT5);                                    /* Reset bits P1.4 , P1.5  */
		    P1SEL &= ~(BIT5);                                                                    /* Select P1.4 , P1.5 as I/O Function for Phase B*/

		    /* Reset switches for phase C (LOW-HIGH)*/
		    P1OUT &= ~(BIT2 | BIT3);                                    /* Reset bits P1.2 , P1.3  */
		    P1SEL &= ~(BIT2 | BIT3);                                                                    /* Select P1.2 , P1.3 as I/O Function for Phase C*/


	    	P2SEL |= BIT4 | BIT5;                                                           /* Select Synchronous PWM for A phase*/
	        P1OUT |= BIT4;                                                                          /* Set Low side of B phase */
	        break;

	    case 4:            /* C-B */

	    	/* Reset switches for phase A (LOW-HIGH) */

	    	P2OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P2.4 , P2.5  */
		    P2SEL &= ~(BIT4 | BIT5);                                                                    /* Select P2.4 , P2.5 as I/O Function for Phase A*/

		    /* Reset switches for phase B (HIGH)*/
		    P1OUT &= ~(BIT5);                                    /* Reset bits P1.4 , P1.5  */
		    P1SEL &= ~(BIT5);                                                                    /* Select P1.4 , P1.5 as I/O Function for Phase B*/


	    	P1SEL |= BIT2 | BIT3;                                   /* Select Synchronous PWM for C phase*/
	        P1OUT |= BIT4;                                                                          /* Set Low side of B phase */
	        break;

	    case 5:       /* C-A */

	    	/* Reset switches for phase A (HIGH) */

	    	P2OUT &= ~(BIT5);                                    /* Reset bits P2.4 , P2.5  */
		    P2SEL &= ~(BIT5);                                                                    /* Select P2.4 , P2.5 as I/O Function for Phase A*/

		    /* Reset switches for phase B (LOW-HIGH)*/
		    P1OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P1.4 , P1.5  */
		    P1SEL &= ~(BIT4 | BIT5);                                                                    /* Select P1.4 , P1.5 as I/O Function for Phase B*/

	    	P1SEL |= BIT2 | BIT3;                                                           /* Select Synchronous PWM for C phase*/
	        P2OUT |= BIT4;                                                                          /* Set Low side of A phase */

	        break;

	    case 6:         /* B-A */

	    	/* Reset switches for phase A (HIGH) */

	    	P2OUT &= ~(BIT5);                                    /* Reset bits P2.4 , P2.5  */
		    P2SEL &= ~(BIT5);                                                                    /* Select P2.4 , P2.5 as I/O Function for Phase A*/

		    /* Reset switches for phase C (LOW-HIGH)*/
		    P1OUT &= ~(BIT2 | BIT3);                                    /* Reset bits P1.2 , P1.3  */
		    P1SEL &= ~(BIT2 | BIT3);                                                                    /* Select P1.2 , P1.3 as I/O Function for Phase C*/


	    	P1SEL |= BIT4 | BIT5;                                                           /* Select Synchronous PWM for B phase*/
	        P2OUT |= BIT4;                                                                          /* Set Low side of A phase */

	        break;
	    default:
	    	DisableGateDrivers();
	    	break;
	    }
	}
	else
	{
	    switch(commState)
	    {
	    case 1:        /* B-C */

			P1SEL &= ~(BIT4);        /* Select P1.4 , P1.5 as I/O Function for Phase B*/
			P1SEL &= ~(BIT2 | BIT3);        /* Select P1.2 , P1.3 as I/O Function for Phase C*/                                                                    /* Select P2.4  I/O Function for Phase A*/
			P2SEL |= BIT5  ;  // Generate Single PWM for modulating duty cycle and frequency

			P1OUT &= ~BIT4 ; //  Make INLB as GPIO low to switch of phase C switches
			P1OUT |= BIT2  ; //  Make INLC as GPIO low to remove the brakes

	    	P2OUT |= BIT4;
	        P1OUT |= BIT5;

	        SensorlessTrapController.RotationCount++;
	        break;

	    case 2:        /* A-C */

			P1SEL &= ~(BIT4 | BIT5);        /* Select P1.4 , P1.5 as I/O Function for Phase B*/
			P1SEL &= ~(BIT2 | BIT3);        /* Select P1.2 , P1.3 as I/O Function for Phase C*/                                                                    /* Select P2.4  I/O Function for Phase A*/
			P2SEL |= BIT5  ;  // Generate Single PWM for modulating duty cycle and frequency

			P1OUT &= ~BIT5 ; //  Make INHB as GPIO low to switch of phase B switches
			P1OUT &= ~BIT4 ; //  Make INLB as GPIO low to switch of phase C switches
			P1OUT |= BIT2  ; //  Make INLC as GPIO low to remove the brakes

	    	P2OUT |= BIT4;
	        break;

	    case 3:        /* A-B */

			P1SEL &= ~(BIT5);        /* Select P1.4 , P1.5 as I/O Function for Phase B*/
			P1SEL &= ~(BIT2 | BIT3);        /* Select P1.2 , P1.3 as I/O Function for Phase C*/                                                                    /* Select P2.4  I/O Function for Phase A*/
			P2SEL |= BIT5  ;  // Generate Single PWM for modulating duty cycle and frequency

			P1OUT &= ~BIT5 ; //  Make INHB as GPIO low to switch of phase B switches
			P1OUT |= BIT2  ; //  Make INLC as GPIO low to remove the brakes

	    	P2OUT |= BIT4;
	    	P1OUT |= BIT4;
	        break;

	    case 4:            /* C-B */

			P2SEL &= ~BIT4;
			P1SEL &= ~(BIT5);        /* Select P1.4 , P1.5 as I/O Function for Phase B*/
			P1SEL &= ~(BIT2 | BIT3);        /* Select P1.2 , P1.3 as I/O Function for Phase C*/                                                                    /* Select P2.4  I/O Function for Phase A*/
			P2SEL |= BIT5  ;  // Generate Single PWM for modulating duty cycle and frequency

			P2OUT &= ~BIT4 ; // Make INLA as GPIO low to switch of phase A switches
			P1OUT &= ~BIT5 ; //  Make INHB as GPIO low to switch of phase B switches
			P1OUT |= BIT2  ; //  Make INLC as GPIO low to remove the brakes

	    	P1OUT |= BIT4;
	        break;

	    case 5:       /* C-A */

			P2SEL &= ~BIT4;
			P1SEL &= ~(BIT2 | BIT3);        /* Select P1.2 , P1.3 as I/O Function for Phase C*/                                                                    /* Select P2.4  I/O Function for Phase A*/
			P2SEL |= BIT5  ;  // Generate Single PWM for modulating duty cycle and frequency

			P2OUT &= ~BIT4 ; // Make INLA as GPIO low to switch of phase A switches
			P1OUT |= BIT2  ; //  Make INLC as GPIO low to remove the brakes

	    	P1OUT |= BIT4;
	    	P1OUT |= BIT5;
	        break;

	    case 6:         /* B-A */

			P2SEL &= ~BIT4;
			P1SEL &= ~(BIT4);        /* Select P1.4 , P1.5 as I/O Function for Phase B*/
			P1SEL &= ~(BIT2 | BIT3);        /* Select P1.2 , P1.3 as I/O Function for Phase C*/                                                                    /* Select P2.4  I/O Function for Phase A*/
			P2SEL |= BIT5  ;  // Generate Single PWM for modulating duty cycle and frequency

			P2OUT &= ~BIT4 ; // Make INLA as GPIO low to switch of phase A switches
			P1OUT &= ~BIT4 ; //  Make INLB as GPIO low to switch of phase C switches
			P1OUT |= BIT2  ; //  Make INLC as GPIO low to remove the brakes

	    	P1OUT |= BIT5;
	        break;
	    default:
	    	DisableGateDrivers();
	    	break;

	    }
	}
}

/*function
 * SetPWMDutyCycle(UINT16 PWMDutyCycle)
 * Sets the PWM Duty Cycle
 * INPUT: DutyCycle Counter Value
 * */
void SetPWMDutyCycle(uint16_t PWMDutyCycle)
{ //PWM_BLANK_COUNTS is the time before the falling PWM edge for ADC sampling
    TA0CCR1 = PWMDutyCycle;
    TA0CCR2 = PWMDutyCycle;
    TA0CCR3 = PWMDutyCycle;
    TA0CCR4 = PWMDutyCycle;
    TA2CCR1 = PWMDutyCycle;
    TA2CCR2 = PWMDutyCycle;
    TB0CCR1 = ((SensorlessTrapController.CurrentDutyCycle >>2) - (SensorlessTrapController.PWMBlankCounts));
}

/*function
 * UpdateNextCommutation()
 * Increment or decrement commutation based on direction
 * */
void UpdateNextCommutation(void)
{

	if(SensorlessTrapController.Direction == TRUE)
    {
        SensorlessTrapController.CurrentCommState++;
        if(SensorlessTrapController.CurrentCommState > 6)
        {
            SensorlessTrapController.CurrentCommState = 1;
        }
    }
    else
    {
        SensorlessTrapController.CurrentCommState--;
        if(SensorlessTrapController.CurrentCommState < 1)
        {
            SensorlessTrapController.CurrentCommState = 6;
        }
    }
	if((SensorlessTrapController.CurrentCommState ==1) && (ApplicationStatus.currentstate == MOTOR_RUN))
	{
		ReadSPDFDBK();
	}
}

/*function
 * UpdateBEMFADC
 * Selects the floating phase to be monitored based on commutation states
 * */
void UpdateBEMFADC(void)
{
    switch(SensorlessTrapController.CurrentCommState)
    {
    case 1: ADC12MCTL0 = ADC12INCH_0 | ADC12EOS;               //   channel = A0 (Read the BEMF reading from Phase A ) ,
        break;
    case 2: ADC12MCTL0 = ADC12INCH_1 | ADC12EOS;               //   channel = A1 (Read the BEMF reading from Phase B ) ,
        break;
    case 3: ADC12MCTL0 = ADC12INCH_2 | ADC12EOS;               //   channel = A2 (Read the BEMF reading from Phase C ) ,
        break;
    case 4: ADC12MCTL0 = ADC12INCH_0 | ADC12EOS;               //   channel = A0 (Read the BEMF reading from Phase A ) ,
        break;
    case 5: ADC12MCTL0 = ADC12INCH_1 | ADC12EOS;               //   channel = A1 (Read the BEMF reading from Phase B ) ,
        break;
    case 6: ADC12MCTL0 = ADC12INCH_2 | ADC12EOS;               //   channel = A2 (Read the BEMF reading from Phase C ) ,
        break;
    default:
        break;
    }
}

/*function
 * FastReadBEMF
 * Triggers ADC and Samples BEMF of all three Phases
 * is the speed input (potentiometer) for the applications
 * */
void FastReadBEMF(void)
{
    ADC12CTL0 |= ADC12ENC;                                                      // Enable Conversions
    ADC12CTL0 |= ADC12SC;                                                       // Start sampling of channels
    while(ADC12CTL1 & ADC12BUSY_L)
    {
    }
    ;

    ADC12CTL0 &= ~ADC12ENC;                                                      // End sampling of channels
    ADC12CTL0 &= ~ADC12SC;                                                      // Disable conversions

    SensorlessTrapController.GetBEMF = ADC12MEM0 & 0x0FFF;
    /* Filter only last 12 bits */;
    SensorlessTrapController.GetBEMF >>= PWM_FACTOR;            /*12 bit ADC result has to be scaled to 10 bit value */
}

/*function
 * ReadVCC()
 * Triggers ADC and samples supply voltage and evaluates for over or under voltage fault
 * */
void ReadVCC()
{
	ADC12MCTL0 = ADC12INCH_5 + ADC12EOS;         //   channel = A5 (Read the supply voltage)
    ADC12CTL0 |= ADC12ENC;                                                      // Enable Conversions
    ADC12CTL0 |= ADC12SC;                                                       // Start sampling of channels
    while(ADC12CTL1 & ADC12BUSY_L)
    {
    }
    ;

    ADC12CTL0 &= ~ADC12ENC;                                                             // End sampling of channels
    ADC12CTL0 &= ~ADC12SC;                                                      // Disable conversions

    SensorlessTrapController.VCCvoltage = ADC12MEM0 & 0x0FFF;       /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/
    SensorlessTrapController.VCCvoltage >>= PWM_FACTOR;                         /*12 bit ADC result has to be scaled to 10 bit value */
    SensorlessTrapController.CTvoltage =
        (SensorlessTrapController.VCCvoltage >> 1);                                         // Center tap voltage is VCC/2 , used in BEMF integration calculation

    if(SensorlessTrapController.VCCvoltage <
    		SensorlessTrapController.UnderVolLim | SensorlessTrapController.VCCvoltage >
    SensorlessTrapController.OverVolLim)                                                                                                       /* Under Voltage of 10.0V and over voltage at 20.0V*/
    {
        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        ApplicationStatus.currentstate = FAULT;
        ApplicationStatus.fault = VOLTAGE;
    }
    if(SensorlessTrapController.VCCvoltage <
    		SensorlessTrapController.MinPowerSupply)                                                                                                       /* Under Voltage of 10.0V and over voltage at 20.0V*/
       {
           ApplicationStatus.previousstate = ApplicationStatus.currentstate;
           ApplicationStatus.currentstate = FAULT;
           ApplicationStatus.fault = POWER_SUPPLY;
       }
}


/*function
 * ReadVCC2()
 * Triggers ADC and samples the DeviceID pin for specific voltage subranges (188mV for DRV8350S, 350mV for DRV8350H
 * */
void ReadVCC2()
{
    ADC12MCTL0 = ADC12INCH_4 + ADC12EOS;         //   channel = A4 (Read the deviceID)
    ADC12CTL0 |= ADC12ENC;                                                      // Enable Conversions
    ADC12CTL0 |= ADC12SC;                                                       // Start sampling of channels
    while(ADC12CTL1 & ADC12BUSY_L)
    {
    }
    ;

    ADC12CTL0 &= ~ADC12ENC;                                                             // End sampling of channels
    ADC12CTL0 &= ~ADC12SC;                                                      // Disable conversions

    SensorlessTrapController.deviceIDADC = ADC12MEM0 & 0x0FFF;       /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/
    SensorlessTrapController.deviceIDADC >>= PWM_FACTOR;                         /*12 bit ADC result has to be scaled to 10 bit value */
}

/*function
 * SetMotorSpeed()
 * Reads the Input motor speed from the GUI to set the Target duty cycle
 * */
void SetMotorSpeed(void)
{
	if(SensorlessTrapController.Calibrate_Motor_run)
	{
	    SensorlessTrapController.TargetDutyCycle = (SensorlessTrapController.PWMPeriod >> 1);   // Update the Duty cycle with Motor speed set from the MDBU serial
	}
	else
	{
		SensorlessTrapController.TargetDutyCycle = SensorlessTrapController.SetMotorSpeed;   // Update the Duty cycle with Motor speed set from the MDBU serial
	}
	if(SensorlessTrapController.TargetDutyCycle >= SensorlessTrapController.MaxDutyCycle)
    {
        SensorlessTrapController.TargetDutyCycle = SensorlessTrapController.MaxDutyCycle;
    }
    else if((SensorlessTrapController.CurrentDutyCycle <= SensorlessTrapController.MinOffDutyCycle) &&
            (ApplicationStatus.currentstate != MOTOR_IDLE))
    {
        //ensure the target duty cycle stays above minimum or shut it off
        if((SensorlessTrapController.TargetDutyCycle <= SensorlessTrapController.CurrentDutyCycle) && (!HostController.Calibrate_Motor))
        {
        	ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        	ApplicationStatus.currentstate = MOTOR_STOP;
        }
    }
    if(SensorlessTrapController.TargetDutyCycle !=
       SensorlessTrapController.CurrentDutyCycle)
    {                                                                                                                                                   //set a flag is the speed has changed
        SensorlessTrapController.SpeedChange = TRUE;
    }
}

/*function
 * ReadCurrentshunt()
 * Reads CSA value and triggers OC faults for Motor current greater than Set Limit
 * */
void ReadCurrentShunt()
{
    ADC12MCTL0 = ADC12INCH_12 + ADC12EOS;               //   channel = A4 (Read the CSA reading from Phase A for over current protection) , End of Sequence
    ADC12CTL0 |= ADC12ENC;                                                      // Enable Conversions
    ADC12CTL0 |= ADC12SC;                                                       // Start sampling of channels
    while(ADC12CTL1 & ADC12BUSY_L)
    {
    }
    ;
    SensorlessTrapController.MotorPhaseCurrent = ADC12MEM0 & 0x0FFF;       /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/

    ADC12CTL0 &= ~ADC12ENC;                                                             // Start sampling of channels
    ADC12CTL0 &= ~ADC12SC;                                                      // Start sampling of channels

    if(SensorlessTrapController.ShuntVariant == TRUE)    // If Current sense is in phase shunt remove the offset.
    {
    	SensorlessTrapController.MotorPhaseCurrent -= 2048;     // subtracting the bias, Vref/2 is added as bias voltage to support bidirectional current sensing
    }
    SensorlessTrapController.MotorPhaseCurrent = abs(
    SensorlessTrapController.MotorPhaseCurrent);
    if((SensorlessTrapController.MotorPhaseCurrent >
       SensorlessTrapController.MotorPhaseCurrentLimit) && (ApplicationStatus.fault == NOFAULT))                                                  /* Motor Phase Current Limit*/
    {
        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        ApplicationStatus.currentstate = FAULT;
        ApplicationStatus.fault = OVERCURRENT;
    }
}

/*function
 * ISCReadPhaseVoltage()
 * Triggers ADC and samples 3 phase voltages and evaluates if motor is already spinning
 * */
void ISCReadPhaseVoltage()
{
    ReadVCC();                                                                                      // Read Supply Voltage Vcc
    ADC12MCTL0 = ADC12INCH_0;                                   //   channel = A0 (Read the Phase A Voltage)
    ADC12MCTL1 = ADC12INCH_1;                                   //   channel = A1 (Read the Phase B Voltage)
    ADC12MCTL2 = ADC12INCH_2 + ADC12EOS;                        //   channel = A2 (Read the Phase C Voltage)

    ADC12CTL0 |= ADC12MSC | ADC12ENC;                           // Enable Conversions and sample multi channels
    ADC12CTL0 |= ADC12SC;                                       // Start sampling of channels
    while(!(ADC12IFG & BIT2))
    {
    }
    ;                                                           // wait for the conversion of all channels

    ADC12CTL0 &= ~ADC12ENC;                             // End sampling of channels
    ADC12CTL0 &= ~ADC12MSC;                             // End  Multiple channels sampling
    ADC12CTL0 &= ~ADC12SC;                              // Disable conversions

    SensorlessTrapController.PhaseA_BEMF = ADC12MEM0 & 0x0FFF;       /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/
    SensorlessTrapController.PhaseB_BEMF = ADC12MEM1 & 0x0FFF;       /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/
    SensorlessTrapController.PhaseC_BEMF = ADC12MEM2 & 0x0FFF;       /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/

    SensorlessTrapController.LineAB_BEMF = abs(SensorlessTrapController.PhaseA_BEMF - SensorlessTrapController.PhaseB_BEMF);
	SensorlessTrapController.LineBC_BEMF = abs(SensorlessTrapController.PhaseB_BEMF - SensorlessTrapController.PhaseC_BEMF);
	SensorlessTrapController.LineCA_BEMF = abs(SensorlessTrapController.PhaseC_BEMF - SensorlessTrapController.PhaseA_BEMF);

}

/*function
 * ReadSPDFDBK()
 * This function captures the value from Timer to Calculate electrical speed in Timer counts
 * */
void ReadSPDFDBK()
{
	//RPM count & Speed measurement of 18 Bit resolution

	if(SensorlessTrapController.TimerOverflowFlag == 0) // A count of 1 indicates timer A1 ticked for 0x010000 clocks
	{
		SensorlessTrapController.SPDFdbk = TA1R;  // Generate the speed count = The max value 0FFFF represents the least possible speed measurement which is 625K / 0x0FFFF = 9.52hz as  timer 1 is run at 625KHz
	}
	SensorlessTrapController.TimerOverflowFlag = 0 ; //Reset the Interrupt falg
	TA1R = 0x0000;				// Reset the timer count
}
/*function
 * CommutationStateDetect()
 * This function compares the 3phase BEMF values to identify the Commutation state and also Measures the BEMF peak to peak value in given timer counts
 * */
inline void CommutationStateDetect()
{
		if(SensorlessTrapController.LineAB_BEMF <= ISC_MIN_LINE_BEMF)                                                // If V(a) ~= V(b) , commutation instance , Assuming 3 bit ADC error
	    {
			if(SensorlessTrapController.PhaseA_BEMF > SensorlessTrapController.PhaseC_BEMF)                // if V(a) > V(c)
			{
				if(SensorlessTrapController.Direction)                                                     // Phase A BEMF should be increasing
				{
					SensorlessTrapController.ISC_Current_State = 1;
					SensorlessTrapController.ISC_Next_State = 2;
				}
				else																					  // Phase B BEMF should be increasing
				{
					SensorlessTrapController.ISC_Current_State = 2;
					SensorlessTrapController.ISC_Next_State = 1;
				}
			}
			else                                                                                          // if V(a) < V(c)
			{
				if(SensorlessTrapController.Direction)                                                    // Phase A BEMF should be decreasing
				{
					SensorlessTrapController.ISC_Current_State = 4;
					SensorlessTrapController.ISC_Next_State = 5;
				}
				else																					 // Phase B BEMF should be decreasing
				{
					SensorlessTrapController.ISC_Current_State = 5;
					SensorlessTrapController.ISC_Next_State = 4;
				}
		    }
			SensorlessTrapController.ISC_Current_BEMF = SensorlessTrapController.PhaseA_BEMF;
			if((abs(SensorlessTrapController.PhaseA_BEMF - SensorlessTrapController.VCCvoltage) <=7) || (SensorlessTrapController.PhaseA_BEMF <=7))
			{
				SensorlessTrapController.ISC_BEMF_Clamped = TRUE;     // If BEMF is observed as Camel hump shaped or McD shaped Waveform implicated BEMF is calmped to nuetral or Vcc
			}
			if(SensorlessTrapController.ISC_Current_State != SensorlessTrapController.ISC_Prev_State)   // If the current state is different with previous state
			{
				SensorlessTrapController.ISC_BEMF_Diff[SensorlessTrapController.ISC_Current_State] = abs(SensorlessTrapController.ISC_Prev_BEMF - SensorlessTrapController.ISC_Current_BEMF);
				SensorlessTrapController.ISC_Prev_BEMF = SensorlessTrapController.ISC_Current_BEMF;
				SensorlessTrapController.ISC_Motor_Speed[SensorlessTrapController.ISC_Current_State] = SensorlessTrapController.BEMF_Counter;       // Motor speed is captured based on PWM counts between ISC State 1 & 2
				SensorlessTrapController.ISC_Prev_State = SensorlessTrapController.ISC_Current_State;   // Save the current state as previous state
    			SensorlessTrapController.ISC_Phase_Match++;
    		    SensorlessTrapController.ISC_Phase_Match_Flag = TRUE;
    			SensorlessTrapController.BEMF_Counter = 0;
    		}
	    }
		else if (SensorlessTrapController.LineBC_BEMF <= ISC_MIN_LINE_BEMF)                                                // If V(b) ~= V(c) , commutation instance , Assuming 3 bit ADC error
	    {
			if(SensorlessTrapController.PhaseB_BEMF > SensorlessTrapController.PhaseA_BEMF)                // if V(b),V(c) > V(a)
			{
				if(SensorlessTrapController.Direction)                                                     // Phase B BEMF should be increasing
				{
					SensorlessTrapController.ISC_Current_State = 5;
					SensorlessTrapController.ISC_Next_State = 6;
				}
				else																					  // Phase C BEMF should be increasing
				{
					SensorlessTrapController.ISC_Current_State = 6;
					SensorlessTrapController.ISC_Next_State = 5;
				}
			}
			else                                                                                          // if V(b), V(c) < V(a)
			{
				if(SensorlessTrapController.Direction)                                                    // Phase B BEMF should be decreasing
				{
					SensorlessTrapController.ISC_Current_State = 2;
					SensorlessTrapController.ISC_Next_State = 3;
				}
				else																					 // Phase C BEMF should be decreasing
				{
					SensorlessTrapController.ISC_Current_State = 3;
					SensorlessTrapController.ISC_Next_State = 2;
				}
			}
			SensorlessTrapController.ISC_Current_BEMF = SensorlessTrapController.PhaseB_BEMF;
			if((abs(SensorlessTrapController.PhaseB_BEMF - SensorlessTrapController.VCCvoltage) <=7) || (SensorlessTrapController.PhaseB_BEMF <=7))
			{
				SensorlessTrapController.ISC_BEMF_Clamped = TRUE;     // If BEMF is observed as Camel hump shaped or McD shaped Waveform implicated BEMF is calmped to nuetral or Vcc
			}
			if(SensorlessTrapController.ISC_Current_State != SensorlessTrapController.ISC_Prev_State)   // If the current state is different with previous state
			{
				SensorlessTrapController.ISC_BEMF_Diff[SensorlessTrapController.ISC_Current_State] = abs(SensorlessTrapController.ISC_Prev_BEMF - SensorlessTrapController.ISC_Current_BEMF);
				SensorlessTrapController.ISC_Prev_BEMF = SensorlessTrapController.ISC_Current_BEMF;
				SensorlessTrapController.ISC_Motor_Speed[SensorlessTrapController.ISC_Current_State] = SensorlessTrapController.BEMF_Counter;       // Motor speed is captured based on PWM counts between ISC State 1 & 2
				SensorlessTrapController.ISC_Prev_State = SensorlessTrapController.ISC_Current_State;   // Save the current state as previous state
    			SensorlessTrapController.ISC_Phase_Match++;
    		    SensorlessTrapController.ISC_Phase_Match_Flag = TRUE;
    			SensorlessTrapController.BEMF_Counter = 0;
    		}
	    }
		else if (SensorlessTrapController.LineCA_BEMF <= ISC_MIN_LINE_BEMF)                                                // If V(c) ~= V(a) , commutation instance , Assuming 3 bit ADC error
	    {
			if(SensorlessTrapController.PhaseA_BEMF > SensorlessTrapController.PhaseB_BEMF)				   //if V(a), V(c) > V(b)
			{
				if(SensorlessTrapController.Direction)                                                     // Phase C BEMF should be increasing
				{
					SensorlessTrapController.ISC_Current_State = 3;
					SensorlessTrapController.ISC_Next_State = 4;
				}
				else																					  // Phase A BEMF should be increasing
				{
					SensorlessTrapController.ISC_Current_State = 4;
					SensorlessTrapController.ISC_Next_State = 3;
				}
			}
			else                                                                                          // if V(a), V(c) < V(b)
			{
				if(SensorlessTrapController.Direction)                                                    // Phase C BEMF should be decreasing
				{
					SensorlessTrapController.ISC_Current_State = 6;
					SensorlessTrapController.ISC_Next_State = 1;
				}
				else																					 // Phase A BEMF should be decreasing
				{
					SensorlessTrapController.ISC_Current_State = 1;
					SensorlessTrapController.ISC_Next_State = 6;
				}
			}
			SensorlessTrapController.ISC_Current_BEMF = SensorlessTrapController.PhaseC_BEMF;
			if((abs(SensorlessTrapController.PhaseC_BEMF - SensorlessTrapController.VCCvoltage) <=7) || (SensorlessTrapController.PhaseC_BEMF <=7))
			{
				SensorlessTrapController.ISC_BEMF_Clamped = TRUE;     // If BEMF is observed as Camel hump shaped or McD shaped Waveform implicated BEMF is calmped to nuetral or Vcc
			}
			if(SensorlessTrapController.ISC_Current_State != SensorlessTrapController.ISC_Prev_State)   // If the current state is different with previous state
				{
				SensorlessTrapController.ISC_BEMF_Diff[SensorlessTrapController.ISC_Current_State] = abs(SensorlessTrapController.ISC_Prev_BEMF - SensorlessTrapController.ISC_Current_BEMF);
				SensorlessTrapController.ISC_Prev_BEMF = SensorlessTrapController.ISC_Current_BEMF;
				SensorlessTrapController.ISC_Motor_Speed[SensorlessTrapController.ISC_Current_State] = SensorlessTrapController.BEMF_Counter;       // Motor speed is captured based on PWM counts between ISC State 1 & 2
				SensorlessTrapController.ISC_Prev_State = SensorlessTrapController.ISC_Current_State;   // Save the current state as previous state
    		    SensorlessTrapController.ISC_Phase_Match_Flag = TRUE;
    			SensorlessTrapController.ISC_Phase_Match++;
    			SensorlessTrapController.BEMF_Counter = 0;
    		}
	    }
}
/*function
 * calculateBEMFThreshold()
 * This Function calculates the BEMF threshold of the motor by calibrating the motor by Accelerating and then spinning in Integration mode in both forward and reverse directions
 * */
inline void calculateBEMFThreshold()
{
	if(SensorlessTrapController.Calibrate_Motor_run == FALSE)     		// First time Caibrate in Open loop Forward direction
	{
        SensorlessTrapController.Calibrate_Motor_Start = TRUE;
		SensorlessTrapController.Calibrate_Motor_run = TRUE;
		SensorlessTrapController.BEMFThreshold = (SensorlessTrapController.ISC_Motor_Speed[7] * SensorlessTrapController.ISC_BEMF_Diff[7]) >> 3;
	}
	else if(SensorlessTrapController.Calibrate_Motor_Direction == FALSE) // Second time Caibrate in Closed loop Forward direction
	{
        SensorlessTrapController.Calibrate_Motor_Start = TRUE;
		SensorlessTrapController.Calibrate_Motor_Direction = TRUE;
		SensorlessTrapController.CaibBEMFThreshold[SensorlessTrapController.Direction] = (SensorlessTrapController.ISC_Motor_Speed[7] * SensorlessTrapController.ISC_BEMF_Diff[7]) >> 3;
		SensorlessTrapController.Direction_flag = FALSE;
		SensorlessTrapController.Direction = !SensorlessTrapController.Direction;   // Change the direction and re calibrate
	}
	else																// Third time Caibrate in Closed loop Forward direction
	{
		SensorlessTrapController.CaibBEMFThreshold[SensorlessTrapController.Direction] = (SensorlessTrapController.ISC_Motor_Speed[7] * SensorlessTrapController.ISC_BEMF_Diff[7]) >> 3;
		SensorlessTrapController.Direction_flag = FALSE;
	    SensorlessTrapController.Direction = !SensorlessTrapController.Direction; // Change the direction to Actual set direction
		SensorlessTrapController.Calibrate_Motor_Direction = 0;
		SensorlessTrapController.Calibrate_Motor_run= 0;
		SensorlessTrapController.BEMFThreshold = (SensorlessTrapController.CaibBEMFThreshold[0] + SensorlessTrapController.CaibBEMFThreshold[1]) >>1 ;
		ApplicationStatus.previousstate =  ApplicationStatus.currentstate;
		ApplicationStatus.currentstate = MOTOR_STOP;
		HostController.Calibrate_Motor = 0;
	}
}
/*function
 * calculateBEMFThreshold()
 * This Function calculates the dutycycle based on the BEMF value and Switches to BEMF Integration method during Initial Speed Control
 * */
inline void CatchInitialSpeed()
{
	SensorlessTrapController.QMATH_TEMP_A = _Q4(SensorlessTrapController.ISC_BEMF_Diff[7]);   // Maximum value of ADC is 12 bit
	SensorlessTrapController.QMATH_TEMP_B = _Q4(SensorlessTrapController.VCCvoltage);     // Maximum value of ADC is 12 bit
	SensorlessTrapController.QMATH_TEMP_C = _Q4div(SensorlessTrapController.QMATH_TEMP_A, SensorlessTrapController.QMATH_TEMP_B);
	SensorlessTrapController.QMATH_TEMP_A = _Q4(SensorlessTrapController.PWMPeriod);
	SensorlessTrapController.QMATH_TEMP_B = _Q4mpy(SensorlessTrapController.QMATH_TEMP_C,SensorlessTrapController.QMATH_TEMP_A);
	SensorlessTrapController.QMATH_TEMP_A = _Q4(1.12);                                     // Assuming Bemf holds 90% of Applied Voltage , Apply 1.11 times the BEMF
	SensorlessTrapController.QMATH_TEMP_C = _Q4mpy(SensorlessTrapController.QMATH_TEMP_A,SensorlessTrapController.QMATH_TEMP_B);
	SensorlessTrapController.CurrentDutyCycle = _Q4int(SensorlessTrapController.QMATH_TEMP_C);
	SensorlessTrapController.CurrentCommState = SensorlessTrapController.ISC_Next_State;
	SensorlessTrapController.AccelDone = TRUE;                                              // Switch to Closed loop
	ApplicationStatus.currentstate = MOTOR_START;

}
inline void CalDutyPerSpeed()
{
    SensorlessTrapController.Motor_Electrical_Speed = ((SensorlessTrapController.Motor_Rated_Speed/120.0F)*SensorlessTrapController.Motor_Poles) ;
	SensorlessTrapController.Hertz_Per_Volt_Counts = ((float_t)SensorlessTrapController.Motor_Electrical_Speed / SensorlessTrapController.Motor_Rated_Voltage) ;
 	SensorlessTrapController.Hertz_Per_Volt_Counts /=  DRV_COUNT_PER_VOLTS; // This parameter defines the Electrical frequency for a given Voltage in digital counts for DRV835x Board
	SensorlessTrapController.Hertz_Per_Unit_Volt_Counts = ( SensorlessTrapController.Hertz_Per_Volt_Counts * 100 / SensorlessTrapController.Motor_Under_Rating );  // Including motor under rating
	SensorlessTrapController.Duty_Per_Speed =  (SensorlessTrapController.PWMPeriod / SensorlessTrapController.Hertz_Per_Unit_Volt_Counts);
}
