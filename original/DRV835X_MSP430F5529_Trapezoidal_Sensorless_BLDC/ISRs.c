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
 * @file        ISRs.c
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
// Timer1_A0 interrupt service routine
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
    if(ApplicationStatus.currentstate == MOTOR_IPD)
    {
        if(SensorlessTrapController.IPDStatus == TIMER_INTERRUPT)
        {
            SensorlessTrapController.IPDStatus = ADC_READ;
            ADC12CTL0 |= ADC12ENC;                              // Enable conversions control registers shouldn't be changes after enabling conversions
            ADC12CTL0 |= ADC12SC;
        }
    }
   	SensorlessTrapController.TimerOverflowFlag = 1;       //This count is used to calculate speed of 18bit resolution
   	SensorlessTrapController.SPDFdbk = 0x0FFFF ; // Set the speed count to maximum value that represents the zero speed
}
// Timer1_A1 interrupt service routine
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
{
    uint16_t interruptValue = TA1IV;
}

// Timer2_A0 interrupt service routine
#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void)
{
	uint16_t interruptValue = TA2IV;
}

/* Timer2_A1 interrupt service routine
   This is for CCR1 Compare match
   Used for sampling the BEMF ADC
 */
#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR(void)
{
}

/* Timer0_B0 interrupt service routine
   ISR to check for stall fault and handle fault recovery timeout
 */
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
	if(SensorlessTrapController.GateToggleFaultDetect)
	{
		if(SensorlessTrapController.GateToggleCounter < (SensorlessTrapController.Counter_1M_Second * 10 )) // If a Gate Toggle fault detect flag is set for 10ms, Disable the flag
		{
			SensorlessTrapController.GateToggleCounter++;
		}
		else
		{
			SensorlessTrapController.GateToggleFaultDetect = FALSE;
			SensorlessTrapController.GateToggleCounter = 0;
			ReadVCC();                                                                      /* Check for any voltage faults */
			if((P2IN & BIT7) == 0)                                           // Check whether If Fault still persist i.e if p2.7 is low
			{
				P2IFG |= BIT7;                                               // Trigger an Interrupt which Identifies the fault
			}
		}
	}
    if(ApplicationStatus.currentstate == MOTOR_RUN)     // If the motor is in run state where pulses are given and still if motor is not spinning , then stall fault is detected
    {
        SensorlessTrapController.StallDetectCounter++;          // Stall detect counter increases for every PWM Interrupt
        if(SensorlessTrapController.StallDetectCounter >=
            		SensorlessTrapController.Counter_1M_Second) // Stall Delay incremented for every 1 milli second
        {
			SensorlessTrapController.StallDetectCounter = 0;
			SensorlessTrapController.StallDetectDelay++;
			if(SensorlessTrapController.StallDetectDelay >=
				SensorlessTrapController.StallDetecttime)
			{
				SensorlessTrapController.StallDetectDelay = 0;
				/* check for STall fault*/
				if(SensorlessTrapController.RotationCount <= SensorlessTrapController.StallDetectRev)         /* < 1 rpm --> stall */
				{
					ApplicationStatus.previousstate = ApplicationStatus.currentstate;
					ApplicationStatus.currentstate = FAULT;
					ApplicationStatus.fault = MOTOR_STALL;
				}
				SensorlessTrapController.RotationCount = 0;
			}
        }
    }

    /*In fault case wait for Fault recovery time and restart software with initialization*/
    if(ApplicationStatus.currentstate == FAULT)
    {
        SensorlessTrapController.RestartDelayCounter++;                       // Every PWM Interrupt the Fault recovery delay counter increases
		SensorlessTrapController.StallDetectCounter = 0;                      // When a Fault is detected , Stall fault is ignored
		SensorlessTrapController.StallDetectDelay = 0;
		if (SensorlessTrapController.RestartDelayCounter > SensorlessTrapController.Counter_1M_Second)
        {
			SensorlessTrapController.RestartDelay++;						// Once Delay counter counts 1 milli second increase the restart Delay
	        SensorlessTrapController.RestartDelayCounter = 0;
			if(SensorlessTrapController.RestartDelay >= SensorlessTrapController.AutoFaultRecoveryTime)
			{
				ADC_Init();
				ReadVCC();                                                                      /* Check for any voltage faults */
				SensorlessTrapController.RestartDelay = 0;
		    	SensorlessTrapController.Calibrate_Motor_run = 0;
				HostController.Calibrate_Motor = 0;
				P1OUT |= BIT0;                              /* OFF LED1 */
				P4OUT |= BIT7;                              /* OFF LED2 */
				if(SensorlessTrapController.SPIVariant == TRUE)                  // Initialize the SPI variables and settings only if the device is "S" Variant
				{
					SPI_Write(SPI_REG_DRV_CTRL,
						  (SPI_Read(SPI_REG_DRV_CTRL) | CLR_FLT_MASK));                           //Try to Clear the faults and warnings
				}
				if((P2IN & BIT7) != 0)                                           // Check whether If Fault still persist i.e if p2.7 is low
				{
					ApplicationStatus.previousstate =
						ApplicationStatus.currentstate;
					ApplicationStatus.currentstate = SYSTEM_INIT;
					ApplicationStatus.fault = NOFAULT;                                              /* restart need to NOFAULT */
				}
			}
        }
    }
}

/* Timer0_A0 interrupt service routine
   This is for only CCRO. This interrupt occurs every PWM period
   Used for time sensitive control in the state machine
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    SensorlessTrapController.SystemCount++;
    switch(ApplicationStatus.currentstate)
    {
    case MOTOR_ALIGN:
        if(SensorlessTrapController.StartAlign == FALSE)
        {
            if(SensorlessTrapController.CurrentDutyCycle < SensorlessTrapController.StartupDutyCycle)
            {             //continue to increase the duty cycle to the specified value
                SensorlessTrapController.CurrentDutyCycle++;
                SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);
            }
            if(SensorlessTrapController.AlignWaitCounter == 0)
            {             //set the commutation sequence once
                PWM_SetCommutation(SensorlessTrapController.AlignSector);
            }
        }
        SensorlessTrapController.AlignWaitCounter++;
        if(SensorlessTrapController.AlignWaitCounter >=  SensorlessTrapController.AlignWaitTime)
        {         //after driving the align sector for align time stop
            SensorlessTrapController.CurrentDutyCycle = 0;
            DisableGateDrivers();
            BrakeMotor();
            SensorlessTrapController.AlignWaitCounter = 0;
            if(SensorlessTrapController.StartAlign)
            {             //after waiting another align wait time signal align complete
                SensorlessTrapController.AlignComplete = TRUE;
                SensorlessTrapController.AlignWaitCounter = 0;
            }
            SensorlessTrapController.StartAlign = TRUE;
        }
        break;
    case MOTOR_ISC:
    	SensorlessTrapController.BEMF_Counter++;                                   // counter to count the number of samples between two commutation points
        break;
    case MOTOR_IPD:
        if(SensorlessTrapController.IPDStatus == START)
        {
            IPD_SetState(SensorlessTrapController.IPDState);
            TA1R = 0x0000;                                                                                                      /* reset TA1 counter , which sets no of clock cycles a phase is switched on*/
            TA1CCTL0 |= CCIE;
            SensorlessTrapController.IPDStatus = TIMER_INTERRUPT;
        }
        if(SensorlessTrapController.IPDStatus == BRAKE)
        {
            SensorlessTrapController.IPDCount++;
            SensorlessTrapController.IPDCoastTime =
                SensorlessTrapController.IPDBrakeTime * SensorlessTrapController.IPDDecayConstant;
            if(SensorlessTrapController.IPDCount >=
               SensorlessTrapController.IPDBrakeTime)
            {
            	DisableGateDrivers();                                  // Turn Off all switches during Coast time
                if(SensorlessTrapController.IPDCount >=
                   SensorlessTrapController.IPDCoastTime)
                {
                    SensorlessTrapController.IPDState++;
                    SensorlessTrapController.IPDCount = 0;
                    SensorlessTrapController.IPDStatus = START;
                }
            }
            if(SensorlessTrapController.IPDState >= 7)
            {
                SensorlessTrapController.IPDStatus = DONE;
                ADC12CTL0 &= ~ADC12ENC;                                          // Disable the ADC conversions after completion of IPD
            }
        }
        break;

    case MOTOR_START:
        SensorlessTrapController.AccelCounter++;
        if((SensorlessTrapController.AccelCounter == SensorlessTrapController.Counter_1M_Second)&&
           (SensorlessTrapController.AccelDone == FALSE))                                                                             //1ms
        {
            SensorlessTrapController.AccelCounter = 0;                                          //reset  period counter
            SensorlessTrapController.AccelVelocityInit += SensorlessTrapController.AccelRate;                     //increase velocity by the acceleration rate
            SensorlessTrapController.AccelDistance +=
                (SensorlessTrapController.AccelVelocityInit - (SensorlessTrapController.AccelRate >> 1)) >>
                3;                                                                                                                       //calculate distance
            if(SensorlessTrapController.AccelDistance > ACCEL_60_DEGREES)
            {                     //if distance is 60 degrees commutate
                SensorlessTrapController.AccelDistance = 0;
                UpdateNextCommutation();
                SensorlessTrapController.IQMATH_TEMP_C = _IQ15((SensorlessTrapController.AccelVelocityInit >> 10));
				SensorlessTrapController.IQMATH_TEMP_B = _IQ15(SensorlessTrapController.Duty_Per_Unit_Speed);
                SensorlessTrapController.IQMATH_TEMP_A = _IQ15mpy(SensorlessTrapController.IQMATH_TEMP_B , SensorlessTrapController.IQMATH_TEMP_C);
		        SensorlessTrapController.CurrentDutyCycle = _IQ15int(SensorlessTrapController.IQMATH_TEMP_A);
                PWM_SetCommutation(SensorlessTrapController.CurrentCommState);
                SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);
                SetMotorSpeed();                                                                                                                                                                                                                            //this enables the motor to stop during open loop
                if(SensorlessTrapController.AccelVelocityInit > SensorlessTrapController.AccelStop)
                {                                       //if our open loop speed reaches threshold switch to closed loop
					if((HostController.Calibrate_Motor) && (!SensorlessTrapController.Calibrate_Motor_run))
					{
						ApplicationStatus.previousstate =
						ApplicationStatus.currentstate;
				        ApplicationStatus.currentstate = MOTOR_ISC;                                             // Move to Motor ISC to calculate BEMF Threshold
						DisableGateDrivers();
						SensorlessTrapController.ISC_Phase_Match = 0;
						SensorlessTrapController.ISCStatus = READ_BEMF;
					}
					else
					{
						UpdateBEMFADC();
						SensorlessTrapController.AccelDone = TRUE;

					}
                }
            }
        }
        break;

    case MOTOR_RUN:
        SensorlessTrapController.ADCcnt++;
        SensorlessTrapController.SpeedDivider++;
        break;

    default:
        break;
    }
}


#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{

}

// Timer0_A1 interrupt service routine
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
}

/* Timer0_B0 interrupt service routine
   This is for only CCR1.      // This interrupt is to monitor the BEMF of the floating phase voltages just before the PWM falling edge so that BEMF will get settled after voltage dynamics
 */
#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
{
    if(ApplicationStatus.currentstate == MOTOR_RUN)
    {     //verify the state machine is in closed loop control
        if(SensorlessTrapController.ADCchange == TRUE)                                                                  // if the commutation sequence just changed (during blanking time)
        {
            switch(SensorlessTrapController.CurrentCommState)                             // Monitor either Vcc, Phase current , or Speed Input during Commutation Blank Time
            {
            case 1: SetMotorSpeed();                                            // Reads Change in speed input
                break;
            case 2: ReadVCC();                                                          // Reads supply voltage value
                break;
            case 3: SetMotorSpeed();                                            // Reads Change in speed input
                break;
            case 4: ReadVCC();                                                          // Reads supply voltage value
                break;
            default:
            	break;
            }
            SensorlessTrapController.ADCchange = FALSE;
        }
        UpdateBEMFADC();                                                                                                // set the ADC MUX for the floating phase
        if(SensorlessTrapController.ADCcnt > SensorlessTrapController.CommutationBlankTime)                                                                                   // if the blanking time is over and the ADC has a new value
        {
        	FastReadBEMF();
            SensorlessTrapController.CommStateDetect =
                SensorlessTrapController.CurrentCommState & 0x01;                                                                    //detect if the commutation state is even or odd
            if(SensorlessTrapController.Direction == FALSE)
            {
                //BEMF Decreasing && BEMF>CT
                if((SensorlessTrapController.CommStateDetect) &&
                   SensorlessTrapController.GetBEMF <=
                   SensorlessTrapController.CTvoltage)
                {
                    SensorlessTrapController.SumBEMF +=
                        SensorlessTrapController.CTvoltage -
                        SensorlessTrapController.GetBEMF;
                }
                //BEMF Increasing && CT>BEMF
                else if((SensorlessTrapController.CommStateDetect == 0) &&
                        SensorlessTrapController.GetBEMF >=
                        SensorlessTrapController.CTvoltage)
                {
                    SensorlessTrapController.SumBEMF +=
                        SensorlessTrapController.GetBEMF -
                        SensorlessTrapController.CTvoltage;
                }
            }
            else
            {
                //BEMF Decreasing && BEMF>CT
                if((SensorlessTrapController.CommStateDetect == 0) &&
                   SensorlessTrapController.GetBEMF <=
                   SensorlessTrapController.CTvoltage)
                {
                    SensorlessTrapController.SumBEMF +=
                        SensorlessTrapController.CTvoltage -
                        SensorlessTrapController.GetBEMF;
                }
                //BEMF Increasing && CT>BEMF
                else if((SensorlessTrapController.CommStateDetect) &&
                        SensorlessTrapController.GetBEMF >=
                        SensorlessTrapController.CTvoltage)
                {
                    SensorlessTrapController.SumBEMF +=
                        SensorlessTrapController.GetBEMF -
                        SensorlessTrapController.CTvoltage;
                }
            }
            if(SensorlessTrapController.SumBEMF >=  SensorlessTrapController.BEMFThreshold)                            //compare the sum to the commutation threshold
            {
                SensorlessTrapController.SumBEMF = 0;
                SensorlessTrapController.ADCcnt = 0;
                UpdateNextCommutation();
                PWM_SetCommutation(SensorlessTrapController.CurrentCommState);
                SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);
                SensorlessTrapController.ADCchange = TRUE;
            }
            if((SensorlessTrapController.CurrentCommState == 5) || (SensorlessTrapController.CurrentCommState ==6))
            {
            	ReadCurrentShunt();                             //Reads CSA value and triggers OC faults for Motor current greater than Set Limit , phase A current is monitored
            }
        }
    }
    TB0CCTL1 &= ~CCIFG;
}

#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void)
{
}

#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{
}

/* ISR for port 2 Interrupts
 * check  for Hall sensor-2 , Hall sensor-3 Interrupt
 * */
#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    if(P2IFG & BIT7)                                         // If Fault pin from P3.4 <==> P2.7  Interrupt is triggered , Take corresponding action by Reading fault status from SPI
    {
    	if(!(P2IN & BIT7))
    	{
			if(SensorlessTrapController.SPIVariant == TRUE)                  // Initialize the SPI variables and settings only if the device is "S" Variant
			{
				unsigned int faultStatus = SPI_Read(SPI_REG_FAULT_STAT); // Check fault register;

				if(faultStatus & FAULT_MASK)                                          // If it is Fault
				{
					if(faultStatus & UVLO_MASK )
					{
						ApplicationStatus.previousstate =
						ApplicationStatus.currentstate;
						ApplicationStatus.currentstate = FAULT;
						ApplicationStatus.fault = VOLTAGE;
						ReadVCC();
					}
					else if(faultStatus & VDS_OCP_MASK)
					{
						ApplicationStatus.previousstate =
							ApplicationStatus.currentstate;
						ApplicationStatus.currentstate = FAULT;
						ApplicationStatus.fault = OVERCURRENT;
					}
					else if(faultStatus & OTSD_MASK)
					{
						ApplicationStatus.previousstate =
							ApplicationStatus.currentstate;
						ApplicationStatus.currentstate = FAULT;
						ApplicationStatus.fault = OVERTEMPERATURE;
					}
					else if(faultStatus & GDF_MASK)
					{
						ApplicationStatus.previousstate =
							ApplicationStatus.currentstate;
						ApplicationStatus.currentstate = FAULT;
						ApplicationStatus.fault = GATE_DRIVER;
					}
					else
					{
						ApplicationStatus.previousstate =
							ApplicationStatus.currentstate;
						ApplicationStatus.currentstate = FAULT;
						ApplicationStatus.fault = UNKNOWN;
					}
				}
				else                      // If the Fault pin is toggling because of warnings then clear the warnings
				{
					if(ApplicationStatus.currentstate != FAULT)
					{
						SPI_Write(SPI_REG_DRV_CTRL,
								  (SPI_Read(SPI_REG_DRV_CTRL) | CLR_FLT_MASK));                       //If device is not in fault status clear and Ignore any warnings arise
					}
				}
			}
			else
			{
				if(SensorlessTrapController.GateToggleFaultDetect)                            // If a fault is triggered within 10ms of Toggling gate drivers, Disable gate drivers but a fault will not be set, If fault persists after 5ms, Fault is triggered
				{
			        DisableGateDrivers();                                                     /* Disable Gate Drivers when a fault is triggered*/
				}
				else
				{
					ApplicationStatus.previousstate = ApplicationStatus.currentstate;
					ApplicationStatus.currentstate = FAULT;
					ApplicationStatus.fault = UNKNOWN;
				}
			}
    	}
    	P2IFG &= ~BIT7;
    }
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    if(ApplicationStatus.currentstate == MOTOR_IPD)
    {
        if(SensorlessTrapController.IPDStatus == ADC_READ)            // wait till Timer reaches certain limit
        {
        	BrakeMotor();
            switch(__even_in_range(ADC12IV,34))
            {
            case  0: break;                                       // Vector  0:  No interrupt
            case  2: break;                                       // Vector  2:  ADC overflow
            case  4: break;                                       // Vector  4:  ADC timing overflow
            case 6:
                SensorlessTrapController.IPDCurrentRiseValue[
                    SensorlessTrapController.IPDState] = ADC12MEM0 & 0x0FFF;                                                           /* Filter only last 12 bits */
                break;
            case 8:
                SensorlessTrapController.IPDCurrentRiseValue[
                    SensorlessTrapController.IPDState] = ADC12MEM1 & 0x0FFF;                                                           /* Filter only last 12 bits */
                break;
            case 10:
                SensorlessTrapController.IPDCurrentRiseValue[
                    SensorlessTrapController.IPDState] = ADC12MEM2 & 0x0FFF;                                                           /* Filter only last 12 bits */
                break;
            case 12:
                SensorlessTrapController.IPDCurrentRiseValue[
                    SensorlessTrapController.IPDState] = ADC12MEM3 & 0x0FFF;                                                           /* Filter only last 12 bits */
                break;
            case 14:
                SensorlessTrapController.IPDCurrentRiseValue[
                    SensorlessTrapController.IPDState] = ADC12MEM4 & 0x0FFF;                                                           /* Filter only last 12 bits */
                break;
            case 16:
                SensorlessTrapController.IPDCurrentRiseValue[
                    SensorlessTrapController.IPDState] = ADC12MEM5 & 0x0FFF;                                                           /* Filter only last 12 bits */
                break;
            default:
                break;
            }
            ADC12CTL0 &= ~ADC12SC;                                 // Make SC bit low to make SHI low
//            P4OUT &= ~BIT1;
        	if(SensorlessTrapController.ShuntVariant == TRUE)
        	{
				SensorlessTrapController.IPDCurrentRiseValue[
					SensorlessTrapController.IPDState] -= 2048;
        	}
            SensorlessTrapController.IPDCurrentRiseValue[
                SensorlessTrapController.IPDState] = abs(
                SensorlessTrapController.IPDCurrentRiseValue[
                    SensorlessTrapController.IPDState]);
            if(SensorlessTrapController.IPDCurrentRiseValue[
                   SensorlessTrapController.IPDState] >
               SensorlessTrapController.MotorPhaseCurrentLimit)                                                                                               /* Motor Phase Current Limit*/
            {
                ApplicationStatus.previousstate =
                    ApplicationStatus.currentstate;
                ApplicationStatus.currentstate = FAULT;
                ApplicationStatus.fault = OVERCURRENT;
            }
            SensorlessTrapController.IPDCount = 0;            //clear the time count
            SensorlessTrapController.IPDStatus = BRAKE;
        }
    }
    else
    {
        ADC12IE = 0x00;

    }
}

#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void)
{
}


#pragma vector = USCI_A0_VECTOR
__interrupt void USCIA_ISR(void)
{
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCIB_ISR(void)
{
}
/* ISR to handle UART interrupts from GUI */
#pragma vector = USCI_A1_VECTOR
__interrupt void USCIA1_ISR(void)
{
	switch ( UCA1IV )
	  {
	    case 2:
			/*
			 * Handle RX interrupt
			 */
			#ifndef MDBUSERIAL_USE_USB
				mdbuSerial_handleRX();
			#endif
		    break;
	   case 4:
			/*
			 * Handle TX interrupt
			 */
			#ifndef MDBUSERIAL_USE_USB
				mdbuSerial_handleTX();
			#endif
			break;
	  }
}

#pragma vector = USCI_B1_VECTOR
__interrupt void USCIB1_ISR(void)
{
}

#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR(void)
{
}

#pragma vector = SYSNMI_VECTOR
__interrupt void SYSNMI_ISR(void)
{
}

#pragma vector = COMP_B_VECTOR
__interrupt void COMP_B_ISR(void)
{
}
