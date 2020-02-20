/**************************************************************************
 * @file        ISRs.c
 * @author      MDBU Software Team
 * @brief       ISR setups for application
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 ******************************************************************************/

#include "global.h"
#include "mdbu_global.h"

// Controller
extern SENSORED_TRAP_Obj sensoredTrapController;
extern APPLICATION_STATUS applicationStatus;

// Host Controller
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

extern void PWM_SetCommutation(uint8_t hallState);

/* This Interrupt routine samples the Vcc supply voltage to detect Voltage faults */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
    uint16_t interruptValue = TA1IV;

    sensoredTrapController.TimerOverflowFlag = 1; /* This count is used to calculate speed of 16bit resolution */
    sensoredTrapController.SPDFdbk = 0xFFFF ; 	/* Set the speed count to maximum value that represents the zero speed */
}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
{
    uint16_t interruptValue = TA1IV;
}

#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void)
{
    uint16_t interruptValue = TA2IV;
}

#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR(void)
{
    uint16_t interruptValue = TA2IV;
}

/* Timer0_B0 interrupt service routine
   ISR to check for stall fault and handle fault recovery timeout
 */
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
    uint16_t interruptValue = TB0IV;

    sensoredTrapController.readVccCounter++;
    if (sensoredTrapController.readVccCounter >= READ_VCC_PERIOD)
    {
        ReadVCC();
        sensoredTrapController.readVccCounter = 0;
    }

#ifdef STALL_DETECTOIN_FLAG
    sensoredTrapController.StallDetectCounter++;
	if(sensoredTrapController.GateToggleFaultDetect)
	{
		if(sensoredTrapController.GateToggleCounter < (sensoredTrapController.Counter_1M_Second * 10 )) // If a Gate Toggle fault detect flag is set for 10ms, Disable the flag
		{
			sensoredTrapController.GateToggleCounter++;
		}
		else
		{
			sensoredTrapController.GateToggleFaultDetect = FALSE;
			sensoredTrapController.GateToggleCounter = 0;
			ReadVCC();                                                                      /* Check for any voltage faults */
			if((P2IN & BIT7) == 0)                                           // Check whether If Fault still persist i.e if p2.7 is low
			{
				P2IFG |= BIT7;                                               // Trigger an Interrupt which Identifies the fault
			}
		}
	}

    if(sensoredTrapController.StallDetectCounter >= sensoredTrapController.stallDetectTimerThreshold)
    {
        /* check for STall fault*/
        sensoredTrapController.StallDetectCounter = 0;
        if(sensoredTrapController.CurrentDutyCycle >= sensoredTrapController.minStallDetectDuty &&
           applicationStatus.currentstate != MOTOR_RAMP_UP &&  applicationStatus.currentstate != HALL_ALIGN &&
           applicationStatus.currentstate != FAULT)                                                                                                                        // If there is already a fault state , Stall fault shouldn't overwrite it
        {
            if(sensoredTrapController.RotationCount <=
               sensoredTrapController.stallDetectRevThreshold)                                                     /* < 1 rpm --> stall */
            {
                applicationStatus.previousstate =
                    applicationStatus.currentstate;
                applicationStatus.currentstate = FAULT;
                applicationStatus.fault = MOTOR_STALL;
            }
        }
        sensoredTrapController.RotationCount = 0;
    }
#endif
    /*In fault case wait 1000 milli sec and restart software with initialization*/
    if(applicationStatus.currentstate == FAULT)
    {
        sensoredTrapController.RestartDelay++;
        sensoredTrapController.StallDetectCounter = 0;                      // When a Fault is detected , Stall fault is ignored
        if(sensoredTrapController.RestartDelay >= sensoredTrapController.autoFaultRecoveryTime)
        {
            ReadVCC();
    		HostController.CalibrateHall = FALSE;                               // End the calibration
            sensoredTrapController.RestartDelay = 0;
            P1OUT |= BIT0;                              /* OFF LED1 */
            P4OUT |= BIT7;                              /* OFF LED2 */
            if(sensoredTrapController.SPIVariant == TRUE)                  // Initialize the SPI variables and settings only if the device is "S" Variant
            {
            	SPI_Write(SPI_REG_DRV_CTRL,
            			(SPI_Read(SPI_REG_DRV_CTRL) | CLR_FLT_MASK));          //Try to Clear the faults and warnings
            }
            if((P2IN & BIT7) != 0)                                           // Check whether If Fault still persist i.e if p2.7 is low
            {
            	applicationStatus.previousstate =
                    applicationStatus.currentstate;
                applicationStatus.currentstate = SYSTEM_INIT;
                applicationStatus.fault = NOFAULT;                           /* restart need to NOFAULT */

            }
        }
    }
}

/* Timer0_B0 interrupt service routine
   This is for only CCRO.
 */
#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
{
    uint16_t interruptValue = TBIV;
}

/* Timer0_A0 interrupt service routine for Timer A0 period match interrupt
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    uint16_t interruptValue = TBIV;
    /*increase period counter */
    sensoredTrapController.Accelerationcounter++;
	ReadCurrentShunt();                                    /* Monitor Motor Phase Current Value */
    switch(applicationStatus.currentstate)
    {
    case HALL_ALIGN:
    	if(sensoredTrapController.HallAlignStartFlag)
    	{
            if(sensoredTrapController.CurrentDutyCycle < sensoredTrapController.CALDutyCycle)
            {             //continue to increase the duty cycle to the specified value
            	sensoredTrapController.CurrentDutyCycle++;
                SetPWMDutyCycle(sensoredTrapController.CurrentDutyCycle);
            }
			sensoredTrapController.HallAligncounter++;
			if(sensoredTrapController.HallAligncounter >=
			   sensoredTrapController.HallCalibrationCycles)
			{
				sensoredTrapController.HallAlignDoneFlag = TRUE;
				sensoredTrapController.HallAlignStartFlag = FALSE;
			}
    	}
    	break;
    case MOTOR_RAMP_UP:

        if(sensoredTrapController.Accelerationcounter >=
           sensoredTrapController.Accelerationdivider)
        {
            sensoredTrapController.Accelerationcounter = 0;                  /*reset  period counter */
            if(sensoredTrapController.CurrentDutyCycle == 0)
            {
                sensoredTrapController.CurrentDutyCycle =
                    sensoredTrapController.MINDutyCycle;
            }
            else
            {
                sensoredTrapController.CurrentDutyCycle +=
                    sensoredTrapController.RampRate;                         /*increase duty cycle by acceleration rate*/
            }
            /* set timer channel compare value */
            TA0CCR1 = sensoredTrapController.CurrentDutyCycle;
            TA0CCR2 = sensoredTrapController.CurrentDutyCycle;
            TA0CCR3 = sensoredTrapController.CurrentDutyCycle;
            TA0CCR4 = sensoredTrapController.CurrentDutyCycle;
            TA2CCR1 = sensoredTrapController.CurrentDutyCycle;
            TA2CCR2 = sensoredTrapController.CurrentDutyCycle;
            /*move to Motor RUN if target duty cycle is reached */
            if(sensoredTrapController.CurrentDutyCycle >=
               sensoredTrapController.TargetDutyCycle)
            {
                applicationStatus.previousstate =
                    applicationStatus.currentstate;
                applicationStatus.currentstate = MOTOR_RUN;
            }

            /* Move to MOTOR_RAMP_DOWM if MOTOR STOP received*/
            if(HostController.StartStopMotor == 1)
            {
                applicationStatus.previousstate = applicationStatus.currentstate;
                applicationStatus.currentstate = MOTOR_RAMP_DOWN;
            }
        }
        break;
    case MOTOR_RAMP_DOWN:
        if(sensoredTrapController.Accelerationcounter >=
           sensoredTrapController.Accelerationdivider)
        {
            sensoredTrapController.Accelerationcounter = 0;              /*reset  period counter*/
            sensoredTrapController.CurrentDutyCycle -=
                sensoredTrapController.RampRate;                         /*decrease duty cycle by acceleration rate */

            if(sensoredTrapController.CurrentDutyCycle <
               (sensoredTrapController.MINDutyCycle))
            {
                sensoredTrapController.CurrentDutyCycle = 0;
            }

            //set timer channel compare value
            TA0CCR1 = sensoredTrapController.CurrentDutyCycle;
            TA0CCR2 = sensoredTrapController.CurrentDutyCycle;
            TA0CCR3 = sensoredTrapController.CurrentDutyCycle;
            TA0CCR4 = sensoredTrapController.CurrentDutyCycle;
            TA2CCR1 = sensoredTrapController.CurrentDutyCycle;
            TA2CCR2 = sensoredTrapController.CurrentDutyCycle;

            /*move to Motor RUN if target duty cycle is reached or to Motor STOP if duty cycle equals 0 */
            if(sensoredTrapController.CurrentDutyCycle == 0)
            {
                applicationStatus.previousstate =
                    applicationStatus.currentstate;
                applicationStatus.currentstate = MOTOR_STOP;
            }
            else if((sensoredTrapController.CurrentDutyCycle <=
                     sensoredTrapController.TargetDutyCycle) &&
                    (HostController.StartStopMotor == 0))
            {
                applicationStatus.previousstate =
                    applicationStatus.currentstate;
                applicationStatus.currentstate = MOTOR_RUN;
            }
        }
        break;
    case MOTOR_DIRECTION:
        if(sensoredTrapController.Accelerationcounter >=
           sensoredTrapController.Accelerationdivider)
        {
            sensoredTrapController.Accelerationcounter = 0;                                     //reset  period counter

            if(sensoredTrapController.CurrentDutyCycle != 0x00)
            {
                sensoredTrapController.CurrentDutyCycle -=
                    sensoredTrapController.RampRate;                                            //decrease duty cycle by acceleration rate
                //set timer channel compare value
                TA0CCR1 = sensoredTrapController.CurrentDutyCycle;
                TA0CCR2 = sensoredTrapController.CurrentDutyCycle;
                TA0CCR3 = sensoredTrapController.CurrentDutyCycle;
                TA0CCR4 = sensoredTrapController.CurrentDutyCycle;
                TA2CCR1 = sensoredTrapController.CurrentDutyCycle;
                TA2CCR2 = sensoredTrapController.CurrentDutyCycle;
                //change direction if duty cycle is 0 and set new commutation --> move back to target duty cycle
            }
            else
            {
                if(sensoredTrapController.Direction_flag != 0)                      /* This flag determines if direction change is required */
                {
                    sensoredTrapController.Direction =
                        !sensoredTrapController.Direction;
                    sensoredTrapController.Direction_flag = 0;
                }
                if(sensoredTrapController.PWM_Mode == 0)  // If six PWM mode
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
					P1OUT |= BIT2;                  // Motor brakes are removed
            		P2OUT |= (BIT5);
            		P2SEL |= (BIT5);
                	if(sensoredTrapController.Direction == TRUE )
					{
						P1OUT |= BIT3;                  // Set the direction to high using INHC on DRV83xx EVM
					}
					else
					{
						P1OUT &= ~BIT3;                  // Set the direction to low using INHC on DRV83xx EVM
					}
				}

                if(sensoredTrapController.Direction == TRUE)
                {
                    P1OUT |= BIT0;                                                                             /* Turn  on LED1 */
                    P4OUT &= ~BIT7;                                                                            /* Turn off LED 2*/
                }
                else
                {
                    P1OUT &= ~BIT0;                                                                            /* Turn  off LED1 */
                    P4OUT |= BIT7;                                                                             /* Turn on LED 2*/
                }

    			if(HostController.StartStopMotor == 0)
    			{
    				UpdateNextCommutation();
    				applicationStatus.previousstate =
    						applicationStatus.currentstate;
    				applicationStatus.currentstate = MOTOR_RAMP_UP;
    			}
            }
        }
        break;
    default:
        break;
    }
}

/* ISR for port 1 Interrupts
 * check  for direction change input and Hall sensor-1 Interrupt
 * */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    // Direction Change Interrupt
    if(P1IFG & BIT1)
    {
        sensoredTrapController.Direction_flag =
            !sensoredTrapController.Direction_flag;                                             // This flag monitors for change in direction, If direction change is applied multiple times in quick succession before speed becoming zero, this flag determines the true direction by the time motor is ramped up.
        applicationStatus.previousstate = applicationStatus.currentstate;
        applicationStatus.currentstate = MOTOR_DIRECTION;
        P1IFG &= ~BIT1;                                     // Clear interrupt flags
    }
}

/* ISR for port 2 Interrupts
 * check  for Hall sensor-2 , Hall sensor-3 Interrupt
 * */
#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    //Hall Sensor-1. Hall Sensor-2, Hall Sensor-3 Interrupt
    if(P2IFG & (BIT0 | BIT2 | BIT6))
    {
        if(P2IFG & BIT0)
        {
            P2IES ^= BIT0;
        }
        else if(P2IFG & BIT2)
        {
            P2IES ^= BIT2;
        }
        else if(P2IFG & BIT6)
        {
            P2IES ^= BIT6;
        }

        P2IFG &= ~(BIT0 | BIT2 | BIT6);                 // Clear interrupt flags

        if(applicationStatus.currentstate == SYSTEM_INIT)
        {
            applicationStatus.previousstate = applicationStatus.currentstate;
            applicationStatus.currentstate = FAULT;
            applicationStatus.fault = UNKNOWN;
        }
        else if(applicationStatus.currentstate == HALL_ALIGN)
        {
        	sensoredTrapController.HallAlignInterruptFlag = TRUE;
        }
        else
        {
            UpdateNextCommutation();
        }
    }

    if(P2IFG & BIT7)                                         // If Fault pin from P3.4 <==> P2.7  Interrupt is triggered , Take corresponding action by Reading fault status from SPI
    {
    	if(!(P2IN & BIT7))
    	{
    		if(sensoredTrapController.SPIVariant == TRUE)                  // Initialize the SPI variables and settings only if the device is "S" Variant
    		{
    			unsigned int faultStatus = SPI_Read(SPI_REG_FAULT_STAT);     // Check fault register;

				if(faultStatus & FAULT_MASK)                                          // If it is Fault
				{
					if(faultStatus & VDS_OCP_MASK)
					{
						applicationStatus.previousstate =
							applicationStatus.currentstate;
						applicationStatus.currentstate = FAULT;
						applicationStatus.fault = OVERCURRENT;
					}
					else if(faultStatus & UVLO_MASK)
					{
						applicationStatus.previousstate =
							applicationStatus.currentstate;
						applicationStatus.currentstate = FAULT;
						applicationStatus.fault = VOLTAGE;
					}
					else if(faultStatus & OTSD_MASK)
					{
						applicationStatus.previousstate =
							applicationStatus.currentstate;
						applicationStatus.currentstate = FAULT;
						applicationStatus.fault = OVERTEMPERATURE;
					}
					else if(faultStatus & GDF_MASK)
					{
						applicationStatus.previousstate =
							applicationStatus.currentstate;
						applicationStatus.currentstate = FAULT;
						applicationStatus.fault = GATE_DRIVER;
					}
					else
					{
						applicationStatus.previousstate =
							applicationStatus.currentstate;
						applicationStatus.currentstate = FAULT;
						applicationStatus.fault = UNKNOWN;
					}
				}
				else                      // If the Fault pin is toggling because of warnings then clear the warnings
				{
					if(applicationStatus.currentstate != FAULT)
					{
						SPI_Write(SPI_REG_DRV_CTRL,
								  (SPI_Read(SPI_REG_DRV_CTRL) | CLR_FLT_MASK));                                   //If device is not in fault status clear and Ignore any warnings arise
					}
				}
		  }
		  else
		  {
				if(sensoredTrapController.GateToggleFaultDetect)                            // If a fault is triggered within 10ms of Toggling gate drivers, Disable gate drivers but a fault will not be set, If fault persists after 5ms, Fault is triggered
				{
			        DisableGateDrivers();                                                     /* Disable Gate Drivers when a fault is triggered*/
				}
				else
				{
					applicationStatus.previousstate = applicationStatus.currentstate;
					applicationStatus.currentstate = FAULT;
					applicationStatus.fault = UNKNOWN;
				}
		  }
    	}
      P2IFG &= ~BIT7;
    }
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
}

#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void)
{
}

#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{
}

#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void)
{
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    switch(__even_in_range(ADC12IV,34))
    {
    case  0: break;                                 // Vector  0:  No interrupt
    case  2: break;                                 // Vector  2:  ADC overflow
    case  4: break;                                 // Vector  4:  ADC timing overflow
    case  6:                                        // Vector  6:  ADC12IFG0

        sensoredTrapController.TargetDutyCycle = ADC12MEM0 & 0x0FFF;            /* Filter only last 12 bits */

        sensoredTrapController.TargetDutyCycle >>= PWM_FACTOR;           /* As the Period register is stored with 1024 , 12 bit ADC result has to be scaled to 10 bit value */
        if(sensoredTrapController.TargetDutyCycle >
           sensoredTrapController.MAXDutyCycle)
        {
            sensoredTrapController.TargetDutyCycle =
                sensoredTrapController.MAXDutyCycle;
        }
        break;

    case 8:                                                                             // Vector  8:  ADC12IFG1
        sensoredTrapController.VCCvoltage = ADC12MEM1 & 0x0FFF;             /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/
        sensoredTrapController.VCCvoltage >>= PWM_FACTOR;               /*12 bit ADC result has to be scaled to 10 bit value */
#ifdef VCC_MONITOR_FLAG
        if(sensoredTrapController.VCCvoltage <
           sensoredTrapController.underVoltageLimit | sensoredTrapController.VCCvoltage >
           sensoredTrapController.overVoltageLimit)                                                                                                         /* Under Voltage of 8.0V and over voltage at 20.0V*/
        {
            applicationStatus.previousstate = applicationStatus.currentstate;
            applicationStatus.currentstate = FAULT;
            applicationStatus.fault = VOLTAGE;
        }
        if(sensoredTrapController.VCCvoltage <
        		sensoredTrapController.MinPowerSupply)                                                                                                       /* Under Voltage of 10.0V and over voltage at 20.0V*/
           {
               applicationStatus.previousstate = applicationStatus.currentstate;
               applicationStatus.currentstate = FAULT;
               applicationStatus.fault = POWER_SUPPLY;
           }

#endif
        break;

    case 10:                                                                            // Vector  10:  ADC12IFG2
        sensoredTrapController.MotorPhaseCurrent = ADC12MEM2 & 0x0FFF;             /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/
        if(sensoredTrapController.MotorPhaseCurrent > 50)             		 // If motor phase current is started biasing
        {
            if(sensoredTrapController.ShuntVariant == TRUE)    // If Current sense is not in phase shunt remove the offset.
            {
                sensoredTrapController.MotorPhaseCurrent -= 2048;     // subtracting the bias, Vref/2 is added as bias voltage to support bidirectional current sensing
            }
            sensoredTrapController.MotorPhaseCurrent = abs(
                sensoredTrapController.MotorPhaseCurrent);
            if((sensoredTrapController.MotorPhaseCurrent >
               sensoredTrapController.MotorPhaseCurrentLimit) && (applicationStatus.fault == NOFAULT))                /* Motor Phase Current Limit*/
            {
                applicationStatus.previousstate =
                    applicationStatus.currentstate;
                applicationStatus.currentstate = FAULT;
                applicationStatus.fault = OVERCURRENT;
            }
        }
        break;
    default:
        break;
    }
}

#pragma vector = COMP_B_VECTOR
__interrupt void COMP_B_ISR(void)
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
