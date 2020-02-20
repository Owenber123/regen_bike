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
 * @file        DRV83xx_MSP430F5529_trap_sensorless_main.c
 * @author      MDBU Software Team
 * @brief       SPI API file for SPI Module.
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 * @date		May 2016
 ******************************************************************************/
#include "global.h"

// Controller
SENSORLESS_TRAP_Obj SensorlessTrapController;
APPLICATION_STATUS ApplicationStatus;
MDBUSERIAL_RXSTATE mdbuSerial_RxState;
HOSTCONTROL_STATUS HostControl_Status;
mdbuSerial_RxPacket mdbuSerial_RxPkt;
HOST_CONTROLLER_Obj HostController;

// Registers
FLT_STAT_REG0_Obj Fault_Status_Reg;
VGS_STAT_REG1_Obj VGS_Status_Reg;
DRV_CTRL_REG2_Obj Driver_Control_Reg;
GATE_DRV_HS_REG3_Obj Gate_Drive_HS_Reg;
GATE_DRV_LS_REG4_Obj Gate_Drive_LS_Reg;
OCP_CTRL_REG5_Obj OCP_Control_Reg;
CSA_CTRL_REG6_Obj CSA_Control_Reg;
DRV_CONFIG_REG7_Obj DRV_Config_Reg;
REG_MAP_Obj Reg_Map_Cache;

/*function
 * DRV8x_State_Machine(void)
 * Handles the state machine and transitions of the Application
 * */
void DRV8x_State_Machine(void)
{
    // process current motor state
	if(ApplicationStatus.fault != NOFAULT)    // If any fault is detected change the state to fault
	{
		ApplicationStatus.previousstate = ApplicationStatus.currentstate;
		ApplicationStatus.currentstate = FAULT;
	}
    switch(ApplicationStatus.currentstate)
    {
    case SYSTEM_INIT:
        sensorlessTrapController_Init();                                                // Initialize Motor Variables
        ReadVCC();                                                                                      // Read Supply Voltage Vcc
        if(HostController.EnabledGateDrivers)
        {
            drv83xx_regRestoreFromCache();													   /* Restore device register values from the cached ones */
        }

        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        ApplicationStatus.currentstate = SYSTEM_IDLE;                                            // Move to Motor INIT state and initialize Motor variables
        break;
    case SYSTEM_IDLE:
    	 if(HostController.Calibrate_Motor)
    	 {
     		ReadVCC();
     	    CalDutyPerSpeed();                // This inline function calculates the applicable dutycycle based on the input speed using Device used, Motor Rated voltage, Rated speed , Under rating , Applied DC bus voltage
     		SensorlessTrapController.Duty_Per_Unit_Speed = (SensorlessTrapController.Duty_Per_Speed / SensorlessTrapController.VCCvoltage);
     		SensorlessTrapController.Duty_Per_Unit_Speed = (SensorlessTrapController.Duty_Per_Speed / SensorlessTrapController.VCCvoltage);
            SensorlessTrapController.IQMATH_TEMP_C = _IQ15((SensorlessTrapController.AccelVelocity >> 10));
			SensorlessTrapController.IQMATH_TEMP_B = _IQ15(SensorlessTrapController.Duty_Per_Unit_Speed);
            SensorlessTrapController.IQMATH_TEMP_A = _IQ15mpy(SensorlessTrapController.IQMATH_TEMP_B , SensorlessTrapController.IQMATH_TEMP_C);
	        SensorlessTrapController.StartupDutyCycle = _IQ15int(SensorlessTrapController.IQMATH_TEMP_A);
     		ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        	if(SensorlessTrapController.Direction_flag)
        	{
        		SensorlessTrapController.Direction =
        	                !SensorlessTrapController.Direction;
        		SensorlessTrapController.Direction_flag = FALSE;
        	}
            DisableGateDrivers();
            SetMotorSpeed();
            sensorlessTrapController_Init();                                                // Initialize Motor Variables
            SensorlessTrapController.ISC_Phase_Match = 0;
            SensorlessTrapController.Calibrate_Motor_Start = TRUE;
            SensorlessTrapController.ISCStatus = READ_BEMF;
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_ISC;                                             // Move to Motor ISC
    	 }
    	 else if(HostController.Start_Stop_Motor)     // If Motor is in Stop Mode i.e. 1
    	 {
    		ApplicationStatus.previousstate = ApplicationStatus.currentstate;
    		ApplicationStatus.currentstate = SYSTEM_IDLE;
    		ReadVCC();
    	 }
    	 else							    // If Motor is in Start Mode i.e. 0
    	 {
     		ReadVCC();
     	    CalDutyPerSpeed();                // This inline function calculates the applicable dutycycle based on the input speed using Device used, Motor Rated voltage, Rated speed , Under rating , Applied DC bus voltage
     		SensorlessTrapController.Duty_Per_Unit_Speed = (SensorlessTrapController.Duty_Per_Speed / SensorlessTrapController.VCCvoltage);
            SensorlessTrapController.IQMATH_TEMP_C = _IQ15((SensorlessTrapController.AccelVelocity >> 10));
			SensorlessTrapController.IQMATH_TEMP_B = _IQ15(SensorlessTrapController.Duty_Per_Unit_Speed);
            SensorlessTrapController.IQMATH_TEMP_A = _IQ15mpy(SensorlessTrapController.IQMATH_TEMP_B , SensorlessTrapController.IQMATH_TEMP_C);
	        SensorlessTrapController.StartupDutyCycle = _IQ15int(SensorlessTrapController.IQMATH_TEMP_A);
     		ApplicationStatus.previousstate = ApplicationStatus.currentstate;
    		ApplicationStatus.currentstate = MOTOR_IDLE;
    	 }
                                                                                                  // Read Supply Voltage Vcc
    	 break;
    case MOTOR_IDLE:

        SetMotorSpeed();
        if(HostController.Start_Stop_Motor)     // If Motor is in Stop Mode i.e. 1
        {
         		ApplicationStatus.previousstate = ApplicationStatus.currentstate;
          		ApplicationStatus.currentstate = SYSTEM_IDLE;
        }
        if((SensorlessTrapController.TargetDutyCycle > SensorlessTrapController.MinOnDutyCycle) && (!HostController.Start_Stop_Motor) )
        {
        	if(SensorlessTrapController.Direction_flag)
        	{
        		SensorlessTrapController.Direction =
        	                !SensorlessTrapController.Direction;
        		SensorlessTrapController.Direction_flag = FALSE;
        	}                                                                       // Read ADC input for speed input (potentiometer)
            if(SensorlessTrapController.Direction == TRUE)
            {
                P1OUT |= BIT0;                                                         /* Turn  on LED1 */
                P4OUT &= ~BIT7;                                                                                 /* Turn off LED 2*/
            }
            else
            {
                P1OUT &= ~BIT0;                                                        /* Turn  off LED1 */
                P4OUT |= BIT7;                                                                                  /* Turn on LED 2*/
            }

            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_ISC;                                             // Move to Motor ISC
            DisableGateDrivers();
            SensorlessTrapController.ISC_Phase_Match = 0;
            SensorlessTrapController.ISCStatus = READ_BEMF;
            ReadVCC();                                                                                      // Read Supply Voltage Vcc
        }
        ReadVCC();                                                                                          // Read Supply Voltage Vcc
        break;
    case MOTOR_ISC:
        // wait until a done signal is recieved from the ISC routine
        switch(SensorlessTrapController.ISCStatus)
        {
        case READ_BEMF:                  // Read the BEMF of phase windings during motor startup,

      		ISCReadPhaseVoltage();
      		/* *************If the BEMF Value is very less then Switch to Motor start up ********************/
    		if((SensorlessTrapController.LineAB_BEMF <= ISC_MIN_LINE_BEMF) && (SensorlessTrapController.LineBC_BEMF<= ISC_MIN_LINE_BEMF) && (SensorlessTrapController.LineCA_BEMF <= ISC_MIN_LINE_BEMF))          // If the line voltages are equal implies that Va = Vb = Vc and motor is not spinning , Assuming 3 bit ADC error
    		{
    			if((!HostController.Start_Stop_Motor) || HostController.Calibrate_Motor)
    			{
					DisableGateDrivers();
					sensorlessTrapController_Init();                                                // Initialize Motor Variables
					SensorlessTrapController.ISCStatus = RUN_IPD;                                                 // If Motor is not spinning Continue with IPD
    			}
    			else
    			{
					ApplicationStatus.previousstate =
   				    ApplicationStatus.currentstate;
					ApplicationStatus.currentstate = MOTOR_STOP;
					HostController.Calibrate_Motor = 0;
    			}
    		}  /********** If there is Significant BEMF at the Start-up Switch to Integration Method ** If motor is in Calibration mode calculate BEMF Threshold ******/
    		else
    		{
    			CommutationStateDetect();               // This Function Identifies the Current commutation states based on the BEMF matching

				if((SensorlessTrapController.ISC_Phase_Match == ISC_ZEROTH_PHASE_MATCH) && (SensorlessTrapController.ISC_Phase_Match_Flag))            // 0th matching state , Set the BEMF clamped value to False
				{
					SensorlessTrapController.ISC_BEMF_Clamped = FALSE;
				}
				if((SensorlessTrapController.ISC_Phase_Match == ISC_FIRST_PHASE_MATCH) && (SensorlessTrapController.ISC_Phase_Match_Flag))            // 1st matching state , Detect the next commutation state based on direction
				{
					SensorlessTrapController.ISC_Phase_Match_Flag = FALSE;
					P4OUT ^= BIT1;
					SensorlessTrapController.ISC_Comm_Match_State = SensorlessTrapController.ISC_Next_State;
				}
				else if((SensorlessTrapController.ISC_Phase_Match == ISC_SECOND_PHASE_MATCH) && (SensorlessTrapController.ISC_Phase_Match_Flag))       // 2nd Matching state , Verify the commutation is sequence is correct
				{
					SensorlessTrapController.ISC_Phase_Match_Flag = FALSE;
					P4OUT ^= BIT1;
					SensorlessTrapController.ISC_Motor_Speed[7] = 0;
					SensorlessTrapController.ISC_BEMF_Diff[7] = 0;
					for (SensorlessTrapController.ISC_Counter = 1; SensorlessTrapController.ISC_Counter < 7 ; SensorlessTrapController.ISC_Counter++ )
					{
						SensorlessTrapController.ISC_Motor_Speed[7] += SensorlessTrapController.ISC_Motor_Speed[SensorlessTrapController.ISC_Counter];
						SensorlessTrapController.ISC_BEMF_Diff[7] += SensorlessTrapController.ISC_BEMF_Diff[SensorlessTrapController.ISC_Counter];
					}
					SensorlessTrapController.ISC_Motor_Speed[7] /= 6;
					SensorlessTrapController.ISC_BEMF_Diff[7] /=6;
					if (SensorlessTrapController.ISC_BEMF_Clamped == FALSE)
					{
								SensorlessTrapController.ISC_BEMF_Diff[7] = ((SensorlessTrapController.ISC_BEMF_Diff[7] * 3) >>1); // If BEMF is visible as pure sinusoidal waveforms across the phases of Inverter bridge multiply 60 degree BEMF diff with 1.5 to get McD wave equivalent value
					}

					if(SensorlessTrapController.ISC_Motor_Speed[7] < SensorlessTrapController.ISC_Threshold_MAX_Speed_Count)
					{
						SensorlessTrapController.ISC_Phase_Match = 0;
					}
					else
					{
						if(HostController.Calibrate_Motor)     // If Motor calibration is commanded Calculate BEMF Threshold
						{
							calculateBEMFThreshold();
						}
						else if((SensorlessTrapController.ISC_Comm_Match_State == SensorlessTrapController.ISC_Current_State)&&
							(SensorlessTrapController.ISC_Motor_Speed[7] < SensorlessTrapController.ISC_Threshold_Speed_Count) && (!HostController.Start_Stop_Motor))
						{
							CatchInitialSpeed();             // If the Motor is spinning already with sufficient speed to produce BEMF switch to Integration method
						}
					}
				}
      		}
            break;

        case RUN_IPD:                                                                   // ISCdone = 1 indicates the motor is stopped
//            P4OUT &= ~BIT1;
			if(HostController.Calibrate_Motor)
			{
				if(SensorlessTrapController.Calibrate_Motor_Start)
				{
					SensorlessTrapController.Calibrate_Motor_Start = FALSE;  // if the Acceleration routine fails set as Fault
				}
				else
				{
					ApplicationStatus.previousstate = ApplicationStatus.currentstate;
					ApplicationStatus.currentstate = FAULT;                                                       // Move to motor Align
					ApplicationStatus.fault = ACCELERATION;
				}
			}
		    SensorlessTrapController.AccelVelocityInit =  SensorlessTrapController.AccelVelocity; // Initial value of openloop acceleration from GUI
			if(SensorlessTrapController.Align_IPD)
			{
	        	TimerB0_Init();
	            TimerA0_Init();
	            TimerA2_Init();
	        	if (SensorlessTrapController.ShuntVariant == TRUE)                                  // Reset current setting after IPD
	            {
	            	SPI_Write(SPI_REG_CSA_CTRL, Reg_Map_Cache.CSA_Control_Reg6);
	            }
				SensorlessTrapController.AlignComplete = 0;
				SensorlessTrapController.StartAlign = FALSE;
				ApplicationStatus.previousstate = ApplicationStatus.currentstate;
				ApplicationStatus.currentstate = MOTOR_ALIGN;                                                       // Move to motor Align
			}
			else
			{
				SensorlessTrapController.SystemCount = 0;
				IPD_Init();
				SensorlessTrapController.IPDStart = TRUE;
				ApplicationStatus.previousstate = ApplicationStatus.currentstate;
				ApplicationStatus.currentstate = MOTOR_IPD;                                                         // Move to motor Initial position detection
				SensorlessTrapController.IPDState = 1;
				SensorlessTrapController.IPDStatus = START;
			}
			break;                    // End of RUN_IPD
        }                             // End of Switch case for ISC
        break;

    case MOTOR_ALIGN:
        if(SensorlessTrapController.AlignComplete == TRUE)                                                              // wait for the align routine to finish
        {
            SensorlessTrapController.CurrentDutyCycle = SensorlessTrapController.StartupDutyCycle;
            SensorlessTrapController.CurrentCommState = SensorlessTrapController.AlignSector;
            UpdateNextCommutation();                                                                                                            // update the commutation sequence to ~90 degrees from the rotor
            UpdateNextCommutation();
            SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);                                         // set the pwm duty cycle to the open loop duty cycle
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_START;                                                               //change to Motor IDLE state and wait for start duty cycle
            if(SensorlessTrapController.Direction_flag)
			{
				SensorlessTrapController.Direction_flag = FALSE;
				SensorlessTrapController.Direction =
								!SensorlessTrapController.Direction;                                                              // This flag monitors for change in direction, If direction change is applied multiple times in quick succession before speed becoming zero, this flag determines the true direction by the time motor is ramped up.				        __delay_cycles(25000000);                              /* A delay of 1s is applied before changing the direction of spin*/
			}
			if((HostController.Start_Stop_Motor) && !(HostController.Calibrate_Motor))
			{
				ApplicationStatus.previousstate =
					   ApplicationStatus.currentstate;
				ApplicationStatus.currentstate = MOTOR_STOP;
			}

        }
        break;
    case MOTOR_IPD:

        if(SensorlessTrapController.IPDStatus == DONE)
        {
            SensorlessTrapController.IPDState = 1;
            while(SensorlessTrapController.IPDState < 7)                                        // find the minimum rise time from IPD
            {
                if(SensorlessTrapController.IPDCurrentRiseValue[
                       SensorlessTrapController.IPDState] >
                   SensorlessTrapController.IPDCurrentRiseValue[
                       SensorlessTrapController.IPDMaxCRV])
                {
                    SensorlessTrapController.IPDMaxCRV =
                        SensorlessTrapController.IPDState;                                                            // set the Maximum Current Rise value if it is detected
                    SensorlessTrapController.IPDState++;                             // skip any large rise times
                }
                else
                {
                    SensorlessTrapController.IPDState++;
                }
            }
            switch(SensorlessTrapController.IPDMaxCRV)                                          // based on the Maximum Current Rise value set the commutation state
            {
            case 1:                             //B-C
                if(SensorlessTrapController.IPDCurrentRiseValue[5] >=
                   SensorlessTrapController.IPDCurrentRiseValue[4])
                {
                    SensorlessTrapController.CurrentCommState = 0x03;                                           //drive state A-B in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x02;                                    //drive state A-C in actual Set commutation
                }
                break;
            case 2:                            //C-A
                if(SensorlessTrapController.IPDCurrentRiseValue[4] >=
                   SensorlessTrapController.IPDCurrentRiseValue[6])
                {
                    SensorlessTrapController.CurrentCommState = 0x01;                                    //drive state B-C in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x06;                                    //drive state B-A in actual Set commutation
                }
                break;
            case 3:                            //A-B
                if(SensorlessTrapController.IPDCurrentRiseValue[6] >=
                   SensorlessTrapController.IPDCurrentRiseValue[5])
                {
                    SensorlessTrapController.CurrentCommState = 0x05;                                    //drive state C-A in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x04;                                    //drive state C-B in actual Set commutation
                }
                break;
            case 4:                             //B-A
                if(SensorlessTrapController.IPDCurrentRiseValue[1] >=
                   SensorlessTrapController.IPDCurrentRiseValue[2])
                {
                    SensorlessTrapController.CurrentCommState = 0x02;                                    //drive state A-C in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x01;                                    //drive state B-C in actual Set commutation
                }
                break;
            case 5:                            //A-C
                if(SensorlessTrapController.IPDCurrentRiseValue[3] >=
                   SensorlessTrapController.IPDCurrentRiseValue[1])
                {
                    SensorlessTrapController.CurrentCommState = 0x04;                                    //drive state C-B in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x03;                                    //drive state A-B in actual Set commutation
                }
                break;
            case 6:                            //C-B
                if(SensorlessTrapController.IPDCurrentRiseValue[2] >=
                   SensorlessTrapController.IPDCurrentRiseValue[3])
                {
                    SensorlessTrapController.CurrentCommState = 0x06;                                    //drive state B-A in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x05;                                    //drive state C-A in actual Set commutation
                }
                break;
            default:
                break;
            }
            if(SensorlessTrapController.Direction == FALSE)                              // for reverse direction , start 180 degrees opposite
            {
                UpdateNextCommutation();
                UpdateNextCommutation();
                UpdateNextCommutation();
            }
        	if (SensorlessTrapController.ShuntVariant == TRUE)                                  // Reset current setting after IPD
            {
            	SPI_Write(SPI_REG_CSA_CTRL, Reg_Map_Cache.CSA_Control_Reg6);
            }
            ADC_Init();
            TIMER_SPD_Init();																		   // Timer Initialization to read the motor electrical speed
            UpdateBEMFADC();                                                                                                // set the ADC MUX for the floating phase
            SensorlessTrapController.CurrentDutyCycle = SensorlessTrapController.StartupDutyCycle;
            SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);                                         // set the pwm duty cycle to the open loop duty cycle
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_START;                                                               //change to Motor IDLE state and wait for start duty cycle
            if(SensorlessTrapController.Direction_flag)
			{
				SensorlessTrapController.Direction_flag = FALSE;
				SensorlessTrapController.Direction =
								!SensorlessTrapController.Direction;                                                              // This flag monitors for change in direction, If direction change is applied multiple times in quick succession before speed becoming zero, this flag determines the true direction by the time motor is ramped up.				        __delay_cycles(25000000);                              /* A delay of 1s is applied before changing the direction of spin*/
			}
			if((HostController.Start_Stop_Motor) && !(HostController.Calibrate_Motor))
			{
				ApplicationStatus.previousstate =
					   ApplicationStatus.currentstate;
				ApplicationStatus.currentstate = MOTOR_STOP;
			}
        }
        break;

    case MOTOR_START:
        if(SensorlessTrapController.AccelDone == TRUE)                                                                  // wait for a flag indicating open loop speed is reached
        {
        	    SensorlessTrapController.ADCchange = FALSE;
	            ADC_Init();
	            TIMER_SPD_Init();																		   // Timer Initialization to read the motor electrical speed
	            UpdateBEMFADC();                                                                                                // set the ADC MUX for the floating phase
	            PWM_SetCommutation(SensorlessTrapController.CurrentCommState);
	            SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);
				SensorlessTrapController.StallDetectCounter = 0;
				ApplicationStatus.previousstate = ApplicationStatus.currentstate;
				ApplicationStatus.currentstate = MOTOR_RUN;                                                         // Move to motor run

        }
        break;

    case MOTOR_RUN:

        if((SensorlessTrapController.Direction_flag)||(HostController.Start_Stop_Motor && (!HostController.Calibrate_Motor)))
        {
            if(SensorlessTrapController.SpeedDivider > SensorlessTrapController.RampRateDelay)
            {
                SensorlessTrapController.SpeedDivider = 0;
                if(SensorlessTrapController.CurrentDutyCycle >= (SensorlessTrapController.MinOffDutyCycle))  // Addidng hysteresis for going to Motor direction state
                {
                    SensorlessTrapController.CurrentDutyCycle -= RAMP_RATE;
                }
                else
                {
                	if(SensorlessTrapController.Direction_flag)
                   	{
                		SensorlessTrapController.Direction_flag = FALSE;
                		SensorlessTrapController.Direction =
                		                !SensorlessTrapController.Direction;                                                              // This flag monitors for change in direction, If direction change is applied multiple times in quick succession before speed becoming zero, this flag determines the true direction by the time motor is ramped up.				        __delay_cycles(25000000);                              /* A delay of 1s is applied before changing the direction of spin*/
                        if(SensorlessTrapController.Direction == TRUE)
                        {
                            P1OUT |= BIT0;                                                         /* Turn  on LED1 */
                            P4OUT &= ~BIT7;                                                                                 /* Turn off LED 2*/
                        }
                        else
                        {
                            P1OUT &= ~BIT0;                                                        /* Turn  off LED1 */
                            P4OUT |= BIT7;                                                                                  /* Turn on LED 2*/
                        }
                        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
                        ApplicationStatus.currentstate = MOTOR_STOP;                                     // move to motor initialization
                   	}
                	if(HostController.Start_Stop_Motor)
                	{
                		ApplicationStatus.previousstate =
               	               ApplicationStatus.currentstate;
                		ApplicationStatus.currentstate = MOTOR_STOP;
                	}
                }
            }
        }
        else if(SensorlessTrapController.SpeedChange &&
                (SensorlessTrapController.SpeedDivider > SensorlessTrapController.RampRateDelay))                                                                 // if the commanded speed is changed and wait for the accel divisor
        {
            if(SensorlessTrapController.TargetDutyCycle >
               SensorlessTrapController.CurrentDutyCycle)                                                                      // If motor is accelerating , increase duty cycle
            {
                SensorlessTrapController.CurrentDutyCycle += RAMP_RATE;
            }
            else if(SensorlessTrapController.TargetDutyCycle <
                    SensorlessTrapController.CurrentDutyCycle)                                                                      // If motor is descelerating , decrease duty cycle
            {
                SensorlessTrapController.CurrentDutyCycle -= RAMP_RATE;
            }
            else if(SensorlessTrapController.TargetDutyCycle ==
                    SensorlessTrapController.CurrentDutyCycle)                                                                       // if motor speed is constant , keep duty cycle intact
            {
                SensorlessTrapController.SpeedChange = FALSE;                                                                                                                           // Speed change is triggered in Read pot speed function
                if(SensorlessTrapController.Calibrate_Motor_run)
				{
					ApplicationStatus.previousstate =
						   ApplicationStatus.currentstate;
					ApplicationStatus.currentstate = MOTOR_STOP;
				}
            }
            SensorlessTrapController.SpeedDivider = 0;                                                                                                                              // Speed divider counts the
        }
        break;

    case MOTOR_STOP:
        DisableGateDrivers();
    	ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        ApplicationStatus.currentstate = SYSTEM_IDLE;                         // move to motor initialization
        break;

    default: break;

    case FAULT:
        DisableGateDrivers();                                                     /* Disable Gate Drivers when a fault is triggered*/
        /*handles the faults and sets fault LEDS*/
        switch(ApplicationStatus.fault)
        {
        case NOFAULT:
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = SYSTEM_INIT;                                                         // Move to motor run
        	break;

        case VOLTAGE:
            P1OUT ^= BIT0;                                              /* Toggle LED1*/
            P4OUT ^= BIT7;                                              /* Toggle LED2 */
            break;


        case MOTOR_STALL:
            P1OUT |= BIT0;                                              /* Turn ON LED1*/
            P4OUT ^= BIT7;                                              /* Toggle LED2 */

            break;
        case OVERCURRENT:
            P1OUT ^= BIT0;                                              /* Toggle LED1 */
            P4OUT |= BIT7;                                              /* Turn ON LED2 */

            break;
        case OVERTEMPERATURE:
            P1OUT |= BIT0;                                              /* Turn ON LED1*/
            P4OUT &= ~BIT7;                                             /* Turn OFF LED2*/

            break;
        case GATE_DRIVER:
            P1OUT |= BIT0;                                              /* Turn ON LED1*/
            P4OUT |= BIT7;                                              /* Turn ON LED2*/

            break;
        case POWER_SUPPLY:
        	P1OUT |= BIT0;                                              /* Turn ON LED1*/
        	P4OUT |= BIT7;                                              /* Turn ON LED2*/
        	break;

        case UNKNOWN:                                                                     /* If the fault is Triggered from sources other than described above read the Fault register IC values from Fault Variables */
        	P1OUT |= BIT0;                                             /* Turn ON LED1*/
            P4OUT &= ~BIT7;                                             /* Turn OFF LED2*/

            break;

        default: break;
        }
    }
}

void HostController_StateMachine(void)
{
/*function
 * DRV8x_State_Machine(void)
 * Handles the state machine and transitions of the Application
 * */
	uint8_t j;
	 if(mdbuserial_usb_rxReceived)
	 {
		mdbuserial_usb_rxReceived = FALSE;
		unsigned int rxByteCount = cdcReceiveDataInBuffer(mdbuSerial_usbRxBuffer, sizeof(mdbuSerial_usbRxBuffer), CDC0_INTFNUM);

		if(rxByteCount)
		{
			for (j = 0; j < rxByteCount; j++)
			{
				mdbuSerial_handleRX(mdbuSerial_usbRxBuffer[j]);
			}
		}
	 }

 switch(HostControl_Status)
 {
	case HOST_IDLE:
	{

		uint8_t *data,length;
		uint8_t i,cmd;
		HostControl_Status = HOST_IDLE;
		if(mdbuSerial_rxStateStop)
		{

			cmd =  mdbuSerial_RxPkt.pkt_cmd;
			data =  mdbuSerial_RxPkt.pkt_data;
			length = mdbuSerial_RxPkt.pkt_len;
			mdbuSerial_rxStateStop = 0;
			for(i=0;i<MDBUSERIAL_NUM_CALLBACKS && MDBUSERIAL_NUM_CALLBACKS != 0;i++)
			{
				if(mdbuserial_callbacktable[i].cmd == cmd)
				{
					mdbuserial_callbacktable[i].callback(data, (size_t)(length));
					//goto packet_clean;
				}
			}
			if(HostController.EnabledGateDrivers)
			{
				HostControl_Status = HOST_ACTIVE;
				if (SensorlessTrapController.SPIVariant == TRUE) {
                    drv83xx_regRestoreFromCache();
                }
			}
		}
	}

	   break;
	 case HOST_ACTIVE:
	 {
		 uint8_t *data,length;
		 uint8_t i,cmd;

		 if(mdbuSerial_rxStateStop)
		 {
			cmd =  mdbuSerial_RxPkt.pkt_cmd;
			data =  mdbuSerial_RxPkt.pkt_data;
			length = mdbuSerial_RxPkt.pkt_len;

			mdbuSerial_rxStateStop = 0;
			for(i=0;i<MDBUSERIAL_NUM_CALLBACKS && MDBUSERIAL_NUM_CALLBACKS != 0;i++)
			{
				if(mdbuserial_callbacktable[i].cmd == cmd)
				{
					mdbuserial_callbacktable[i].callback(data, (size_t)(length));
					//goto packet_clean;
				}
			}
			if(HostController.EnabledGateDrivers == 0)
				HostControl_Status = HOST_IDLE;
		}
	  break;
 	}
	}


}

/*function
 * main()
 * Initializes the Application and calls periodically the state machine function
 * */
int main()
{
	Init_Application();                 // Initialize the State Machine
    mdbuSerial_init(); 					// Initialise MDBU Serial Protocol
    HostControllerInit();				// Initialise Host Controller
    while(1)
    {
    	DRV8x_State_Machine();        // call background state machine
    	HostController_StateMachine();
    }
}
