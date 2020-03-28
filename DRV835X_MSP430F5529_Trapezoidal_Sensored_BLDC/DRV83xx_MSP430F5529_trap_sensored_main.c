/*************************************************************************
 * @file   drv83xx_MSP430F5529_trap_sensored_main.c
 * @author MDBU Software Team
 * @brief  Trapezoidal control of BLDC based on Hall-sensor input
 * @note   Copyright (c) 2016 Texas Instruments Incorporated.
 *         All rights reserved.
 ****************************************************************************/

#include "global.h"
#include "mdbu_global.h"
#include "stdio.h"

// Controller
SENSORED_TRAP_Obj sensoredTrapController;
APPLICATION_STATUS applicationStatus;

//E-Bike
BIKE_CONTROLLER bikeController;
BIKE_STATUS Bike_Status;

// Tracking cmds sent by host
uint8_t CMDS[100];

// Host Controller
HOSTCONTROL_STATUS HostControl_Status;
HOST_CONTROLLER_Obj HostController;
extern MDBUSERIAL_RXSTATE mdbuSerial_RxState;
extern mdbuSerial_RxPacket mdbuSerial_RxPkt;

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
void DRV8x_StateMachine(void)
{
	/* Move to FAULT state if any fault reported by the ISR. ISR is also
	 * changing the state to FAULT. But one extra level of protection to prevent
	 * over writing of the state variable during context switchings */
	if (applicationStatus.fault)
	{
        applicationStatus.previousstate = applicationStatus.currentstate;
        applicationStatus.currentstate = FAULT;
	}

    switch(applicationStatus.currentstate)                    							 	/* process current motor state*/
    {
    case SYSTEM_INIT:
        sensoredTrapController_Init();                                                     /*Initialize Motor Variables*/

        if(HostController.EnabledGateDrivers)
        {
            drv83xx_regRestoreFromCache();													   /* Restore device register values from the cached ones */
        }

        applicationStatus.previousstate = applicationStatus.currentstate;                  /* Move to Motor INIT
                                                                                              state and initialize Motor variables*/
        applicationStatus.currentstate = SYSTEM_IDLE;
        break;
    case SYSTEM_IDLE:
        /* If received MOTOR START command, move to MOTOR_INIT */
        if(HostController.StartStopMotor == 0)
        {
            applicationStatus.previousstate = applicationStatus.currentstate;
            applicationStatus.currentstate = MOTOR_IDLE;
	    }
        if(HostController.CalibrateHall)
        {
        	applicationStatus.previousstate = applicationStatus.currentstate;
            applicationStatus.currentstate = HALL_ALIGN;
	    }
        break;
    case MOTOR_IDLE:
        /* Update direction on LEDs*/
        if(sensoredTrapController.Direction == TRUE)
        {
            P1OUT |= BIT0;                                                              /* Turn  on LED1 */
            P4OUT &= ~BIT7;                                                             /* Turn off LED 2*/
        }
        else
        {
            P1OUT &= ~BIT0;                                                             /* Turn  off LED1 */
            P4OUT |= BIT7;                                                              /* Turn on LED 2*/
        }
        if (sensoredTrapController.TargetDutyCycle > sensoredTrapController.MINDutyCycle)
        {
            if((HostController.CalibrateHall)|| ((sensoredTrapController.HallCalibComplete == FALSE)  && (sensoredTrapController.AutoHallCalib)))
            {
    			sensoredTrapController.CurrentDutyCycle = 0;
                SetPWMDutyCycle(sensoredTrapController.CurrentDutyCycle);
    			sensoredTrapController.HallAlignInterruptFlag = FALSE;
                sensoredTrapController.HallAlignStartFlag = TRUE;
                sensoredTrapController.HallAligncounter = 0;
                sensoredTrapController.HallAlignState = 1;
                Hall_AlignSetState(sensoredTrapController.HallAlignState);
            	applicationStatus.previousstate = applicationStatus.currentstate;
                applicationStatus.currentstate = HALL_ALIGN;
    	    }
            /* Move to MOTOR_RAMP_UP if MOTOR START received*/
            else if(HostController.StartStopMotor == 0)
            {
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
            	}
            	UpdateNextCommutation();
                TIMER_SPD_Init();																		   // Timer Initialization to read the motor electrical speed

                applicationStatus.previousstate = applicationStatus.currentstate;
                applicationStatus.currentstate = MOTOR_RAMP_UP;

            }
         }
         break;
    case HALL_ALIGN:
    	if(sensoredTrapController.HallAlignDoneFlag)                // Wait until the first Phase Vector A Angle 0 deg is set to read 1st Hall status
    	{
				sensoredTrapController.HallAlignDoneFlag = FALSE;
				sensoredTrapController.HallAligncounter = 0;
				sensoredTrapController.HallAlignValue[sensoredTrapController.HallAlignState] = ((P2IN & BIT0) >> HALL_SHIFT_FOR_PIN_A) |
																		((P2IN & BIT2) >> HALL_SHIFT_FOR_PIN_B) |
																		((P2IN & BIT6) >> HALL_SHIFT_FOR_PIN_C);        /* get hall effect pin status and move */
				 sensoredTrapController.HallAlignState++;              // Set second phase vector Angle 60 deg to read 2nd Hall state
				 sensoredTrapController.CurrentDutyCycle = 0;
				 SetPWMDutyCycle(sensoredTrapController.CurrentDutyCycle);
				 Hall_AlignSetState(sensoredTrapController.HallAlignState);
				 if(sensoredTrapController.HallAlignState <= 3)       // If only 1 phase vector is applied, Apply 2nd vector , if second is applied aplly 1st vector again
				 {
						sensoredTrapController.HallAlignStartFlag = TRUE;
						sensoredTrapController.HallAligncounter = 0;
				 }
				 else												 // If both the Voltage vectors are applied , Calibrate the Hall signal shift based on Hall readings
				 {
					 if((sensoredTrapController.HallAlignInterruptFlag) && (sensoredTrapController.HallAlignValue[1] == sensoredTrapController.HallAlignValue[3])) // if rotor moves from one state to other
					 {
						sensoredTrapController.HallAlignInterruptFlag = FALSE;
						sensoredTrapController.CurrentDutyCycle = 0;
						SetPWMDutyCycle(sensoredTrapController.CurrentDutyCycle);
						DisableGateDrivers();
						if((sensoredTrapController.HallAlignValue[1] == 3) || (sensoredTrapController.HallAlignValue[1] == 4)) // MSB bit different from middle & LSB bits - HALL C
						{
							sensoredTrapController.ShiftForPINC = 4;      // Bit Shift needed to reposition PinC as Hall C
							if((sensoredTrapController.HallAlignValue[2] == 2) || (sensoredTrapController.HallAlignValue[2] == 5)) // LSB bit position changed - Hall A
							{
								sensoredTrapController.ShiftForPINA = 0;  // Bit Shift needed To reposition PinA as Hall A
								sensoredTrapController.ShiftForPINB = 1;  // Bit Shift needed To reposition PinB as Hall B
							}
							else if((sensoredTrapController.HallAlignValue[2] == 1) || (sensoredTrapController.HallAlignValue[2] == 6)) // Middle bit position changed - Hall A
							{
								sensoredTrapController.ShiftForPINB = 2;   // Bit Shift needed To reposition PinB as Hall A
								sensoredTrapController.ShiftForPINA = 1;   // Bit Shift needed To reposition PinA as Hall B
							}
						}
						else if((sensoredTrapController.HallAlignValue[1] == 2) || (sensoredTrapController.HallAlignValue[1] == 5)) // Middle bit different from MSB & LSB bits - HALL C
						{
							sensoredTrapController.ShiftForPINB = 0;    // Bit Shift needed To reposition PinB as Hall C
							if((sensoredTrapController.HallAlignValue[2] == 3) || (sensoredTrapController.HallAlignValue[2] == 4)) // LSB bit position changed - Hall A
							{
								sensoredTrapController.ShiftForPINA = 0;  // Bit Shift needed To reposition PinA as Hall A
								sensoredTrapController.ShiftForPINC = 5;  // Bit Shift needed To reposition PinC as Hall B
							}
							else if((sensoredTrapController.HallAlignValue[2] == 6) || (sensoredTrapController.HallAlignValue[2] == 1)) // MSB bit position changed - Hall A
							{
								sensoredTrapController.ShiftForPINC = 6;   // Bit Shift needed To reposition PinC as Hall A
								sensoredTrapController.ShiftForPINA = 1;   // Bit Shift needed To reposition PinA as Hall B
							}
						}
						else if((sensoredTrapController.HallAlignValue[1] == 1) || (sensoredTrapController.HallAlignValue[1] == 6)) // LSB bit different from middle & MSB bits - HALL C
						{
							sensoredTrapController.ShiftForPINA = 2;   // Bit Shift needed To reposition PinA as Hall C
							if((sensoredTrapController.HallAlignValue[2] == 5) || (sensoredTrapController.HallAlignValue[2] == 2)) // MSB bit position changed - Hall A
							{
								sensoredTrapController.ShiftForPINC = 6;  // Bit Shift needed To reposition PinC as Hall A
								sensoredTrapController.ShiftForPINB = 1;  // Bit Shift needed To reposition PinB as Hall B
							}
							else if((sensoredTrapController.HallAlignValue[2] == 3) || (sensoredTrapController.HallAlignValue[2] == 4)) // Middle bit position changed - Hall A
							{
								sensoredTrapController.ShiftForPINB = 2;   // Bit Shift needed To reposition PinB as Hall A
								sensoredTrapController.ShiftForPINC = 5;   // Bit Shift needed To reposition PinC as Hall B
							}

						}
						sensoredTrapController.HallCalibComplete = TRUE;
						HostController.CalibrateHall = FALSE;                               // End the calibration
						P2IFG |= 0x00;
						P2IES |= (P2IN & (BIT0 | BIT2 | BIT6));     // Port Interrupts edges selected based on the Current position of Hall Sensors
						applicationStatus.previousstate = applicationStatus.currentstate;                  /* Move to System Idle state and initialize Motor variables*/
						applicationStatus.currentstate = SYSTEM_IDLE;
					 }
					else                                                 // If no interrupt is observed during hall calibration, rotor is blocked / no sufficient Hall align time
					{
						sensoredTrapController.HallAlignInterruptFlag = FALSE;
						applicationStatus.previousstate = applicationStatus.currentstate;
						applicationStatus.currentstate = FAULT;
						applicationStatus.fault = HALL_CALIBRATION;
					}
				}
    	}
        break;
    case MOTOR_RUN:

        if(sensoredTrapController.Direction == TRUE)
        {
            P1OUT |= BIT0;                                                                   /* Turn  on LED1 */
            P4OUT &= ~BIT7;                                                                  /* Turn off LED 2*/
        }
        else
        {
            P1OUT &= ~BIT0;                                                                  /* Turn  off LED1 */
            P4OUT |= BIT7;                                                                   /* Turn on LED 2*/
        }

        /* Move to MOTOR_RAMP_DOWM if MOTOR STOP received*/
        if(HostController.StartStopMotor == 1)
        {
            applicationStatus.previousstate = applicationStatus.currentstate;
            applicationStatus.currentstate = MOTOR_RAMP_DOWN;
        }

        /*Measure every 100th PWM periods for changed speed input
           and change to acceleration or brake state*/
        if(sensoredTrapController.Accelerationcounter >
           SPEED_INPUT_SAMP_INTERVAL)
        {
            sensoredTrapController.Accelerationcounter = 0;

            if(sensoredTrapController.TargetDutyCycle <
               sensoredTrapController.CurrentDutyCycle)
            {
                applicationStatus.previousstate =
                    applicationStatus.currentstate;
                applicationStatus.currentstate = MOTOR_RAMP_DOWN;
                P1OUT &= ~BIT0;                                                              /* Turn  off LED1 */
                P4OUT &= ~BIT7;                                                              /* Turn off LED 2*/
            }
            else if(sensoredTrapController.TargetDutyCycle >
                    sensoredTrapController.CurrentDutyCycle)
            {
                applicationStatus.previousstate =
                    applicationStatus.currentstate;
                applicationStatus.currentstate = MOTOR_RAMP_UP;
                P1OUT &= ~BIT0;                                                              /* Turn  off LED1 */
                P4OUT &= ~BIT7;                                                              /* Turn off LED 2*/
            }
            else
            {
              if(sensoredTrapController.CurrentDutyCycle < sensoredTrapController.MINDutyCycle)
              {
                    applicationStatus.previousstate =
                        applicationStatus.currentstate;
                    applicationStatus.currentstate = MOTOR_RAMP_DOWN;
                    P1OUT &= ~BIT0;                                                              /* Turn  off LED1 */
                    P4OUT &= ~BIT7;                                                              /* Turn off LED 2*/
              }
            }
            if (sensoredTrapController.TargetDutyCycle > sensoredTrapController.MAXDutyCycle)
            {
                    sensoredTrapController.TargetDutyCycle = sensoredTrapController.MAXDutyCycle;
            }
        }
        break;
    case MOTOR_STOP:
        applicationStatus.previousstate = applicationStatus.currentstate;
        applicationStatus.currentstate = SYSTEM_IDLE;
        DisableGateDrivers();
        break;
    case FAULT:
        DisableGateDrivers();                                                                /* Disable Gate Drivers when a fault is triggered*/
        /*handles the faults and sets fault LEDS*/
        switch(applicationStatus.fault)
        {
        case NOFAULT:           break;
        case HALLSENSORINVALID:
            P1OUT &= ~BIT0;                                                                  /* Turn OFF LED1*/
            P4OUT |= BIT7;                                                                   /* Turn ON LED2*/

            break;
        case MOTOR_STALL:
            P1OUT |= BIT0;                                                                   /* Turn ON LED1*/
            P4OUT ^= BIT7;                                                                   /* Toggle LED2 */

            break;
        case VOLTAGE:

            P1OUT ^= BIT0;                                      							 /* Toggle LED1*/
            P4OUT ^= BIT7;                                      							 /* Toggle LED2 */

            break;
        case OVERCURRENT:
            P1OUT ^= BIT0;                                      							 /* Toggle LED1 */
            P4OUT |= BIT7;                                      							 /* Turn ON LED2 */

            break;
        case OVERTEMPERATURE:
            P1OUT |= BIT0;                                      							 /* Turn ON LED1*/
            P4OUT &= ~BIT7;                                    								 /* Turn OFF LED2*/

            break;
        case GATE_DRIVER:
            P1OUT |= BIT0;                                     								 /* Turn ON LED1*/
            P4OUT |= BIT7;                                     								 /* Turn ON LED2*/

            break;

        case POWER_SUPPLY:
            P1OUT |= BIT0;                                     								 /* Turn ON LED1*/
            P4OUT |= BIT7;                                     								 /* Turn ON LED2*/

            break;

        case UNKNOWN:                                           							 /* If the fault is Triggered from sources other than described above read the Fault register IC values from Fault Variables */
            P1OUT &= ~BIT0;                                                                  /* Turn OFF LED1*/
            P4OUT &= ~BIT7;                                                                  /* Turn OFF LED2*/

            break;

        default: break;
        }
    }
}


/*function
 * BikeController_StateMachine(void)
 * Handles the Bike state machine
 * */
BikeController_StateMachine(void)
{
    switch(Bike_Status.current)
    {
        case SYSTEM_INIT:
	    bike_init();			// Initialize UART/THROTTLE/BREAK
            Bike_Status.current = BIKE_IDLE;		// Move to next state
	    Bike_Status.previous = SYSTEM_INIT;
            break;
        case SYSTEM_IDLE:
            // This state should be used to save power.  If the bike is in idle we are waiting for an interrupt
	    // Because Bike_Status cannot be set in an interrupt we must check if an interrupt has occured
	    if (1) // ("break interrupt" or "throttle interrupt")
		Bike_Status.previous = SYSTEM_IDLE;
	    	Bike_Status.current = ACCELERATING;
            break;
	case SWITCHING:
	    // Precautions must be taken during the switching between accelerating and regenerating.
	    // Must deactive gates and ...
	    if (Bike_Status.previous == ACCELERATING)
		Bike_Status.current = REGENERATING;
	    Bike_Status.previous = SWITCHING;
	    break;
        case ACCELERATING:
            // ????  drv83xx_regRestoreFromCache();
            ReadPotiSpeed();
	    // update_speed();
	    if (0) // ("break interrupt")
		Bike_Status.previous = ACCELERATING;
		Bike_Status.current = SWITCHING;
            break;
        case REGENERATING:
            // Set Mode to 6 PWM
            //Bike_Status = BIKE_IDLE;
	    if (0) // ("throttle interrupt"
		Bike_Status.previous = REGENERATING;
		Bike_Status.current = SWITCHING;
            break;
    }

}

/*function
 * HostController_StateMachine(void)
 * Handles the HOST state machine
 * */
void HostController_StateMachine(void)
{
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
					}
				}
				if(HostController.EnabledGateDrivers)
				{
					HostControl_Status = HOST_ACTIVE;
					drv83xx_regRestoreFromCache();                      // Default values are read by GUI after enabling the pre drivers
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
					    //printf("%X\n",cmd);
						mdbuserial_callbacktable[i].callback(data, (size_t)(length));
						for(j = 0; j < 99; j++)
						{
						    CMDS[j + 1] = CMDS[j];
						}
						CMDS[j] = cmd;
					}
				}
				if(HostController.EnabledGateDrivers == 0)
				{
					HostControl_Status = HOST_IDLE;
				}
			}
			break;
		}
	}
}

/*function
 * void main()
 * Initializes the Application and calls periodically the state machine function
 * */
void main()
{
    Application_Init();                 	/* Initialize application state machine states*/
    mdbuSerial_init(); 				// Initialize MDBU Serial Protocol
    HostControllerInit();			// Initialize Host Controller
    BikeControllerInit();               	// Initialize Bike Controller
    Bike_Status.current = BIKE_INIT;          	// Initialize State
    Bike_Status.previous = BIKE_INIT;		// Set previous bike status to be safe
    while(1)
    {
        DRV8x_StateMachine();                       	 /* call background state machine */
        BikeController_StateMachine();
        // HostController_StateMachine();
    }
}
