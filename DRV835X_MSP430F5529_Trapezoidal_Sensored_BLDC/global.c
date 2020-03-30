/**************************************************************************
 * @file        global.c
 * @author      MDBU Software Team
 * @brief       global functions used for application
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 ******************************************************************************/
#include "global.h"

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


// Controller
extern SENSORED_TRAP_Obj sensoredTrapController;
extern APPLICATION_STATUS applicationStatus;
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

void EnableGateDrivers()
{
    /* Gate Drive Enable using Port 1.6 */
	//P1OUT |= BIT6;          // Enable Gate Drivers
}
void DisableGateDrivers()
{
	sensoredTrapController.CurrentDutyCycle = 0;
	SetPWMDutyCycle(sensoredTrapController.CurrentDutyCycle);
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
		P2OUT &= ~(BIT5);
		P2SEL &= ~(BIT5);
		P1OUT &= ~BIT2  ; //  Make INLC as GPIO High to A the brakes
	}
}
/*function
 * SetPWMDutyCycle(UINT16 PWMDutyCycle)
 * Sets the PWM Duty Cycle
 * INPUT: DutyCycle Counter Value
 * */
void SetPWMDutyCycle(uint16_t PWMDutyCycle)
{
    TA0CCR1 = PWMDutyCycle;
    TA0CCR2 = PWMDutyCycle;
    TA0CCR3 = PWMDutyCycle;
    TA0CCR4 = PWMDutyCycle;
    TA2CCR1 = PWMDutyCycle;
    TA2CCR2 = PWMDutyCycle;
}

void UpdateNextCommutation()
{

    sensoredTrapController.currentHallstate = ((P2IN & BIT0) << sensoredTrapController.ShiftForPINA)  |
                                              ((P2IN & BIT2) >> sensoredTrapController.ShiftForPINB) |
                                              ((P2IN & BIT6) >> sensoredTrapController.ShiftForPINC);        /* get hall effect pin status and move */
    switch(sensoredTrapController.currentHallstate)
    {
    case 0:
        //disallowed State leads to Hall sensor Fault
        applicationStatus.previousstate = applicationStatus.currentstate;
        applicationStatus.currentstate = FAULT;
		sensoredTrapController.HallCalibComplete = FALSE;
        applicationStatus.fault = HALLSENSORINVALID;
        break;
    case 1:
        //check for Direction and sets commutation
        if(sensoredTrapController.Direction == TRUE)
        {
            PWM_SetCommutation(sensoredTrapController.currentHallstate);
        }
        else
        {
            PWM_SetCommutation(6);
        }
        //RPM count
        sensoredTrapController.RotationCount++;

        // Read Electrical Speed
        if((applicationStatus.currentstate == MOTOR_RUN) ||
           (applicationStatus.currentstate == MOTOR_RAMP_UP) ||
           (applicationStatus.currentstate == MOTOR_RAMP_DOWN) ||
		   (applicationStatus.currentstate == MOTOR_DIRECTION))
        {
          ReadSPDFDBK();
        }
        break;

    case 2:
        if(sensoredTrapController.Direction == TRUE)
        {
            PWM_SetCommutation(sensoredTrapController.currentHallstate);
        }
        else
        {
            PWM_SetCommutation(5);
        }
        break;

    case 3:
        if(sensoredTrapController.Direction == TRUE)
        {
            PWM_SetCommutation(sensoredTrapController.currentHallstate);
        }
        else
        {
            PWM_SetCommutation(4);
        }
        break;

    case 4:
        if(sensoredTrapController.Direction == TRUE)
        {
            PWM_SetCommutation(sensoredTrapController.currentHallstate);
        }
        else
        {
            PWM_SetCommutation(3);
        }
        break;

    case 5:
        if(sensoredTrapController.Direction == TRUE)
        {
            PWM_SetCommutation(sensoredTrapController.currentHallstate);
        }
        else
        {
            PWM_SetCommutation(2);
        }
        break;

    case 6:
        if(sensoredTrapController.Direction == TRUE)
        {
            PWM_SetCommutation(sensoredTrapController.currentHallstate);
        }
        else
        {
            PWM_SetCommutation(1);
        }
        break;

    case 7:
        applicationStatus.previousstate = applicationStatus.currentstate;
        applicationStatus.currentstate = FAULT;
		sensoredTrapController.HallCalibComplete = FALSE;
        applicationStatus.fault = HALLSENSORINVALID;
        break;

    default:
        break;
    }
}
/*function
 * Hall_AlignSetState(uint8_t hallState)
 * Set PWM commutation
 * INPUT: Hall state
 * */
void Hall_AlignSetState(uint8_t hallState)
{
    if (sensoredTrapController.PWM_Mode == 0)
    {
        /* Implementing Synchronous PWM i.e. to Toggle between High side and low side of a Phase with Dead Band*/

        switch(hallState)
        {
        case 1:          /* U+ V- W- */

        	DisableGateDrivers();

            P2SEL |= BIT4 | BIT5;                                      /* Select Synchronous PWM for U phase*/
            P1OUT |= BIT4;                                             /* Set Low side of V phase */
            P1OUT |= BIT2;                                             /* Set Low side of W phase */
            break;
        case 2:         /* U+ V+ W- */

            P2SEL |= BIT4 | BIT5;                                      /* Select Synchronous PWM for U phase*/
        	P1SEL |= BIT4 | BIT5;                                      /* Select Synchronous PWM for V phase*/
            P1OUT |= BIT2;                                             /* Set Low side of W phase */
            break;
        case 3:          /* U+ V- W- */

        	P1SEL &= ~(BIT4 | BIT5);                                      /* Select Synchronous PWM for V phase*/
            P2SEL |= BIT4 | BIT5;                                      /* Select Synchronous PWM for U phase*/
            P1OUT |= BIT4;                                             /* Set Low side of V phase */
            P1OUT |= BIT2;                                             /* Set Low side of W phase */
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
void PWM_SetCommutation(uint8_t hallState)
{
    if (sensoredTrapController.PWM_Mode == 0)
    {

        /* Implementing Synchronous PWM i.e. to Toggle between High side and low side of a Phase with Dead Band*/

        switch(hallState)
        {
        case 1:       	  /* W-U */

            /* Reset switches for phase U (HIGH) */
            P2OUT &= ~(BIT5);                                    /* Reset bits P2.4 , P2.5  */
            P2SEL &= ~(BIT5);                                    /* Select P2.4 , P2.5 as I/O Function for Phase U*/

            /* Reset switches for phase V (LOW-HIGH)*/
            P1OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P1.4 , P1.5  */
            P1SEL &= ~(BIT4 | BIT5);                                    /* Select P1.4 , P1.5 as I/O Function for Phase V*/

        	P1SEL |= BIT2 | BIT3;                                  /* Select Synchronous PWM for W phase*/
            P2OUT |= BIT4;                                         /* Set Low side of U phase */

            break;
        case 2:          /* U-V */

            /* Reset switches for phase V (LOW-HIGH)*/
            P1OUT &= ~(BIT5);                                    /* Reset bits P1.4 , P1.5  */
            P1SEL &= ~(BIT5);                                    /* Select P1.4 , P1.5 as I/O Function for Phase V*/
            /* Reset switches for phase W (LOW-HIGH)*/

            P1OUT &= ~(BIT2 | BIT3);                                    /* Reset bits P1.2 , P1.3  */
            P1SEL &= ~(BIT2 | BIT3);                                    /* Select P1.2 , P1.3 as I/O Function for Phase W*/

        	P2SEL |= BIT4 | BIT5;                                  /* Select Synchronous PWM for U phase*/
            P1OUT |= BIT4;                                         /* Set Low side of V phase */
            break;
        case 3:          /* W-V */

            /* Reset switches for phase U (LOW-HIGH) */
            P2OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P2.4 , P2.5  */
            P2SEL &= ~(BIT4 | BIT5);                                    /* Select P2.4 , P2.5 as I/O Function for Phase U*/

            /* Reset switches for phase V (LOW-HIGH)*/
            P1OUT &= ~(BIT5);                                    /* Reset bits P1.4 , P1.5  */
            P1SEL &= ~(BIT5);                                    /* Select P1.4 , P1.5 as I/O Function for Phase V*/

        	P1SEL |= BIT2 | BIT3;                                   /* Select Synchronous PWM for W phase*/
            P1OUT |= BIT4;                                          /* Set Low side of V phase */
            break;
        case 4:          /* V-W */

            /* Reset switches for phase U (LOW-HIGH) */
            P2OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P2.4 , P2.5  */
            P2SEL &= ~(BIT4 | BIT5);                                    /* Select P2.4 , P2.5 as I/O Function for Phase U*/

            /* Reset switches for phase W (LOW-HIGH)*/

            P1OUT &= ~(BIT3);                                    /* Reset bits P1.2 , P1.3  */
            P1SEL &= ~(BIT3);                                    /* Select P1.2 , P1.3 as I/O Function for Phase W*/

        	P1SEL |= BIT4 | BIT5;                                   /* Select Synchronous PWM for V phase*/
            P1OUT |= BIT2;                                          /* Set Low side of W phase */
            break;
        case 5:         /* V-U */

            /* Reset switches for phase U (LOW-HIGH) */
            P2OUT &= ~(BIT5);                                    /* Reset bits P2.4 , P2.5  */
            P2SEL &= ~(BIT5);                                    /* Select P2.4 , P2.5 as I/O Function for Phase U*/

            /* Reset switches for phase W (LOW-HIGH)*/

            P1OUT &= ~(BIT2 | BIT3);                                    /* Reset bits P1.2 , P1.3  */
            P1SEL &= ~(BIT2 | BIT3);                                    /* Select P1.2 , P1.3 as I/O Function for Phase W*/

        	P1SEL |= BIT4 | BIT5;                                   /* Select Synchronous PWM for V phase*/
            P2OUT |= BIT4;                                          /* Set Low side of U phase */
            break;
        case 6:         /* U-W */

            /* Reset switches for phase V (LOW-HIGH)*/
            P1OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P1.4 , P1.5  */
            P1SEL &= ~(BIT4 | BIT5);                                    /* Select P1.4 , P1.5 as I/O Function for Phase V*/

            /* Reset switches for phase W (LOW-HIGH)*/
            P1OUT &= ~(BIT3);                                    /* Reset bits P1.2 , P1.3  */
            P1SEL &= ~(BIT3);                                    /* Select P1.2 , P1.3 as I/O Function for Phase W*/

        	P2SEL |= BIT4 | BIT5;                                   /* Select Synchronous PWM for of U phase*/
            P1OUT |= BIT2;                                          /* Set Low side of W phase */
            break;
        default:
        	DisableGateDrivers();

        }
    }
}

/*function
 * ReadPotiSpeed()
 * Triggers and ADC4 sample and low pass filters lower 3 bits
 * is the speed input (potentiometer) for the applications
 * */
void ReadPotiSpeed()
{
    TA1CCTL0 &= ~CCIE;                      /* Disable Read Vcc ISR as it can conficts with SPI*/
    ADC12CTL0 |= ADC12SC;                   // Start ADC conversion to read Pot
    ADC12IE = ADC12IE0;                     // Enable Interrupt after last conversion
    TA1CCTL0 |= CCIE;                       /* Re-enable Read Vcc ISR period match interrupt */
}

/*function
 * ReadVCC()
 * Triggers and internal VCC ADC sample and evaluates for over or under voltage fault
 * */
void ReadVCC()
{
    ADC12CTL0 |= ADC12SC;                   // Start conversion
    ADC12IE = ADC12IE1;                     // Enable Interrupt after last conversion
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

    sensoredTrapController.deviceIDADC = ADC12MEM0 & 0x0FFF;       /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/
    sensoredTrapController.deviceIDADC >>= PWM_FACTOR;                         /*12 bit ADC result has to be scaled to 10 bit value */
}

/*function
 * ReadCurrentshunt()
 * Reads CSA value and triggers OC faults for Motor current greater than Set Limit
 * */
void ReadCurrentShunt()
{
    ADC12CTL0 |= ADC12SC;                   // Start conversion
    ADC12IE = ADC12IE2;                     // Enable Interrupt after last conversion
}

/*function
 * ReadSPDFDBK()
 * Reads Electrical Speed of the motor
 * */
void ReadSPDFDBK()
{
	/* RPM count & Speed measurement of 16 Bit resolution */

	if(!sensoredTrapController.TimerOverflowFlag) /* A count of three indicates timer A1 ticked for 0x010000 clocks */
	{
		sensoredTrapController.SPDFdbk = TA1R;    /* Generate the speed count = The max value 0xFFFF represents the least possible speed measurement which is 625K / 0xFFFF = 9.52Hz as  timer 1 is run at 625KHz */
	}
	sensoredTrapController.TimerOverflowFlag = 0 ; /* Reset the Interrupt flag */
	TA1R = 0x0000;				                   /* Reset the timer count */
}

/*function
 * drv83xx_getGPIO(unsigned char gpioPort, unsigned char gpioNum)
 * Device specific Get GPIO function
 * */
unsigned char drv83xx_getGPIO(unsigned char gpioPort, unsigned char gpioNum)
{
    if (gpioPort == 0x01)
    {
        if ((gpioNum & P1IN) == 0)
        {
          if (gpioNum == EN_DRV)
          {
            HostController.EnabledGateDrivers = 0;
          }
            return 0;
        }
        else
        {
          if (gpioNum == EN_DRV)
          {
            HostController.EnabledGateDrivers = 1;
          }
            return 1;
        }
    }
    else if (gpioPort == 0x02)
    {
        if ((gpioNum & P2IN) == 0)
            return 0;
        else
            return 1;
    }
    else
    {
        // do nothing
        return 0;
    }
}

/*function
 * drv83xx_getGPIO(unsigned char gpioPort, unsigned char gpioNum, unsigned char gpioVal)
 * Device specific Set GPIO function
 * */
void drv83xx_setGPIO(unsigned char gpioPort, unsigned char gpioNum, unsigned char gpioVal)
{
	    if (gpioPort == 0x01)
	    {
	        if (gpioVal == 0)
	        {
	            P1OUT &= ~gpioNum;
              if (gpioNum == EN_DRV)
              {
                HostController.EnabledGateDrivers = 0;
				sensoredTrapController.GateToggleFaultDetect = TRUE;     // Set the Gate toggle fault detect for 10ms
				sensoredTrapController.GateToggleCounter = 0;
              }
	        }
	        else
	        {
	            P1OUT |= gpioNum;
              if (gpioNum == EN_DRV)
              {

                HostController.EnabledGateDrivers = 1;
				sensoredTrapController.GateToggleFaultDetect = TRUE;     // Set the Gate toggle fault detect for 10ms
				sensoredTrapController.GateToggleCounter = 0;
              }
	        }
	    }
	    else if (gpioPort == 0x02)
	    {
	        if (gpioVal == 0)
	            P2OUT &= ~gpioNum;
	        else
	            P2OUT |= gpioNum;
	    }
	    else
	    {
	        // do nothing
	    }

}

/*function
 * drv83xx_StartMotor()
 * Device specific Start Motor function
 * */
void drv83xx_StartMotor()
{
	HostController.StartStopMotor = 0;
}

/*function
 * drv83xx_StartMotor()
 * Device specific Stop Motor function
 * */
void drv83xx_StopMotor()
{
	HostController.StartStopMotor = 1;
}

/*function
 * drv83xx_regRestoreFromCache()
 * Restores the device register values by rewriting them with the cached values.
 * */
void drv83xx_regRestoreFromCache()
{
    if(sensoredTrapController.SPIVariant == TRUE)                  // Initialize the SPI variables and settings only if the device is "S" Variant
    {
		/* Write all the cached register values to the device */
		SPI_Write(SPI_REG_DRV_CTRL, Reg_Map_Cache.Driver_Control_Reg2);
		SPI_Write(SPI_REG_GATE_DRV_HS, Reg_Map_Cache.Gate_Drive_HS_Reg3);
		SPI_Write(SPI_REG_GATE_DRV_LS, Reg_Map_Cache.Gate_Drive_LS_Reg4);
		SPI_Write(SPI_REG_OCP_CTRL, Reg_Map_Cache.OCP_Control_Reg5);

		/* This register exists only in DRV83xxS */
		if (sensoredTrapController.ShuntVariant == TRUE)
		{
			SPI_Write(SPI_REG_CSA_CTRL, Reg_Map_Cache.CSA_Control_Reg6);
		}
		if(sensoredTrapController.DeviceVariant == DRV835X_DEVICE)
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
    if(sensoredTrapController.SPIVariant == TRUE)                  // Initialize the SPI variables and settings only if the device is "S" Variant
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
				if (sensoredTrapController.ShuntVariant == TRUE)
				{
					Reg_Map_Cache.CSA_Control_Reg6 = regValue;
				}
				break;
			case 7:
				/*This register exist only in DRV835xS*/
				if(sensoredTrapController.DeviceVariant == DRV835X_DEVICE)
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
    if (applicationStatus.fault == NOFAULT)
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
    if((HostController.StartStopMotor == 0) && (address == 0x02))  // Allow SPI write function to motor control modes address 0x02 only when the motor is in stop state
    {
        ;
    }
    else
    {
        /* Cache the value in the firmware */
        if (applicationStatus.fault == NOFAULT)
        {
            drv83xx_regToCache(address, value);
        }

        /* Write the value to the device */
        SPI_Write(address, value);
    }
}

/*function
 * drv83xx_getMtrParam(unsigned char num)
 * Returns device specific motor parameter value based on the passed parameter
 * number
 * */
unsigned long drv83xx_getMtrParam(unsigned char num)
{
	switch(num)
	{
		case MTR_PARAM_DIR:
			return sensoredTrapController.Direction;
		case MTR_PARAM_SPEED:
			return sensoredTrapController.TargetDutyCycle;
		case MTR_PARAM_DEVICE_ID:
			return sensoredTrapController.DeviceID;
		case MTR_MIN_DUTY_CYCLE:
			return sensoredTrapController.MINDutyCycle;
		case MTR_MAX_DUTY_CYCLE:
			return sensoredTrapController.MAXDutyCycle;
		case MTR_RAMP_RATE_DELAY:
			return sensoredTrapController.Accelerationdivider;
		case MTR_UNDER_VOLTAGE_LIMIT:
			return sensoredTrapController.underVoltageLimit;
		case MTR_OVER_VOLTAGE_LIMIT:
			return sensoredTrapController.overVoltageLimit;
		case MTR_MIN_STALLDETECT_DUTY:
			return sensoredTrapController.minStallDetectDuty;
		case MTR_STALLDETECT_REV_THRESHOLD:
			return sensoredTrapController.stallDetectRevThreshold;
		case MTR_STALLDETECT_TIMER_THRESHOLD:
			return sensoredTrapController.stallDetectTimerThreshold;
		case MTR_AUTO_FAULT_RECOVERY_TIME:
			return sensoredTrapController.autoFaultRecoveryTime;
		case MTR_MOTOR_PHASE_CURRENT_LIMIT:
			return sensoredTrapController.MotorPhaseCurrentLimit;
		case MTR_FAULT_STATUS:
			return applicationStatus.fault;
		case MTR_PARAM_ELEC_SPEED:
			return sensoredTrapController.SPDFdbk;
        case MTR_PARAM_PWM_FREQ:
            return sensoredTrapController.PWMPeriod;
        case MTR_START_STOP_MOTOR:
			return HostController.StartStopMotor;
        case MTR_HALL_CALIBRATE:
			return HostController.CalibrateHall;
        case MTR_CALIBRATE_DUTY_CYCLE:
			return sensoredTrapController.CALDutyCycle;
        case MTR_CALIBRATION_CYCLES:
        	return sensoredTrapController.HallCalibrationCycles;
        case MTR_AUTO_CALIBRATION:
			return sensoredTrapController.AutoHallCalib;
        case DEVICE_FULL_SCALE_VOLTAGE:
        	return FULL_SCALE_VOLTAGE;
		default:
	        return(0);
	}
}

/*function
 * drv83xx_setMtrParam(unsigned char num, unsigned long value)
 * Sets the device specific motor parameter value based on the passed parameter
 * number
 * */
void drv83xx_setMtrParam(unsigned char num, unsigned long value)
{
	switch(num)
	{
		case MTR_PARAM_DIR:
            if(sensoredTrapController.Direction_flag == TRUE)
            {
              if(sensoredTrapController.Direction == value)
              {
                sensoredTrapController.Direction_flag = FALSE;

                if (HostController.StartStopMotor == 0)
                {
                  applicationStatus.previousstate = applicationStatus.currentstate;
                  applicationStatus.currentstate = MOTOR_RAMP_UP;
                }
              }
            }
            else
            {
              if(sensoredTrapController.Direction != value)
              {
                sensoredTrapController.Direction_flag = TRUE;

                applicationStatus.previousstate = applicationStatus.currentstate;
                applicationStatus.currentstate = MOTOR_DIRECTION;
              }
            }
            if(sensoredTrapController.PWM_Mode != 0)
			{
				if(sensoredTrapController.Direction == TRUE )
				{
					P1OUT |= BIT3;                  // Set the direction to high using INHC on drv83xxx EVM
				}
				else
				{
					P1OUT &= ~BIT3;                  // Set the direction to low using INHC on drv83xxx EVM
				}
			}
			break;
		case MTR_PARAM_SPEED:
			sensoredTrapController.TargetDutyCycle = value;
			break;
		case MTR_MIN_DUTY_CYCLE:
			sensoredTrapController.MINDutyCycle = value;
			break;
		case MTR_MAX_DUTY_CYCLE:
			sensoredTrapController.MAXDutyCycle = value;
			break;
		case MTR_RAMP_RATE_DELAY:
			sensoredTrapController.Accelerationdivider = value;
			break;
		case MTR_UNDER_VOLTAGE_LIMIT:
			sensoredTrapController.underVoltageLimit = value;
			break;
		case MTR_OVER_VOLTAGE_LIMIT:
			sensoredTrapController.overVoltageLimit = value;
			break;
		case MTR_MIN_STALLDETECT_DUTY:
			sensoredTrapController.minStallDetectDuty = value;
			break;
		case MTR_STALLDETECT_REV_THRESHOLD:
			sensoredTrapController.stallDetectRevThreshold = value;
			break;
		case MTR_STALLDETECT_TIMER_THRESHOLD:
			sensoredTrapController.stallDetectTimerThreshold = value;
			break;
		case MTR_AUTO_FAULT_RECOVERY_TIME:
			sensoredTrapController.autoFaultRecoveryTime = value;
			break;
		case MTR_MOTOR_PHASE_CURRENT_LIMIT:
			sensoredTrapController.MotorPhaseCurrentLimit = value;
			break;
        case MTR_PARAM_PWM_FREQ:
            sensoredTrapController.PWMPeriod = value;
			sensoredTrapController.Counter_1M_Second = (25000 / sensoredTrapController.PWMPeriod);
            TimerA0_Init();             // Timer A0 Initialization to generate 4 PWM's for switches
            TimerA2_Init();             // Timer A2 Initialization to generate 2 PWM's for switches
            break;
        case MTR_HALL_CALIBRATE:
			HostController.CalibrateHall = value;
			if(HostController.CalibrateHall)
			{
				sensoredTrapController.CurrentDutyCycle = 0;
				SetPWMDutyCycle(sensoredTrapController.CurrentDutyCycle);
				sensoredTrapController.HallAlignInterruptFlag = FALSE;
				sensoredTrapController.HallAlignStartFlag = TRUE;
				sensoredTrapController.HallAligncounter = 0;
				sensoredTrapController.HallAlignState = 1;
				Hall_AlignSetState(sensoredTrapController.HallAlignState);
			}
			else
			{
    			DisableGateDrivers();
				applicationStatus.previousstate = applicationStatus.currentstate;                  /* Move to System Idle state and initialize Motor variables*/
				applicationStatus.currentstate = SYSTEM_IDLE;
				sensoredTrapController.HallAlignStartFlag = FALSE;
			}
        	break;
        case MTR_CALIBRATE_DUTY_CYCLE:
			sensoredTrapController.CALDutyCycle = value;
			break;
        case MTR_CALIBRATION_CYCLES:
        	sensoredTrapController.HallCalibrationCycles = value;
        	break;
        case MTR_AUTO_CALIBRATION:
        	sensoredTrapController.AutoHallCalib =value;
        	break;
        default:
	        break;
	}
}

/*function
 * drv83xx_setCtrlType(unsigned char value)
 * Sets the Control Type
 * */
void drv83xx_setCtrlType(unsigned char value)
{
	sensoredTrapController.PWM_Mode = value;
	PWM_Init();
	DisableGateDrivers();
    if(sensoredTrapController.PWM_Mode == 0) // If 6x PWM mode configure the Mode pin to pull down
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
	if(sensoredTrapController.SPIVariant == TRUE)                  // Only if the device is "S" Variant
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

/*function
 * drv83xx_setCtrlType(unsigned char value)
 * Gets the Control Type
 * */
unsigned char drv83xx_getCtrlType(void)
{
	return sensoredTrapController.PWM_Mode;
}
