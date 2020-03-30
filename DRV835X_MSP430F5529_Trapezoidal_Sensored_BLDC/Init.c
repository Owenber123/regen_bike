/**************************************************************************
 * @file        Init.c
 * @author      MDBU Software Team
 * @brief       Implementation of  drv83xx, MSP430F5529 MCU and AFE initialization.
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 ******************************************************************************/

#include "global.h"


// Controller
extern SENSORED_TRAP_Obj sensoredTrapController;
extern APPLICATION_STATUS applicationStatus;
extern BIKE_CONTROLLER bikeController;

// Host Controller
extern HOST_CONTROLLER_Obj HostController;
extern HOSTCONTROL_STATUS HostControl_Status;

// Registers
extern FLT_STAT_REG0_Obj Fault_Status_Reg;
extern VGS_STAT_REG1_Obj VGS_Status_Reg;
extern DRV_CTRL_REG2_Obj Driver_Control_Reg;
extern GATE_DRV_HS_REG3_Obj Gate_Drive_HS_Reg;
extern GATE_DRV_LS_REG4_Obj Gate_Drive_LS_Reg;
extern OCP_CTRL_REG5_Obj OCP_Control_Reg;
extern CSA_CTRL_REG6_Obj CSA_Control_Reg;
extern DRV_CONFIG_REG7_Obj DRV_Config_Reg;
extern REG_MAP_Obj Reg_Map_Cache;

/* function
 * Application_Init()
 * initializes the application structure
 * */
void Application_Init()
{
    applicationStatus.currentstate = SYSTEM_INIT;
    applicationStatus.previousstate = SYSTEM_INIT;
    applicationStatus.fault = NOFAULT;

    /*
     * One time initialization of the parameters for MDBU Serial
     *
     * */

    sensoredTrapController.IdriveP_Min_Value = (GATE_TO_DRAIN_CHARGE * 1000 / RISE_TIME);    // The Idrive calculation Can be seen in datasheet
    sensoredTrapController.IdriveN_Min_Value = (GATE_TO_DRAIN_CHARGE * 1000 / FALL_TIME);    // The Idrive calculation Can be seen in datasheet
    sensoredTrapController.Tdrive_Max_Value = (2* MAX(RISE_TIME,FALL_TIME));                 // The Tdrive value is calculated based on datasheet , For DRv83xxS devices Tdrive is twice of Rise time/ Fall time

    ADC_Init2();
    ReadVCC2();

    if (sensoredTrapController.deviceIDADC > 100 && sensoredTrapController.deviceIDADC < 500 )
    {
    	sensoredTrapController.DeviceVariant = DRV835X_DEVICE;      //  DRV835x device family

//        SensorlessTrapController.IdriveP_Setting = 3;
//
//        SensorlessTrapController.IdriveN_Setting = 3;
//
//        SensorlessTrapController.Tdrive_Setting = 4;
    }
    else
    {
    		sensoredTrapController.DeviceVariant = DRV83XX_DEVICE;  // unknown device family

    		for(Register_Counter = 0; Register_Counter < NUMCONFIG_IDRIVE_VALUES ; Register_Counter++)
    		{
    			if(sensoredTrapController.IdriveP_Min_Value >= DRV832xS_IdriveP_RegData[Register_Counter])
    			{
    				sensoredTrapController.IdriveP_Setting = Register_Counter;
    			}
    		}
    		for(Register_Counter = 0; Register_Counter < NUMCONFIG_IDRIVE_VALUES ; Register_Counter++)
    		{
    			if(sensoredTrapController.IdriveN_Min_Value >= DRV832xS_IdriveN_RegData[Register_Counter])
    			{
    				sensoredTrapController.IdriveN_Setting = Register_Counter;
    			}
    		}
    		for(Register_Counter = 0; Register_Counter < NUMCONFIG_TDRIVE_VALUES ; Register_Counter++)
    		{
    			if(sensoredTrapController.Tdrive_Max_Value >= DRV835xS_Tdrive_RegData[Register_Counter])
    			{
    				sensoredTrapController.Tdrive_Setting = Register_Counter;
    			}
    		}
    		sensoredTrapController.DeviceVariant = DRV832X_DEVICE;  // drv83xx device family
    }

    sensoredTrapController.ShuntVariant = 0;

     if (sensoredTrapController.deviceIDADC > 100 && sensoredTrapController.deviceIDADC < 280) { // S variant
         sensoredTrapController.SPIVariant = 1;
     } else { // H variant
         sensoredTrapController.SPIVariant = 0;
     }

    sensoredTrapController.DeviceID = sensoredTrapController.DeviceVariant;
    sensoredTrapController.DeviceID = ((sensoredTrapController.DeviceID<<1) + sensoredTrapController.ShuntVariant);
    sensoredTrapController.DeviceID = ((sensoredTrapController.DeviceID<<1) + sensoredTrapController.SPIVariant);
    mdbu_setTrgtInfo(PROD_ID,ALGO_ID,sensoredTrapController.DeviceID,
    		FW_VER_MAJ ,FW_VER_MIN,FW_VER_PATCH);
//    sensoredTrapController.DeviceID = ((sensoredTrapController.DeviceID << 1) + ((P3IN & BIT7) >> 7)); // hector
//    sensoredTrapController.DeviceID = ((sensoredTrapController.DeviceID << 1) + ((P8IN & BIT2) >> 2));

    /* initialized value of Duty cycle that should be reached for the selected potentiometer command setting.*/
    sensoredTrapController.TargetDutyCycle = 0;

    /* How many PWM cycles before increasing duty cycle. Initialized to every 100th PWM period.*/
    sensoredTrapController.Accelerationdivider = RAMP_RATE_DELAY;

    /* Low speed setting depending on application */
    sensoredTrapController.MINDutyCycle = MIN_DUTY_CYCLE;
    /* High speed setting depending on application //0x400 = 1024  --> 50% */
    sensoredTrapController.MAXDutyCycle = MAX_DUTY_CYCLE;
    /* Default shift required to read Hall state from GPIO pins */
    sensoredTrapController.ShiftForPINA = HALL_SHIFT_FOR_PIN_A;
    sensoredTrapController.ShiftForPINB = HALL_SHIFT_FOR_PIN_B;
    sensoredTrapController.ShiftForPINC = HALL_SHIFT_FOR_PIN_C;
    /* Sets the duty cycle during the calibration of Hall states*/
    sensoredTrapController.CALDutyCycle = CAL_DUTY_CYCLE;
    /* Defines number of PWM cycles Motor Hall Calibration is Executed */
    sensoredTrapController.HallCalibrationCycles = HALL_CALIB_CYCLES;
    /* Maximum allowed motor phase current*/
    sensoredTrapController.MotorPhaseCurrentLimit = MOTOR_PHASE_CURRENT_LIMIT;
    sensoredTrapController.HallAlignStartFlag = 0;
    sensoredTrapController.HallAlignDoneFlag = 0;

    /* Under/Over Voltage limits */
    sensoredTrapController.underVoltageLimit = ((UNDER_VOLTAGE_LIMIT * (float_t) 4095.0)/ FULL_SCALE_VOLTAGE);
    sensoredTrapController.overVoltageLimit = ((OVER_VOLTAGE_LIMIT * (float_t) 4095.0)/ FULL_SCALE_VOLTAGE);
    sensoredTrapController.MinPowerSupply = ((MIN_POWER_SUPPLY * (float_t)4095.0)/FULL_SCALE_VOLTAGE);
    /* Minimum Duty Cycle above which stall will be detected */
    sensoredTrapController.minStallDetectDuty = MIN_STALLDETECT_DUTY;

    /* Number of revolutions below which stall fault will be flagged */
    sensoredTrapController.stallDetectRevThreshold = STALLDETECT_REV_THRESHOLD;

    /* Time in milli seconds above which if motor doesnt spin min revolutions specified above(stallDetectRevThreshold) a stall fault is triggered */
    sensoredTrapController.stallDetectTimerThreshold = STALLDETECT_TIMER_THRESHOLD;

    /* Time in milli seconds after which system reinitialises itself if fault gets cleared */
    sensoredTrapController.autoFaultRecoveryTime = AUTO_FAULT_RECOVERY_TIME;

    /* This Variable holds the Ctrl type : 0 for 6PWM mode, 1 for 1 PWM mode*/
    sensoredTrapController.PWM_Mode = 0;

    /* PWM Switching Frequency settings*/
    sensoredTrapController.PWMPeriod = PWM_PERIOD;   		/* This Variable holds the Motor parameter to set the PWM switching frequency as 25Mhz / PWM_PERIOD */
    sensoredTrapController.Counter_1M_Second = (25000 / sensoredTrapController.PWMPeriod);

    /* Initialize MDBU Serial physical layer */
    #ifdef MDBUSERIAL_USE_USB
      USB_setup(TRUE,TRUE);       /* MDBU Serial Protocol over USB */
    #else
      UART1_Init();                /* MDBU Serial Protocol over UART */
    #endif

    /* Gate Drive Enable using Port 1.6 */
    P1DIR |= BIT6;
    P1OUT &= ~BIT6;

    DRV8x_Digital_Init();                                              				   /*Initialize MSP430F5529 peripherals */
    /* Analog initialization */
    DRV8x_Analog_Init();
}

/*function
 *  MotorVariabes_Init()
 * initializes the motor structure
 * */
void sensoredTrapController_Init()
{
    /* initialized value to reading from hall sensor (get hall effect pin state and shift to LSB) */
    sensoredTrapController.currentHallstate = 00;
    /* Set the direction flag value based on the Direction pin switch P1.1.*/
    sensoredTrapController.Direction_flag = FALSE;
    /* initialized value of Current DutyCycle 0% */
    sensoredTrapController.CurrentDutyCycle = 0x00;
    /* How fast increasing duty cycle per PWM cycle times acceleration divider initialized 1 */
    sensoredTrapController.RampRate = RAMP_RATE;
    /* Initialize a counter to compare with Acceleration Divider. Initialized to 0.*/
    sensoredTrapController.Accelerationcounter = 0x00;
    /*This keeps track of how many rotations (electrical) of the motor is completed since the last startup. */
    sensoredTrapController.RotationCount = 0x00;
    /*This is a counter used in Timer4 to check for the motor stall. */
    sensoredTrapController.StallDetectCounter = 0x00;
    /* This is the variable that is used to count the time from when a fault is cleared to when the
       motor starts again */
    sensoredTrapController.RestartDelay = 0x00;
    /* How much voltage applied for EVM */
    sensoredTrapController.VCCvoltage = 0x00;

    /* Read VCC voltage every 100ms */
    sensoredTrapController.readVccCounter = 0;

    /* This variable is used to measure the speed in timer counts of 16 bit resolution, Timer counts is inversely proportional to speed i.e. Maximum speed count represents minimum speed in frequency, 16 bit max value = 0xFFFFh */
    sensoredTrapController.SPDFdbk = 0xFFFF ;

    /* Interrupt counter to ensure the speed measurement with 16bit resolution*/
    sensoredTrapController.TimerOverflowFlag = 0;
}

void ADC_Init(void)
{
    ADC12CTL0 = ADC12ON + ADC12MSC + ADC12SHT0_0;       // Turn on ADC12, extend sampling time
                                                        // to avoid overflow of results
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1;               // Use sampling timer, Single sequence
    ADC12MCTL0 = ADC12INCH_14;                           // ref+=AVcc, channel = A6 (Read the Pot for sensing speed input) // P6.5 connected to 0-3.3V supply
    ADC12MCTL1 = ADC12INCH_5;                           // ref+=AVcc, channel = A5 (Read the Vcc Voltage from drv83xxX )
    ADC12MCTL2 = ADC12INCH_12 + ADC12EOS;               // ref+=AVcc, channel = A12 (Read the Current Shunt amplifier reading from Phase A ) , End of Sequence
    ADC12CTL0 |= ADC12ENC;                              // Enable conversions
}

void ADC_Init2(void)
{

    ADC12CTL0 = ADC12ON | ADC12SHT0_0 | ADC12SHT1_0;                    // Turn on ADC12, avoid overflow of results , select 16 clock cycles for sampling
    ADC12CTL1 = ADC12SHP | ADC12CONSEQ_1;                               // Use sampling timer, Single sequence, start conversion from memory address 0 , Select clock as SMCLK
    ADC12MCTL0 = ADC12INCH_4 + ADC12EOS;                                //   channel = A4 (Read the deviceID) ,
    ADC12IE = 0x00;

}

/* Function Clock Initialization to make use of 25MHZ setting DCO in Clock module of MSP4320F5529 */

void UCS_Init(void)
{
    WDTCTL = WDTPW + WDTHOLD;                       // Stop WDT
    SetVcoreUp (0x01);
    SetVcoreUp (0x02);
    SetVcoreUp (0x03);

    UCSCTL3 = SELREF_2;                             // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                              // Set ACLK = REFO

    __bis_SR_register(SCG0);                        // Disable the FLL control loop
    UCSCTL0 = 0x0000;                               // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_7;                            // Select DCO range 50MHz operation
    UCSCTL2 = FLLD_0 + 762;                         // Set DCO Multiplier for 25MHz
                                                    // (N + 1) * FLLRef = Fdco
                                                    // (762 + 1) * 32768 = 25MHz
                                                    // Set FLL Div = fDCOCLK/2
    __bic_SR_register(SCG0);                        // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 25 MHz / 32,768 Hz ~ 780k MCLK cycles for DCO to settle
    __delay_cycles(782000);

    // Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                          // Clear fault flags
    }
    while(SFRIFG1 & OFIFG);                         // Test oscillator fault flag
}

/* Function Vcore to Use 25mhz DCO in clock module of MSP430F5529 use Vcore up settings to make DCO support 25Mhz*/
void SetVcoreUp(unsigned int level)
{
    // Open PMM registers for write
    PMMCTL0_H = PMMPW_H;
    // Set SVS/SVM high side new level
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
    // Set SVM low side to new level
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
    // Wait till SVM is settled
    while((PMMIFG & SVSMLDLYIFG) == 0)
    {
        ;
    }
    // Clear already set flags
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
    // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;
    // Wait till new level reached
    if((PMMIFG & SVMLIFG))
    {
        while((PMMIFG & SVMLVLRIFG) == 0)
        {
            ;
        }
    }
    // Set SVS/SVM low side to new level
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
    // Lock PMM registers for write access
    PMMCTL0_H = 0x00;
}

void GPIO_Init(void)
{
/* LED initialization ports P1.0 and P4.7 */
    P1OUT &= ~BIT0;
    P1DIR |= BIT0;                               //  Set P1.0 to output for LED1
    P4OUT &= ~BIT7;
    P4DIR |= BIT7;                               // Set P4.7 to output direction for LED2

    // set up bit 0 of P4 as output
    P4DIR |= BIT0;
    // intialize bit 0 of P4 to 1
    P4OUT = 0x01;

/* PWM Initialization Using Ports P1.3, P1.5 , P2.5 for A , B , C Phases High side , P1.2, P1.4 , P2.4 for A , B , C Phases Low side respectively */

    PWM_Init();   // Configure PWM pins appropriately based on PWM mode of operation

/* Configure Port 2.0 , 2.2, 2.6 as inputs to read hall state A,B,C phases respectively */
    P2DIR &= ~(BIT0 | BIT2 | BIT6);
    P2REN |= BIT0 | BIT2 | BIT6;
    P2OUT |= BIT0 | BIT2 | BIT6;
    P2IE |= BIT0 | BIT2 | BIT6;
    P2IFG |= 0x00;
	P2IES |= (P2IN & (BIT0 | BIT2 | BIT6));     // Port Interrupts edges selected based on the Current position of Hall Sensors


/*  Enable Button Switch S2 at P1.1 for sensing Direction Change by enabling Interrupts */
    P1DIR &= ~BIT1;
    P1REN |= BIT1;                              // When a GPIO pin is configured as Input, Enable resistance to the pin by setting Px.REN and PX.OUT
    P1OUT |= BIT1;
    P1IE |= BIT1;
    P1IFG |= 0x00;
    P1IES |= BIT1;

/* Configure Port 2.7 as input for sensing faults and enable Interrupt */
    P2DIR &= ~BIT7;
    P2REN |= BIT7;                              // When a GPIO pin is configured as Input, Enable resistance to the pin by setting Px.REN and PX.OUT
    P2OUT |= BIT7;
    P2IE |= BIT7;
    P2IFG |= 0x00;
    P2IES |= BIT7;

/* UART selection for host interface*/
    P4SEL = BIT5+BIT4;                        // P4.4,5 = USCI_A1 TXD/RXD
/* Configure the Mode pin for drv83xxH devices */
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
}

/*function
 * UART1_Init()
 * Initializes UART for host interface.
 * */
void UART1_Init(void)
{
	UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
	UCA1CTL1 |= UCSSEL_2;                     // SMCLK
	UCA1BR0 = 162 ;                           // 25MHz - 9600 Baud 25M/9600= 2604.1667 , 2604.1667/16 = 162.76 , 0.76 *16 = 12.16
	UCA1BR1 = 0;
	UCA1MCTL = UCBRS_0 + UCBRF_12 + UCOS16;   // Modln UCBRSx=0, UCBRFx=12,
	UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
}

/*function
 * UART0_Init()
 * Initializes UART for host interface.
 * */
void UART0_Init(void)
{
/*
    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 162 ;                           // 25MHz - 9600 Baud 25M/9600= 2604.1667 , 2604.1667/16 = 162.76 , 0.76 *16 = 12.16
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_0 + UCBRF_12 + UCOS16;   // Modln UCBRSx=0, UCBRFx=12,
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
*/

    USCI_A_UART_initParam *params;
    params->clockPrescalar = 162;
    params->firstModReg = 12;
    params->secondModReg = 0;
    params->selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    params->overSampling = USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    //USCI_A_UART_init(USCI_A0_OFFSET, params);
}


/*function
 * PWM_Init()
 * Initializes PWM pins for different PWM modes.
 * */
void PWM_Init(void)
{
    if(sensoredTrapController.PWM_Mode == 0)  // If six PWM mode set the INLA , INLB , INHB as outputs
    {
        P1DIR |= BIT2 | BIT3 | BIT4 | BIT5;          //  P1.2,P1.3,P1.4,P1.5 to output direction for driving PWM to drv83xx
        P2DIR |= BIT4 | BIT5;                        //  P2.4,P2.5 to output direction for driving PWM to drv83xx

        P1OUT &= ~(BIT2 | BIT3 | BIT4 | BIT5);
        P1SEL |= BIT2 | BIT3 | BIT4 | BIT5;          // Set P1.2,P1.3,P1.4,P1.5 for Generating PWM for B phase High side and C phase High side respectively
        P2OUT &= ~(BIT4 | BIT5);
        P2SEL |= BIT4 | BIT5;                        // Set P2.4,P2.5 for Generating PWM for A phase High side
    }
    else                            // If 1xPWM mode set appropriate pin configurations for Gate driver Pins INxx
    {
// Configure P1.2,P1.3 as outputs to set Direction and applying brakes
    	P1OUT |= BIT2;                  // Motor brakes are removed  by default
        if(sensoredTrapController.Direction == TRUE)
        {
            P1OUT |= BIT3;                  // Set the direction to high using INHC on drv83xx EVM
        }
        else
        {
            P1OUT &= ~BIT3;                  // Set the direction to low using INHC on drv83xx EVM
        }
        P1DIR |= BIT2 | BIT3 ;          //  P1.2,P1.3 to output direction for applying brakes & setting direction to drv83xx
// Set the 1 PWM pin on INHA
        P2OUT &= ~(BIT5);
        P2DIR |= BIT5;                     //  Set P2.5 to output direction for driving 1 PWM to drv83xx
        P2SEL |= BIT5;                     // Set P2.5 for Generating 1 PWM for A phase High side
// Configure INLA,INLB,INHB which are connected to Hall inputs
        P1DIR &= ~(BIT4 | BIT5) ;          //  Set P1.4 , P1.5 as Input pins which are directly connected to Hall B & C
        P2DIR &= ~BIT4;                    //  Set P2.4 as Input pin which is connected to Hall A
        P2REN &= ~BIT4 ;
        P1REN &= ~(BIT4 | BIT5);			   // Pull up the Input pins which are connected to Hall pins as High Impedence states
		P2IE |= BIT0 | BIT2 | BIT6;
    }

}
/* USCI Initialization
   The USCI is initialized with the following setting
   3-pin, 8-bit SPI master,Clock polarity high,MSB
 */
void SPI_Init(void)
{
	/* SPI Ports Initialization * Port 3.0, 3.1, 3.2 is used for SIMO SOMI and SCLK respectively , Port 2.0 is Used for nSCS enable*/
    P3OUT &= ~(BIT0 | BIT2);
    P3DIR |= BIT0 | BIT2;                       /*  Set SIMO, CLK as outputs */
    P3DIR &= ~BIT1;                             /* SOMI, slave out master in defined as input P3.1 */
    P3SEL |= BIT0 | BIT1;                       // P3.0,3.1 option select for UCBOSIMO and UCBOSOMI
    P3SEL |= BIT2;                              // P3.2 option select for UCBOCLK function
    P2OUT |= BIT3;
    P2DIR |= BIT3;                              // Set P2.3 to output direction for nSCS

    UCB0CTL1 |= UCSWRST;                          // **Put state machine in reset**
    UCB0CTL0 |= UCMST + UCSYNC + UCMSB;           // 3-pin, 8-bit SPI master
    // Clock polarity high, MSB
    UCB0CTL1 |= UCSSEL_3;                         // MCLK
    UCB0BR0 = 10;                                 // Master Clock divided by 10 used by USCI clk
    UCB0BR1 = 0;                                  //
    UCB0CTL1 &= ~UCSWRST;                         // **Initialize USCI state machine**

    drv83xxSPISet();                              // make nSCS pin of drv83xx low to start communication with master SPI;
    drv83xxSPIReset();                            // make nSCS pin of drv83xx High to stop communication with master SPI;
    SPIDelay();
}

/*function
 * TIMERB_Init()
 * Initializes the TimerB interrupt to Support Fault recovery time and Stall fault detection
 * */
void TIMERB_Init(void)
{
    TBCTL = TBSSEL_2 | TBCLR | MC_1;            /* set continuous up count mode */
    TBCCR0 = TIME_COUNT_1MS;                    /*  1 ms period for compare 0 */

    TBCCTL0 |= CCIE;                            /* enable TimerB Period match interrupt */
    TB0R = 0x0000;                              /* reset counter */
}

/*function
 * TimerA0_Init()
 * initializes the TimerA0 to generate PWM pulses from TA0.1, TA0.2, TA0.3, TA0.4  also generate Period Interrupt for State machine
 * */
void TimerA0_Init(void)
{
    TA0CTL = TASSEL_2 | TACLR | MC_1;           	/* set SMCLK to run timer, Clear the Timer , continuous up count mode   */
    TA0CCR0 = sensoredTrapController.PWMPeriod;     /* set Period value as PWM_period*/

    TA0CCTL0 |= CCIE;                           /* Enable TimerA0 period match interrupt for State machine */
    TA0R = 0x0000;                              /* reset counter */

    /*set out put mode to GPIO low */
    TA0CCTL1 = OUTMOD_0;
    TA0CCTL2 = OUTMOD_0;
    TA0CCTL3 = OUTMOD_0;
    TA0CCTL4 = OUTMOD_0;

    /* reset compare */
    TA0CCR1 = 0x00;
    TA0CCR2 = 0x00;
    TA0CCR3 = 0x00;
    TA0CCR4 = 0x00;

    TA0CCTL1 = OUTMOD_2;
    TA0CCTL2 = OUTMOD_6;
    TA0CCTL3 = OUTMOD_2;
    TA0CCTL4 = OUTMOD_6;
}

/*function
 * TIMER_SPD_Init()
 * initializes TimerA1 for electrical speed measurement
 * */
void TIMER_SPD_Init()
{
	TA1CTL = TASSEL_2 | TACLR | MC_1 | ID_3 ;	/* set continues UP count mode , Clk Div / 2 */
	TA1CCR0 = 0x0FFFF;                      	/* This sets  Timer to count for 65535 counts at 25Mhz */
	TA1EX0 = TAIDEX_4;					 		          /* Divide clock by 5 , to make overall clock run at 25Mhz/(2*5) = 25MHz */
	TA1CCTL0 |= CCIE;					 		            /* enable TimerA1 period match interrupt */
	TA1R = 0x0000;						 		            /* reset counter */
}

/*function
 * TimerA2_Init()
 * initializes the TimerA2 to generate PWM pulses from TA2.2
 * */
void TimerA2_Init(void)
{
    TA2CTL = TASSEL_2 | TACLR | MC_1;           /* set SMCLK to run timer, Clear the Timer , continuous up count mode   */
    TA2CCR0 = sensoredTrapController.PWMPeriod; /* set Period value as PWM_period*/

    TA2R = 0x0000;                              /* reset Timer 2 counter */
    TA0R = TA2R;                          		/* reset Timer 0 counter to TA2R to sync the timers as PWM's were generated by both the timers*/

    /*set out put mode to GPIO low */
    TA2CCTL1 = OUTMOD_0;
    TA2CCTL2 = OUTMOD_0;

    /* reset compare */
    TA2CCR1 = 0x00;
    TA2CCR2 = 0x00;
    TA2CCTL1 = OUTMOD_2;                                                                                /* Select Output mode as Toggle/ReSet for all the switches*/
    TA2CCTL2 = OUTMOD_6;
}

/*function
 * DRV8x_Digital_Init(void)
 * initializes the MSP430F5529 IP's
 * */
void DRV8x_Digital_Init(void)
{
    UCS_Init();                 // Clock Initialization
    GPIO_Init();                // GPIO ports Initialization
    ADC_Init();                 // ADC Initialization
    TIMERB_Init();              // Timer B0 Initialization for Stall fault detection and Fault recovery time
    TimerA0_Init();             // Timer A0 Initialization to generate 4 PWM's for switches
    TimerA2_Init();             // Timer A2 Initialization to generate 2 PWM's for A phase

    __bis_SR_register(GIE);     /*enable global interrupt */
}

void DRV8x_Analog_Init(void)
{
    if(sensoredTrapController.SPIVariant == TRUE)                  // Initialize the SPI variables and settings only if the device is "S" Variant
    {
    	SPI_Init();             // Initialize EVM8305 SPI in slave mode
    	drv83xx_Register_Read();
    	drv83xx_Register_Init();
    	drv83xx_Register_Write();
    }
}

void drv83xx_Register_Read(void)
{
    uint16_t regValue;

    // Read Register 0x00
    regValue = SPI_Read(SPI_REG_FAULT_STAT);

    Fault_Status_Reg.REG0_FAULT = (regValue & FAULT_MASK) >> 10;
    Fault_Status_Reg.REG0_VDS_OCP = (regValue & VDS_OCP_MASK) >> 9;
    Fault_Status_Reg.REG0_GDF = (regValue & GDF_MASK) >> 8;
    Fault_Status_Reg.REG0_UVLO = (regValue & UVLO_MASK) >> 7;
    Fault_Status_Reg.REG0_OTSD = (regValue & OTSD_MASK) >> 6;
    Fault_Status_Reg.REG0_VDS_HA = (regValue & VDS_HA_MASK) >> 5;
    Fault_Status_Reg.REG0_VDS_LA = (regValue & VDS_LA_MASK) >> 4;
    Fault_Status_Reg.REG0_VDS_HB = (regValue & VDS_HB_MASK) >> 3;
    Fault_Status_Reg.REG0_VDS_LB = (regValue & VDS_LB_MASK) >> 2;
    Fault_Status_Reg.REG0_VDS_HC = (regValue & VDS_HC_MASK) >> 1;
    Fault_Status_Reg.REG0_VDS_LC = (regValue & VDS_LC_MASK) >> 0;

    // Read Register 0x01
    regValue = SPI_Read(SPI_REG_VGS_STAT);

    VGS_Status_Reg.REG1_SA_OC = (regValue & SA_OC_MASK) >> 10;
    VGS_Status_Reg.REG1_SB_OC = (regValue & SB_OC_MASK) >> 9;
    VGS_Status_Reg.REG1_SC_OC = (regValue & SC_OC_MASK) >> 8;
    VGS_Status_Reg.REG1_OTW = (regValue & OTW_MASK) >> 7;
    VGS_Status_Reg.REG1_CPUV = (regValue & CPUV_MASK) >> 6;
    VGS_Status_Reg.REG1_VGS_HA = (regValue & VGS_HA_MASK) >> 5;
    VGS_Status_Reg.REG1_VGS_LA = (regValue & VGS_LA_MASK) >> 4;
    VGS_Status_Reg.REG1_VGS_HB = (regValue & VGS_HB_MASK) >> 3;
    VGS_Status_Reg.REG1_VGS_LB = (regValue & VGS_LB_MASK) >> 2;
    VGS_Status_Reg.REG1_VGS_HC = (regValue & VGS_HC_MASK) >> 1;
    VGS_Status_Reg.REG1_VGS_LC = (regValue & VGS_LC_MASK) >> 0;

    // Read Register 0x02
    regValue = SPI_Read(SPI_REG_DRV_CTRL);

    Driver_Control_Reg.REG2_OCP_ACT = (regValue & OCP_ACT_MASK) >> 10;
    Driver_Control_Reg.REG2_DIS_CPUV = (regValue & DIS_CPUV_MASK) >> 9;
    Driver_Control_Reg.REG2_DIS_GDF = (regValue & DIS_GDF_MASK) >> 8;
    Driver_Control_Reg.REG2_OTW_REP = (regValue & OTW_REP_MASK) >> 7;
    Driver_Control_Reg.REG2_PWM_MODE = (regValue & PWM_MODE_MASK) >> 5;
    Driver_Control_Reg.REG2_PWM_COM = (regValue & PWM_COM_MASK) >> 4;
    Driver_Control_Reg.REG2_PWM_DIR = (regValue & PWM_DIR_MASK) >> 3;
    Driver_Control_Reg.REG2_COAST = (regValue & COAST_MASK) >> 2;
    Driver_Control_Reg.REG2_BRAKE = (regValue & BRAKE_MASK) >> 1;
    Driver_Control_Reg.REG2_CLR_FLT = (regValue & CLR_FLT_MASK) >> 0;

    // Read Register 0x03
    regValue = SPI_Read(SPI_REG_GATE_DRV_HS);

    Gate_Drive_HS_Reg.REG3_LOCK = (regValue & LOCK_MASK) >> 8;
    Gate_Drive_HS_Reg.REG3_IDRIVEP_HS = (regValue & IDRIVEP_HS_MASK) >> 4;
    Gate_Drive_HS_Reg.REG3_IDRIVEN_HS = (regValue & IDRIVEN_HS_MASK) >> 0;

    // Read Register 0x04
    regValue = SPI_Read(SPI_REG_GATE_DRV_LS);

    Gate_Drive_LS_Reg.REG4_CBC = (regValue & CBC_MASK) >> 10;
    Gate_Drive_LS_Reg.REG4_TDRIVE = (regValue & TDRIVE_MASK) >> 8;
    Gate_Drive_LS_Reg.REG4_IDRIVEP_LS = (regValue & IDRIVEP_LS_MASK) >> 4;
    Gate_Drive_LS_Reg.REG4_IDRIVEN_LS = (regValue & IDRIVEN_LS_MASK) >> 0;

    // Read Register 0x05
    regValue = SPI_Read(SPI_REG_OCP_CTRL);

    OCP_Control_Reg.REG5_TRETRY = (regValue & TRETRY_MASK) >> 10;
    OCP_Control_Reg.REG5_DEAD_TIME = (regValue & DEAD_TIME_MASK) >> 8;
    OCP_Control_Reg.REG5_OCP_MODE = (regValue & OCP_MODE_MASK) >> 6;
    OCP_Control_Reg.REG5_OCP_DEG = (regValue & OCP_DEG_MASK) >> 4;
    OCP_Control_Reg.REG5_VDS_LVL = (regValue & VDS_LVL_MASK) >> 0;

	if (sensoredTrapController.ShuntVariant == TRUE)       /* This register exists only in DRV83x3S */
	{
		// Read Register 0x06
		regValue = SPI_Read(SPI_REG_CSA_CTRL);

		CSA_Control_Reg.REG6_CSA_FET = (regValue & CSA_FET_MASK) >> 10;
		CSA_Control_Reg.REG6_VREF_DIV = (regValue & VREF_DIV_MASK) >> 9;
		CSA_Control_Reg.REG6_LS_REF = (regValue & LS_REF_MASK) >> 8;
		CSA_Control_Reg.REG6_CSA_GAIN = (regValue & CSA_GAIN_MASK) >> 6;
		CSA_Control_Reg.REG6_DIS_SEN = (regValue & DIS_SEN_MASK) >> 5;
		CSA_Control_Reg.REG6_CSA_CAL_A = (regValue & CSA_CAL_A_MASK) >> 4;
		CSA_Control_Reg.REG6_CSA_CAL_B = (regValue & CSA_CAL_B_MASK) >> 3;
		CSA_Control_Reg.REG6_CSA_CAL_C = (regValue & CSA_CAL_C_MASK) >> 2;
		CSA_Control_Reg.REG6_SEN_LVL = (regValue & SEN_LVL_MASK) >> 0;
	}
	if(sensoredTrapController.DeviceVariant == DRV835X_DEVICE)           /* This register exists only in DRV835xS */
    {
		// Read Register 0x07
		 regValue = SPI_Read(SPI_REG_DRV_CONFIG);
		 DRV_Config_Reg.REG7_RSVD = (regValue & RSVD_MASK) >> 1;
		 DRV_Config_Reg.REG7_CAL_MODE = (regValue & CAL_MODE_MASK) >> 0;
    }
}

void drv83xx_Register_Init(void)
{
	if(sensoredTrapController.DeviceVariant == DRV832X_DEVICE)			/* This register initialisations only for DRV832xS */
	{
		// Set Register 0x02
		Driver_Control_Reg.REG2_OCP_ACT = DRV832xS_RSVD;
		Driver_Control_Reg.REG2_DIS_CPUV = DRV832xS_DIS_CPUV;
		Driver_Control_Reg.REG2_DIS_GDF = DRV832xS_DIS_GDF;
		Driver_Control_Reg.REG2_OTW_REP = DRV832xS_OTW_REP;
		Driver_Control_Reg.REG2_PWM_MODE = DRV832xS_PWM_MODE;
		Driver_Control_Reg.REG2_PWM_COM = DRV832xS_PWM_COM;
		Driver_Control_Reg.REG2_PWM_DIR = DRV832xS_PWM_DIR;
		Driver_Control_Reg.REG2_COAST = DRV832xS_COAST_BIT;
		Driver_Control_Reg.REG2_BRAKE = DRV832xS_BRAKE_BIT;
		Driver_Control_Reg.REG2_CLR_FLT = DRV832xS_CLR_FLT;

		// Set Register 0x03
		Gate_Drive_HS_Reg.REG3_LOCK = DRV832xS_LOCK_BIT;
		Gate_Drive_HS_Reg.REG3_IDRIVEP_HS = 3;
		Gate_Drive_HS_Reg.REG3_IDRIVEN_HS = 3;

		// Set Register 0x04
		Gate_Drive_LS_Reg.REG4_CBC = DRV832xS_CBC;
		Gate_Drive_LS_Reg.REG4_TDRIVE = sensoredTrapController.Tdrive_Setting;
		Gate_Drive_LS_Reg.REG4_IDRIVEP_LS = 3;
		Gate_Drive_LS_Reg.REG4_IDRIVEN_LS = 3;

		// Set Register 0x05
		OCP_Control_Reg.REG5_TRETRY = DRV832xS_TRETRY;
		OCP_Control_Reg.REG5_DEAD_TIME = DRV832xS_DEAD_TIME;
		OCP_Control_Reg.REG5_OCP_MODE = DRV832xS_OCP_MODE;
		OCP_Control_Reg.REG5_OCP_DEG = DRV832xS_OCP_DEG;
		OCP_Control_Reg.REG5_VDS_LVL = DRV832xS_VDS_LVL;

		if (sensoredTrapController.ShuntVariant == TRUE)       /* This register exists only in DRV83x3S */
		{
			// Set Register 0x06
			CSA_Control_Reg.REG6_CSA_FET = DRV832xS_CSA_FET;
			CSA_Control_Reg.REG6_VREF_DIV = DRV832xS_VREF_DIV;
			CSA_Control_Reg.REG6_LS_REF = DRV832xS_LS_REF;
			CSA_Control_Reg.REG6_CSA_GAIN = DRV832xS_CSA_GAIN;
			CSA_Control_Reg.REG6_DIS_SEN = DRV832xS_DIS_SEN;
			CSA_Control_Reg.REG6_CSA_CAL_A = DRV832xS_CSA_CAL_A;
			CSA_Control_Reg.REG6_CSA_CAL_B = DRV832xS_CSA_CAL_B;
			CSA_Control_Reg.REG6_CSA_CAL_C = DRV832xS_CSA_CAL_C;
			CSA_Control_Reg.REG6_SEN_LVL = DRV832xS_SEN_LVL;
		}
	}
    else if(sensoredTrapController.DeviceVariant == DRV835X_DEVICE)              /* This register initialisations only for DRV835xS */
    {
		// Set Register 0x02
		Driver_Control_Reg.REG2_OCP_ACT = DRV835xS_OCP_ACT;
		Driver_Control_Reg.REG2_DIS_CPUV = DRV835xS_DIS_CPUV;
		Driver_Control_Reg.REG2_DIS_GDF = DRV835xS_DIS_GDF;
		Driver_Control_Reg.REG2_OTW_REP = DRV835xS_OTW_REP;
		Driver_Control_Reg.REG2_PWM_MODE = DRV835xS_PWM_MODE;
		Driver_Control_Reg.REG2_PWM_COM = DRV835xS_PWM_COM;
		Driver_Control_Reg.REG2_PWM_DIR = DRV835xS_PWM_DIR;
		Driver_Control_Reg.REG2_COAST = DRV835xS_COAST_BIT;
		Driver_Control_Reg.REG2_BRAKE = DRV835xS_BRAKE_BIT;
		Driver_Control_Reg.REG2_CLR_FLT = DRV835xS_CLR_FLT;

		// Set Register 0x03
		Gate_Drive_HS_Reg.REG3_LOCK = DRV835xS_LOCK_BIT;
		Gate_Drive_HS_Reg.REG3_IDRIVEP_HS = 3;
		Gate_Drive_HS_Reg.REG3_IDRIVEN_HS = 3;

		// Set Register 0x04
		Gate_Drive_LS_Reg.REG4_CBC = DRV835xS_CBC;
		Gate_Drive_LS_Reg.REG4_TDRIVE = sensoredTrapController.Tdrive_Setting;
		Gate_Drive_LS_Reg.REG4_IDRIVEP_LS = 3;
		Gate_Drive_LS_Reg.REG4_IDRIVEN_LS = 3;

		// Set Register 0x05
		OCP_Control_Reg.REG5_TRETRY = DRV835xS_TRETRY;
		OCP_Control_Reg.REG5_DEAD_TIME = DRV835xS_DEAD_TIME;
		OCP_Control_Reg.REG5_OCP_MODE = DRV835xS_OCP_MODE;
		OCP_Control_Reg.REG5_OCP_DEG = DRV835xS_OCP_DEG;
		OCP_Control_Reg.REG5_VDS_LVL = DRV835xS_VDS_LVL;

		if (sensoredTrapController.ShuntVariant == TRUE)       /* This register exists only in DRV83x3S */
		{
			// Set Register 0x06
			CSA_Control_Reg.REG6_CSA_FET = DRV835xS_CSA_FET;
			CSA_Control_Reg.REG6_VREF_DIV = DRV835xS_VREF_DIV;
			CSA_Control_Reg.REG6_LS_REF = DRV835xS_LS_REF;
			CSA_Control_Reg.REG6_CSA_GAIN = DRV835xS_CSA_GAIN;
			CSA_Control_Reg.REG6_DIS_SEN = DRV835xS_DIS_SEN;
			CSA_Control_Reg.REG6_CSA_CAL_A = DRV835xS_CSA_CAL_A;
			CSA_Control_Reg.REG6_CSA_CAL_B = DRV835xS_CSA_CAL_B;
			CSA_Control_Reg.REG6_CSA_CAL_C = DRV835xS_CSA_CAL_C;
			CSA_Control_Reg.REG6_SEN_LVL = DRV835xS_SEN_LVL;
		}
		// Set Register 0x07
		DRV_Config_Reg.REG7_RSVD = DRV835xS_RSVD;
		DRV_Config_Reg.REG7_CAL_MODE = DRV835xS_CAL_MODE;
    }
}
void drv83xx_Register_Write(void)
{
    uint16_t regValue;

    // Write Register 0x02
    regValue = (Driver_Control_Reg.REG2_OCP_ACT << 10) |
               (Driver_Control_Reg.REG2_DIS_CPUV << 9) |
               (Driver_Control_Reg.REG2_DIS_GDF << 8) | \
               (Driver_Control_Reg.REG2_OTW_REP << 7) |
               (Driver_Control_Reg.REG2_PWM_MODE << 5) |
               (Driver_Control_Reg.REG2_PWM_COM << 4) | \
               (Driver_Control_Reg.REG2_PWM_DIR << 3) |
               (Driver_Control_Reg.REG2_COAST << 2) |
               (Driver_Control_Reg.REG2_BRAKE << 1) | \
               (Driver_Control_Reg.REG2_CLR_FLT);

    SPI_Write(SPI_REG_DRV_CTRL, regValue);
    Reg_Map_Cache.Driver_Control_Reg2 = regValue;

    // Write Register 0x03
    regValue = (Gate_Drive_HS_Reg.REG3_LOCK << 8) |
               (3 << 4) |
               (3);

    SPI_Write(SPI_REG_GATE_DRV_HS, regValue);
    Reg_Map_Cache.Gate_Drive_HS_Reg3 = regValue;

    // Write Register 0x04
    regValue = (Gate_Drive_LS_Reg.REG4_CBC << 10) |
               (Gate_Drive_LS_Reg.REG4_TDRIVE << 8) |
               (3 << 4) | \
               (3);

    SPI_Write(SPI_REG_GATE_DRV_LS, regValue);
    Reg_Map_Cache.Gate_Drive_LS_Reg4 = regValue;

    // Write Register 0x05
    regValue = (OCP_Control_Reg.REG5_TRETRY << 10) |
               (OCP_Control_Reg.REG5_DEAD_TIME << 8) |
               (OCP_Control_Reg.REG5_OCP_MODE << 6) | \
               (OCP_Control_Reg.REG5_OCP_DEG << 4) |
               (OCP_Control_Reg.REG5_VDS_LVL);

    SPI_Write(SPI_REG_OCP_CTRL, regValue);
    Reg_Map_Cache.OCP_Control_Reg5 = regValue;

    if (sensoredTrapController.ShuntVariant == TRUE)
    {
    // Write Register 0x06
    regValue = (CSA_Control_Reg.REG6_CSA_FET << 10) |
               (CSA_Control_Reg.REG6_VREF_DIV << 9) |
               (CSA_Control_Reg.REG6_LS_REF << 8) | \
               (CSA_Control_Reg.REG6_CSA_GAIN << 6) |
               (CSA_Control_Reg.REG6_DIS_SEN << 4) |
               (CSA_Control_Reg.REG6_CSA_CAL_A << 3) | \
               (CSA_Control_Reg.REG6_CSA_CAL_C << 2) |
               (CSA_Control_Reg.REG6_SEN_LVL);

	/* This register exists only in DRV83x3S */

		SPI_Write(SPI_REG_CSA_CTRL, regValue);
		Reg_Map_Cache.CSA_Control_Reg6 = regValue;
	}
    if(sensoredTrapController.DeviceVariant == DRV835X_DEVICE)      /* This register exists only in DRV835xS */
    {
	// Write Register 0x07
	    regValue = (DRV_Config_Reg.REG7_RSVD << 1) |
	               (DRV_Config_Reg.REG7_CAL_MODE);
        SPI_Write(SPI_REG_DRV_CONFIG, regValue);
        Reg_Map_Cache.DRV_Config_Reg7 = regValue;
    }
}

void HostControllerInit(void)
{
	HostController.EnabledGateDrivers = 0x00;
	HostController.StartStopMotor = 0x01;

	HostControl_Status = HOST_IDLE;
}

void BikeControllerInit(void)
{
    // Initialize UART
    UART0_Init();

    bikeController.idling = 1;
    bikeController.accelerating = 0;
    bikeController.regenerating = 0;
    bikeController.speed = 0;
    bikeController.regen_level = 0;
}
