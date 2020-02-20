/**************************************************************************
 * @file        Init.h
 * @author      MDBU Software Team
 * @brief       Initialize DRV8323, MSP430F5529 MCU and AFE initialization.
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 ******************************************************************************/

void DRV8x_Digital_Init(void);  // initialize MSP430 */
void UCS_Init(void);                            // Clock Initialization
void GPIO_Init(void);            // initialize GPIO ports
void ADC_Init(void);             // initialize ADC
void TimerA0_Init(void);         // initialize Timer A0  to generate 4 PWM's for switches
void TIMERA1_Init(void);         // initialize Timer A1  for Vcc reading
void TimerA2_Init(void);         // initialize Timer A2  to generate 2 PWM's for A phase
void TIMERB_Init(void);          // initialize Timer B0  for Stall fault detection and Fault recovery time
void SetVcoreUp(unsigned int level);   // initialize  SetVcoreUp for powering module to increase the DCO clock to 25Mhz
void DRV8x_Analog_Init(void);   // initialize DRV8x Analog front end Initialization */
void SPI_Init(void);                    /* Initialize SPI  function definition */
void Application_Init(void);    /* initialize application variables*/
void sensoredTrapController_Init(void); /* initialize sensored Trapezoidal motor controller*/
void drv83xx_Register_Read(void);
void drv83xx_Register_Init(void);
void drv83xx_Register_Write(void);
void HostControllerInit(void);  // Initialize Host Controller
void BikeControllerInit(void);  // Initialize Bike Controller
void TIMER_SPD_Init(void);      /* Initialize timer to read the electrical speed of the motor */
void PWM_Init(void);            /* Initialize PWM pins for multiple PWM modes */
void UART_Init(void);           // Initialize UART
