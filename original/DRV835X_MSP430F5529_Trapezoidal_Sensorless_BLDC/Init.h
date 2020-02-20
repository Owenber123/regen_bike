/* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

void Init_Application(void);
void SensorlessTrapController_Init(void);
void Init_IPD(void);
void DRV8x_Digital_Init(void);  // initialize MSP430 */
void UCS_Init(void);                    // Clock Initialization
void GPIO_Init(void);            // initialize GPIO ports
void UART_Init(void);
void ADC_Init(void);             // initialize ADC
void TIMER_IPD_Init();                   // initialize Timer for IPD to measure the time for which a pulse is to be applied
void TimerA0_Init(void);         // initialize Timer A0  to generate 4 PWM's for switches
void TIMERA1_Init(void);         // initialize Timer A1  for Vcc reading
void TimerA2_Init(void);         // initialize Timer A2  to generate 2 PWM's for A phase
void TimerB0_Init(void);          // initialize Timer B0  for Stall fault detection and Fault recovery time
void SetVcoreUp(unsigned int level);   // initialize  SetVcoreUp for powering module to increase the DCO clock to 25Mhz
void DRV8x_Analog_Init(void);   // initialize DRV8x Analog front end initialization */
void SPI_Init(void);                    /* Initialize SPI  function definition */
void Application_Init(void);    /* initialize application variables*/
void IPD_Init(void);                     // Initialize parameters for using IPD
void ADC_IPD_Init(void);         // Initialize ADC settings for IPD
void Shunt_Amplifier_Control_Init(void); //Initialize SPI settings for CSA
void IPD_Shunt_Amplifier_Control_Init(void); //Initialize SPI settings for CSA during IPD
void drv83xx_Register_Read(void);
void drv83xx_Register_Init(void);
void drv83xx_Register_Write(void);
void TIMER_SPD_Init(void);     // Initialize timer to read the electrical speed of the motor
void UART_Init(void);
void ADC_ACCEL_Init(void);     // Initialize ADC to sample currents during open loop acceleration
void TIMER_ACCEL_Init(void);
void TIMER_ACCEL_SPD_Init(void); // Initializes timer B0 to capture speed during motor acceleration
void ADC_ISC_Init(void);         // Initialize the ADC for Initial speed control settings
