#include "bike.h"





void bike_init(void)
{
    drv83xx_setGPIO(0x01, EN_DRV, 1);
    HostController.StartStopMotor = 0;
    HostController.EnabledGateDrivers = 1;
}

void update_speed(void)
{
	// Get Value from GPIO pin
	// Calculte speed from that ADC value
	// Set the PWM Duty Cycle
	sensoredTrapController.TargetDutyCycle = 500;
}

void send_handle_bar()
{
	
	// Send Params to Arduino
	// SPI or UART?
}


