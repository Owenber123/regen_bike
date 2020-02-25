#include "global.h"



// UART Initializer
struct USCI_A_UART_initParam ebike;



void initialize_bike(void)
{
    drv83xx_setGPIO(0x01, EN_DRV, 1);
    HostController.StartStopMotor = 0;
    HostController.EnabledGateDrivers = 1;

}
