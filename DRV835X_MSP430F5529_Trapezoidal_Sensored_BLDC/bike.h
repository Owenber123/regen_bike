#include "global.h"

#define USCI_A0_OFFSET 0x05C0h


// Initialize Bike for use
void bike_init(void);

// Update the Speed of the driver based on the throttle input pin
void update_speed(void);

// Send Parameters to Arduino on handlebar
void send_handle_bar(void); 
