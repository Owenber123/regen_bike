/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#ifndef MDBUSERIAL_MDBUSERIAL_HAL_H_
#define MDBUSERIAL_MDBUSERIAL_HAL_H_

#define MDBUSERIAL_TX_BUFFER_SIZE 16
#define MDBUSERIAL_RX_BUFFER_SIZE 16

#define MDBUSERIAL_INLINE_PROTOCOL
#define MDBUSERIAL_TX_INTERRUPTS
#define MDBUSERIAL_USE_CALLBACKS
#define MDBUSERIAL_USE_USB

//typedef unsigned char uint8_t;
//typedef unsigned short uint16_t;
typedef unsigned int size_t;

#include <msp430.h>
#include "dataTypeDefinition.h"

// USB specific includes
#include <usb.h>
#include <USB_app/usbConstructs.h>
#include <USB_config/descriptors.h>

extern uint8_t mdbuSerial_usbTxBuffer[MDBUSERIAL_TX_BUFFER_SIZE * 2];
extern uint8_t mdbuSerial_usbRxBuffer[MDBUSERIAL_RX_BUFFER_SIZE * 2];

#pragma FUNC_ALWAYS_INLINE(mdbuSerial_sendusb)
inline void mdbuSerial_sendusb(unsigned char *data, unsigned int length)
{
	cdcSendDataInBackground(data, length, CDC0_INTFNUM, 1000);
}

#ifndef MDBUSERIAL_TX_INTERRUPTS
inline uint8_t mdbuSerial_putDone()
{
	return (IFG2 & UCA0TXIFG) ? 1 : 0;
}
#else //MDBUSERIAL_TX_INTERRUPTS
// PB inline void mdbuSerial_intTXEnable()
// PB
#if 0
void mdbuSerial_intTXEnable()
{
	UCA1IE |= UCTXIE; // Enable TX interrupt
}

// PB inline void mdbuSerial_intTXDisable()
void mdbuSerial_intTXDisable()
{
	UCA1IE &= ~UCTXIE; // Disable TX interrupt
}
void mdbuSerial_put(uint8_t data)
{
	UCA1TXBUF = data;
}
#endif // PB
#endif //MDBUSERIAL_TX_INTERRUPTS

// PB inline void mdbuSerial_put(uint8_t data)

/*
// PB #pragma FUNC_ALWAYS_INLINE(mdbuSerial_get)
// PB inline uint8_t mdbuSerial_get()
uint8_t mdbuSerial_get()
{
	return UCA1RXBUF;
}
*/

#endif /* MDBUSERIAL_MDBUSERIAL_HAL_H_ */
