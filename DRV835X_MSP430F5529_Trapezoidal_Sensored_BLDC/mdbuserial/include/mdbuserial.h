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

#ifndef MDBUSERIAL_INCLUDE_MDBUSERIAL_H_
#define MDBUSERIAL_INCLUDE_MDBUSERIAL_H_

#define MDBUSERIAL_MAJOR_VERSION 0
#define MDBUSERIAL_MINOR_VERSION 1

#include "mdbuserial_hal.h"

typedef uint8_t mdbuSerialCmd_t;
// Serial Protocol State Machine

typedef enum
{
	RX_PACKET_IDLE = 0,
	RX_PACKET_START_RECIEVED,
	RX_PACKET_DATA,
	RX_PACKET_ESCAPED,
	RX_PACKET_STOP
} MDBUSERIAL_RXSTATE;

#define MDBUSERIAL_BUFOVERFLOW 0x1
//#define MDBUSERIAL_NEW_DELIM   0x2
#define MDBUSERIAL_EMPTY       0x4
//#define MDBUSERIAL_ESCAPED     0x8

uint8_t mdbuSerial_rxbuffer[MDBUSERIAL_RX_BUFFER_SIZE];
uint8_t rxbuf_len;
uint8_t mdbuSerial_rxStatus;
volatile uint8_t mdbuSerial_rxStateStop;
volatile uint8_t mdbuserial_usb_rxReceived;
#ifdef MDBUSERIAL_USE_CALLBACKS

typedef void (*mdbuCallbackFcn_t)(uint8_t* data, size_t length);

typedef struct{
	uint8_t cmd;
	mdbuCallbackFcn_t callback;
}_mdbuserial_callbacktable_t;

extern const _mdbuserial_callbacktable_t mdbuserial_callbacktable[];
extern const uint8_t MDBUSERIAL_NUM_CALLBACKS;

#define MDBUSERIAL_START_CALL_TABLE const _mdbuserial_callbacktable_t mdbuserial_callbacktable[] = {
#define MDBUSERIAL_ADD_CALLBACK(CMD,CALLBACK) {CMD,CALLBACK},
#define MDBUSERIAL_END_CALL_TABLE }; const uint8_t MDBUSERIAL_NUM_CALLBACKS=(sizeof mdbuserial_callbacktable / sizeof (_mdbuserial_callbacktable_t));

#endif //MDBUSERIAL_USE_CALLBACKS
//Start: PB Added functions removed from mdbu ....-hal.h

extern void mdbuSerial_intTXEnable();
extern void mdbuSerial_intTXDisable();
extern void mdbuSerial_put(uint8_t data);

//End:
extern void mdbuSerial_init();
#ifdef MDBUSERIAL_USE_USB
	extern inline void mdbuSerial_handleRX(uint8_t data);
#else
	extern inline void mdbuSerial_handleRX();
#endif
extern inline void mdbuSerial_handleTX();
extern uint8_t mdbuSerial_available();
//mdbuSerialCmd_t mdbuSerial_read(uint8_t* buf, size_t length);
extern uint8_t mdbuSerial_write(mdbuSerialCmd_t cmd, uint8_t* data, size_t length);


#endif /* MDBUSERIAL_INCLUDE_MDBUSERIAL_H_ */
