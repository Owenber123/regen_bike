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

#ifndef MDBUSERIAL_INCLUDE_MDBUSERIAL_PROTOCOL_H_
#define MDBUSERIAL_INCLUDE_MDBUSERIAL_PROTOCOL_H_

#define MDBUSERIAL_DELIMINATOR 0x7E
#define MDBUSERIAL_ESCAPE      0x7D
#define MDBUSERIAL_ESCAPE_BIT  0x20

#define MDBUSERIAL_PACKETOFFSET_CMD 			2
#define MDBUSERIAL_PACKETOFFSET_PAYLOAD_LEN 	0
#define MDBUSERIAL_PACKETOFFSET_SEQ 			1
#define MDBUSERIAL_PACKETOFFSET_PAYLOAD			3
#define MDBUSERIAL_PACKETOFFSET_PAYLOAD_END		-1 //offset from stop
#define MDBUSERIAL_PACKETOFFSET_CRC 			-0 //offset from stop

#define MDBUSERIAL_HEADER_SIZE 3
#define MDBUSERIAL_HEADER_ACK 0

//ACK flags for protocol
#define MDBUSERIAL_STATERR_OVERFLOW 0x01
#define MDBUSERIAL_STATERR_SEQUENCE 0x02
#define MDBUSERIAL_STATERR_LENGTH   0x04
#define MDBUSERIAL_STATERR_CRC      0x08

#define MDBUSERIAL_MIN_PACKET_SIZE 4

//Polynomial = x^8 + x^7 + x^6 + x^3 + x^2 + x + 1
#define MDBUSERIAL_CRC_POLYNOMIAL	0xE780

#include "mdbuserial.h"

inline uint8_t mdbuSerial_processRXPacket( uint8_t* start, size_t len);
extern inline uint8_t mdbuserial_writePacket(uint8_t* header,uint8_t* data, size_t length, uint8_t CRC);
extern inline uint8_t mdbuserial_popRXPacket();
extern inline void mdbuSerial_handleTX();

uint8_t mdbuserial_txSEQ;
uint8_t mdbuserial_txIsRunning;

// PB Added Host Controller functions
#define MDBUSERIAL_NUM_CMDLIST 12;

typedef struct{
	 uint8_t pkt_cmd;
	 uint8_t pkt_len;
	 uint8_t *pkt_data;
} mdbuSerial_RxPacket;


#endif /* MDBUSERIAL_INCLUDE_MDBUSERIAL_PROTOCOL_H_ */
