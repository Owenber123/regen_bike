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

#include "mdbuserial_protocol.h"
extern mdbuSerial_RxPacket mdbuSerial_RxPkt;

uint8_t* mdbuserial_packetHead;

uint8_t mdbuSerial_available()
{
	return (mdbuserial_packetHead == (uint8_t*)0) ? 0 : 1;
}

#pragma FUNC_ALWAYS_INLINE(mdbuserial_getNextSEQ)
inline uint8_t mdbuserial_getNextSEQ(){return mdbuserial_txSEQ++;}

#ifndef DONT_USE_CRC8_TABLE
static const uint8_t CRC8TABLE[256] =
{
		0x0,0xcf,0x51,0x9e,0xa2,0x6d,0xf3,0x3c,0x8b,0x44,0xda,
		0x15,0x29,0xe6,0x78,0xb7,0xd9,0x16,0x88,0x47,0x7b,
		0xb4,0x2a,0xe5,0x52,0x9d,0x3,0xcc,0xf0,0x3f,0xa1,
		0x6e,0x7d,0xb2,0x2c,0xe3,0xdf,0x10,0x8e,0x41,0xf6,
		0x39,0xa7,0x68,0x54,0x9b,0x5,0xca,0xa4,0x6b,0xf5,
		0x3a,0x6,0xc9,0x57,0x98,0x2f,0xe0,0x7e,0xb1,0x8d,
		0x42,0xdc,0x13,0xfa,0x35,0xab,0x64,0x58,0x97,0x9,
		0xc6,0x71,0xbe,0x20,0xef,0xd3,0x1c,0x82,0x4d,0x23,
		0xec,0x72,0xbd,0x81,0x4e,0xd0,0x1f,0xa8,0x67,0xf9,
		0x36,0xa,0xc5,0x5b,0x94,0x87,0x48,0xd6,0x19,0x25,
		0xea,0x74,0xbb,0xc,0xc3,0x5d,0x92,0xae,0x61,0xff,
		0x30,0x5e,0x91,0xf,0xc0,0xfc,0x33,0xad,0x62,0xd5,
		0x1a,0x84,0x4b,0x77,0xb8,0x26,0xe9,0x3b,0xf4,0x6a,
		0xa5,0x99,0x56,0xc8,0x7,0xb0,0x7f,0xe1,0x2e,0x12,
		0xdd,0x43,0x8c,0xe2,0x2d,0xb3,0x7c,0x40,0x8f,0x11,
		0xde,0x69,0xa6,0x38,0xf7,0xcb,0x4,0x9a,0x55,0x46,
		0x89,0x17,0xd8,0xe4,0x2b,0xb5,0x7a,0xcd,0x2,0x9c,
		0x53,0x6f,0xa0,0x3e,0xf1,0x9f,0x50,0xce,0x1,0x3d,
		0xf2,0x6c,0xa3,0x14,0xdb,0x45,0x8a,0xb6,0x79,0xe7,
		0x28,0xc1,0xe,0x90,0x5f,0x63,0xac,0x32,0xfd,0x4a,
		0x85,0x1b,0xd4,0xe8,0x27,0xb9,0x76,0x18,0xd7,0x49,
		0x86,0xba,0x75,0xeb,0x24,0x93,0x5c,0xc2,0xd,0x31,
		0xfe,0x60,0xaf,0xbc,0x73,0xed,0x22,0x1e,0xd1,0x4f,
		0x80,0x37,0xf8,0x66,0xa9,0x95,0x5a,0xc4,0xb,0x65,
		0xaa,0x34,0xfb,0xc7,0x8,0x96,0x59,0xee,0x21,0xbf,
		0x70,0x4c,0x83,0x1d,0xd2
};
#pragma FUNC_ALWAYS_INLINE(calcCRC8)
inline uint8_t calcCRC8( uint8_t* buf,  size_t len, uint8_t crc)
{
    while (len--)
        crc = CRC8TABLE[*buf++ ^ crc];

    return crc;
}
#else
#pragma FUNC_ALWAYS_INLINE(calcCRC8)
inline uint8_t calcCRC8(uint8_t* buf, size_t len, uint8_t startValue)
{
	size_t i,j;
	uint16_t CRC = (uint16_t)startValue << 8;

    for(i=len;i>0;i--)
    {
        CRC ^= (uint16_t)buf[len-i] << 8;
        for(j=8;j>0;j--)
        {
            if(CRC & 0x8000)
            {
                CRC ^= MDBUSERIAL_CRC_POLYNOMIAL;
            }
            CRC <<= 1;
        }
    }
    return (uint8_t)(CRC >> 8);
}
#endif //DONT_USE_CRC8_TABLE


// PB #pragma FUNC_ALWAYS_INLINE(mdbuSerial_processRXPacket)
// PB inline uint8_t mdbuSerial_processRXPacket(uint8_t* start, size_t length)
uint8_t mdbuSerial_processRXPacket( uint8_t* start,  size_t length)
{
	static uint8_t prev_SEQ = 0xFF;
	uint8_t ack[2];
	ack[0] = mdbuserial_getNextSEQ();
	ack[1] = 0;

	//overflow (packet less than min packet size?)
	if(length < MDBUSERIAL_MIN_PACKET_SIZE)
	{
		++prev_SEQ;
		ack[1] |= MDBUSERIAL_STATERR_OVERFLOW;
	}
	else
	{
		uint8_t tmpval = calcCRC8(start,length-1,0);
		//need to hardcode number of CRC bytes (or #ifdef) since we need to cast the pointer:
		if(tmpval != *(start + length - 1 + MDBUSERIAL_PACKETOFFSET_CRC))
		{
			//CRC error
			ack[1] |= MDBUSERIAL_STATERR_CRC;
		}
		if(++prev_SEQ != *(start + MDBUSERIAL_PACKETOFFSET_SEQ))
		{
			//SEQ error
			ack[1] |= MDBUSERIAL_STATERR_SEQUENCE;

			//on fail start at 0
			prev_SEQ = 0xFF;
		}
	}

	//write ACK
	mdbuserial_writePacket(MDBUSERIAL_HEADER_ACK,ack,2,0);

	if((ack[1] == 0) || ((ack[1] == MDBUSERIAL_STATERR_SEQUENCE) && (*(start + MDBUSERIAL_PACKETOFFSET_CMD) == 0)))
	{
		//good packet recieved
		//TODO: store packet locations as a stack in place

		#ifdef PB //MDBUSERIAL_USE_CALLBACKS
		uint8_t i;
		for(i=0;i<MDBUSERIAL_NUM_CALLBACKS && MDBUSERIAL_NUM_CALLBACKS != 0;i++)
		{
			if(mdbuserial_callbacktable[i].cmd == *(start + MDBUSERIAL_PACKETOFFSET_CMD))
			{
				mdbuserial_callbacktable[i].callback(start+MDBUSERIAL_PACKETOFFSET_PAYLOAD,(size_t)*(start+MDBUSERIAL_PACKETOFFSET_PAYLOAD_LEN));
				goto packet_clean;
			}
		}
		//TODO: How to handle packets without a registerd command (for now... discard)
	//	#else

		//return 0;
		#endif //MDBUSERIAL_USE_CALLBACKS
#if 1
	//uint8_t i;
	//for(i=0;i< MDBUSERIAL_NUM_CMDLIST;i++)
   //if(mdbuSerial_rxStateStop == 0)
	{
		mdbuSerial_RxPkt.pkt_cmd = *(start + MDBUSERIAL_PACKETOFFSET_CMD);
		mdbuSerial_RxPkt.pkt_len = (size_t)*(start+MDBUSERIAL_PACKETOFFSET_PAYLOAD_LEN);
		mdbuSerial_RxPkt.pkt_data = start + MDBUSERIAL_PACKETOFFSET_PAYLOAD;
		mdbuSerial_rxStateStop = 1;
		goto packet_clean;
	}
	#endif
	}

	//reject/discard packet
packet_clean:
	mdbuserial_popRXPacket();
	mdbuserial_packetHead = (uint8_t*)0;

	return ack[1];
}

uint8_t mdbuSerial_write(mdbuSerialCmd_t cmd, uint8_t* data, size_t length)
{
	uint8_t header[MDBUSERIAL_HEADER_SIZE];
	uint8_t CRC;

	header[MDBUSERIAL_PACKETOFFSET_PAYLOAD_LEN] = length;
	header[MDBUSERIAL_PACKETOFFSET_SEQ] = mdbuserial_txSEQ++;
	header[MDBUSERIAL_PACKETOFFSET_CMD] = cmd;

	CRC = calcCRC8(header,MDBUSERIAL_HEADER_SIZE,0);
	CRC = calcCRC8(data,length,CRC);

	return mdbuserial_writePacket(header,data,length,CRC);
}
