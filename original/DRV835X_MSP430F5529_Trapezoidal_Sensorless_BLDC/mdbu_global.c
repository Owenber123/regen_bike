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


#include "msp430.h"
#include "msp430f5529.h"
#include "mdbuserial.h"
#include "mdbu_global.h"
#include "global.h"
#include "Init.h"



/*
// Serial Command Functions
#define CMD_READ_REGISTER   0x01
#define CMD_WRITE_REGISTER  0x02
#define CMD_GET_GPIO        0x03
#define CMD_SET_GPIO        0x04
#define CMD_STOP_MOTOR      0x20
#define CMD_START_MOTOR     0x21
#define CMD_GET_MTR_PARAM   0x22
#define CMD_SET_MTR_PARAM   0x23
#define CMD_GET_CTRL_TYPE   0x24
#define CMD_SET_CTRL_TYPE   0x25
#define CMD_GET_CTRL_REF    0x26
#define CMD_SET_CTRL_REF    0x27
*/
// GPIO Numbers
//#define GPIO_EN_DRV     0x01 // Unused
//#define GPIO_NFAULT     0x02

MDBUSERIAL_START_CALL_TABLE
	MDBUSERIAL_ADD_CALLBACK(CMD_PING, cmd_ping)
    MDBUSERIAL_ADD_CALLBACK(CMD_READ_REGISTER, cmd_readRegister)
    MDBUSERIAL_ADD_CALLBACK(CMD_WRITE_REGISTER, cmd_writeRegister)
    MDBUSERIAL_ADD_CALLBACK(CMD_GET_GPIO, cmd_getGPIO)
    MDBUSERIAL_ADD_CALLBACK(CMD_SET_GPIO, cmd_setGPIO)
    MDBUSERIAL_ADD_CALLBACK(CMD_STOP_MOTOR, cmd_stopMotor)
    MDBUSERIAL_ADD_CALLBACK(CMD_START_MOTOR, cmd_startMotor)
    MDBUSERIAL_ADD_CALLBACK(CMD_GET_MTR_PARAM, cmd_getMtrParam)
    MDBUSERIAL_ADD_CALLBACK(CMD_SET_MTR_PARAM, cmd_setMtrParam)
    MDBUSERIAL_ADD_CALLBACK(CMD_GET_CTRL_TYPE, cmd_getCtrlType)
    MDBUSERIAL_ADD_CALLBACK(CMD_SET_CTRL_TYPE, cmd_setCtrlType)
    MDBUSERIAL_ADD_CALLBACK(CMD_GET_CTRL_REF, cmd_getCtrlRef)
    MDBUSERIAL_ADD_CALLBACK(CMD_SET_CTRL_REF, cmd_setCtrlRef)
MDBUSERIAL_END_CALL_TABLE




// Control Types
#define CTRL_MODE0 0x00
#define CTRL_MODE1 0x01
#define CTRL_MODEZ 0x02

// Control Ref
#define CTRL_REF_EN_IN2 0x00
#define CTRL_REF_PH_IN1 0x01

APPLICATION_STATUS ApplicationStatus;

TRGT_INFO_Obj targetInfo =
{
	.endianness = 0,	/* MSP430 is Little Endian*/
	.deviceId = 0,
	.fwVerMajor = 1,
	.fwVerMinor = 2,
	.fwVerPatch = 3
};

void mdbu_setTrgtInfo(uint8_t plId, uint8_t algoId,
		  uint8_t partId, uint8_t fwVerMajor,
		  uint8_t fwVerMinor, uint8_t fwVerPatch)
{
	uint16_t a = 1;
	if(*(uint8_t *)&a == 1)     // if the variable is stored in Little endianness, the LSB of the x is stored first.
		targetInfo.endianness = 0x00;
	else
		targetInfo.endianness = 0x01;
	targetInfo.deviceId = 0;

	// find out while running code what values deviceID has for both devices.
	// hardcode the values depending on ADC pin reading

	targetInfo.deviceId = (plId & 0x07);
	targetInfo.deviceId = ((targetInfo.deviceId <<5 ) | (algoId & 0x01F));
	targetInfo.deviceId = ((targetInfo.deviceId <<8 ) | (partId & 0x0FF));
	targetInfo.fwVerMajor = fwVerMajor;
	targetInfo.fwVerMinor = fwVerMinor;
	targetInfo.fwVerPatch = fwVerPatch;
}

/***Command Definition***
* Command: CMD_PING
*
* Incoming Packet Format:
* No incoming payload..
*
* Outgoing Packet Format:
* payload[0] = MCU endianness
* payload[1] = Target Device ID
* payload[2] = Firmware version Major
* payload[3] = Firmware version Minor
* payload[4] = Firmware version Patch
*
*************************/
void cmd_ping(uint8_t* data, size_t length)
{
    /* Populate the Target Info here. e.g. deviceId */

    /* Return back the Target Info as a payload */
    mdbuSerial_write(CMD_PING, (unsigned char *) &targetInfo, sizeof(targetInfo));
}

/***Command Definition***
* Command: CMD_READ_REGISTER
*
* Incoming Packet Format:
* data[0] = Address
*
* Outgoing Packet Format:
* rspPayload[0]...rspPayload[3] = Read value as 4 byte payload
*
*************************/
void cmd_readRegister(uint8_t* data, size_t length)
{
    uint32_t rspPayload;

    if(length < 1)
        return;

    /* Pass register address to the device specific function to read the register */
       rspPayload = (uint32_t) drv83xx_registerRead(data[0]);

    /* Return back the payload as per the device register width */
       mdbuSerial_write(CMD_READ_REGISTER, (unsigned char *) &rspPayload, drv83xx_REGISTER_WIDTH);
}

/***Command Definition***
* Command: CMD_WRITE_REGISTER
*
* Incoming Packet Format:
* data[0] = Address
* data[1]...data[4] = Write Value
*
* Outgoing Packet Format:
* None
*
*************************/
uint8_t data1, data2;
void cmd_writeRegister(uint8_t* data, size_t length)
{

    if(length < (drv83xx_REGISTER_WIDTH + 1))
        return;

    /*
     * Pass register address and value to the device specific function based on
     * the device register width
     */
    switch (drv83xx_REGISTER_WIDTH) {
        case 1:
            drv83xx_registerWrite(data[0], data[1]);
        case 2:
            drv83xx_registerWrite(data[0], *((uint16_t *)(&data[1])));
        case 4:
            drv83xx_registerWrite(data[0], *((uint32_t *)(&data[1])));
    }

}

/***Command Definition***
* Command: CMD_GET_GPIO
*
* Incoming Packet Format:
* data[0] = GPIO
* data[1] = GPIO
*       .
*       .
*       .
*
* Outgoing Packet Format:
* payload[0] = value
* payload[1] = value
*       .
*       .
*       .
*
*************************/
void cmd_getGPIO(uint8_t* data, size_t length)
{

    size_t lentmp = length;
    uint8_t* buf = data;

    for(; length>0; length--, buf++)
    {
        switch(*buf++)
        {
        case GPIO_EN_DRV:
            *buf = drv83xx_getGPIO(0x01, EN_DRV);
            break;
        default:
            break;
        }
        length--;
    }

    mdbuSerial_write(CMD_GET_GPIO, data, lentmp);

}

/***Command Definition***
* Command: CMD_SET_GPIO
*
* Incoming Packet Format:
* data[0] = GPIO
* data[1] = Value
* data[2] = GPIO
* data[3] = Value
*       .
*       .
*       .
*
* Outgoing Packet Format:
* None
*
*************************/
void cmd_setGPIO(uint8_t* data, size_t length)
{

    for(;length>1; length--, data++)
    {
        switch(*data++)
        {
        case GPIO_EN_DRV:
        	drv83xx_setGPIO(0x01, EN_DRV, *data);
            break;

        default:
            break;
        }
        length--;
    }

}

/***Command Definition***
* Command: CMD_STOP_MOTOR
*
* Incoming Packet Format:
*
* Outgoing Packet Format:
*
*************************/
void cmd_stopMotor(uint8_t* data, size_t length)
{
	drv83xx_StopMotor();

}

/***Command Definition***
* Command: CMD_START_MOTOR
*
* Incoming Packet Format:
* data[0] = 0x9A
* data[1] = 0xA3
*
* Outgoing Packet Format:
*
*************************/
void cmd_startMotor(uint8_t* data, size_t length)
{

    if(data[0] == 0x9A && data[1] == 0xA3)
    {
    	drv83xx_StartMotor();
    }

}

/***Command Definition***
* Command: CMD_GET_MTR_PARAM
*
* All data MSB first 32-bit values
* Incoming Packet Format:
* data[0] = Parameter
* data[1] = Empty
* data[2] = Empty
* data[3] = Empty
* data[4] = Empty
* data[5] = Parameter
* data[6] = Empty
* data[7] = Empty
* data[8] = Empty
* data[9] = Empty
*       .
*       .
*       .
*
* Outgoing Packet Format:
* None
*
*************************/
void cmd_getMtrParam(uint8_t* data, size_t packetLength)
{
    uint8_t* buf = data;
    uint16_t lentmp = packetLength;

    for(; lentmp>4; lentmp--)
    {
        uint32_t retval32;
        int i;
        retval32 = drv83xx_getMtrParam(*buf++);
        lentmp-=4;

        //copy retval32 into data
        for(i=0;i<sizeof(uint32_t);i++)
        {
            *buf++ = *(((uint8_t*) & retval32)+i);
        }
    }
    mdbuSerial_write(CMD_GET_MTR_PARAM, data, packetLength);
}

/***Command Definition***
* Command: CMD_SET_MTR_PARAM
*
* All data MSB first 32-bit values
* Incoming Packet Format:
* data[0] = Parameter
* data[1] = Value
* data[2] = Value
* data[3] = Value
* data[4] = Value
* data[5] = Parameter
* data[6] = Value
* data[7] = Value
* data[8] = Value
* data[9] = Value
*       .
*       .
*       .
*
* Outgoing Packet Format:
* None
*
*************************/
void cmd_setMtrParam(uint8_t* data, size_t length)
{
    for(;length>4;length--)
    {
        uint8_t paramNum = *data++;
        uint32_t paramValue = *((uint32_t*)data);
        data+=4;
        drv83xx_setMtrParam(paramNum,paramValue);
        length-=4;
    }

    //gMotorParams.updateMotorParam = true;
}

/***Command Definition***
* Command: CMD_GET_CTRL_TYPE
*
* Incoming Packet Format:
*
* Outgoing Packet Format:
* payload[0] = Control Type
*
*************************/
void cmd_getCtrlType(uint8_t* data, size_t length)
{
    uint8_t buf = 0;

    buf = drv83xx_getCtrlTypeParam();

    mdbuSerial_write(CMD_GET_CTRL_TYPE, &buf, 1);

}


/***Command Definition***
* Command: CMD_SET_CTRL_TYPE
*
* Incoming Packet Format:
* data[0] = Control Type
*
* Outgoing Packet Format:
*
*************************/
void cmd_setCtrlType(uint8_t* data, size_t length)
{

    if(length > 1)
        return;

    drv83xx_setCtrlTypeParam(data[0]);

}

/***Command Definition***
* Command: CMD_GET_CTRL_REF
*
* Incoming Packet Format:
*
* Outgoing Packet Format:
* payload[0] = Control Ref
* payload[1] = Control Ref
* payload[2] = Control Ref
* payload[3] = Control Ref
*
*************************/
void cmd_getCtrlRef(uint8_t* data, size_t length)
{
	#if 0
    uint8_t buf[4];

    buf[0] = gCtrlRef.enIn2;
    buf[1] = gCtrlRef.in1;
    buf[2] = gCtrlRef.ph;
    buf[3] = 0x00;

    mdbuSerial_write(CMD_GET_CTRL_REF, buf, 4);
    #endif
}


/***Command Definition***
* Command: CMD_SET_CTRL_TYPE
*
* Incoming Packet Format:
* data[0] = Control Ref
* data[1] = Control Ref
* data[2] = Control Ref
* data[3] = Control Ref
*
* Outgoing Packet Format:
*
*************************/
void cmd_setCtrlRef(uint8_t* data, size_t length)
{
	#if 0
    if(length != 4)
        return;

    gCtrlRef.enIn2 = data[0];
    gCtrlRef.in1   = data[1];
    gCtrlRef.ph    = data[2];
    // unused
    //test = data[3];

    gCtrlRef.updateCtrlRef = true;
    #endif
}


