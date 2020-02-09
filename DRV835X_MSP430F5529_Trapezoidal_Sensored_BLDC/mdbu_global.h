#ifndef MDBU_GLOBAL__H_
#define MDBU_GLOBAL__H_

#include "msp430.h"
#include "dataTypeDefinition.h"
#include "mdbuserial.h"
#include "mdbuserial_protocol.h"

// Serial Command Functions
#define CMD_PING   			0x00
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

void cmd_ping(uint8_t* data, size_t length);
void cmd_readRegister(uint8_t* data, size_t length);
void cmd_writeRegister(uint8_t* data, size_t length);
void cmd_getGPIO(uint8_t* data, size_t length);
void cmd_setGPIO(uint8_t* data, size_t length);
void cmd_stopMotor(uint8_t* data, size_t length);
void cmd_startMotor(uint8_t* data, size_t length);
void cmd_getMtrParam(uint8_t* data, size_t length);
void cmd_setMtrParam(uint8_t* data, size_t length);
void cmd_getCtrlType(uint8_t* data, size_t length);
void cmd_setCtrlType(uint8_t* data, size_t length);
void cmd_getCtrlRef(uint8_t* data, size_t length);
void cmd_setCtrlRef(uint8_t* data, size_t length);
void mdbu_setTrgtInfo(uint8_t plId, uint8_t algoId,
		  uint8_t partId, uint8_t fwVerMajor,
		  uint8_t fwVerMinor, uint8_t fwVerPatch);

extern const _mdbuserial_callbacktable_t mdbuserial_callbacktable[];

typedef struct __attribute__ ((__packed__)) TRGT_INFO_Obj
{
	uint8_t 	endianness; 	/* MCU Endianness */
	uint16_t 	deviceId;		/* Device ID. Combination of PL_ID (3 bits), ALGO_ID (5 bits) and PART_ID (8 bits) from MSB side */
	uint8_t 	fwVerMajor;		/* Firmware version Major */
	uint8_t 	fwVerMinor;		/* Firmware version Minor */
	uint8_t 	fwVerPatch;		/* Firmware version Patch */
} TRGT_INFO_Obj;

#endif
