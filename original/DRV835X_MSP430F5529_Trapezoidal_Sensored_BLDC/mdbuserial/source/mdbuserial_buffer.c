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

#include "mdbuserial.h"
#include "global.h"
#include "mdbuserial_protocol.h"

#if 1 //TODO: SINGLE_ELEMENT_BUFFER

//buffer flags
#define MDBUSERIAL_BUFOVERFLOW 0x1
//#define MDBUSERIAL_NEW_DELIM   0x2
#define MDBUSERIAL_EMPTY       0x4
//#define MDBUSERIAL_ESCAPED     0x8
MDBUSERIAL_RXSTATE mdbuSerial_RxState;
#if 0
uint8_t mdbuSerial_rxbuffer[MDBUSERIAL_RX_BUFFER_SIZE];
uint8_t rxbuf_len;
uint8_t mdbuSerial_rxStatus;
#endif
struct{
	uint8_t data[MDBUSERIAL_TX_BUFFER_SIZE];
	uint8_t* head;
	uint8_t* tail;
	size_t size;
}mdbuSerial_txbuffer;

#ifdef MDBUSERIAL_USE_USB
	uint8_t mdbuSerial_usbTxBuffer[MDBUSERIAL_TX_BUFFER_SIZE * 2];
	uint8_t mdbuSerial_usbRxBuffer[MDBUSERIAL_RX_BUFFER_SIZE * 2];
#endif

// PB Added for Debug
uint8_t mdbuSerial_get()
{
	return UCA1RXBUF;
}

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
// End : PB added for debug
#pragma FUNC_ALWAYS_INLINE(mdbuserial_popRXPacket)
inline uint8_t mdbuserial_popRXPacket()
{
	rxbuf_len=0;
	mdbuSerial_rxStatus = MDBUSERIAL_EMPTY;
	return rxbuf_len;
}

#pragma FUNC_ALWAYS_INLINE(mdbuserial_TXBufSize)
inline uint8_t mdbuserial_TXBufSize(){return mdbuSerial_txbuffer.size;}

#pragma FUNC_ALWAYS_INLINE(mdbuserial_popTXBuf)
inline uint8_t mdbuserial_popTXBuf()
{
	uint8_t retval;
	if(mdbuSerial_txbuffer.size > 0)
	{
		retval = *(mdbuSerial_txbuffer.head);
		mdbuSerial_txbuffer.head++;
		if(mdbuSerial_txbuffer.head >= mdbuSerial_txbuffer.data + MDBUSERIAL_TX_BUFFER_SIZE)
		{
			mdbuSerial_txbuffer.head = mdbuSerial_txbuffer.data;
		}

		//handle corner case: before pop head = tail (size = 1)
		if(mdbuSerial_txbuffer.size == 1)
		{
			mdbuSerial_txbuffer.tail = mdbuSerial_txbuffer.head;
		}

		mdbuSerial_txbuffer.size--;
	}
	else // mdbuSerial_txbuffer.size == 0
	{
		retval = 0xFF;
	}

	return retval;
}

#pragma FUNC_ALWAYS_INLINE(mdbuserial_pushTXBuf)
inline uint8_t mdbuserial_pushTXBuf(uint8_t data)
{
	//buffer full
	if(mdbuSerial_txbuffer.size > MDBUSERIAL_TX_BUFFER_SIZE)
	{
		return MDBUSERIAL_BUFOVERFLOW;
	}
	else if(mdbuSerial_txbuffer.size == 0)
	{
		*(mdbuSerial_txbuffer.tail) = data;
		mdbuSerial_txbuffer.size++;
	}
	else
	{
		mdbuSerial_txbuffer.size++;
		mdbuSerial_txbuffer.tail++;
		if(mdbuSerial_txbuffer.tail >= mdbuSerial_txbuffer.data + MDBUSERIAL_TX_BUFFER_SIZE)
		{
			mdbuSerial_txbuffer.tail = mdbuSerial_txbuffer.data;
		}
		*(mdbuSerial_txbuffer.tail) = data;
	}
	return 0;
}

void mdbuSerial_init()
{
	mdbuSerial_rxStatus = 0;
	mdbuSerial_txbuffer.head = mdbuSerial_txbuffer.tail = mdbuSerial_txbuffer.data;
	rxbuf_len = 0;
	mdbuserial_txSEQ = 0x0;
	mdbuSerial_rxStatus |= MDBUSERIAL_EMPTY;
	mdbuserial_txIsRunning = 0;
	mdbuSerial_RxState = RX_PACKET_IDLE;
	mdbuSerial_rxStateStop = 0;
}

#pragma FUNC_ALWAYS_INLINE(mdbuserial_writePacket)
inline uint8_t mdbuserial_writePacket(uint8_t* header,uint8_t* data, size_t length, uint8_t CRC)
{
	size_t i;
	uint8_t status = 0;

	#ifdef MDBUSERIAL_USE_USB
		// Atomic operation. Disable USB In Endpoint (TX) interrupt
		uint16_t usbInEpIntrState = usbDisableInEndpointInterrupt(stUsbHandle[CDC0_INTFNUM].edb_Index);
	#endif

	status |= mdbuserial_pushTXBuf(MDBUSERIAL_DELIMINATOR);
	for(i=MDBUSERIAL_HEADER_SIZE;i>0&&header!=MDBUSERIAL_HEADER_ACK;i--,header++)
	{
		if(*header == MDBUSERIAL_DELIMINATOR || *header == MDBUSERIAL_ESCAPE)
		{
			status |= mdbuserial_pushTXBuf(MDBUSERIAL_ESCAPE);
			status |= mdbuserial_pushTXBuf((*header) ^ MDBUSERIAL_ESCAPE_BIT);
		}
		else
		{
			status |= mdbuserial_pushTXBuf(*header);
		}
	}
	for(i=length;i>0;i--,data++)
	{
		if(*data == MDBUSERIAL_DELIMINATOR || *data == MDBUSERIAL_ESCAPE)
		{
			status |= mdbuserial_pushTXBuf(MDBUSERIAL_ESCAPE);
			status |= mdbuserial_pushTXBuf((*data) ^ MDBUSERIAL_ESCAPE_BIT);
		}
		else
		{
			status |= mdbuserial_pushTXBuf(*data);
		}
	}
	if(header != MDBUSERIAL_HEADER_ACK)
	{
		if(CRC == MDBUSERIAL_DELIMINATOR || CRC == MDBUSERIAL_ESCAPE)
		{
			status |= mdbuserial_pushTXBuf(MDBUSERIAL_ESCAPE);
			status |= mdbuserial_pushTXBuf(CRC ^ MDBUSERIAL_ESCAPE_BIT);
		}
		else
		{
			status |= mdbuserial_pushTXBuf(CRC);
		}
	}
	status |= mdbuserial_pushTXBuf(MDBUSERIAL_DELIMINATOR);

	if(mdbuserial_txIsRunning == 0)
	{

	#ifdef MDBUSERIAL_TX_INTERRUPTS
		#ifndef MDBUSERIAL_USE_USB
			mdbuSerial_intTXEnable();
		#endif
		#ifndef MDBUSERIAL_TX_INTERRUPTS
			if(!mdbuSerial_putDone())
				return;
		#endif

		#ifdef MDBUSERIAL_USE_USB
			i = 0;
			unsigned char length = mdbuserial_TXBufSize();

			while (mdbuserial_TXBufSize() != 0)
			{
				mdbuSerial_usbTxBuffer[i++] = mdbuserial_popTXBuf();
			}
			mdbuSerial_sendusb(mdbuSerial_usbTxBuffer, length);
		#else
			mdbuSerial_put(mdbuserial_popTXBuf());
		#endif
	#endif

		mdbuserial_txIsRunning = 1;
	}


	#ifdef MDBUSERIAL_USE_USB
		usbRestoreInEndpointInterrupt(usbInEpIntrState);
	#endif

	return status;
}

void mdbuSerial_handleTX()
{
	#ifndef MDBUSERIAL_TX_INTERRUPTS
		if(!mdbuSerial_putDone())
			return;
	#endif

	if(mdbuserial_TXBufSize() == 0)
	{
		#ifdef MDBUSERIAL_TX_INTERRUPTS
			#ifndef MDBUSERIAL_USE_USB
				mdbuSerial_intTXDisable();
			#endif
		#endif
			mdbuserial_txIsRunning = 0;
	}
	else
	{
		#ifdef MDBUSERIAL_USE_USB
			unsigned char i = 0;
			unsigned char length = mdbuserial_TXBufSize();
			while (mdbuserial_TXBufSize() != 0)
			{
				mdbuSerial_usbTxBuffer[i++] = mdbuserial_popTXBuf();
			}
			mdbuSerial_sendusb(mdbuSerial_usbTxBuffer, length);
		#else
			mdbuSerial_put(mdbuserial_popTXBuf());
		#endif
	}
}

#pragma FUNC_ALWAYS_INLINE(mdbuSerial_handleRX)
#ifdef MDBUSERIAL_USE_USB
	inline void mdbuSerial_handleRX(uint8_t data)
#else
	inline void mdbuSerial_handleRX()
#endif
{
#ifndef MDBUSERIAL_USE_USB
  //static enum{RX_PACKET_IDLE, RX_PACKET_START_RECIEVED,RX_PACKET_DATA, RX_PACKET_ESCAPED, RX_PACKET_STOP}rxState = RX_PACKET_IDLE;
  uint8_t data;
  //mdbuSerial_RxState = RX_PACKET_IDLE;
  data = mdbuSerial_get();
#endif

  //stop if the buffer is full
  if(rxbuf_len >= MDBUSERIAL_RX_BUFFER_SIZE)
  {
	  mdbuSerial_rxStatus |= MDBUSERIAL_BUFOVERFLOW;
  }

  switch(mdbuSerial_RxState)
  {
	  case RX_PACKET_IDLE:
		  if(data != MDBUSERIAL_DELIMINATOR)
			  return;
		  //single packet buffer, if rxbuf_len != 0, we still have data not yet processed, set buffer overflow
		  //but don't stop the interface - need a way to clear this
		  if(rxbuf_len != 0)
		  {
			  mdbuSerial_rxStatus |= MDBUSERIAL_BUFOVERFLOW;
		  }
		  else
		  {
			  mdbuSerial_rxStatus &= ~MDBUSERIAL_BUFOVERFLOW;
		  }
		  //else move to state RX_PACKET_DATA on next loop, data implicitly discarded
		  mdbuSerial_RxState = RX_PACKET_DATA;
		  break;
	  case RX_PACKET_ESCAPED:
		  //move to state RX_PACKET_DATA this loop
		  data ^= MDBUSERIAL_ESCAPE_BIT;
		  mdbuSerial_RxState = RX_PACKET_DATA;
		  //don't store data or increment rxbuf_len if there is a packet in the buffer or buffer overflow
		  if(!(mdbuSerial_rxStatus & MDBUSERIAL_BUFOVERFLOW))
			  mdbuSerial_rxbuffer[rxbuf_len++] = data;
		  break;
	  case RX_PACKET_DATA:
		  if(data == MDBUSERIAL_ESCAPE)
		  {
			  //implicitly discard data
			  mdbuSerial_RxState = RX_PACKET_ESCAPED;
			  return;
		  }
		  else if(data == MDBUSERIAL_DELIMINATOR)
		  {
			  //implicitly discard data and go to RX_PACKET_STOP state this loop
			  mdbuSerial_RxState = RX_PACKET_STOP;
		  }
		  else
		  {
			  //don't store data or increment rxbuf_len if there is a packet in the buffer or buffer overflow
			  if(!(mdbuSerial_rxStatus & MDBUSERIAL_BUFOVERFLOW))
				  mdbuSerial_rxbuffer[rxbuf_len++] = data;
			  break;
		  }
	  case RX_PACKET_STOP:
		  mdbuSerial_rxStatus &= ~MDBUSERIAL_EMPTY;

	// PB
#if 1
			#ifdef MDBUSERIAL_INLINE_PROTOCOL
			  if(mdbuSerial_rxStatus & MDBUSERIAL_BUFOVERFLOW)
			  {
				  mdbuSerial_processRXPacket(mdbuSerial_rxbuffer,0);
			  }
			  else
			  {
				  mdbuSerial_processRXPacket(mdbuSerial_rxbuffer,rxbuf_len);
			  }
			#endif //MDBUSERIAL_INLINE_PROTOCOL
		  mdbuSerial_RxState = RX_PACKET_IDLE;
#endif
		  break;
  }

}

#endif //SINGLE_ELEMENT_BUFFER
