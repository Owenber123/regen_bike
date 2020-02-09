/**************************************************************************
 * @file        SPI_API.c
 * @author      MDBU Software Team
 * @brief       SPI API file for SPI Module.
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 ******************************************************************************/
#ifndef drv83xx_MSP430F5529_TRAPEZOIDAL_SENSORED_BLDC_SPI_API_H_
#define drv83xx_MSP430F5529_TRAPEZOIDAL_SENSORED_BLDC_SPI_API_H_

#define SPI_DELAY 100
#define SPI_BUSY_FLAG 0x01
#include "global.h"

void SPIDelay()
{
    volatile unsigned int Delay_Count;

    for(Delay_Count = SPI_DELAY; Delay_Count > 0; Delay_Count--)
    {
        ;                                                                                // Wait for slave to initialize
    }
}

/*!
   \fn static inline void drv83xxSPIReset()
   \brief Holds the  SPI slave in reset
 */
void drv83xxSPIReset()
{
    P2OUT |= BIT3;                            // make nSCS pin of drv83xx High to stop communication with master SPI;
}

/*!
   \fn static inline void drv83xxSPIset()
   \brief Releases the AFE SPI slave from reset, so that it can begin accepting characters
 */
void drv83xxSPISet()
{
    P2OUT &= ~BIT3;                           // make nSCS pin of drv83xx low to start communication with master SPI;
}

void SPI_Write(unsigned short address,
               unsigned short data)
{
    volatile unsigned char dataMSB,dataLSB;
    address = ((address << 3) & 0x078);
    dataLSB = (data & 0x00FF);
    dataMSB = ((data >> 8) & 0x07) | address;

    drv83xxSPISet();                          // make nSCS pin of drv83xx low to start communication with master SPI;
    while(!(UCB0IFG & UCTXIFG))
    {
        ;                                     // USCI_A0 TX buffer ready?
    }
    UCB0TXBUF = dataMSB;                      // Transmit first Byte
    while(!(UCB0IFG & UCTXIFG))
    {
        ;                                     // USCI_A0 TX buffer ready?
    }
    UCB0TXBUF = dataLSB;                      // Transmit Second Byte
    while(UCB0STAT & SPI_BUSY_FLAG)
    {
        ;                                     // Wait till Transmission is complete
    }
    drv83xxSPIReset();                        // make nSCS pin of drv83xx High to stop communication with master SPI;
    SPIDelay();
}

unsigned short SPI_Read(unsigned char address)
{
    volatile unsigned short dataMSB,dataLSB,data;

    address = ((address << 3) & 0x078);
    drv83xxSPISet();                          // make nSCS pin of drv83xx low to start communication with master SPI;
    while(!(UCB0IFG & UCTXIFG))
    {
        ;                                    // USCI_A0 TX buffer ready?
    }
    UCB0TXBUF = address | BIT7;              // Transmit the Address of the register to be read , Or with BIT7 to indicate read operation
    while(UCB0STAT & SPI_BUSY_FLAG)
    {
        ;                                    // Wait till Transmission is complete
    }
    dataMSB = UCB0RXBUF & 0x07;              // Recieve the First byte
    UCB0TXBUF = address;                     // Transmit Second character
    while(UCB0STAT & SPI_BUSY_FLAG)
    {
        ;                                    // Wait till Transmission is complete
    }
    dataLSB = UCB0RXBUF & 0xFF;              // Recieve the Second byte
    drv83xxSPIReset();                       // make nSCS pin of drv83xx High to stop communication with master SPI;
    SPIDelay();
    data = ((dataMSB << 8) & 0x0700) | dataLSB;
    return(data);
}

#endif /*_SPI_API_H_*/
