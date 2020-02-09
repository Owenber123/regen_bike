/**************************************************************************
 * @file        SPI_API.h
 * @author      MDBU Software Team
 * @brief       SPI API header file to define functions used for SPI Module.
 * @note        Copyright (c) 2016 Texas Instruments Incorporated.
 *              All rights reserved.
 ******************************************************************************/
#ifndef drv83xx_MSP430F5529_TRAPEZOIDAL_SENSORED_BLDC_SPI_API_H_
#define drv83xx_MSP430F5529_TRAPEZOIDAL_SENSORED_BLDC_SPI_API_H_

#include "global.h"
void SPIDelay();
void drv83xxSPIReset();
void drv83xxSPISet();
void SPI_Write(unsigned short address,
               unsigned short data);
unsigned short SPI_Read(unsigned char address);

/*************************************************************
* drv83xxS Analog Subsystem SPI REGISTER ADDRESS
*************************************************************/

/* Analog Subsystem Instructions */
#define SPI_REG_FAULT_STAT      (0x00)         /*  */
#define SPI_REG_VGS_STAT        (0x01)         /*  */
#define SPI_REG_DRV_CTRL        (0x02)         /*  */
#define SPI_REG_GATE_DRV_HS     (0x03)         /*  */
#define SPI_REG_GATE_DRV_LS     (0x04)         /*  */
#define SPI_REG_OCP_CTRL        (0x05)         /*  */
#define SPI_REG_CSA_CTRL        (0x06)         /*  Register specific to DRV83x3S devices */
#define SPI_REG_DRV_CONFIG      (0x07)		   /*  Register specific to DRV835xS devices */

/* Analog Subsystem Instructions - Bit Definitions */

/* SPI_REG_00 : FAULT STATUS 1 */
#define FAULT_MASK          (0x0400)         /*  */
#define VDS_OCP_MASK        (0x0200)         /*  */
#define GDF_MASK            (0x0100)         /*  */
#define UVLO_MASK           (0x0080)         /*  */
#define OTSD_MASK           (0x0040)         /*  */
#define VDS_HA_MASK         (0x0020)         /*  */
#define VDS_LA_MASK         (0x0010)         /*  */
#define VDS_HB_MASK         (0x0008)         /*  */
#define VDS_LB_MASK         (0x0004)         /*  */
#define VDS_HC_MASK         (0x0002)         /*  */
#define VDS_LC_MASK         (0x0001)         /*  */

/* SPI_REG_01 : VGS STATUS 2 */
#define SA_OC_MASK          (0x0400)         /*  */
#define SB_OC_MASK          (0x0200)         /*  */
#define SC_OC_MASK          (0x0100)         /*  */
#define OTW_MASK            (0x0080)         /*  */
#define CPUV_MASK           (0x0040)         /*  */
#define VGS_HA_MASK         (0x0020)         /*  */
#define VGS_LA_MASK         (0x0010)         /*  */
#define VGS_HB_MASK         (0x0008)         /*  */
#define VGS_LB_MASK         (0x0004)         /*  */
#define VGS_HC_MASK         (0x0002)         /*  */
#define VGS_LC_MASK         (0x0001)         /*  */

/* SPI_REG_02 : DRIVER CONTROL */
#define OCP_ACT_MASK        (0x0400)         /*  */
#define DIS_CPUV_MASK       (0x0200)         /*  */
#define DIS_GDF_MASK        (0x0100)         /*  */
#define OTW_REP_MASK        (0x0080)         /*  */
#define PWM_MODE_MASK       (0x0060)         /*  */
#define PWM_MODE_1X			(0x0040)
#define PWM_MODE_6X			(0x0000)
#define PWM_COM_MASK        (0x0010)         /*  */
#define PWM_DIR_MASK        (0x0008)         /*  */
#define COAST_MASK          (0x0004)         /*  */
#define BRAKE_MASK          (0x0002)         /*  */
#define CLR_FLT_MASK        (0x0001)         /*  */

/* SPI_REG_03 : GATE DRIVE HS */
#define LOCK_MASK           (0x0700)         /*  */
#define IDRIVEP_HS_MASK     (0x00F0)         /*  */
#define IDRIVEN_HS_MASK     (0x000F)         /*  */

/* SPI_REG_04 : GATE DRIVE LS */
#define CBC_MASK            (0x0400)         /*  */
#define TDRIVE_MASK         (0x0300)         /*  */
#define IDRIVEP_LS_MASK     (0x00F0)         /*  */
#define IDRIVEN_LS_MASK     (0x000F)         /*  */

/* SPI_REG_05 : OCP CONTROL */
#define TRETRY_MASK         (0x0400)         /*  */
#define DEAD_TIME_MASK      (0x0300)         /*  */
#define OCP_MODE_MASK       (0x00C0)         /*  */
#define OCP_DEG_MASK        (0x0030)         /*  */
#define VDS_LVL_MASK        (0x000F)         /*  */

/* SPI_REG_06 : CSA CONTROL */
#define CSA_FET_MASK        (0x0400)         /*  */
#define VREF_DIV_MASK       (0x0200)         /*  */
#define LS_REF_MASK         (0x0100)         /*  */
#define CSA_GAIN_MASK       (0x00C0)         /*  */
#define DIS_SEN_MASK        (0x0020)         /*  */
#define CSA_CAL_A_MASK      (0x0010)         /*  */
#define CSA_CAL_B_MASK      (0x0008)         /*  */
#define CSA_CAL_C_MASK      (0x0004)         /*  */
#define SEN_LVL_MASK        (0x0003)         /*  */

/* SPI_REG_07 : DRIVER CONFIG  */
#define RSVD_MASK           (0x07FE)         /*  */
#define CAL_MODE_MASK       (0x0001)         /*  */
#endif
