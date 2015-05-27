/*
 * spiFpga.c
 *
 *  Created on: 2015Äê5ÔÂ18ÈÕ
 *      Author: Administrator
 */

#include "cyu3system.h"
#include "cyu3os.h"
//#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3spi.h"
//#include "cyu3gpio.h"
#include "spiFpga.h"
#include "..\Utilities\crc16.h"
#include "gpifgpio.h"
#include "cyu3utils.h"

#define FPGA_CS_CTL(isHigh)		CyU3PSpiSetSsnLine(isHigh)

#define FPGA_CMD_TITLE			0xA5
#define FPGA_CMD_WREG			0x00
#define FPGA_CMD_RREG			0x01
#define FPGA_CMD_WDAT			0x02
#define FPGA_CMD_RDAT			0x03

CyU3PReturnStatus_t
fpgaWriteRegister(uint16_t reg, uint16_t val)
{
	uint8_t buf[4*2];
	uint16_t crc;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	buf[0] = FPGA_CMD_TITLE;
	buf[1] = FPGA_CMD_WREG;
	buf[2] = (reg >> 8) & 0xFF;
	buf[3] = (reg >> 0) & 0xFF;
	buf[4] = (val >> 8) & 0xFF;
	buf[5] = (val >> 0) & 0xFF;
	crc = CalcCRC16(0xFFFF, buf, sizeof(buf) - 2);
	buf[6] = (crc >> 8) & 0xFF;
	buf[7] = (crc >> 0) & 0xFF;

    FPGA_CS_CTL (CyFalse);
    status = CyU3PSpiTransmitWords (buf, sizeof(buf));
    FPGA_CS_CTL (CyTrue);
    return status;

}

CyU3PReturnStatus_t
fpgaReadRegister(uint16_t reg, uint16_t* val)
{
	uint8_t buf[5*2];
	uint16_t crc;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	buf[0] = FPGA_CMD_TITLE;
	buf[1] = FPGA_CMD_RREG;
	buf[2] = (reg >> 8) & 0xFF;
	buf[3] = (reg >> 0) & 0xFF;
	buf[4] = (0 >> 8) & 0xFF;
	buf[5] = (0 >> 0) & 0xFF;

    FPGA_CS_CTL (CyFalse);

    status = CyU3PSpiTransmitWords (buf, 3*2);
    status = CyU3PSpiReceiveWords (&buf[3*2], 2*2);

   *val = (((uint16_t)buf[3*2])<<8) + buf[3*2 + 1];
   crc =  (((uint16_t)buf[4*2])<<8) + buf[4*2 + 1];

    FPGA_CS_CTL (CyTrue);

    if(crc != CalcCRC16(0xFFFF, buf, sizeof(buf) - 2))
    	return CY_U3P_ERROR_CRC;
    return status;
}

CyU3PReturnStatus_t
fpgaWriteDataCmd(uint32_t address, uint32_t length)
{
	uint8_t buf[6*2];
	uint16_t crc;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	buf[0] = FPGA_CMD_TITLE;
	buf[1] = FPGA_CMD_WDAT;
	buf[2] = (address >> 24) & 0xFF;
	buf[3] = (address >> 16) & 0xFF;
	buf[4] = (address >> 8) & 0xFF;
	buf[5] = (address >> 0) & 0xFF;
	buf[6] = (length >> 24) & 0xFF;
	buf[7] = (length >> 16) & 0xFF;
	buf[8] = (length >> 8) & 0xFF;
	buf[9] = (length >> 0) & 0xFF;
	crc = CalcCRC16(0xFFFF, buf, sizeof(buf) - 2);
	buf[10] = (crc >> 8) & 0xFF;
	buf[11] = (crc >> 0) & 0xFF;

    FPGA_CS_CTL (CyFalse);
    status = CyU3PSpiTransmitWords (buf, sizeof(buf));
		ClockCycle();
		ClockCycle();
    FPGA_CS_CTL (CyTrue);
    return status;
}

CyU3PReturnStatus_t
fpgaReadDataCmd(uint32_t address, uint32_t length)
{
	uint8_t buf[6*2];
	uint16_t crc;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	buf[0] = FPGA_CMD_TITLE;
	buf[1] = FPGA_CMD_RDAT;
	buf[2] = (address >> 24) & 0xFF;
	buf[3] = (address >> 16) & 0xFF;
	buf[4] = (address >> 8) & 0xFF;
	buf[5] = (address >> 0) & 0xFF;
	buf[6] = (length >> 24) & 0xFF;
	buf[7] = (length >> 16) & 0xFF;
	buf[8] = (length >> 8) & 0xFF;
	buf[9] = (length >> 0) & 0xFF;
	crc = CalcCRC16(0xFFFF, buf, sizeof(buf) - 2);
	buf[10] = (crc >> 8) & 0xFF;
	buf[11] = (crc >> 0) & 0xFF;

    FPGA_CS_CTL (CyFalse);
    status = CyU3PSpiTransmitWords (buf, sizeof(buf));
		ClockCycle();
		ClockCycle();
    FPGA_CS_CTL (CyTrue);
    return status;
}


