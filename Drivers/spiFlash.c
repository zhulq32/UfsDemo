/*
 * spiFlash.c
 *
 *  Created on: 2015Äê5ÔÂ18ÈÕ
 *      Author: Administrator
 */

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3spi.h"
#include "cyu3gpio.h"

extern CyU3PDmaChannel glSpiTxHandle;   /* SPI Tx channel handle */
extern CyU3PDmaChannel glSpiRxHandle;   /* SPI Rx channel handle */

#define CY_FX_USB_SPI_TIMEOUT                   (5000)

#define glSpiPageSize				256
#define SPI_FLASH_CS_PIN		57

#define SPI_FLASH_CS_CTL(isHigh)	CyU3PGpioSetValue (SPI_FLASH_CS_PIN, isHigh)
//#define SPI_FLASH_CS_CTL(isHigh)	((isHigh==CyTrue)?(GPIO->lpp_gpio_simple[SPI_FLASH_CS_PIN] |= CYFX_GPIO_HIGH):(GPIO->lpp_gpio_simple[SPI_FLASH_CS_PIN] &=~CYFX_GPIO_HIGH))

CyU3PReturnStatus_t
spiFlashInit ( void )
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PGpioClock_t gpioClock;
    CyU3PGpioSimpleConfig_t gpioConfig;
    /* Initialize the GPIO module. */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc     = CY_U3P_SYS_CLK;
    gpioClock.halfDiv    = 0;

    apiRetStatus = CyU3PGpioInit(&gpioClock, NULL);
    if (apiRetStatus != 0)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "CyU3PGpioInit failed, error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    gpioConfig.outValue    = CyTrue;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;

    apiRetStatus = CyU3PGpioSetSimpleConfig(SPI_FLASH_CS_PIN, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "CyU3PGpioSetSimpleConfig for GPIO Id %d failed, error code = %d\n",
        		SPI_FLASH_CS_PIN, apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    return apiRetStatus;
}

CyU3PReturnStatus_t
spiReadID(uint32_t* id)
{
	uint8_t buf[1] = {0x9F};
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    SPI_FLASH_CS_CTL (CyFalse);
    status = CyU3PSpiTransmitWords (buf, 1);
    status = CyU3PSpiReceiveWords ((uint8_t*)id, 3);
    SPI_FLASH_CS_CTL (CyTrue);
	return status;
}

CyU3PReturnStatus_t
spiWriteEnable( void )
{
	uint8_t buf[1] = {0x06};
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    SPI_FLASH_CS_CTL (CyFalse);
    status = CyU3PSpiTransmitWords (buf, 1);
    SPI_FLASH_CS_CTL (CyTrue);
	return status;
}

CyU3PReturnStatus_t
spiReadStatus( uint8_t* oStatus )
{
	uint8_t buf[1] = {0x05};
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    SPI_FLASH_CS_CTL (CyFalse);
    status = CyU3PSpiTransmitWords (buf, 1);
    status = CyU3PSpiReceiveWords (oStatus, 1);
    SPI_FLASH_CS_CTL (CyTrue);
	return status;
}

CyU3PReturnStatus_t
spiEraseSector(uint32_t sectorAddress)
{
	uint8_t buf[4] = {0xD8,0,0,0};
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    status = spiWriteEnable();
    if(status != CY_U3P_SUCCESS)
    	return status;

    buf[1] = (sectorAddress >> 16) & 0xFF;
    buf[2] = (sectorAddress >> 8) & 0xFF;
    buf[3] = (sectorAddress >> 0) & 0xFF;
    SPI_FLASH_CS_CTL (CyFalse);
    status = CyU3PSpiTransmitWords (buf, 4);
    SPI_FLASH_CS_CTL (CyTrue);
    if(status != CY_U3P_SUCCESS)
    	return status;

    do{
    	status = spiReadStatus(buf);
        if(status != CY_U3P_SUCCESS)
        	return status;
    }while(buf[0] & 0x01);
    return status;
}

CyU3PReturnStatus_t
spiFastRead(uint32_t address, uint8_t* data, uint32_t length)
{
    CyU3PDmaBuffer_t buf_p;
	uint8_t buf[5] = {0x0B,0,0,0,0};
    uint16_t pageCount = (length / glSpiPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (length == 0)
        return CY_U3P_SUCCESS;
    if ((length % glSpiPageSize) != 0)
        pageCount ++;

    buf_p.buffer = data;
    buf_p.status = 0;
    buf_p.size  = glSpiPageSize;
    buf_p.count = glSpiPageSize;

	buf[1] = (address >> 16) & 0xFF;
	buf[2] = (address >> 8) & 0xFF;
	buf[3] = (address >> 0) & 0xFF;

	SPI_FLASH_CS_CTL (CyFalse);
	status = CyU3PSpiTransmitWords (buf, 5);
	if (status != CY_U3P_SUCCESS)
		goto EndOfFunction;

    while(pageCount){
		CyU3PSpiSetBlockXfer (0, glSpiPageSize);

		status = CyU3PDmaChannelSetupRecvBuffer (&glSpiRxHandle,
				&buf_p);
		if (status != CY_U3P_SUCCESS)
			goto EndOfFunction;

		status = CyU3PDmaChannelWaitForCompletion (&glSpiRxHandle,
				CY_FX_USB_SPI_TIMEOUT);
		if (status != CY_U3P_SUCCESS)
			goto EndOfFunction;
		CyU3PSpiDisableBlockXfer (CyFalse, CyTrue);

		address += glSpiPageSize;
        buf_p.buffer += glSpiPageSize;
        pageCount --;
    }

EndOfFunction:
    SPI_FLASH_CS_CTL (CyTrue);
    return status;
}

CyU3PReturnStatus_t
spiPageProgram(uint32_t address, uint8_t* data, uint32_t length)
{
    CyU3PDmaBuffer_t buf_p;
	uint8_t buf[4] = {0x02,0,0,0};
    uint16_t pageCount = (length / glSpiPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (length == 0)
        return CY_U3P_SUCCESS;
    if ((length % glSpiPageSize) != 0)
        pageCount ++;

    buf_p.buffer = data;
    buf_p.status = 0;
    buf_p.size  = glSpiPageSize;
    buf_p.count = glSpiPageSize;

	while(pageCount){
		buf[0] = 0x02;
		buf[1] = (address >> 16) & 0xFF;
		buf[2] = (address >> 8) & 0xFF;
		buf[3] = (address >> 0) & 0xFF;

		SPI_FLASH_CS_CTL (CyFalse);
		status = CyU3PSpiTransmitWords (buf, 4);
		if (status != CY_U3P_SUCCESS)
			goto EndOfFunction;

		CyU3PSpiSetBlockXfer (0, glSpiPageSize);

        status = CyU3PDmaChannelSetupSendBuffer (&glSpiTxHandle,
                &buf_p);
		if (status != CY_U3P_SUCCESS)
			goto EndOfFunction;

        status = CyU3PDmaChannelWaitForCompletion(&glSpiTxHandle,
                CY_FX_USB_SPI_TIMEOUT);
		if (status != CY_U3P_SUCCESS)
			goto EndOfFunction;

		CyU3PSpiDisableBlockXfer (CyFalse, CyTrue);
	    SPI_FLASH_CS_CTL (CyTrue);

	    do{
	    	status = spiReadStatus(buf);
	        if(status != CY_U3P_SUCCESS)
	        	return status;
	    }while(buf[0] & 0x01);

		address += glSpiPageSize;
        buf_p.buffer += glSpiPageSize;
        pageCount --;
    }

EndOfFunction:
    SPI_FLASH_CS_CTL (CyTrue);
    return status;
}
