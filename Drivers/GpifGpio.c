/*
 * GpifGpio.c
 *
 *  Created on: 2015Äê5ÔÂ21ÈÕ
 *      Author: Administrator
 */

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3gpio.h"
#include "cyu3error.h"
#include "gpio_regs.h"
#include "cyu3utils.h"
#include "gpifgpio.h"


CyU3PReturnStatus_t GpifGpioInit( void ){
    CyU3PGpioClock_t gpioClock;
    CyU3PGpioSimpleConfig_t gpioConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the GPIO module. */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc     = CY_U3P_SYS_CLK;
    gpioClock.halfDiv    = 0;

    apiRetStatus = CyU3PGpioInit(&gpioClock, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    	return apiRetStatus;

    gpioConfig.outValue    = CyFalse;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;

    CyU3PDeviceGpioOverride(GPIF_CLK,CyTrue);
    apiRetStatus = CyU3PGpioSetSimpleConfig(GPIF_CLK, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    	return apiRetStatus;

    gpioConfig.outValue    = CyTrue;
    CyU3PDeviceGpioOverride(GPIF_nOE,CyTrue);
    apiRetStatus = CyU3PGpioSetSimpleConfig(GPIF_nOE, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    	return apiRetStatus;

    CyU3PDeviceGpioOverride(GPIF_nRD,CyTrue);
    apiRetStatus = CyU3PGpioSetSimpleConfig(GPIF_nRD, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    	return apiRetStatus;

    apiRetStatus = CyU3PGpioSetSimpleConfig(GPIF_nWR, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    	return apiRetStatus;

    CyU3PDeviceGpioOverride(GPIF_nCS,CyTrue);
    apiRetStatus = CyU3PGpioSetSimpleConfig(GPIF_nCS, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    	return apiRetStatus;

    /* Configure GPIO 55 as input(MISO) */
    gpioConfig.outValue    = CyFalse;
    gpioConfig.inputEn     = CyTrue;
    gpioConfig.driveLowEn  = CyFalse;
    gpioConfig.driveHighEn = CyFalse;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;

    apiRetStatus = CyU3PGpioSetSimpleConfig(GPIF_CMDACK, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    	return apiRetStatus;

    apiRetStatus = CyU3PGpioSetSimpleConfig(GPIF_FF_VLD, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    	return apiRetStatus;

    apiRetStatus = CyU3PGpioSetSimpleConfig(GPIF_EOP, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    	return apiRetStatus;

    apiRetStatus = CyU3PGpioSetSimpleConfig(GPIF_DIR, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    	return apiRetStatus;

    return apiRetStatus;
}

CyBool_t GetCmdAck( void ){
	CyBool_t res;
	CyU3PGpioGetValue (GPIF_CMDACK, &res);
	return res;
}

CyBool_t GetRnW( void ){
	CyBool_t res;
	CyU3PGpioGetValue (GPIF_DIR, &res);
	return res;
}

CyBool_t GetFifoValid( void ){
	CyBool_t res;
	CyU3PGpioGetValue (GPIF_FF_VLD, &res);
	return res;
}

CyBool_t GetEndOfPac( void ){
	CyBool_t res;
	CyU3PGpioGetValue (GPIF_EOP, &res);
	return res;
}

void TrigCs( CyBool_t isHigh ){
    CyU3PGpioSetValue (GPIF_nCS, isHigh);
}

void TrigOE( CyBool_t isHigh ){
    CyU3PGpioSetValue (GPIF_nOE, isHigh);
}

void TrigRead( CyBool_t isHigh ){
    CyU3PGpioSetValue (GPIF_nRD, isHigh);
}

void TrigWrite( CyBool_t isHigh ){
    CyU3PGpioSetValue (GPIF_nWR, isHigh);
}

void TrigClk( CyBool_t isHigh ){
    CyU3PGpioSetValue (GPIF_CLK, isHigh);
}

CyBool_t WaitFifoValid(uint32_t cycle){
	while(cycle--){
		if(GetFifoValid() == CyTrue)
			return CyTrue;
		TrigClk(CyTrue);
		CyU3PBusyWait (1);
		TrigClk(CyFalse);
		CyU3PBusyWait (1);
	}
	return CyFalse;
}

CyBool_t WaitEndOfPac(uint32_t cycle){
	while(cycle--){
		if(GetEndOfPac() == CyTrue)
			return CyTrue;
		TrigClk(CyTrue);
		CyU3PBusyWait (1);
		TrigClk(CyFalse);
		CyU3PBusyWait (1);
	}
	return CyFalse;
}
