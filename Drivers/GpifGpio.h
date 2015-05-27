/*
 * GpifGpio.h
 *
 *  Created on: 2015Äê5ÔÂ21ÈÕ
 *      Author: Administrator
 */

#ifndef GPIFGPIO_H_
#define GPIFGPIO_H_

#define GPIF_CMDACK		25

#define GPIF_CLK		16
#define GPIF_FF_VLD		21
#define GPIF_EOP		24
#define GPIF_DIR		22
#define GPIF_nOE		19
#define GPIF_nRD		20
#define GPIF_nWR		23
#define GPIF_nCS		17

#define ClockCycle()	do{\
							TrigClk(CyTrue);\
							CyU3PBusyWait (1);\
							TrigClk(CyFalse);\
							CyU3PBusyWait (1);\
						}while(0)


CyU3PReturnStatus_t GpifGpioInit( void );
CyBool_t WaitFifoValid(uint32_t cycle);
CyBool_t WaitEndOfPac(uint32_t cycle);
CyBool_t GetCmdAck( void );
CyBool_t GetRnW( void );
CyBool_t GetFifoValid( void );
CyBool_t GetEndOfPac( void );
void TrigCs( CyBool_t isHigh );
void TrigOE( CyBool_t isHigh );
void TrigRead( CyBool_t isHigh );
void TrigWrite( CyBool_t isHigh );
void TrigClk( CyBool_t isHigh );


#endif /* GPIFGPIO_H_ */
