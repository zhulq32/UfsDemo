/*
 * spiFlash.h
 *
 *  Created on: 2015Äê5ÔÂ18ÈÕ
 *      Author: Administrator
 */

#ifndef SPIFLASH_H_
#define SPIFLASH_H_

CyU3PReturnStatus_t
spiFlashInit ( void );

CyU3PReturnStatus_t
spiReadID(uint32_t* id);

CyU3PReturnStatus_t
spiWriteEnable( void );

CyU3PReturnStatus_t
spiReadStatus( uint8_t* status );

CyU3PReturnStatus_t
spiEraseSector(uint32_t sectorAddress);

CyU3PReturnStatus_t
spiFastRead(uint32_t address, uint8_t* data, uint32_t length);

CyU3PReturnStatus_t
spiPageProgram(uint32_t address, uint8_t* data, uint32_t length);


#endif /* SPIFLASH_H_ */
