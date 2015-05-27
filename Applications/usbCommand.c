/*
 * usbCommand.c
 *
 *  Created on: 2015年4月21日
 *      Author: Administrator
 */


#include "usbCommand.h"
#include <string.h>
#define min(a,b)				( ((a)>(b))?(b):(a) )			//取小
extern uint32_t getUsbTransferLength();

const char cstrProduct[16] = "DP_UFS";
uint8_t    *sim_sdram;

#define PROCESS_DECLARE(name)	name(\
		const unsigned char* cucpCbw, unsigned char ucCbwLength,\
		unsigned char* ucpCsw, unsigned char* ucpCswLength,\
		read_callback read, write_callback write, void* dev, unsigned int uiLength)

unsigned int PROCESS_DECLARE(usbc_get_type_version)
{
	*ucpCswLength = 3 + strlen(cstrProduct)+1;
	ucpCsw[0] = 1;//VERSION_H;
	ucpCsw[1] = 2;//VERSION_M;
	ucpCsw[2] = 3;//VERSION_L;
	memcpy(&ucpCsw[3],cstrProduct,strlen(cstrProduct)+1);
	return 0;
}

unsigned int PROCESS_DECLARE(usbc_get_system_status)
{
	return 0;
}

unsigned int PROCESS_DECLARE(usbc_wr_sdram)
{
	unsigned int res,len = getUsbTransferLength();
	unsigned int address = *(unsigned int*)cucpCbw;

	address &= (32768 - 1);
	while(uiLength){
		res = read(dev,sim_sdram+address,min(uiLength,len),NULL);
		if(res) return res;
		uiLength -= min(uiLength,len);
		address += min(uiLength,len);
		address &= (32768 - 1);
	}
	return 0;
}

unsigned int PROCESS_DECLARE(usbc_rd_sdram)
{
	unsigned int res,len = getUsbTransferLength();
	unsigned int address = *(unsigned int*)cucpCbw;

	address &= (32768 - 1);
	while(uiLength){
		res = write(dev,sim_sdram+address,min(uiLength,len),NULL);
		if(res) return res;
		uiLength -= min(uiLength,len);
		address += min(uiLength,len);
		address &= (32768 - 1);
	}
	return 0;
}

typedef struct {
	unsigned short command;
	process_command process;
}USB_COMMAND;

static const USB_COMMAND usb_command[] = {
	//system command
	{0x0001,usbc_get_type_version},
	{0x0002,usbc_get_system_status},
	{0x0120,usbc_wr_sdram},
	{0x0121,usbc_rd_sdram},
};

process_command GetCommandProcess(unsigned short command){
	for(int i=0;i<sizeof(usb_command)/sizeof(usb_command[0]);i++)
		if(usb_command[i].command == command)
			return usb_command[i].process;
	return (process_command)0;
}
