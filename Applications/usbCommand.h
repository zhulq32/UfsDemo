/*
 * usbCommand.h
 *
 *  Created on: 2015Äê4ÔÂ21ÈÕ
 *      Author: Administrator
 */

#ifndef USBCOMMAND_H_
#define USBCOMMAND_H_


typedef struct __st_usb_vector{
	unsigned char  ucSlot;
	unsigned char  ucOperation;
	unsigned char  ucLeftOpNum;
	unsigned char  ucStep;
	unsigned short usFlag;
	unsigned short usError;
	unsigned int   uiLastSuccess;
	unsigned int   uiTotalPass;
	unsigned int   uiTotalFail;
}USB_VECTOR;

typedef struct __st_usb_trig_ram_prj{
	unsigned char  ucSlot;
	unsigned char  ucFlag;
	unsigned short usOperation;
	unsigned short usOperateArea;
	unsigned short usOffsetInPage;
	unsigned int   uiStartPage;
	unsigned int   uiImageLength;
	unsigned int   uiImageChecksum;
	unsigned int   uiSdRamAddress;
	unsigned int   uiSdRamLength;
	unsigned int   uiSdRamChecksum;
}TRIG_RAMPRJ;

typedef struct __st_usb_trig_ff_prj{
	unsigned char  ucSlot;
	unsigned char  ucFlag;
	unsigned short usOperation;
	unsigned short usOperateArea;
	unsigned short usOffsetInPage;
	unsigned int   uiStartPage;
	unsigned int   uiImageLength;
	unsigned int   uiImageChecksum;
}TRIG_FFPRJ;

typedef struct __st_usb_dp_socket{
	//sizeof(DP_SOCKET)-2*(sizeof(unsigned int)) = 8*4
	unsigned char  ucSlot;
	unsigned char  ucError;
	unsigned char  ucMachine;
	unsigned char  ucStep;
	unsigned short usLeftImageNum;
	unsigned short usFlag;
	unsigned int   uiLastSuccess;
	unsigned int   uiTotalPass;
	unsigned int   uiTotalFail;

	unsigned int   uiTimeout;
	unsigned int   uiTaskTick;
	unsigned short usDelay;
	unsigned short Rfu;
}USB_DP_SOCKET;

typedef struct __st_usb_dp_image{
	//sizeof(DP_IMAGE)-3*(sizeof(unsigned int))
	unsigned short usImageIndex;
	unsigned short usFlags;
	unsigned short usOperation[2];
	unsigned short usTargetBank;
	unsigned short usOffsetInPage;
	unsigned int   uiStartPage;
	unsigned int   uiImageLength;
	unsigned int   uiImageOffset;
	unsigned int   uiImageCheckSum;
	unsigned int   uiSdRamAddress;
	unsigned int   uiSdRamCheckSum;

	unsigned int   uiBadBlockManagement;
	unsigned short usMaxErrorBits;
	unsigned short usErrorBitsInBytes;
	unsigned int   uiRemappedBlock;
	unsigned int   uiRemappedDirection;

	unsigned int   uiParameterLength;
}USB_DP_IMAGE;

typedef struct __st_usb_dp_sktimg{
	USB_DP_SOCKET socket;
	USB_DP_IMAGE image;
}USB_DP_SKTIMG;


typedef unsigned int (*read_callback)(void* dev,unsigned char* data,unsigned int length,unsigned int* readed);
typedef unsigned int (*write_callback)(void* dev,const unsigned char* data,unsigned int length,unsigned int* readed);
typedef unsigned int (*process_command)(const unsigned char* cucpCbw,unsigned char ucCbwLength,
								 unsigned char* ucpCsw,unsigned char* ucpCswLength,
								 read_callback,write_callback,void* dev, unsigned int uiLength);

process_command GetCommandProcess(unsigned short command);


#endif /* USBCOMMAND_H_ */
