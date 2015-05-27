/*
 ## Cypress USB 3.0 Platform source file (cyfxmscdemo.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2011,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file illustrates the Mass Storage Class Driver example.
   The example makes use of the the internal device memory to provide the storage space.
   A minimum of 32KB is required for the device to be formatted as FAT on the windows host */


#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3usbconst.h>
#include <cyu3uart.h>
#include <cyu3utils.h>

#include "UsbTask.h"
#include "..\Applications\usbCommand.h"
#include "..\Applications\DPD.h"

static CyU3PEvent      appEvent;                             /* MSC application DMA Event group */

static uint32_t        glMscMaxSectors;                         /* Size of RAM Disk in 512 byte sectors. */
static uint16_t        glMscSectorSize;                         /* RAM Disk sector size: 512 bytes. */



static CyU3PUSBSpeed_t glUsbSpeed;                              /* Current USB Bus speed */
static CyBool_t        glDevConfigured;                         /* Flag to indicate Set Config event handled */
//static uint32_t        glCswDataResidue;                        /* Residue length for CSW */
static CyU3PDmaChannel glChHandleMscOut, glChHandleMscIn;       /* Channel handles */

static uint8_t           glCmdDirection;                        /* SCSI Command Direction */
static uint32_t          glDataTxLength;                        /* SCSI Data length */
uint8_t                  glInPhaseError = CyFalse;              /* Invalid command received flag. */
static volatile CyBool_t glMscChannelCreated = CyFalse;         /* Whether DMA channels have been created. */
static uint8_t           glReqSenseIndex = CY_FX_MSC_SENSE_DEVICE_RESET; /* Current sense data index. */


void threadUsbComm_Entry (uint32_t);     /* Forward declaration for the MscAppThread entry function */

uint32_t getUsbTransferLength(){
    /* Based on the Bus Speed configure the endpoint packet size */
    if (glUsbSpeed == CY_U3P_FULL_SPEED)
    {
        return 64;
    }
    else if (glUsbSpeed == CY_U3P_HIGH_SPEED)
    {
    	return 512;
    }
    else if (glUsbSpeed == CY_U3P_SUPER_SPEED)
    {
    	return 1024;
    }
    else
    {
        /* Error Handling */
        return 0;
    }
}

CyBool_t
cbkUsbSetup (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
    );

void
cbkUsbEvent (
    CyU3PUsbEventType_t evtype, /* Event type */
    uint16_t            evdata  /* Event data */
    );


/* MSC application error handler */
void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        );

process_command
GetDpdCommandProcess(
    uint8_t *buffer, /* Pointer to the CBW buffer */
    uint16_t count   /* Length of the CBW */
    );

CyFxMscCswReturnStatus_t
usbSendCsw(
    DPDCsw* csw    /* Command Status */
    );


static void dmaSetConfigured(CyU3PDmaChannelConfig_t*  dmaConfig)
{
	uint32_t len;
    CyU3PReturnStatus_t      apiRetStatus;

    len = getUsbTransferLength();
    if(len == 0){
        CyU3PDebugPrint (4, "Error! USB Not connected\r\n");
        CyFxAppErrorHandler(CY_U3P_ERROR_INVALID_CONFIGURATION);
    }
    else{
        dmaConfig->size = getUsbTransferLength();
        glMscSectorSize = getUsbTransferLength();     /* Sector size */
        glMscMaxSectors = (CY_FX_MSC_CARD_CAPACITY/getUsbTransferLength());   /* Maximum no. of sectors on the storage device */
        CyU3PDebugPrint (4, "SetConf event received %d\r\n",dmaConfig->size);
    }

    /* Create a DMA Manual IN channel between USB Producer socket
       and the CPU */
    /* DMA size is set above based on the USB Bus Speed */
    dmaConfig->count = CY_FX_MSC_DMA_BUF_COUNT;
    dmaConfig->prodSckId = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_PROD_0 | CY_FX_MSC_EP_BULK_OUT_SOCKET);
    dmaConfig->consSckId = CY_U3P_CPU_SOCKET_CONS;
    dmaConfig->dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaConfig->notification = 0;
    dmaConfig->cb = NULL;
    dmaConfig->prodHeader = 0;
    dmaConfig->prodFooter = 0;
    dmaConfig->consHeader = 0;
    dmaConfig->prodAvailCount = 0;

    /* Create the channel */
    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleMscIn,
                                          CY_U3P_DMA_TYPE_MANUAL_IN,
                                          dmaConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "DMA IN Channel Creation Failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Create a DMA Manual OUT channel between CPU and USB consumer socket */
    dmaConfig->count = CY_FX_MSC_DMA_BUF_COUNT;
    dmaConfig->prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaConfig->consSckId = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_MSC_EP_BULK_IN_SOCKET);
    dmaConfig->dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaConfig->cb = NULL;
    dmaConfig->prodHeader = 0;
    dmaConfig->prodFooter = 0;
    dmaConfig->consHeader = 0;
    dmaConfig->prodAvailCount = 0;

    /* Create the channel */
    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleMscOut,
                                          CY_U3P_DMA_TYPE_MANUAL_OUT,
                                          dmaConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "DMA OUT Channel Creation Failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Flush the Endpoint memory */
    CyU3PUsbFlushEp(CY_FX_MSC_EP_BULK_OUT);
    CyU3PUsbFlushEp(CY_FX_MSC_EP_BULK_IN);

    glMscChannelCreated = CyTrue;
    CyU3PDebugPrint (4, "SetConfig handling complete\r\n");

}


unsigned int usb_read_cbk(void* dev,unsigned char* data,unsigned int length,unsigned int* readed){
    CyU3PDmaBuffer_t dmaMscInBuffer;
    CyU3PReturnStatus_t apiRetStatus;

    /* Prepare the DMA buffer */
    dmaMscInBuffer.buffer = data;
    dmaMscInBuffer.status = 0;

    uint32_t len = getUsbTransferLength();
    if(len == 0)
    {
        CyU3PDebugPrint (4, "USB Not connected\r\n");
        return CY_U3P_ERROR_FAILURE;
    }
    else
        dmaMscInBuffer.size = len;

    /* Setup IN buffer to receive the data */
    apiRetStatus  = CyU3PDmaChannelSetupRecvBuffer (&glChHandleMscIn, &dmaMscInBuffer);
    if (apiRetStatus == CY_U3P_SUCCESS)
    {
        /* Wait for data to be received */
        apiRetStatus  = CyU3PDmaChannelWaitForRecvBuffer (&glChHandleMscIn,&dmaMscInBuffer, CYU3P_WAIT_FOREVER);
    }
    if(readed) *readed = dmaMscInBuffer.count;
    ((DPDCsw*)&dev)->dwDataResidue -= dmaMscInBuffer.count;
    /* Return Status */
    return apiRetStatus;
}
unsigned int usb_write_cbk(void* dev,const unsigned char* data,unsigned int length,unsigned int* readed){
    CyU3PDmaBuffer_t dmaMscOutBuffer;
    CyU3PReturnStatus_t apiRetStatus;

    /* Prepare the DMA Buffer */
    dmaMscOutBuffer.buffer = (uint8_t*)data;
    dmaMscOutBuffer.status = 0;

    uint32_t len = getUsbTransferLength();
    if(len == 0)
    {
        CyU3PDebugPrint (4, "USB Not connected\r\n");
        return CY_U3P_ERROR_FAILURE;
    }
    else
    	dmaMscOutBuffer.size = len;

    dmaMscOutBuffer.count = length;

    /* Setup OUT buffer to send the data */
    /* OUT buffer setup when there is no abort */
    apiRetStatus = CyU3PDmaChannelSetupSendBuffer (&glChHandleMscOut, &dmaMscOutBuffer);
    if (apiRetStatus == CY_U3P_SUCCESS)
    {
        /* Wait for data to be sent */
        apiRetStatus = CyU3PDmaChannelWaitForCompletion (&glChHandleMscOut,CYU3P_WAIT_FOREVER);
    }
    if(readed) *readed = dmaMscOutBuffer.count;
    ((DPDCsw*)&dev)->dwDataResidue -= dmaMscOutBuffer.count;
    /* Return Status */
    return apiRetStatus;
}

static void reset_csw(DPDCbw* cbw,DPDCsw* csw){
	csw->dwDataResidue = cbw->dwDataTransferLength;
	csw->byTag = cbw->byTag++;
    csw->byResLength = 0;
	memset(csw->abyResult,0,sizeof(csw->abyResult));
	csw->wCmdStatus = 0;
}

void
DebugPortInit (void);

void
SpiPortInit (void);

void
ApplicationInit (void);

/*
 * Entry function for the MscAppThread
 */
void
threadUsbComm_Entry (
        uint32_t input)
{
	extern uint8_t    *sim_sdram;
    CyU3PReturnStatus_t      apiRetStatus;
    uint32_t                 txApiRetStatus, eventFlag;
    process_command          usbCommand;
    DPDCbw                   cbw;
    DPDCsw                   csw;

    CyU3PDmaChannelConfig_t  dmaConfig;
    CyU3PDmaBuffer_t         dmaMscInBuffer;

    CyBool_t                 readQueued = CyFalse;

    /* Initialize the Debug Module */
    //DebugPortInit();
    SpiPortInit();

    /* Initialize the MSC Application */
    ApplicationInit();

    csw.dwSignature = DPD_CSW_SIGNATURE;

    sim_sdram = CyU3PMemAlloc(16384);
    if(sim_sdram == NULL){
    	CyU3PDebugPrint(1, "Create SDRAM fail.\r\n");
    	while(1);
    }

    for (;;)
    {
        /* Wait for a Set Configuration Event */
        txApiRetStatus = CyU3PEventGet (&appEvent, CY_FX_MSC_SET_CONFIG_EVENT,CYU3P_EVENT_AND_CLEAR,
                                        &eventFlag, CYU3P_WAIT_FOREVER);

        if (txApiRetStatus == CY_U3P_SUCCESS)
        	dmaSetConfigured(&dmaConfig);

        /* Initialize the DMA IN buffer members */
        dmaMscInBuffer.buffer = (uint8_t*)&cbw;
        dmaMscInBuffer.status = 0;
        dmaMscInBuffer.count = 0;
        dmaMscInBuffer.size = dmaConfig.size;

        /* Whenever we have restarted the USB connection, the read needs to be queued afresh. */
        readQueued = CyFalse;

        for (;;)
        {
            if (glInPhaseError == CyTrue)
            {
                /* We will need to queue the read again after the stall is cleared. */
                readQueued = CyFalse;
                continue;
            }

            /* Setup IN buffer */
            if (!readQueued)
            {
                dmaMscInBuffer.buffer = (uint8_t*)&cbw;
                apiRetStatus = CyU3PDmaChannelSetupRecvBuffer (&glChHandleMscIn, &dmaMscInBuffer);
                if (apiRetStatus == CY_U3P_SUCCESS)
                {
                    /* We can allow the USB link to move to U1/U2 while waiting for a command. */
                    readQueued = CyTrue;
                    CyU3PUsbLPMEnable ();
                }
            }

            if (readQueued)
            {
                /* Wait for CBW received on the IN buffer */
                apiRetStatus = CyU3PDmaChannelWaitForRecvBuffer (&glChHandleMscIn, &dmaMscInBuffer, CYU3P_WAIT_FOREVER);
                if (apiRetStatus == CY_U3P_SUCCESS)
                {
                    CyU3PDebugPrint (1, "Get cbw%d (0x%X).\r\n",cbw.byTag,cbw.wCommand);
                    /* Command received. Disable LPM and move link back to U0. */
                    readQueued = CyFalse;
                    CyU3PUsbLPMDisable ();
                    if (glUsbSpeed == CY_U3P_SUPER_SPEED)
                        CyU3PUsbSetLinkPowerState (CyU3PUsbLPM_U0);

                    /* Validate command */
                    usbCommand = GetDpdCommandProcess((uint8_t*)&cbw, dmaMscInBuffer.count);
                    if (usbCommand == (process_command)0)
                    {
                        CyU3PDebugPrint (1, "Unsupport USB Command (0x%X).\r\n",cbw.wCommand);
                    }
                    else{
                    	reset_csw(&cbw,&csw);
                    	apiRetStatus = usbCommand(cbw.abyArgument,cbw.byCBLength,
                    			csw.abyResult,&csw.byResLength,
								((cbw.bmFlags&0x80)?NULL:usb_read_cbk),
								((cbw.bmFlags&0x80)?usb_write_cbk:NULL),
								&csw,cbw.dwDataTransferLength);
                    	if((apiRetStatus == CY_U3P_SUCCESS)||(csw.dwDataResidue == 0)){
                    		csw.wCmdStatus = apiRetStatus;
                    		apiRetStatus = usbSendCsw(&csw);
                    		if(apiRetStatus == CY_U3P_SUCCESS)
                    			CyU3PDebugPrint (1, "Sent csw (0x%X).\r\n",csw.wCmdStatus);
                    	}
                    }
                }
                else
                {
                    /* Command not received as yet. Don't treat this as an error. */
                    apiRetStatus = CY_U3P_SUCCESS;
                }
            }

            /* Check for Reset conditions */
            /* Reset due to USB Reset Event or USB Cable Unplug */
            if (apiRetStatus != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (4, "Stalling MSC endpoints: %d\r\n", apiRetStatus);

                /* Stall both the IN and OUT Endpoints. */
                CyU3PUsbStall (CY_FX_MSC_EP_BULK_OUT, 1, 0);
                CyU3PUsbStall (CY_FX_MSC_EP_BULK_IN, 1, 0);

                if (glMscChannelCreated)
                {
                    /* Reset the DMA channels */
                    /* Reset the IN channel */
                    CyU3PDmaChannelReset(&glChHandleMscIn);

                    /* Reset the OUT Channel */
                    CyU3PDmaChannelReset(&glChHandleMscOut);
                }

                /* Request Sense Index */
                glReqSenseIndex = CY_FX_MSC_SENSE_DEVICE_RESET;
            }

            /* Check if USB Disconnected */
            if (glUsbSpeed == CY_U3P_NOT_CONNECTED)
            {
                if (glMscChannelCreated)
                {
                    /* Destroy the IN channel */
                    CyU3PDmaChannelDestroy (&glChHandleMscIn);

                    /* Destroy the OUT Channel */
                    CyU3PDmaChannelDestroy (&glChHandleMscOut);

                    glMscChannelCreated = CyFalse;
                }

                /* Request Sense Index */
                glReqSenseIndex = CY_FX_MSC_SENSE_DEVICE_RESET;

                break;
            }
        }
    }
}


/* Callback to handle the USB Setup Requests and Mass Storage Class requests */
CyBool_t
cbkUsbSetup (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
    )
{
    CyBool_t mscHandleReq = CyFalse;
    uint8_t maxLun = 0;
    CyU3PReturnStatus_t apiRetStatus;
    uint32_t txApiRetStatus;
    CyU3PEpConfig_t endPointConfig;

    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex, wLength; /* Decode the fields from the setup request. */

    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength  = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);

    /* Check for Set Configuration request */
    if (bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glDevConfigured)
                CyU3PUsbAckSetup ();
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);

            mscHandleReq = CyTrue;
        }

        /* Check for Set Configuration request */
        if (bRequest == CY_U3P_USB_SC_SET_CONFIGURATION)
        {
            /* Check if Set Config event is handled */
            if (glDevConfigured == CyFalse )
            {
                glDevConfigured = CyTrue;

                /* Configure the endpoints based on the USB speed */

                /* Get the Bus speed */
                glUsbSpeed = CyU3PUsbGetSpeed();

                /* Based on the Bus Speed configure the endpoint packet size */
                uint32_t len = getUsbTransferLength();
                if(len == 0)
                    CyU3PDebugPrint (4, "Error! USB Not connected\r\n");
                else
                	endPointConfig.pcktSize = len;

                /* Set the Endpoint Configurations */

                /* Producer Endpoint configuration */
                endPointConfig.enable = 1;
                endPointConfig.epType = CY_U3P_USB_EP_BULK;
                endPointConfig.streams = 0;
                endPointConfig.burstLen = 1;

                /* Configure the Endpoint */
                apiRetStatus = CyU3PSetEpConfig(CY_FX_MSC_EP_BULK_OUT,&endPointConfig);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    /* Error Handling */
                    CyU3PDebugPrint (4, "USB Set Endpoint config failed, Error Code = %d\r\n", apiRetStatus);
                    CyFxAppErrorHandler(apiRetStatus);
                }

                /* Consumer Endpoint configuration */
                endPointConfig.enable = 1;
                endPointConfig.epType = CY_U3P_USB_EP_BULK;
                endPointConfig.streams = 0;
                endPointConfig.burstLen = 1;

                /* Configure the Endpoint */
                apiRetStatus = CyU3PSetEpConfig(CY_FX_MSC_EP_BULK_IN,&endPointConfig);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    /* Error Handling */
                    CyU3PDebugPrint (4, "USB Set Endpoint config failed, Error Code = %d\r\n", apiRetStatus);
                    CyFxAppErrorHandler(apiRetStatus);
                }

                /* Set Event to indicate Set Configuration */
                txApiRetStatus = CyU3PEventSet(&appEvent,CY_FX_MSC_SET_CONFIG_EVENT,CYU3P_EVENT_OR);
                if (txApiRetStatus != CY_U3P_SUCCESS)
                {
                    /* Error handling */
                    CyU3PDebugPrint (4, "Bulk Loop App Set Event Failed, Error Code = %d\r\n", txApiRetStatus);
                }

                CyU3PDebugPrint (4, "SetConfig event set\r\n");
            }

        }
        else if (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
        {
            if (glInPhaseError)
            {
                /* Acknowledge the Setup Request */
                CyU3PUsbAckSetup ();
                return CyTrue;
            }
            /* Check Clear Feature on IN EP */
            if (wIndex == CY_FX_MSC_EP_BULK_IN)
            {
                /* Clear stall */
                CyU3PUsbStall(CY_FX_MSC_EP_BULK_IN,CyFalse,CyTrue);

                mscHandleReq = CyTrue;

                /* Acknowledge the Setup Request */
                CyU3PUsbAckSetup ();
            }

            /* Check Clear Feature on OUT EP */
            if (wIndex == CY_FX_MSC_EP_BULK_OUT)
            {
                /* Clear stall */
                CyU3PUsbStall(CY_FX_MSC_EP_BULK_OUT,CyFalse,CyTrue);

                mscHandleReq = CyTrue;

                /* Acknowledge the Setup Request */
                CyU3PUsbAckSetup ();
            }
        }
        else
        {
            /* All other requests are ignored by application */
            /* No operation */
        }
    }
    /* Check for MSC Requests */
    else if (bType == CY_FX_MSC_USB_CLASS_REQ)
    {
        mscHandleReq = CyFalse;
        apiRetStatus = CY_U3P_SUCCESS;

        if ((bTarget == CY_U3P_USB_TARGET_INTF) &&
                (wIndex == CY_FX_USB_MSC_INTF) && (wValue == 0))
        {
            /* Get MAX LUN Request */
            if (bRequest == CY_FX_MSC_GET_MAX_LUN_REQ)
            {
                if (wLength == 1)
                {
                    mscHandleReq = CyTrue;
                    /* Send response */
                    apiRetStatus = CyU3PUsbSendEP0Data(0x01, &maxLun);
                    if (apiRetStatus != CY_U3P_SUCCESS)
                    {
                        /* Error handling */
                        CyU3PDebugPrint (4, "Send EP0 Data Failed, Error Code = %d\r\n", apiRetStatus);
                    }
                }
            }
            /* BOT Reset Request */
            else if (bRequest == CY_FX_MSC_BOT_RESET_REQ)
            {
                mscHandleReq = CyTrue;
                glInPhaseError = CyFalse;
                if (wLength == 0)
                {
                    CyU3PUsbAckSetup ();

                    if (glMscChannelCreated)
                    {
                        CyU3PDmaChannelReset(&glChHandleMscOut);
                        CyU3PDmaChannelReset(&glChHandleMscIn);
                    }

                    CyU3PUsbFlushEp (CY_FX_MSC_EP_BULK_OUT);
                    CyU3PUsbStall (CY_FX_MSC_EP_BULK_OUT,  CyFalse, CyTrue);

                    CyU3PUsbFlushEp (CY_FX_MSC_EP_BULK_IN);
                    CyU3PUsbStall (CY_FX_MSC_EP_BULK_IN,  CyFalse, CyTrue);

                    /* Request Sense Index */
                    glReqSenseIndex = CY_FX_MSC_SENSE_DEVICE_RESET;
                }
                else
                {
                     CyU3PUsbStall (0x00, CyTrue, CyFalse);
                }
            }
            else
            {
                /* No operation */
            }
        }

        if ((mscHandleReq == CyFalse) || (apiRetStatus != CY_U3P_SUCCESS))
        {
            /* This is a failed handling. Stall the EP. */
            CyU3PUsbStall (0, CyTrue, CyFalse);
        }
    }
    else
    {
        /* No operation */
    }

    return mscHandleReq;
}

/* This function validates the CBW received w.r.t signature and CBW length */
process_command
GetDpdCommandProcess(
    uint8_t *buffer, /* Pointer to the CBW buffer */
    uint16_t count   /* Length of the CBW */
    )
{
    /* Verify signature */
    if (*(unsigned int*)buffer != DPD_CBW_SIGNATURE)
    {
        return (process_command)0;
    }
    else
    {
        /* Verify count */
        if (count != DPD_CBW_SIZE)
        {
        	return (process_command)0;
        }
    }

    return GetCommandProcess(((DPDCbw*)buffer)->wCommand);
}

/* This function frames the CSW and sends it to the host */
CyFxMscCswReturnStatus_t
usbSendCsw (
    DPDCsw* csw    /* Command Status */
    )
{
	CyU3PDmaBuffer_t dmaMscOutBuffer;
	CyU3PReturnStatus_t apiRetStatus;

	dmaMscOutBuffer.buffer = (uint8_t*)csw;
	dmaMscOutBuffer.status = 0;
	dmaMscOutBuffer.count = sizeof(DPDCsw);


	dmaMscOutBuffer.size = getUsbTransferLength();

    /* Check for phase error */
    if (csw->wCmdStatus == CY_FX_CBW_CMD_PHASE_ERROR)
    {
        glInPhaseError = CyTrue;

        CyU3PUsbSetEpNak (CY_FX_MSC_EP_BULK_IN, CyTrue);
        CyU3PBusyWait (125);

        /* Stall the IN endpoint */
        CyU3PUsbStall(CY_FX_MSC_EP_BULK_IN, CyTrue, CyFalse);

        /* Stall the OUT endpoint */
        CyU3PUsbStall(CY_FX_MSC_EP_BULK_OUT, CyTrue, CyFalse);

        CyU3PDmaChannelReset(&glChHandleMscIn);
        CyU3PDmaChannelReset(&glChHandleMscOut);

        CyU3PUsbFlushEp(CY_FX_MSC_EP_BULK_IN);
        CyU3PUsbFlushEp(CY_FX_MSC_EP_BULK_OUT);

        CyU3PUsbSetEpNak (CY_FX_MSC_EP_BULK_IN, CyFalse);
    }
    /* Check for Command failed */
    else if (csw->wCmdStatus == CY_FX_CBW_CMD_FAILED)
    {
        /* Only when data is expected or to be sent stall the EPs */
        if (glDataTxLength != 0)
        {
            /* Check direction */
            if (glCmdDirection == 0x00)
            {
                /* Stall the OUT endpoint */
                CyU3PUsbStall(CY_FX_MSC_EP_BULK_OUT, CyTrue, CyFalse);
            }
            else
            {
                CyU3PUsbSetEpNak (CY_FX_MSC_EP_BULK_IN, CyTrue);
                CyU3PBusyWait (125);

                /* Stall the IN endpoint */
                CyU3PUsbStall(CY_FX_MSC_EP_BULK_IN, CyTrue, CyFalse);

                CyU3PDmaChannelReset (&glChHandleMscIn);
                CyU3PUsbFlushEp(CY_FX_MSC_EP_BULK_IN);

                CyU3PUsbSetEpNak (CY_FX_MSC_EP_BULK_IN, CyFalse);
            }
        }
    }
    else
    {
        /* No operation. Proceed to send the status */
    }

    /* OUT buffer setup when there is no abort */
    apiRetStatus = CyU3PDmaChannelSetupSendBuffer (&glChHandleMscOut, &dmaMscOutBuffer);
    if (apiRetStatus == CY_U3P_SUCCESS)
    {
        /* Wait for data to be sent */
    	apiRetStatus = CyU3PDmaChannelWaitForCompletion (&glChHandleMscOut,CYU3P_WAIT_FOREVER);
    }
    /* Send the status to the host */
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
    	return CY_FX_CBW_CMD_MSC_RESET;
    }

    /* Return status */
    return CY_FX_CBW_CMD_PASSED;
}


/* This is the Callback function to handle the USB Events */
void
cbkUsbEvent (
    CyU3PUsbEventType_t evtype, /* Event type */
    uint16_t            evdata  /* Event data */
    )
{
    CyU3PDebugPrint (4, "USB event %d %d\r\n", evtype, evdata);

    /* Check for Reset / Suspend / Disconnect / Connect Events */
    if ((evtype == CY_U3P_USB_EVENT_RESET) || (evtype == CY_U3P_USB_EVENT_SUSPEND) ||
        (evtype == CY_U3P_USB_EVENT_DISCONNECT) || (evtype == CY_U3P_USB_EVENT_CONNECT))
    {
        if (glMscChannelCreated)
        {
            /* Abort the IN Channel */
            CyU3PDmaChannelAbort(&glChHandleMscIn);

            /* Abort the OUT Channel */
            CyU3PDmaChannelAbort(&glChHandleMscOut);
        }

        /* Request Sense Index */
        glReqSenseIndex = CY_FX_MSC_SENSE_DEVICE_RESET;

        /* Init the USB Speed */
        glUsbSpeed = CY_U3P_NOT_CONNECTED;

        /* Clear Flag */
        glDevConfigured = CyFalse;
    }
}

/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t
cbkAppLpmReq (
        CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}
#if 0
/* This function initializes the Debug Module for the MSC Application */
void
DebugPortInit()
{

    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus;

    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set UART Configuration */
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;

    /* Set the UART configuration */
    apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set the UART transfer */
    apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Initialize the Debug application */
    apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Disable Preamble */
    CyU3PDebugPreamble (CyFalse);
}
#endif

/* This function initializes the USB Module, sets the enumeration descriptors,
   configures the Endpoints and configures the DMA module for the
   MSC Application */
void
ApplicationInit (void)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint32_t txApiRetStatus;

    /* Create FX MSC events */
    txApiRetStatus = CyU3PEventCreate(&appEvent);
    if (txApiRetStatus != 0)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "MSC Appln Create Event Failed, Error Code = %d\r\n", txApiRetStatus);
    }

    /* Start the USB functionality */
    apiRetStatus = CyU3PUsbStart();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Function Failed to Start, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Setup the Callback to Handle the USB Setup Requests */
    /* Set Fast enumeration to False */
    CyU3PUsbRegisterSetupCallback(cbkUsbSetup, CyFalse);

    /* Setup the Callback to Handle the USB Events */
    CyU3PUsbRegisterEventCallback(cbkUsbEvent);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback(cbkAppLpmReq);
    /* Set the USB Enumeration descriptors */

    /* Device Descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set Device Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Device Descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set Device Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Device Qualifier Descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set Device Qualifier Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Other Speed Descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set Other Speed Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Configuration Descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set Configuration Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Configuration Descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set Configuration Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* BOS Descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set Configuration Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String Descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set String Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String Descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set String Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String Descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set String Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
#if 0
    /* String Descriptor 3 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 3, (uint8_t *)CyFxUSBSerialNumberDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set String Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
#endif
    /* Connect the USB Pins */
    /* Enable Super Speed operation */
    apiRetStatus = CyU3PConnectState(1,CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Connect failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
}

