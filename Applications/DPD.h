/*
 * DPD.h
 *
 *  Created on: 2015Äê4ÔÂ21ÈÕ
 *      Author: Administrator
 */

#ifndef DPD_H_
#define DPD_H_
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
/// \page "DPD Protocol Codes"
/// This page lists the Transport Protocol codes for DPD.
/// (Table 3.1, DediProg Device Spec. Overview)
///
/// !Protocols
/// - DPD_PROTOCOL_CBI_COMPLETION
/// - DPD_PROTOCOL_CBI
/// - DPD_PROTOCOL_BULK_ONLY
/*
/// Control/Bulk/Interrupt (CBI) Transport (with command complete interrupt)
#define DPD_PROTOCOL_CBI_COMPLETION             0x00
/// Control/Bulk/Interrupt (CBI) Transport (no command complete interrupt)
#define DPD_PROTOCOL_CBI                        0x01
/// Bulk-Only Transport
#define DPD_PROTOCOL_BULK_ONLY                  0x50
//------------------------------------------------------------------------------
*/

/// Test unit control:
#define CTRL_NOT_READY                          0x00
#define CTRL_GOOD                               0x01
#define CTRL_BUSY                               0x02

//------------------------------------------------------------------------------
/// \page "DPD CBW Definitions"
/// This page lists the Command Block Wrapper (CBW) definitions.
///
/// !Constants
/// - DPD_CBW_SIZE
/// - DPD_CBW_SIGNATURE
///
/// !Fields
/// - DPD_CBW_DEVICE_TO_HOST

/// Command Block Wrapper Size
#define DPD_CBW_SIZE                            sizeof(DPDCbw)
/// 'Dedi' 0x69646544
#define DPD_CBW_SIGNATURE                       0x69646544

/// CBW bmCBWFlags field
#define DPD_CBW_DEVICE_TO_HOST                  (1 << 7)
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// \page "DPD CSW Definitions"
/// This page lists the Command Status Wrapper (CSW) definitions.
///
/// !Constants
/// - DPD_CSW_SIZE
/// - DPD_CSW_SIGNATURE
///
/// !Command Block Status Values
/// (Table 5.3 , USB Mass Storage Class Bulk-Only Transport)
/// - DPD_CSW_COMMAND_PASSED
/// - DPD_CSW_COMMAND_FAILED
/// - DPD_CSW_PHASE_ERROR

/// Command Status Wrapper Size
#define DPD_CSW_SIZE                            sizeof(DPDCsw)
/// 'Prog' 0x676F7250
#define DPD_CSW_SIGNATURE                       0x676F7250

/// Command Passed (good status)
#define DPD_CSW_COMMAND_PASSED                  0
/// Command Failed
#define DPD_CSW_COMMAND_FAILED                  0x80
/// Phase Error
#define DPD_CSW_PHASE_ERROR                     0x81

#define DPD_CSW_RES_DEFN						0
#define DPD_CSW_RES_ERR							1
#define DPD_CSW_RES_PRC							2
#define DPD_CSW_RES_RESULT						3
#define DPD_CSW_RES_NONE						0xFF

//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
//      Structures
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Command Block Wrapper (CBW),
/// See Table 5.1, USB Mass Storage Class Bulk-Only Transport.
///
/// The CBW shall start on a packet boundary and shall end as a
/// short packet with exactly 31 (1Fh) bytes transferred.
//------------------------------------------------------------------------------
typedef struct {

    /// 'Dedi' 0x69646544 (little endian)
    unsigned int dwSignature;
    /// Number of bytes transfer append the DPDCbw
    unsigned int dwDataTransferLength;
    /// Must be the same as DPDCsw.byTag
    unsigned char byTag;
	/// Version of it.
    unsigned char byVersion;
    /// Indicates the directin of the transfer:
    /// 0x80=IN=device-to-host,
    /// 0x00=OUT=host-to-device
    unsigned char bmFlags;
    /// Length of CommandBlock
    unsigned char byCBLength;
	/// wCommand
	unsigned short wCommand;
    /// Command block
    unsigned char abyArgument[50];

} DPDCbw;

//------------------------------------------------------------------------------
/// Command Status Wrapper (CSW),
/// See Table 5.2, USB Mass Storage Class Bulk-Only Transport.
//------------------------------------------------------------------------------
typedef struct
{
    /// 'Prog' 0x676F7250 (little endian)
    unsigned int dwSignature;
    /// For Data-Out the device shall report in the dCSWDataResidue the
    /// difference between the amount of data expected as stated in the
    /// dCBWDataTransferLength, and the actual amount of data processed by
    /// the device. For Data-In the device shall report in the dCSWDataResidue
    /// the difference between the amount of data expected as stated in the
    /// dCBWDataTransferLength and the actual amount of relevant data sent by
    /// the device. The dCSWDataResidue shall not exceed the value sent in the
    /// dCBWDataTransferLength.
    unsigned int dwDataResidue;
    /// Must be the same as DPDCbw.byTag
    unsigned char byTag;
	/// Length Of the Result
    unsigned char byResLength;
    /// Indicates the success or failure of the command.
	unsigned short wCmdStatus;

	unsigned char abyResult[52];

} DPDCsw;


#endif /* DPD_H_ */
