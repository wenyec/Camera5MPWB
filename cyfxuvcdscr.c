/*
 ## Cypress FX3 Camera Kit Source file (cyfxuvcdscr.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
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

/*
 * This file contains the USB descriptors the videology 24C2.05WDIG camera with USB 3.0 interface
 * This is a 1080P camera @ 30 FPS
 * date : 15-May-2013
 * eddited by: Rene Damen
 * rev 02: add synchronous transfer and add additional control features
 * 		features that has been added are:
 * 		wb, aex-mode, blc mode, mirror/flip, zoom
 */

#include "uvc.h"
#include "camera_ptzcontrol.h"

/* Standard Device Descriptor */
const uint8_t CyFxUSBDeviceDscr[] =
    {
        0x12,                           /* Descriptor Size */
        CY_U3P_USB_DEVICE_DESCR,        /* Device Descriptor Type */
        0x00,0x02,                      /* USB 2.0 */
        0xEF,                           /* Device Class */
        0x02,                           /* Device Sub-class */
        0x01,                           /* Device protocol */
        0x40,                           /* Maxpacket size for EP0 : 64 bytes */
        0xBD,0x1B,                      /* Vendor ID 0xB4,0x04(Cypress); 0xBD,0x1B (Videology)*/
#ifndef CAM720
        0x61,0x30,                      /* Product ID 0xFA,0x00(Cypress FX3); 0x61,0x30, (color USB3.0 Cam 1080p NTSC)*/
#else
        0x62,0x30,                      /* Product ID 0xFA,0x00(Cypress FX3); 0x62,0x30, (color USB3.0 Cam 720p NTSC)*/
#endif
        0x00,0x00,                      /* Device release number */
        0x01,                           /* Manufacture string index */
        0x02,                           /* Product string index */
        0x00,                           /* Serial number string index */
        0x01                            /* Number of configurations */
    };

/* Device Descriptor for SS */
const uint8_t CyFxUSBDeviceDscrSS[] =
    {
        0x12,                           /* Descriptor Size */
        CY_U3P_USB_DEVICE_DESCR,        /* Device Descriptor Type */
        0x00,0x03,                      /* USB 3.0 */
        0xEF,                           /* Device Class */
        0x02,                           /* Device Sub-class */
        0x01,                           /* Device protocol */
        0x09,                           /* Maxpacket size for EP0 : 2^9 Bytes */
        0xBD,0x1B,                      /* Vendor ID 0xB4,0x04(Cypress); 0xBD,0x1B (Videology)*/
#ifndef CAM720
        0x61,0x30,                      /* Product ID 0xFA,0x00(Cypress FX3); 0x61,0x30, (color USB3.0 Cam 1080p NTSC)*/
#else
        0x62,0x30,                      /* Product ID 0xFA,0x00(Cypress FX3); 0x62,0x30, (color USB3.0 Cam 720p NTSC)*/
#endif
        0x00,0x00,                      /* Device release number */
        0x01,                           /* Manufacture string index */
        0x02,                           /* Product string index */
        0x00,                           /* Serial number string index */
        0x01                            /* Number of configurations */
    };

/* Standard Device Qualifier Descriptor */
const uint8_t CyFxUSBDeviceQualDscr[] =
    {
        0x0A,                           /* Descriptor Size */
        CY_U3P_USB_DEVQUAL_DESCR,       /* Device Qualifier Descriptor Type */
        0x00,0x02,                      /* USB 2.0 */
        0xEF,                           /* Device Class */
        0x02,                           /* Device Sub-class */
        0x01,                           /* Device protocol */
        0x40,                           /* Maxpacket size for EP0 : 64 bytes */
        0x01,                           /* Number of configurations */
        0x00                            /* Reserved */
    };

/* Standard Full Speed Configuration Descriptor */
const uint8_t CyFxUSBFSConfigDscr[] =
    {

        /* Configuration Descriptor Type */
        0x09,                           /* Descriptor Size */
        CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
        0x09,0x00,                      /* Length of this descriptor and all sub descriptors */
        0x00,                           /* Number of interfaces */
        0x01,                           /* Configuration number */
        0x00,                           /* COnfiguration string index */
        0x80,                           /* Config characteristics - Bus powered */
        0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */
    };

/* Standard High Speed Configuration Descriptor */
const uint8_t CyFxUSBHSConfigDscr[] =
    {

        /* Configuration Descriptor Type */
        0x09,                           /* Descriptor Size */
        CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
#ifdef USB_DEBUG_INTERFACE
        0xDC, 0x00//0xE4,0x00,//0xF6,0x00,                      /* Length of this descriptor and all sub descriptors */
        0x03,                           /* Number of interfaces */
#else
        0xDD, 0x00,//0xDF,0x00,                      /* Length of this descriptor and all sub descriptors */
        0x02,                           /* Number of interfaces */
#endif
        0x01,                           /* Configuration number */
        0x00,                           /* COnfiguration string index */
        0x80,                           /* Config characteristics - Bus powered */
        0xFA,                           /* Max power consumption of device (in 2mA unit) : 500mA */

        /* Interface Association Descriptor */
        0x08,                           /* Descriptor Size */
        CY_FX_INTF_ASSN_DSCR_TYPE,      /* Interface Association Descr Type: 11 */
        0x00,                           /* I/f number of first VideoControl i/f */
        0x02,                           /* Number of Video i/f */
        0x0E,                           /* CC_VIDEO : Video i/f class code */
        0x03,                           /* SC_VIDEO_INTERFACE_COLLECTION : Subclass code */
        0x00,                           /* Protocol : Not used */
        0x00,                           /* String desc index for interface */

        /* Standard Video Control Interface Descriptor */
        0x09,                           /* Descriptor size */
        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
        0x00,                           /* Interface number */
        0x00,                           /* Alternate setting number */
        0x01,                           /* Number of end points */
        0x0E,                           /* CC_VIDEO : Interface class */
        0x01,                           /* CC_VIDEOCONTROL : Interface sub class */
        0x00,                           /* Interface protocol code */
        0x00,                           /* Interface descriptor string index */

        /* Class specific VC Interface Header Descriptor */
        0x0D,                           /* Descriptor size */
        0x24,                           /* Class Specific I/f Header Descriptor type */
        0x01,                           /* Descriptor Sub type : VC_HEADER */
        0x00,0x01,                      /* Revision of class spec : 1.0 */
        0x50,0x00,                      /* Total Size of class specific descriptors (till Output terminal) */
        0x10,0xf7,0x6c,0x04, //=74.25MHz            /* Clock frequency : 0x00,0x6C,0xDC,0x02=48MHz(Deprecated) */
        0x01,                           /* Number of streaming interfaces */
        0x01,                           /* Video streaming I/f 1 belongs to VC i/f */

        /* Input (Camera) Terminal Descriptor */
        0x12,                           /* Descriptor size */
        0x24,                           /* Class specific interface desc type */
        0x02,                           /* Input Terminal Descriptor type */
        0x01,                           /* ID of this terminal */
        0x01,0x02,                      /* Camera terminal type */
        0x00,                           /* No association terminal */
        0x00,                           /* String desc index : Not used */
#ifdef UVC_PTZ_SUPPORT
        (uint8_t)(wObjectiveFocalLengthMin&0xFF),
        (uint8_t)((wObjectiveFocalLengthMin>>8)&0xFF),
        (uint8_t)(wObjectiveFocalLengthMax&0xFF),
        (uint8_t)((wObjectiveFocalLengthMax>>8)&0xFF),
        (uint8_t)(wOcularFocalLength&0xFF),
        (uint8_t)((wOcularFocalLength>>8)&0xFF),
#else
        0x00, 0x00, //0x64,0x00,                      /* No absolute optical zoom supported */
        0x00, 0x00, //0xE8,0x03,                      /* No absolute optical zoom supported */
        0x00, 0x00, //0x0A,0x00,                      /* No absolute optical zoom supported */
#endif
        0x03,                           /* Size of controls field for this terminal : 3 bytes */
#ifdef UVC_PTZ_SUPPORT
        0x00,0x0A,0x00,
#else
        0x9e,0x05,0x00,                 /* controls supported org.:0x9e,0x01,0x00*/
#endif

        /* Processing Unit Descriptor */
        0x0C,                           /* Descriptor size */
        0x24,                           /* Class specific interface desc type */
        0x05,                           /* Processing Unit Descriptor type */
        0x02,                           /* ID of this terminal */
        0x01,                           /* Source ID : 1 : Conencted to input terminal */
        0x58,0x02,                      /* Digital multiplier original: 0x00,0x40,*/
        0x03,                           /* Size of controls field for this terminal : 3 bytes */
        0xdf,0x45,0x00,                 /* Brightness Contrast Saturation sharpness, BLC, gain, mains freq, wb -0x1B,0x37,0x00
         	 	 	 	 	 	 	 	   controls supported   0x01,0x00,0x00 or 0x1B, 0x37, 0x00*/
        0x00,                           /* String desc index : Not used */

        /* Extension Unit Descriptor */
        0x1C,                           /* Descriptor size */
        0x24,                           /* Class specific interface desc type */
        0x06,                           /* Extension Unit Descriptor type */
        0x03,                           /* ID of this terminal */
        /* 16 byte GUID based on the GUID data structure
         *  {3757CA7A-1AA3-495b-968D-8E361F967650} */
        0x7A,0xCA,0x57,0x37,
        0xA3,0x1A,0x5B,0x49,
        0x96,0x8D,0x8E,0x36,
        0x1F,0x96,0x76,0x50,
        0x0f,                           /* Number of controls in this terminal */
        0x01,                           /* Number of input pins in this terminal */
        0x02,                           /* Source ID : 2 : Connected to Proc Unit */
        0x03,                           /* Size of controls field for this terminal : 3 bytes */
        0xff,0xd7,0x0f,		            /* controls supported */
        0x00,                           /* String desc index : Not used */

        /* Output Terminal Descriptor */
        0x09,                           /* Descriptor size */
        0x24,                           /* Class specific interface desc type */
        0x03,                           /* Output Terminal Descriptor type */
        0x04,                           /* ID of this terminal */
        0x01,0x01,                      /* USB Streaming terminal type */
        0x00,                           /* No association terminal */
        0x03,                           /* Source ID : 3 : Connected to Extn Unit */
        0x00,                           /* String desc index : Not used */

        /* Video Control Status Interrupt Endpoint Descriptor */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        CY_FX_EP_CONTROL_STATUS,        /* Endpoint address and description */
        CY_U3P_USB_EP_INTR,             /* Interrupt End point Type */
        0x00,0x04,                      /* Max packet size = 1024: 0x00,0x04, (64: 0x40, 0x00) bytes */
        0x01,                           /* Servicing interval: 8 ms*/

        /* Class Specific Interrupt Endpoint Descriptor */
        0x05,                           /* Descriptor size */
        0x25,                           /* Class Specific Endpoint Descriptor Type */
        CY_U3P_USB_EP_INTR,             /* End point Sub Type */
        0x40,0x00,                      /* Max packet size = 1024:  (64: 0x40, 0x00) bytes */

        /* Standard Video Streaming Interface Descriptor (Alternate Setting 0) */
        0x09,                           /* Descriptor size */
        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
        0x01,                           /* Interface number */
        0x00,                           /* Alternate setting number */
        0x01,                           /* Number of end points : Zero Bandwidth */
        0x0E,                           /* Interface class : CC_VIDEO */
        0x02,                           /* Interface sub class : CC_VIDEOSTREAMING */
        0x00,                           /* Interface protocol code : Undefined */
        0x00,                           /* Interface descriptor string index */

       /* Class-specific Video Streaming Input Header Descriptor */
        0x0E,                           /* Descriptor size */
        0x24,                           /* Class-specific VS I/f Type */
        0x01,                           /* Descriptotor Subtype : Input Header */
        0x01,                           /* 1 format desciptor follows */
        0x57,0x00,                      /* Total size of Class specific VS descr: 0x51 Bytes*/
        CY_FX_EP_BULK_VIDEO,            /* EP address for BULK video data */
        0x00,                           /* No dynamic format change supported */
        0x04,                           /* Output terminal ID : 4 */
        0x02,                           /* Still image capture method 2 supported */
        0x01,                           /* Hardware trigger supported */
        0x01,                           /* Hardware to initiate still image capture supported */
        0x01,                           /* Size of controls field : 1 byte */
        0x00,                           /* D2 : Compression quality supported */


       /* Class specific Uncompressed VS format descriptor */
        0x1B,                           /* Descriptor size */
        0x24,                           /* Class-specific VS I/f Type */
        0x04,                           /* Subtype : uncompressed format I/F */
        0x01,                           /* Format desciptor index (only one format is supported) */
        0x01,                           /* number of frame descriptor followed */
        0x59,0x55,0x59,0x32,            /* GUID used to identify streaming-encoding format: YUY2  */
        0x00,0x00,0x10,0x00,
        0x80,0x00,0x00,0xAA,
        0x00,0x38,0x9B,0x71,
        0x10,                           /* Number of bits per pixel used to specify color in the decoded video frame.
                                           0 if not applicable: 10 bit per pixel */
        0x01,                           /* Optimum Frame Index for this stream: 1 */
        0x08,                           /* X dimension of the picture aspect ratio: Non-interlaced in
			        	   progressive scan */
        0x06,                           /* Y dimension of the picture aspect ratio: Non-interlaced in
					   progressive scan*/
        0x00,                           /* Interlace Flags: Progressive scanning, no interlace */
        0x00,                           /* duplication of the video stream restriction: 0 - no restriction */

       /* Class specific Uncompressed VS Frame descriptor */
        0x1E,                           /* Descriptor size */
        0x24,                           /* Descriptor type*/
        0x05,                           /* Subtype: uncompressed frame I/F */
        0x01,                           /* Frame Descriptor Index */
        0x02,                           /* Still image capture method 2/3 supported, fixed frame rate */
#ifndef CAM720
#ifdef GPIFIIM
        0x80,0x02,
        0xE0,0x01,
#else
        0xC0,0x03,                      /* Width in pixel: 960(0xC0,0x03) or 640(0x80,0x02) */
        0x1C,0x02,                      /* Height in pixel 540(0x1C,0x02) or 480(0xE0,0x01) */
#endif
#else
        0x80,0x02,                      /* Width in pixel: 640(0x80,0x02) */
        0x68,0x01,                      /* Height in pixel 360(0x68,0x01) */
#endif
        0x00,0xE0,0xD4,0x0E,//0x00,0x10,0x5C,0x0C,            /* Min bit rate bits/s. Not specified, taken from MJPEG */
        0x00,0xE0,0xD4,0x0E,            /* Max bit rate bits/s. Not specified, taken from MJPEG */
        0x00,0xD2,0x0F,0x00,            /* Maximum video or still frame size in bytes(Deprecated) */
#ifndef CAM720
        0x15,0x16,0x05,0x00,            /* Default Frame Interval 30fps 0x15,0x16,0x05,0x00,*/
        0x01,                           /* Frame interval(Frame Rate) types: Only one frame interval supported */
        0x15,0x16,0x05,0x00,            /* Shortest Frame Interval 30fps :0x15,0x16,0x05,0x00, 25fps: 0x80,0x1a,0x06,0x00,*/
#else
        0x15,0x16,0x05,0x00,            /* Default Frame Interval 60fps 0x0A,0x8B,0x02,0x00 */
        0x01,                           /* Frame interval(Frame Rate) types: Only one frame interval supported */
        0x15,0x16,0x05,0x00,            /* Shortest Frame Interval 60fps :0x0A,0x8B,0x02,0x00*/
#endif
#if 1
        /* Class specific VS Still Image Frame descriptor for Method 2/3*/
         0x0A,                           /* Descriptor size */
         0x24,                           /* Descriptor type*/
         0x03,                           /* Subtype: still image frame I/F */
         0/*0x87 CY_FX_EP_BULK_IMAGE*/,            /* EP address for BULK still image the method 2 has to be set to 0 */
         0x01,                           /* Number of image size pattern */
#ifndef CAM720
#ifdef GPIFIIM
        0x80,0x02,
        0xE0,0x01,
#else
        0xC0,0x03,                      /* Width in pixel: 960(0xC0,0x03) or 640(0x80,0x02) */
        0x1C,0x02,                      /* Height in pixel 540(0x1C,0x02) or 480(0xE0,0x01) */
#endif
#else
        0x80,0x02,                      /* Width in pixel: 640(0x80,0x02) */
        0x68,0x01,                      /* Height in pixel 360(0x68,0x01) */
#endif
         0x00,                           /* Number of compression pattern of this format */
         //0x00,                           /* The uncompression still image */
#endif

         /* color matching descriptor */
         0x06,                           /* Descriptor size */
         0x24,                           /* Descriptor type */
         0x0D,                           /* Subtype: VC color match */
         0x00,                           /* color primaries: unspecified */
         0x00,                           /* color primaries: unspecified */
         0x00,                           /* color primaries: unspecified */

         /* Endpoint Descriptor for BULK Streaming Video Data */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        CY_FX_EP_BULK_VIDEO,            /* Endpoint address and description */
        0x02,                           /* BULK End point */
        (uint8_t)(512 & 0x00FF),        /* High speed max packet size is always 512 bytes. */
        (uint8_t)((512 & 0xFF00)>>8),
        0x01                            /* Servicing interval for data transfers */
#if 0 //for still image capture method 3
        /* Endpoint Descriptor for BULK Streaming still image Data */
        ,
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        0x87/*CY_FX_EP_BULK_IMAGE*/,            /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* BULK End point */
        (uint8_t)(512 & 0x00FF),        /* High speed max packet size is always 512 bytes. */
        (uint8_t)((512 & 0xFF00)>>8),
        0x00                           /* Servicing interval for data transfers */
#endif
#ifdef USB_DEBUG_INTERFACE
        ,
        0x09,                           /* Descriptor size */
        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
        0x02,                           /* Interface number */
        0x00,                           /* Alternate setting number */
        0x02,                           /* Number of end points */
        0xFF,                           /* Interface class */
        0x00,                           /* Interface sub class */
        0x00,                           /* Interface protocol code */
        0x00,                           /* Interface descriptor string index */

        /* Endpoint descriptor for producer EP */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
        CY_FX_EP_DEBUG_CMD,             /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* Bulk endpoint type */
        0x00,0x02,                      /* Max packet size = 512 bytes */
        0x00,                           /* Servicing interval for data transfers : 0 for bulk */
    
        /* Endpoint descriptor for consumer EP */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
        CY_FX_EP_DEBUG_RSP,             /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* Bulk endpoint type */
        0x00,0x02,                      /* Max packet size = 512 bytes */
        0x00                            /* Servicing interval for data transfers : 0 for Bulk */
#endif
    };

/* BOS for SS */

#define CY_FX_BOS_DSCR_TYPE             15
#define CY_FX_DEVICE_CAPB_DSCR_TYPE     16
#define CY_FX_SS_EP_COMPN_DSCR_TYPE     48

/* Device Capability Type Codes */
#define CY_FX_WIRELESS_USB_CAPB_TYPE    1
#define CY_FX_USB2_EXTN_CAPB_TYPE       2
#define CY_FX_SS_USB_CAPB_TYPE          3
#define CY_FX_CONTAINER_ID_CAPBD_TYPE   4

const uint8_t CyFxUSBBOSDscr[] =
{
        0x05,                           /* Descriptor Size */
        CY_FX_BOS_DSCR_TYPE,            /* Device Descriptor Type */
        0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
        0x02,                           /* Number of device capability descriptors */

        /* USB 2.0 Extension */
        0x07,                           /* Descriptor Size */
        CY_FX_DEVICE_CAPB_DSCR_TYPE,    /* Device Capability Type descriptor */
        CY_FX_USB2_EXTN_CAPB_TYPE,      /* USB 2.0 Extension Capability Type */
        0x00,0x00,0x00,0x00,            /* Supported device level features  */

        /* SuperSpeed Device Capability */
        0x0A,                           /* Descriptor Size */
        CY_FX_DEVICE_CAPB_DSCR_TYPE,    /* Device Capability Type descriptor */
        CY_FX_SS_USB_CAPB_TYPE,         /* SuperSpeed Device Capability Type */
        0x00,                           /* Supported device level features  */
        0x0E,0x00,                      /* Speeds Supported by the device : SS, HS and FS */
        0x03,                           /* Functionality support */
        0x00,                           /* U1 Device Exit Latency */
        0x00,0x00                       /* U2 Device Exit Latency */
};

/* Super Speed Configuration Descriptor */
const uint8_t CyFxUSBSSConfigDscr[] =
    {

        /* Configuration Descriptor Type */
        0x09,                           /* Descriptor Size */
        CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
#ifdef USB_DEBUG_INTERFACE
        0xFC,0x00,//0x0E,0x01,                      /* Total length of this and all sub-descriptors. */
        0x03,                           /* Number of interfaces */
#else
        0x50,0x01,//0xF1,0x00,//0xD9,0x00,//                      /* Length of this descriptor and all sub descriptors */
        0x02,                           /* Number of interfaces */
#endif
        0x01,                           /* Configuration number */
        0x00,                           /* Configuration string index */
        0x80,                           /* Config characteristics - Bus powered */
        0x64,                           /* Max power consumption of device (in 8mA unit) : 400mA(0x32)? */

        /* Interface Association Descriptor */
        0x08,                           /* Descriptor Size */
        CY_FX_INTF_ASSN_DSCR_TYPE,      /* Interface Association Descr Type: 11 */
        0x00,                           /* I/f number of first VideoControl i/f */
        0x02,                           /* Number of Video i/f */
        0x0E,                           /* CC_VIDEO : Video i/f class code */
        0x03,                           /* SC_VIDEO_INTERFACE_COLLECTION : Subclass code */
        0x00,                           /* Protocol : Not used */
        0x00,                           /* String desc index for interface */

        /* Standard Video Control Interface Descriptor */
        0x09,                           /* Descriptor size */
        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
        0x00,                           /* Interface number */
        0x00,                           /* Alternate setting number */
        0x01,                           /* Number of end points */
        0x0E,                           /* CC_VIDEO : Interface class */
        0x01,                           /* CC_VIDEOCONTROL : Interface sub class */
        0x00,                           /* Interface protocol code */
        0x00,                           /* Interface descriptor string index */

        /* Class specific VC Interface Header Descriptor */
        0x0D,                           /* Descriptor size */
        0x24,                           /* Class Specific I/f Header Descriptor type */
        0x01,                           /* Descriptor Sub type : VC_HEADER */
        0x00,0x01,                      /* Revision of class spec : 1.0 */
        0x50,0x00,                      /* Total Size of class specific descriptors (till Output terminal) */
        0x00,0xd8,0xb8,0x05,  //(96Mhz) /* Clock frequency : 74.25MHz(Deprecated) (0x10,0xf7,0x6c,0x04,)*/
        0x01,                           /* Number of streaming interfaces */
        0x01,                           /* Video streaming I/f 1 belongs to VC i/f */

        /* Input (Camera) Terminal Descriptor */
        0x12,                           /* Descriptor size */
        0x24,                           /* Class specific interface desc type */
        0x02,                           /* Input Terminal Descriptor type */
        0x01,                           /* ID of this terminal */
        0x01,0x02,                      /* Camera terminal type */
        0x00,                           /* no association terminal output terminal: usb video streaming*/
        0x00,                           /* String desc index : Not used */
#ifdef UVC_PTZ_SUPPORT
        (uint8_t)(wObjectiveFocalLengthMin&0xFF),
        (uint8_t)((wObjectiveFocalLengthMin>>8)&0xFF),
        (uint8_t)(wObjectiveFocalLengthMax&0xFF),
        (uint8_t)((wObjectiveFocalLengthMax>>8)&0xFF),
        (uint8_t)(wOcularFocalLength&0xFF),
        (uint8_t)((wOcularFocalLength>>8)&0xFF),
#else
        0x00,0x00,//0x64,0x00,                      /* No absolute optical zoom supported */
        0x00,0x00,//0x8E,0x03,                      /* No absolute optical zoom supported */
        0x00,0x00,//0x0A,0x00,                      /* No absolute optical zoom supported */
#endif
        0x03,                           /* Size of controls field for this terminal : 3 bytes */
#ifdef UVC_PTZ_SUPPORT
        0x00,0x0A,0x00,
#else
        0x00,0x00,0x00,/*0x9e,0x05,0x00,*/                 /*  controls supported 0x9e,0x01,0x00,*/
#endif

        /* Processing Unit Descriptor *//* FEATERS DESCRIPTION, FEATERS DESCRIPTION, FEATERS DESCRIPTION, FEATERS DESCRIPTION, FEATERS DESCRIPTION, */
        0x0C,                           /* Descriptor size */
        0x24,                           /* Class specific interface desc type */
        0x05,                           /* Processing Unit Descriptor type */
        0x02,                           /* ID of this terminal */
        0x01,                           /* Source ID : 1 : Connected to input terminal */
        0x58,0x02,                      /* Digital multiplier org:0x00,0x40,*/
        0x03,                           /* Size of controls field for this terminal : 3 bytes */
#ifdef  COLOR
        0xd3,0x05,0x00,					/* no hue and saturation control available */
#else
        0x13,0x05,0x00,/*0xdf,0x45,0x00,*/                 /* Brightness Contrast Saturation sharpness, BLC, gain, mains freq, wb -0x1B,0x37,0x00
         	 	 	 	 	 	 	 	   controls supported   0x01,0x00,0x00 or 0x1B, 0x37, 0x00*/
#endif
        0x00,                           /* String desc index : Not used */

        /* Extension Unit Descriptor */
        0x1D,                           /* Descriptor size */
        0x24,                           /* Class specific interface desc type */
        0x06,                           /* Extension Unit Descriptor type */
        0x03,                           /* ID of this terminal */
        /* 16 byte GUID based on the GUID data structure
         *  {3757CA7A-1AA3-495b-968D-8E361F967650} */
        0x7A,0xCA,0x57,0x37,
        0xA3,0x1A,0x5B,0x49,
        0x96,0x8D,0x8E,0x36,
        0x1F,0x96,0x76,0x50,
        /* 16 byte GUID this is not based on the GUID layout of the data structure
        0x50,0x76,0x96,0x1F,
        0x36,0x8E,0x8D,0x96,
        0x5B,0x49,0xA3,0x1A,
        0x7A,0xCA,0x57,0x37,*/
        0x0b,                           /* Number of controls in this terminal */
        0x01,                           /* Number of input pins in this terminal */
        0x02,                           /* Source ID : 1: connected to input terminal 2 : Connected to Proc Unit  0x02*/
        0x04,                           /* Size of controls field for this terminal : 3 bytes */
        0xfd,0xdf,0xff/*7*/,
        0x0e, // 0x04,0x86,0x00,           /* controls supported */
        0x00,                           /* String desc index : Not used */

        /* Output Terminal Descriptor */
        0x09,                           /* Descriptor size */
        0x24,                           /* Class specific interface desc type */
        0x03,                           /* Output Terminal Descriptor type */
        0x04,                           /* ID of this terminal */
        0x01,0x01,                      /* USB Streaming terminal type */
        0x00,                           /* No association terminal */
        0x03,                           /* Source ID : 3 : Connected to Extn Unit */
        0x00,                           /* String desc index : Not used */

        /* Video Control Status Interrupt Endpoint Descriptor */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        CY_FX_EP_CONTROL_STATUS,        /* Endpoint address and description */
        CY_U3P_USB_EP_INTR,             /* Interrupt End point Type */
        0x00,0x04,                      /* Max packet size = 1024 bytes */
        0x01,                           /* Servicing interval 8 ms*/

        /* Super Speed Endpoint Companion Descriptor */
        0x06,                           /* Descriptor size */
        CY_U3P_SS_EP_COMPN_DESCR,       /* SS Endpoint Companion Descriptor Type */
        0x00,                           /* Max no. of packets in a Burst : 1 */
        0x00,                           /* Attribute: N.A. */
        0x00,                           /* Bytes per interval:1024 */
        0x04,

        /* Class Specific Interrupt Endpoint Descriptor */
        0x05,                           /* Descriptor size */
        0x25,                           /* Class Specific Endpoint Descriptor Type */
        CY_U3P_USB_EP_INTR,             /* End point Sub Type */
        0x40,0x00,                      /* Max packet size = 64 bytes */


        /* Standard Video Streaming Interface Descriptor (Alternate Setting 0) */
        0x09,                           /* Descriptor size */
        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
        0x01,                           /* Interface number */
        0x00,                           /* Alternate setting number */
        0x01,                           /* Number of end points video + still image*/
        0x0E,                           /* Interface class : CC_VIDEO */
        0x02,                           /* Interface sub class : CC_VIDEOSTREAMING */
        0x00,                           /* Interface protocol code : Undefined */
        0x00,                           /* Interface descriptor string index */

       /* Class-specific Video Streaming Input Header Descriptor */
        0x0E,                           /* Descriptor size */
        0x24,                           /* Class-specific VS I/f Type */
        0x01,                           /* Descriptotor Subtype : Input Header */
        0x01,                           /* 1 format desciptor follows */
        0xBD,0x00,//0x47,0x00,                      /* Total size of Class specific VS descr */
        CY_FX_EP_BULK_VIDEO,            /* EP address for BULK video data */
        0x00,                           /* No dynamic format change supported */
        0x04,                           /* Output terminal ID : 4 */
        0x02,                           /* Still image capture method 2 supported */
        0x01,                           /* Hardware trigger supported */
        0x01,                           /* Hardware to initiate still image capture supported */
        0x01,                           /* Size of controls field : 1 byte */
        0x00,                           /* D2 : Compression quality supported */


        /* Class specific Uncompressed VS format descriptor */
        0x1B,                           /* Descriptor size */
        0x24,                           /* Class-specific VS I/f Type */
        0x04,                           /* Subtype : uncompressed format I/F */
        0x01,                           /* Format desciptor index */
        0x04,                           /* Number of frame descriptor followed */
        0x59,0x55,0x59,0x32,            /* GUID used to identify streaming-encoding format: YUY2 --> Is this correct, the order I got from Wim, next 3 lines are reversed?????????? */
        0x00,0x00,0x10,0x00,			/* From Wim: 00 00 00 10,*/
        0x80,0x00,0x00,0xAA,			/*         : 80 00 00 AA,*/
        0x00,0x38,0x9B,0x71,			/*         : 00 38 9B 71, --> first line from Wim was 32 59 55 59*/
        0x10,                           /* Number of bits per pixel  WAS 0x10 **********************************************************************/
        0x01,                           /* Optimum Frame Index for this stream: 1 */
        0x10,                           /* X dimension of the picture aspect ratio; Non-interlaced */
        0x09,                           /* Y dimension of the pictuer aspect ratio: Non-interlaced */
        0x00,                           /* Interlace Flags: Progressive scanning, no interlace */
        0x00,                           /* duplication of the video stream restriction: 0 - no restriction */

        /* Class specific Uncompressed VS frame descriptor 1 for Full Res 2592x1944*/
        0x1E,                           /* Descriptor size */
        0x24,                           /* Descriptor type*/
        0x05,                           /* Subtype: uncompressed frame I/F */
        0x01,                           /* Frame Descriptor Index */
        0x02,                           /* Still image capture method 1 is not supported in this descriptor, fixed frame rate */
        0x20, 0x0A,                     /* Width in pixel  2592(0x20, 0x0A) 1920 (0x80, 0x07) 1280 (0x00, 0x05)**********************************************************************************/
        0x98, 0x07,                     /* Height in pixel 1944(0x98, 0x07) 1080 (0x38, 0x04) 720 (0xd0, 0x02) **********************************************************************************/
        0x00,0xD0,0x14,0x48,            /* Min bit rate bits/s. 2250*1150*16*15= 0x00,0xD0,0x14,0x48 ******************************************************/
        0x00,0xD0,0x14,0x48, 		//0x00,0xD0,0x14,0x48,            /* Max bit rate bits/s.=2250*1150*16*15= 0x4814D000 ******************************************************/
        0x00,0xC6,0x99,0x00,            /* Maximum video or still frame size in bytes(Deprecated)*/
        0x2A,0x2C,0x0A,0x00,            /* Default Frame Interval 30fps 0x15,0x16,0x05,0x00,*/
        0x01,                           /* Frame interval(Frame Rate) types: Only one frame interval supported */
        0x2A,0x2C,0x0A,0x00,            /* Shortest Frame Interval 30fps :0x15,0x16,0x05,0x00, 25fps: 0x80,0x1a,0x06,0x00,*/

        /* Class specific Uncompressed VS frame descriptor 2 for 1080p*/
        0x1E,                           /* Descriptor size */
        0x24,                           /* Descriptor type*/
        0x05,                           /* Subtype: uncompressed frame I/F */
        0x02,                           /* Frame Descriptor Index */
        0x02,                           /* Still image capture method 1 is not supported in this descriptor, fixed frame rate */
        0x80, 0x07,                     /* Width in pixel  1920 (0x80, 0x07) 1280 (0x00, 0x05)**********************************************************************************/
        0x38, 0x04,                     /* Height in pixel 1080 (0x38, 0x04) 720 (0xd0, 0x02) **********************************************************************************/
        0x00,0x80,0x53,0x3B,            /* Min bit rate bits/s.=1920*1080*16*30= 0x3B538000 ******************************************************/
        0x00,0x80,0x53,0x3B, 		//0x00,0x28,0x26,0x84,            /* Max bit rate bits/s.=2250*1150*16*15= 0x4814D000 ******************************************************/
        0x00,0xA4,0x1F,0x00,            /* Maximum video or still frame size in bytes(Deprecated)*/
        0x15,0x16,0x05,0x00,            /* Default Frame Interval 30fps 0x15,0x16,0x05,0x00,*/
        0x01,                           /* Frame interval(Frame Rate) types: Only one frame interval supported */
        0x15,0x16,0x05,0x00,            /* Shortest Frame Interval 30fps :0x15,0x16,0x05,0x00, 25fps: 0x80,0x1a,0x06,0x00,*/

        /* Class specific Uncompressed VS frame descriptor 3 for 720p Res 1290x720*/
        0x1E,                           /* Descriptor size */
        0x24,                           /* Descriptor type*/
        0x05,                           /* Subtype: uncompressed frame I/F */
        0x03,                           /* Frame Descriptor Index */
        0x02,                           /* Still image capture method 1 is not supported in this descriptor, fixed frame rate */
        0x00, 0x05,                      /* Width in pixel: 1280 (0x00, 0x05) */
        0xd0, 0x02,                      /* Height in pixel 720 (0xd0, 0x02) */
        0x00,0x00,0xBC,0x34,            /* Min bit rate bits/s. 1280*720*16*60= 0x34BC0000 *****************/
        0x00,0x00,0xBC,0x34,			//0x00,0xD0,0x14,0x48,           /* Max bit rate bits/s.=2250*1150*16*30= 0x3b538000 *****/
        0x00,0xC6,0x99,0x00,            /* Maximum video or still frame size in bytes(Deprecated)*/
        0x0A,0x8B,0x02,0x00,            /* Default Frame Interval 60fps 0x0A,0x8B,0x02,0x00 */
        0x01,                           /* Frame interval(Frame Rate) types: Only one frame interval supported */
        0x0A,0x8B,0x02,0x00,            /* Shortest Frame Interval 60fps :0x0A,0x8B,0x02,0x00*/

        /* Class specific Uncompressed VS frame descriptor 4 for VGA Res 640x480*/
        0x1E,                           /* Descriptor size */
        0x24,                           /* Descriptor type*/
        0x05,                           /* Subtype: uncompressed frame I/F */
        0x04,                           /* Frame Descriptor Index */
        0x02,                           /* Still image capture method 1 is not supported in this descriptor, fixed frame rate */
        0x80, 0x02,                      /* Width in pixel: 640 (0x80, 0x02) */
        0xE0, 0x01,                      /* Height in pixel 480 (0xE0, 0x01) */
        0x00,0x00,0x28,0x23,            /* Min bit rate bits/s.=640*4800*16*25= 0x23230000 ********/
        0x00,0x00,0x28,0x23, 			//0x00,0x00,0x28,0x23,           /* Max bit rate bits/s.=2250*1150*16*30= 0x3b538000 *****/
        0x00,0xC6,0x99,0x00,            /* Maximum video or still frame size in bytes(Deprecated)*/
        0x85,0x45,0x01,0x00,            /* Default Frame Interval 120fps 0x85,0x45,0x01,0x00 */
        0x01,                           /* Frame interval(Frame Rate) types: Only one frame interval supported */
        0x85,0x45,0x01,0x00,            /* Shortest Frame Interval 120fps :0x85,0x45,0x01,0x00*/


#if 1
        /* Class specific VS Still Image Frame descriptor for Method 2/3*/
         0x16,                           /* Descriptor size */
         0x24,                           /* Descriptor type*/
         0x03,                           /* Subtype: still image frame I/F */
         0x00/*CY_FX_EP_BULK_IMAGE:0x87*/,            /* EP address for BULK still image */
         0x04,                           /* Number of image size pattern */
 //four res' of still image
        0x80, 0x02,                      /* Width in pixel: 640 (0x80, 0x02) */
        0xE0, 0x01,                      /* Height in pixel 480 (0xE0, 0x01) */
        0x00, 0x05,                      /* Width in pixel: 1280 (0x00, 0x05) */
        0xd0, 0x02,                      /* Height in pixel 720 (0xd0, 0x02) */
        0x80, 0x07,                     /* Width in pixel  1920 (0x80, 0x07) 1280 (0x00, 0x05)*/
        0x38, 0x04,                     /* Height in pixel 1080 (0x38, 0x04) 720 (0xd0, 0x02) */
        0x20, 0x0A,                     /* Width in pixel  2592 (0x20, 0x0A) 1280 (0x00, 0x05)*/
        0x98, 0x07,                     /* Height in pixel 1944 (0x98, 0x07) 720 (0xd0, 0x02) */

        0x00,                           /* Number of compression pattern of this format */
         //0x00,                           /* The uncompression still image */
#endif

         /* color matching descriptor */
         0x06,                           /* Descriptor size */
         0x24,                           /* Descriptor type */
         0x0D,                           /* Subtype: VC color match */
         0x00,                           /* color primaries: unspecified */
         0x00,                           /* color primaries: unspecified */
         0x00,                           /* color primaries: unspecified */

        /* Endpoint Descriptor for BULK Streaming Video Data */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        CY_FX_EP_BULK_VIDEO,            /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* BULK End point */
        CY_FX_EP_BULK_VIDEO_PKT_SIZE_L, /* EP MaxPcktSize: 1024B */
        CY_FX_EP_BULK_VIDEO_PKT_SIZE_H, /* EP MaxPcktSize: 1024B */
        0x01,                           /* Servicing interval for data transfers */

        /* Super Speed Endpoint Companion Descriptor for the Streaming Video Data endpoint */
        0x06,                           /* Descriptor size */
        CY_U3P_SS_EP_COMPN_DESCR,       /* SS Endpoint Companion Descriptor Type */
        0x0F,                           /* Max number of packets per burst: 15 */
        0x00,                           /* Attribute: Streams not defined */
        0x00,                           /* No meaning for bulk */
        0x00

#if 0 // for still image method 3
        /* Endpoint Descriptor for BULK Streaming still image Data */
        ,
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        0x87/*CY_FX_EP_BULK_IMAGE*/,            /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* BULK End point */
        CY_FX_EP_BULK_VIDEO_PKT_SIZE_L, /* EP MaxPcktSize: 1024B */
        CY_FX_EP_BULK_VIDEO_PKT_SIZE_H, /* EP MaxPcktSize: 1024B */
        0x00,                           /* Servicing interval for data transfers */

        /* Super Speed Endpoint Companion Descriptor for Streaming still image endpoint */
        0x06,                           /* Descriptor size */
        CY_U3P_SS_EP_COMPN_DESCR,       /* SS Endpoint Companion Descriptor Type */
        0x0F,                           /* Max number of packets per burst: 15 */
        0x00,                           /* Attribute: Streams not defined */
        0x00,                           /* No meaning for bulk */
        0x00
#endif

#ifdef USB_DEBUG_INTERFACE
        ,
        0x09,                           /* Descriptor size */
        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
        0x02,                           /* Interface number */
        0x00,                           /* Alternate setting number */
        0x02,                           /* Number of end points */
        0xFF,                           /* Interface class */
        0x00,                           /* Interface sub class */
        0x00,                           /* Interface protocol code */
        0x00,                           /* Interface descriptor string index */

        /* Endpoint descriptor for producer EP */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
        CY_FX_EP_DEBUG_CMD,             /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* Bulk endpoint type */
        0x00,0x04,                      /* Max packet size = 1024 bytes */
        0x00,                           /* Servicing interval for data transfers : 0 for bulk */
    
        /* Super speed endpoint companion descriptor for producer EP */
        0x06,                           /* Descriptor size */
        CY_U3P_SS_EP_COMPN_DESCR,       /* SS endpoint companion descriptor type */
        0,                              /* No burst support. */
        0x00,                           /* Max streams for bulk EP = 0 (No streams) */
        0x00,0x00,                      /* Service interval for the EP : 0 for bulk */
    
        /* Endpoint descriptor for consumer EP */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
        CY_FX_EP_DEBUG_RSP,             /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* Bulk endpoint type */
        0x00,0x04,                      /* Max packet size = 1024 bytes */
        0x00,                           /* Servicing interval for data transfers : 0 for Bulk */
    
        /* Super speed endpoint companion descriptor for consumer EP */
        0x06,                           /* Descriptor size */
        CY_U3P_SS_EP_COMPN_DESCR,       /* SS endpoint companion descriptor type */
        0,                              /* No burst support. */
        0x00,                           /* Max streams for bulk EP = 0 (No streams) */
        0x00,0x00                       /* Service interval for the EP : 0 for bulk */
#endif
    };


/* Standard Language ID String Descriptor */
const uint8_t CyFxUSBStringLangIDDscr[] =
    {
        0x04,                           /* Descriptor Size */
        CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
        0x09,0x04                       /* Language ID supported */
    };

/* Standard Manufacturer String Descriptor */

const uint8_t CyFxUSBManufactureDscr[] =
    {
        0x38,                           /* Descriptor Size */
        CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
        'V',0x00,
        'i',0x00,
        'd',0x00,
        'e',0x00,
        'o',0x00,
        'l',0x00,
        'o',0x00,
        'g',0x00,
        'y',0x00,
        ' ',0x00,
        'I',0x00,
        'm',0x00,
        'a',0x00,
        'g',0x00,
        'i',0x00,
        'n',0x00,
        'g',0x00,
        ' ',0x00,
        'S',0x00,
        'o',0x00,
        'l',0x00,
        'u',0x00,
        't',0x00,
        'i',0x00,
        'o',0x00,
        'n',0x00,
        's',0x00
    };

/*
const uint8_t CyFxUSBManufactureDscr[] =
    {
        0x10,
        CY_U3P_USB_STRING_DESCR,
        'C',0x00,
        'y',0x00,
        'p',0x00,
        'r',0x00,
        'e',0x00,
        's',0x00,
        's',0x00
    };
*/

/* Standard Product String Descriptor */
const uint8_t CyFxUSBProductDscr[] =
    {
        0x30,                           /* Descriptor Size */
        CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
        'V',0x00,
        'i',0x00,
        'd',0x00,
        'e',0x00,
        'o',0x00,
        'l',0x00,
        'o',0x00,
        'g',0x00,
        'y',0x00,
        ' ',0x00,
        'U',0x00,
        'S',0x00,
        'B',0x00,
        ' ',0x00,
        '3',0x00,
        '.',0x00,
        '0',0x00,
        ' ',0x00,
        'H',0x00,
        'D',0x00,
        'C',0x00,
        'a',0x00,
        'm',0x00
    };

/*
const uint8_t CyFxUSBProductDscr[] =
    {
        0x08,
        CY_U3P_USB_STRING_DESCR,
        'F',0x00,
        'X',0x00,
        '3',0x00
    };
*/
