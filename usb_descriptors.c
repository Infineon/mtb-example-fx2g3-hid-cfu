/***************************************************************************//**
* \file usb_descriptors.c
* \version 1.0
*
* \brief Defines the USB descriptors used in the HID-CFU Application.
*
*******************************************************************************
* \copyright
* (c) (2025), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "cy_pdl.h"
#include "cy_usb_common.h"
#include "cy_usb_usbd.h"
#include "usb_echo_device.h"

#ifdef FX_CFU
#include "cfu.h"
#endif /* FX_CFU */

#define USB_DESC_ATTRIBUTES __attribute__ ((section(".descSection"), used))

/* Binary device object store descriptor */
USB_DESC_ATTRIBUTES uint8_t CyFxUSBBOSDscr[64] __attribute__ ((aligned (32))) =
{
    0x05,                           /* Descriptor size */
    0x0F,                           /* Device descriptor type */
                                    /* Number of device capability descriptors */
    0x16,0x00,                      /* Length of this descriptor and all sub-descriptors */
    0x02,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x02,                           /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */

    /* SuperSpeed device capability */
    0x0A,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x03,                           /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features: No LTM support.  */
    0x0E,0x00,                      /* Speeds supported by the device: SS, HS and FS */
    0x03,                           /* Functionality support */
    0x0A,                           /* U1 Device Exit latency */
    0xFF,0x07,                      /* U2 Device Exit latency */
};


/* Device qualifier descriptor. */
const uint8_t CyFxDevQualDscr[] __attribute__ ((aligned (4))) =
{
    0x0A,                           /* Descriptor size */
    0x06,                           /* Device qualifier descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0x00,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0x01,                           /* Number of configurations */
    0x00                            /* Reserved */
};


/* Binary Object Store (BOS) Descriptor. */
const uint8_t CyFxBOSDscr[] __attribute__ ((aligned (4))) =
{
    0x05,                           /* Descriptor size */
    CY_USB_BOS_DSCR,                /* BOS descriptor type */
    0x0C,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x01,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    CY_DEVICE_CAPB_DSCR,            /* Device capability type descriptor */
    0x02,                           /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00             /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */
};

USB_DESC_ATTRIBUTES uint8_t CyFxLangString[32] __attribute__ ((aligned (32))) =
{
    0x04,
    0x03,
    0x09,
    0x04
};

/* Standard Manufacturer String descriptor */
USB_DESC_ATTRIBUTES uint8_t CyFxMfgString[32] __attribute__ ((aligned (32))) =
{
    0x12,
    0x03,
    'I',
    0x00,
    'N',
    0x00,
    'F',
    0x00,
    'I',
    0x00,
    'N',
    0x00,
    'E',
    0x00,
    'O',
    0x00,
    'N',
    0x00
};

/* Standard Product String desciptor */
USB_DESC_ATTRIBUTES uint8_t CyFxProdString[32] __attribute__ ((aligned (32))) =
{
    0x1A,
    0x03,
    'E',
    0x00,
    'Z',
    0x00,
    '-',
    0x00,
    'U',
    0x00,
    'S',
    0x00,
    'B',
    0x00,
    ' ',
    0x00,
    'F',
    0x00,
    'X',
    0x00,
    '2',
    0x00,
    'G',
    0x00,
    '3',
    0x00
};

/* Standard Interafce String desciptor */
USB_DESC_ATTRIBUTES uint8_t CyFxIntfString[] __attribute__ ((aligned (32))) =
{
    0x1A,
    0x03,
    'E',
    0x00,
    'Z',
    0x00,
    '-',
    0x00,
    'U',
    0x00,
    'S',
    0x00,
    'B',
    0x00,
    ' ',
    0x00,
    'F',
    0x00,
    'X',
    0x00,
    '2',
    0x00,
    'G',
    0x00,
    '3',
    0x00
};

/* USB 2.0 descriptors */
/* Standard device descriptor for USB 2.0 */
const uint8_t CyFxUSB20DeviceDscr[] __attribute__ ((aligned (4))) =
{
    0x12,                           /* Descriptor size */
    0x01,                           /* Device descriptor type */
    0x00,0x02,                      /* USB 2.00 */
    0xEF,                           /* Device class */
    0x02,                           /* Device sub-class */
    0x01,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0xB4,0x04,                      /* Vendor ID */
#if (!USE_WINUSB)
    0x05,0x49,                      /* Product ID */
#else
    0xF1,0x00,                      /*Using a different Product ID when Winusb is used on the Host PC*/
#endif /* USE_WINUSB */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Standard device qualifier descriptor */
const uint8_t CyFxUSBDeviceQualDscr[] __attribute__ ((aligned (4))) =
{
    0x0A,                           /* Descriptor size */
    0x06,       /* Device qualifier descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0x00,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0x01,                           /* Number of configurations */
    0x00                            /* Reserved */
};

/* Standard full speed configuration descriptor */
const uint8_t CyFxUSBFSConfigDscr[] __attribute__ ((aligned (4))) =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
#ifdef FX_CFU
    0x39,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of interfaces */
#else
    0x20,0x00,
    0x01,
#endif /* FX_CFU */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
    0x80,                           /* Config characteristics - bus powered */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

    /* Vendor Interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x02,                           /* Number of endpoints for this interface */
    0xFF,                           /* Interface class */
    0x00,                           /* Interface sub class */
    0x00,                           /* Interface protocol code */
    0x03,                           /* Interface descriptor string index */

    /* Endpoint descriptor for consumer EP */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | BULK_IN_ENDPOINT,        /* Endpoint address and description */
    0x02,                           /* Bulk endpoint type */
    0x00,0x02,                      /* Max packet size = 1024 bytes */
    0x00,                           /* Servicing interval for data transfers : 0 for Bulk */

    /* Endpoint descriptor for producer EP */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    BULK_OUT_ENDPOINT,              /* Endpoint address and description */
    0x02,                           /* Bulk endpoint type */
    0x00,0x02,                      /* Max packet size = 1024 bytes */
    0x00,                           /* Servicing interval for data transfers : 0 for bulk */

#ifdef FX_CFU
    /* HID Interface Descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x01,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of end points */
    0x03,                           /* Interface class */
    0x00,                           /* Interface sub class : None */
    0x02,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* HID Descriptor */
    0x09,                           /* Descriptor size */
    0x21,                           /* Descriptor Type -> HID Descriptor */
    0x11,0x01,                      /* HID Class Spec 1.11 */
    0x00,                           /* Target Country */
    0x01,                           /* Total HID Class Descriptors */
    0x22,                           /* Report Descriptor Type */
    CY_FX_GET_LSB(FX_HID_REPORT_DESCR_SIZE),
    CY_FX_GET_MSB(FX_HID_REPORT_DESCR_SIZE),             /* Total Length of Report Descriptor */

    /* Endpoint Descriptor  */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint Descriptor Type */
    0x80 | BULK_IN_ENDPOINT_2,      /* Endpoint address and description */
    0x03,                           /* Interrupt End point Type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x0A,
#endif /* FX_CFU */
};


/* MS OS String Descriptor */
USB_DESC_ATTRIBUTES uint8_t glOsString[] __attribute__ ((aligned (32))) =
{
    0x12, /* Length. */
    0x03, /* Type - string. */
    'M', 0x00, 'S', 0x00, 'F', 0x00, 'T', 0x00, '1', 0x00, '0', 0x00, '0', 0x00, /* Signature. */
    MS_VENDOR_CODE, /* MS vendor code. */
    0x00 /* Padding. */
};

USB_DESC_ATTRIBUTES uint8_t glOsCompatibilityId[] __attribute__ ((aligned (32))) =
{
    /* Header */
    0x28, 0x00, 0x00, 0x00, /* length Need to be updated based on number of interfaces. */
    0x00, 0x01, /* BCD version */
    0x04, 0x00, /* Index: 4 - compatibility ID */
    0x01, /* count. Need to be updated based on number of interfaces. */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* reserved. */
    /* First Interface */
    0x00, /* Interface number */
    0x01, /* reserved: Need to be 1. */
    0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00, /* comp ID –ID to bind the device with
                                                       WinUSB.*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* sub-compatibility ID - NONE. */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* reserved - needs to be zero. */
};

USB_DESC_ATTRIBUTES uint8_t glOsFeature[] __attribute__ ((aligned (32))) =
{
    /* Header */
    0x8E, 0x00, 0x00, 0x00, /* Length. */
    0x00, 0x01, /* BCD version. 1.0 as per MS */
    0x05, 0x00, /* Index */
    0x01, 0x00, /* count. */
    /* Property section. */
    0x84, 0x00, 0x00, 0x00, /* length */
    0x01, 0x00, 0x00, 0x00, /* dwPropertyDataType: REG_DWORD_LITTLE_ENDIAN */
    0x28, 0x00, /* wPropertyNameLength: 0x30 */

    0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 0x49, 0x00, 0x6E, 0x00,
    0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00,
    0x55, 0x00, 0x49, 0x00, 0x44, 0x00, 0x00, 0x00, /* bPropertyName: DeviceInterfaceGUID */
    0x4E, 0x00, 0x00, 0x00, /* dwPropertyDataLength: 4E */

    '{', 0x00, '0', 0x00, '1', 0x00, '2', 0x00, '3', 0x00, '4', 0x00, '5', 0x00, '6', 0x00,
    '7', 0x00, '-', 0x00, '2', 0x00, 'A', 0x00, '4', 0x00, 'F', 0x00, '-', 0x00, '4', 0x00,
    '9', 0x00, 'E', 0x00, 'E', 0x00, '-', 0x00, '8', 0x00, 'D', 0x00, 'D', 0x00, '3', 0x00,
    '-', 0x00, 'F', 0x00, 'A', 0x00, 'D', 0x00, 'E', 0x00, 'A', 0x00, '3', 0x00, '7', 0x00,
    '7', 0x00, '2', 0x00, '3', 0x00, '4', 0x00, 'A', 0x00, '}', 0x00, 0x00, 0x00
        /* bPropertyData: {01234567-2A4F-49EE-8DD3-FADEA377234A} */
};


/* Standard high speed configuration descriptor */
const uint8_t CyFxUSBHSConfigDscr[] __attribute__ ((aligned (4))) =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
#ifdef FX_CFU
    0x39,0x00,                  /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of interfaces */
#else
    0x20,0x00,
    0x01,
#endif /* FX_CFU */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
    0x80,                           /* Config characteristics - bus powered */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

    /* Vendor Interface descriptor */
    0x09,                            /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x02,                           /* Number of endpoints for this interface */
    0xFF,                           /* Interface class */
    0x00,                           /* Interface sub class */
    0x00,                           /* Interface protocol code */
    0x03,                           /* Interface descriptor string index */

    /* Endpoint descriptor for consumer EP */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | BULK_IN_ENDPOINT,        /* Endpoint address and description */
    0x02,                           /* Bulk endpoint type */
    0x00,0x02,                      /* Max packet size = 1024 bytes */
    0x00,                           /* Servicing interval for data transfers : 0 for Bulk */

    /* Endpoint descriptor for producer EP */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    BULK_OUT_ENDPOINT,              /* Endpoint address and description */
    0x02,                           /* Bulk endpoint type */
    0x00,0x02,                      /* Max packet size = 1024 bytes */
    0x00,                           /* Servicing interval for data transfers : 0 for bulk */

#ifdef FX_CFU
    /* HID Interface Descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x01,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of end points */
    0x03,                           /* Interface class */
    0x00,                           /* Interface sub class : None */
    0x00,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* HID Descriptor */
    0x09,                           /* Descriptor size */
    0x21,                           /* Descriptor Type -> HID Descriptor */
    0x10,0x11,                      /* HID Class Spec 11.1 */
    0x00,                           /* Target Country */
    0x01,                           /* Total HID Class Descriptors */
    0x22,                           /* Report Descriptor Type */
    CY_FX_GET_LSB(FX_HID_REPORT_DESCR_SIZE),
    CY_FX_GET_MSB(FX_HID_REPORT_DESCR_SIZE),             /* Total Length of Report Descriptor */

    /* Endpoint Descriptor  */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint Descriptor Type */
    0x80 | BULK_IN_ENDPOINT_2,      /* Endpoint address and description */
    0x03,                           /* Interrupt End point Type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x0A,
#endif /* FX_CFU */

};

#ifdef FX_CFU
const uint8_t cyFxHidReportDesc[]  __attribute__ ((aligned (4)))   = {
        /* USAGE_PAGE */                                            0x06U, 0x00U, 0xFFU,
        /* USAGE */                                                        0x09U, 0x01U,
        /* COLLECTION */                                                   0xA1U, 0x01U,
        /* REPORT_ID */                                                    0x85U, 0x01U,
        /* USAGE */                                                        0x09U, 0x01U,
        /* LOGICAL_MINIMUM */                                              0x15U, 0x00U,
        /* LOGICAL_MAXIMUM */                                       0x26U, 0xFFU, 0x00U,
        /* REPORT_COUNT */                                                 0x95U, 0x3FU,
        /* REPORT_SIZE */                                                  0x75U, 0x08U,
        /* INPUT */                                                        0x81U, 0x02U,
        /* REPORT_ID */                                                    0x85U, 0x01U,
        /* USAGE */                                                        0x09U, 0x01U,
        /* OUTPUT */                                                       0x91U, 0x02U,
        /* END_COLLECTION */                                                      0xC0U,
        /* USAGE_PAGE */                                                   0x05U, 0x01U,
        /* USAGE */                                                        0x09U, 0x80U,
        /* COLLECTION */                                                   0xA1U, 0x01U,
        /* REPORT_ID */                                                    0x85U, 0x02U,
        /* LOGICAL_MAXIMUM */                                              0x25U, 0x01U,
        /* LOGICAL_MINIMUM */                                              0x15U, 0x00U,
        /* USAGE */                                                        0x09U, 0x82U,
        /* USAGE */                                                        0x09U, 0x83U,
        /* REPORT_SIZE */                                                  0x75U, 0x01U,
        /* REPORT_COUNT */                                                 0x95U, 0x02U,
        /* INPUT */                                                        0x81U, 0x06U,
        /* REPORT_SIZE */                                                  0x75U, 0x06U,
        /* REPORT_COUNT */                                                 0x95U, 0x01U,
        /* INPUT */                                                        0x81U, 0x03U,
        /* END_COLLECTION */                                                      0xC0U,
        /* USAGE_PAGE */                                            0x06U, 0x0BU, 0xFFU,
        /* USAGE */                                                 0x0AU, 0x04U, 0x01U,
        /* COLLECTION */                                                   0xA1U, 0x01U,
        /* LOGICAL_MINIMUM */                                              0x15U, 0x00U,
        /* LOGICAL_MAXIMUM */                                       0x26U, 0xFFU, 0x00U,
        /* REPORT_ID */                                                    0x85U, 0x2AU,
        /* REPORT_SIZE */                                                  0x75U, 0x08U,
        /* REPORT_COUNT */                                                 0x95U, 0x3CU,
        /* USAGE */                                                        0x09U, 0x60U,
        /* INPUT */                                                 0x82U, 0x02U, 0x01U,
        /* USAGE */                                                        0x09U, 0x61U,
        /* OUTPUT */                                                0x92U, 0x02U, 0x01U,
        /* USAGE */                                                        0x09U, 0x62U,
        /* FEATURE */                                               0xB2U, 0x02U, 0x01U,
        /* LOGICAL_MINIMUM */                         0x17U, 0x00U, 0x00U, 0x00U, 0x80U,
        /* LOGICAL_MAXIMUM */                         0x27U, 0xFFU, 0xFFU, 0xFFU, 0x7FU,
        /* REPORT_SIZE */                                                  0x75U, 0x20U,
        /* REPORT_COUNT */                                                 0x95U, 0x04U,
        /* REPORT_ID */                                                    0x85U, 0x2CU,
        /* USAGE_MINIMUM */                                                0x19U, 0x66U,
        /* USAGE_MAXIMUM */                                                0x29U, 0x69U,
        /* INPUT */                                                        0x81U, 0x02U,
        /* REPORT_ID */                                                    0x85U, 0x2DU,
        /* USAGE_MINIMUM */                                                0x19U, 0x8AU,
        /* USAGE_MAXIMUM */                                                0x29U, 0x8DU,
        /* INPUT */                                                        0x81U, 0x02U,
        /* USAGE_MINIMUM */                                                0x19U, 0x8EU,
        /* USAGE_MAXIMUM */                                                0x29U, 0x91U,
        /* OUTPUT */                                                       0x91U, 0x02U,
        /* END_COLLECTION */                                                      0xC0U,
};

const uint8_t cyFxHidInterfaceDescr[]  __attribute__ ((aligned (4)))  =
{
        /* HID Descriptor */
        0x09,                           /* Descriptor size */
        0x21,           /* Descriptor Type */
        0x10,0x11,                      /* HID Class Spec 11.1 */
        0x00,                           /* Target Country */
        0x01,                           /* Total HID Class Descriptors */
        0x22,                           /* Report Descriptor Type */
        CY_FX_GET_LSB(FX_HID_REPORT_DESCR_SIZE),
        CY_FX_GET_MSB(FX_HID_REPORT_DESCR_SIZE),             /* Total Length of Report Descriptor */
};
#endif /* FX_CFU */
