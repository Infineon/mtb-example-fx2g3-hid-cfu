/***************************************************************************//**
* \file cfu.h
* \version 1.0
*
* \brief C header file with CFU handlers and related macros.
*
*******************************************************************************
* \copyright
* (c) (2025), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* Changes: Declared custom functions, defined custom structs and macros
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
*
*******************************************************************************
* \copyright
* This file is part of Component Firmware Update (CFU), licensed under
* the MIT License (MIT).
*
* Copyright (c) Microsoft Corporation. All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*******************************************************************************/

#ifndef _CFU_H_
#define _CFU_H_

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/* Compiler includes */
#include <string.h>
#include <stdlib.h>

/* Library Stack includes */
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usbss_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_usbd_version.h"
#include "cy_flash.h"
#include "cy_fault_handlers.h"
#include "cy_debug.h"


/* Application includes */
#include "usb_app.h"
#include "../app_version.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* Macros */
#ifdef FX_CFU

#define INVALID_IMG                             (0xF0FFFF0F)
#define INVALID_IMG_Msk                         (0xFFFFFFFF)    /* validity & INVALID_IMG_Msk == INVALID_IMG implies image invalid*/

#define VALID_IMG                               (0xF0F00F0F)
#define VALID_IMG_Msk                           (0xFFF0FFFF)    /* validity & VALID_IMG_Msk == VALID_IMG implies image valid */

#define V_STRING_PRESENT                        (0xF0FF0F0F)
#define V_STRING_PRESENT_Msk                    (0xFFFFFFFF)    /* validity & V_STRING == VALID_IMG implies image has v string */

#define SPI_META_SIZE                           (0x18)          /* 24 */

#ifndef HID_IF_NUM
#define HID_IF_NUM                              (0x01)
#endif /* HID_IF_NUM */

#ifndef BULK_IN_ENDPOINT
#define BULK_IN_ENDPOINT                            (0x01)
#endif
#ifndef BULK_OUT_ENDPOINT
#define BULK_OUT_ENDPOINT                           (0x01)
#endif

#ifndef BULK_IN_ENDPOINT_2
#define BULK_IN_ENDPOINT_2                          (0x02)
#endif

#ifndef BULK_OUT_ENDPOINT_2
#define BULK_OUT_ENDPOINT_2                         (0x02)
#endif

#endif /* FX_CFU */

#define GET_HID_REPORT       (0x01)
#define SET_HID_REPORT       (0x09)
#define HID_SET_IDLE         (0x0A)

typedef enum cy_en_cfu_hid_report_id_t
{
    CFG_MODE_FEATURE_ID     =   1,
    FLASH_READ_SET_ID       =   2,
    FLASH_READ_GET_ID       =   3,
    FLASH_WRITE_ID          =   4,
    FLASH_ERASE_ID          =   5,
    FLASH_ERASE_POLL_ID     =   6,
    RESET_ID                =   18,

    CFU_VERSION                 = 0x2A,
    CFU_OFFER_COMMAND           = 0x2D,
    CFU_SET_REPORT_ID_OFFER_CONTENT = 0x2A,

}cy_en_cfu_hid_report_id_t;

/* Note: These defines should match CFU Protocol Spec definitions */
#define CFU_OFFER_METADATA_INFO_CMD                        (0xFF)
#define CFU_SPECIAL_OFFER_CMD                              (0xFE)
#define CFU_SPECIAL_OFFER_GET_STATUS                       (0x03)
#define CFU_SPECIAL_OFFER_NONCE                            (0x02)
#define CFU_SPECIAL_OFFER_NOTIFY_ON_READY                  (0x01)
#define CFW_UPDATE_PACKET_MAX_LENGTH                       (sizeof(cy_cfu_content_command_t))
#define FIRMWARE_OFFER_REJECT_BANK                         (0x04)
#define FIRMWARE_OFFER_REJECT_INV_MCU                      (0x01)
#define FIRMWARE_OFFER_REJECT_MISMATCH                     (0x03)
#define FIRMWARE_OFFER_REJECT_OLD_FW                       (0x00)
#define FIRMWARE_OFFER_TOKEN_DRIVER                        (0xA0)
#define FIRMWARE_OFFER_TOKEN_SPEEDFLASHER                  (0xB0)
#define FIRMWARE_UPDATE_CMD_NOT_SUPPORTED                  (0xFF)
#define FIRMWARE_UPDATE_FLAG_FIRST_BLOCK                   (0x80)
#define FIRMWARE_UPDATE_FLAG_LAST_BLOCK                    (0x40)
#define FIRMWARE_UPDATE_FLAG_VERIFY                        (0x08)
#define FIRMWARE_UPDATE_OFFER_ACCEPT                       (0x01)
#define FIRMWARE_UPDATE_OFFER_BUSY                         (0x03)
#define FIRMWARE_UPDATE_OFFER_COMMAND_READY                (0x04)
#define FIRMWARE_UPDATE_OFFER_REJECT                       (0x02)
#define FIRMWARE_UPDATE_OFFER_SKIP                         (0x00)
#define FIRMWARE_UPDATE_OFFER_SWAP_PENDING                 (0x02)
#define FIRMWARE_UPDATE_STATUS_ERROR_COMPLETE              (0x03)
#define FIRMWARE_UPDATE_STATUS_ERROR_CRC                   (0x05)
#define FIRMWARE_UPDATE_STATUS_ERROR_INVALID               (0x0B)
#define FIRMWARE_UPDATE_STATUS_ERROR_INVALID_ADDR          (0x09)
#define FIRMWARE_UPDATE_STATUS_ERROR_NO_OFFER              (0x0A)
#define FIRMWARE_UPDATE_STATUS_ERROR_PENDING               (0x08)
#define FIRMWARE_UPDATE_STATUS_ERROR_PREPARE               (0x01)
#define FIRMWARE_UPDATE_STATUS_ERROR_SIGNATURE             (0x06)
#define FIRMWARE_UPDATE_STATUS_ERROR_VERIFY                (0x04)
#define FIRMWARE_UPDATE_STATUS_ERROR_VERSION               (0x07)
#define FIRMWARE_UPDATE_STATUS_ERROR_WRITE                 (0x02)
#define FIRMWARE_UPDATE_STATUS_SUCCESS                     (0x00)
#define OFFER_INFO_END_OFFER_LIST                          (0x02)
#define OFFER_INFO_START_ENTIRE_TRANSACTION                (0x00)
#define OFFER_INFO_START_OFFER_LIST                        (0x01)

#define VERSION_FEATURE_USAGE_REPORT_ID                     0x2A
#define CONTENT_OUTPUT_USAGE_REPORT_ID                      0x2A
#define CONTENT_RESPONSE_INPUT_USAGE_REPORT_ID              0x2C
#define OFFER_OUTPUT_USAGE_REPORT_ID                        0x2D
#define OFFER_RESPONSE_INPUT_USAGE_REPORT_ID                0x2A

#define CFU_DEVICE_USAGE_PAGE                               0x00, 0xFA
#define CFU_DEVICE_USAGE                                    0xF5

#define REPORT_ID_VERSIONS_FEATURE                          0x20
#define REPORT_ID_PAYLOAD_OUTPUT                            0x20
#define REPORT_ID_DUMMY_INPUT                               0x20
#define REPORT_ID_PAYLOAD_INPUT                             0x22
#define REPORT_ID_OFFER_OUTPUT                              0x25
#define REPORT_ID_OFFER_INPUT                               0x25

#define OFFER_INPUT_USAGE_MIN                               0x1A
#define OFFER_INPUT_USAGE_MAX                               0x1D
#define OFFER_OUTPUT_USAGE_MIN                              0x1E
#define OFFER_OUTPUT_USAGE_MAX                              0x21
#define PAYLOAD_INPUT_USAGE_MIN                             0x26
#define PAYLOAD_INPUT_USAGE_MAX                             0x29
#define PAYLOAD_OUTPUT_USAGE                                0x31
#define VERSIONS_FEATURE_USAGE                              0x42
#define DUMMY_INPUT_USAGE                                   0x52

#define COMPONENT_ID_MCU                                    0x30
#define COMPONENT_ID_AUDIO                                  0x2

#define FIRMWARE_VERSION_MAJOR                              123
#define FIRMWARE_VERSION_MINOR                              4
#define FIRMWARE_VERSION_VARIANT                            5

#define REPORT_ID_LENGTH                                    0x01

#define FEATURE_REPORT_LENGTH                               0x3C
#define OUTPUT_REPORT_LENGTH                                0x3C
#define INPUT_REPORT_LENGTH                                 0x20

#define FX_HID_REPORT_DESCR_SIZE                           (129) /* Size of cyFxHidReportDesc */

/**
 * \def CY_FX_DWORD_GET_BYTE0
 * \brief Retrieves byte 0 from a 32 bit number.
 */
#define CY_FX_DWORD_GET_BYTE0(d)                           ((uint8_t)((d) & 0xFF))

/**
 * \def CY_FX_DWORD_GET_BYTE1
 * \brief Retrieves byte 1 from a 32 bit number.
 */
#define CY_FX_DWORD_GET_BYTE1(d)                           ((uint8_t)(((d) >>  8) & 0xFF))

/**
 * \def CY_FX_DWORD_GET_BYTE2
 * \brief Retrieves byte 2 from a 32 bit number.
 */
#define CY_FX_DWORD_GET_BYTE2(d)                           ((uint8_t)(((d) >> 16) & 0xFF))

/**
 * \def CY_FX_DWORD_GET_BYTE3
 * \brief Retrieves byte 3 from a 32 bit number.
 */
#define CY_FX_DWORD_GET_BYTE3(d)                           ((uint8_t)(((d) >> 24) & 0xFF))

/* Macros */
#define CFU_METADAT_LOC                                     (0x1007FE00U)
#define CY_FX_GET_LSB(w)                                   ((uint8_t)((w) & 255))
#define CY_FX_GET_MSB(w)                                   ((uint8_t)((w) >> 8))
#define SPI_FLASH_BLOCK_SIZE                                (512)
#define CY_USB_HANDLE_HID_REQ                               (0xC1)
#define CFU_VERSION_RESPONSE_SIZE                           (61)
#define CFU_OFFER_RESPONSE_BUFFER_SIZE                      (256)
#define CFU_OFFER_RESPONSE_SIZE                             (16)

#define CY_FX_USB_SC_GET_DESCRIPTOR                        0x06
#define GET_HID_DESCR                                       (0x21)
#define GET_REPORT_DESC                                     (0x22)

#define SIZE_REPORT_ID                                      (1)
#define NUM_SUBCOMPONENT                                    (0)
#define MAJOR_VERSION                                       (0)
#define MINOR_VERSION_H                                     (0)
#define MINOR_VERSION_L                                     (0)
#define PATCH_VERSION                                       (2)
#define CY_FX_MAKEDWORD(b3, b2, b1, b0)                        ((uint32_t)((((uint32_t)(b3)) << 24) | (((uint32_t)(b2)) << 16) | \
                                                            (((uint32_t)(b1)) << 8) | ((uint32_t)(b0))))

#define FX_IMG_FILE_MAX_SIZE                                (512*1024)
#define FX_IMG_FILE_MAX_NUM_BLOCKS                          (FX_IMG_FILE_MAX_SIZE/SPI_FLASH_BLOCK_SIZE)
#define REPORT_ID_INDEX                                     0

#define CPFWU_REVISION                                      (2u)

/**
 * \def CFU_TIMER_ID
 * \brief Unique CFU update timeout timer ID based on the Desktop Mini Stack
 */
#define CFU_TIMER_ID                                        72u

/**
 * \def MAX_FW_UPDATE_TIME_FAIL_SAFE_MS
 * \brief CFU reset period in ms
 * \details This should be set to a value greater than
 *          500 ms to avoid significant increase in power consumption.
 */
#define MAX_FW_UPDATE_TIME_FAIL_SAFE_MS                    (10000u)

#define CFU_LOG_MESSAGE(msg, ...) \
{ \
    Cy_Debug_AddToLog(3, "[CFU]: "msg"\r\n", ##__VA_ARGS__); \
}

#define CFU_LOG_TRACE_MSG(msg, ...) \
{ \
    Cy_Debug_AddToLog(4, "[CFU][T]: "msg"\r\n", ##__VA_ARGS__); \
}

#define CFU_LOG_ERROR(msg, ...) \
{ \
    Cy_Debug_AddToLog(3, "[CFU][ERROR]: "msg"\r\n", ##__VA_ARGS__); \
}

/* Typedefs */
typedef struct
{
    uint8_t   activeComponentId;
    bool    forceReset;
    bool    updateInProgress;
} cy_cfu_current_offer_info_t;

typedef struct cy_stc_cfu_component_registration
{
    struct cy_stc_cfu_component_registration* pNext;
    const uint8_t componentId;
} cy_cfu_component_registration_t;

#pragma pack(1)
typedef struct
{
    struct
    {
        uint8_t componentCount;
        uint16_t reserved0;
        uint8_t fwUpdateRevision : 4;
        uint8_t reserved1 : 3;
        uint8_t extensionFlag : 1;
    } header;
    uint8_t versionAndProductInfoBlob[20];
} cy_cfu_get_version_response_t;

typedef struct
{
    struct
    {
        uint8_t segmentNumber;
        uint8_t reserved0 : 6;
        uint8_t forceImmediateReset : 1;
        uint8_t forceIgnoreVersion : 1;
        uint8_t componentId;
        uint8_t token;
    } componentInfo;

    uint32_t version;
    uint32_t hwVariantMask;
    struct
    {
        uint8_t protocolRevision : 4;
        uint8_t bank : 2;
        uint8_t reserved0 : 2;
        uint8_t milestone : 3;
        uint8_t reserved1 : 5;
        uint16_t productId;
    } productInfo;

} cy_cfu_offer_command_t;

typedef struct
{
    struct
    {
        uint8_t infoCode;
        uint8_t reserved0;
        uint8_t shouldBe0xFF;
        uint8_t token;
    } componentInfo;

    uint32_t reserved0[3];

} cy_cfu_offer_info_only_command_t;

typedef struct
{
    struct
    {
        uint8_t commandCode;
        uint8_t reserved0;
        uint8_t shouldBe0xFE;
        uint8_t token;
    } componentInfo;

    uint32_t reserved0[3];

} cy_cfu_special_offer_command_t;

typedef struct
{
    union
    {
        struct
        {
            uint8_t reserved0[3];
            uint8_t token;
            uint32_t reserved1;
            uint8_t rejectReasonCode;
            uint8_t reserved2[3];
            uint8_t status;
            uint8_t reserved3[3];
        };
    };
} cy_cfu_offer_response_t;

typedef struct
{
    uint8_t flags;
    uint8_t length;
    uint16_t sequenceNumber;
    uint32_t address;
    uint8_t pData[52];
} cy_cfu_content_command_t;

typedef struct
{
    union
    {
        struct
        {
            uint16_t sequenceNumber;
            uint16_t reserved0;
            uint8_t status;
            uint8_t reserved1[3];
            uint32_t reserved2[2];
        };
    };
} cy_cfu_content_response_t;

typedef enum
{
    COMPOSITE_DEVICE      = (0x01U),
    /* Add more platform component ID's here as and when required */
} cy_en_cfu_platform_id_t;

#pragma pack()

typedef enum
{
    CY_CFU_UPDATE_FAILED = 0,
    CY_CFU_IDLE,
    CY_CFU_UPDATE_IN_PROGRESS,
    CY_CFU_UPDATE_DONE,
    CY_CFU_UPDATE_REJECTED,
}cy_cfu_status_codes_t;

typedef struct cy_stc_cfu_status
{
    cy_cfu_status_codes_t updateStatus;
    uint8_t deviceCount;
    uint32_t version;
    uint32_t upcomingContentSize;
}cy_stc_cfu_status_t;

/* Global variable externs */
extern bool cfu_last_block;
extern bool cfu_first_block;
extern uint8_t cfuBuffer[CFU_OFFER_RESPONSE_BUFFER_SIZE];
extern volatile uint8_t app;
extern volatile uint8_t appmode[7];
extern const uint8_t app2Mode[8] ;
extern const uint8_t app1Mode[8] ;
extern const uint8_t cyFxHidReportDesc[];
extern const uint8_t cyFxHidInterfaceDescr[];

/* Functions */
/**
 * \name Cy_USB_HandleHidRequests
 * \brief Handle HID-CFU requests
 * \note HID requests that are not relavant to CFU are ignored
 * \param pUsbdCtxt
 * \param pUsbApp
 * \param bRequest
 * \param wValue
 * \param wIndex
 * \param wLength
 * \retval isHandled Indicate wether the HID-CFU request has been handled
 */
bool Cy_USB_HandleHidRequests(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength);

/**
 * \name Cy_CFU_ProcessCFWUContent
 * \brief Process the content component firmware update command.
 * \details This function is non reentrant - only to be called
 *          from single thread. If this is not the case
 *          for your implementation - extra care must be
 *          made for thread safety.
 * \param pCommand The command to be processed.
 * \param pResponse The response to be populated.
 * \retval None
 */
void Cy_CFU_ProcessCFWUContent(cy_cfu_content_command_t* pCommand, cy_cfu_content_response_t* pResponse);

/**
 * \name Cy_CFU_ProcessCFWUOffer
 * \brief Process the offer component firmware update command.
 * \details This function is non reentrant - only to be called
 *          from single thread. If this is not the case
 *          for your implementation - extra care must be
 *          made for thread safety.
 * \param pCommand The command to process.
 * \param pResponse The response to populate.
 * \retval None
 */
void Cy_CFU_ProcessCFWUOffer(cy_cfu_offer_command_t* pCommand, cy_cfu_offer_response_t* pResponse);

/**
 * \name Cy_CFU_ProcessCFWUGetFWVersion
 * \brief Process the get firmware version CFU command.
 * \param pResponse The response to populate and return to host
 * \retval None
 */
void Cy_CFU_ProcessCFWUGetFWVersion(cy_cfu_get_version_response_t* pResponse);

/**
 * \name Cy_CFU_HandleCFUVersionRequest
 * \brief Send CFU version and product info to host
 * \param pUsbdCtxt
 * \param pUsbApp
 * \param reportID
 * \retval status Indicate if info is successfully sent to host
 */
uint8_t Cy_CFU_HandleCFUVersionRequest(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t reportID);

/**
 * \name Cy_CFU_HandleCFUOfferOpUsageReq
 * \brief Handler to interface with the MSFT-based CFU offer processing API
 * \param pUsbdCtxt
 * \param pUsbApp
 * \param buf
 * \retval status Indicates successful response transfer after CFU offer processing
 */
uint8_t Cy_CFU_HandleCFUOfferOpUsageReq(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *buf);

/**
 * \name Cy_CFU_HandleCFUContentOpUsage
 * \brief Handler to interface with the MSFT-based CFU content processing API
 * \param pUsbdCtxt
 * \param pUsbApp
 * \param buf
 * \retval status Indicates successful response transfer after CFU content processing
 */
uint8_t Cy_CFU_HandleCFUContentOpUsage(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *buf);

/**
 * \name Cy_USB_HandleCFUOfferNContentRequest
 * \brief Process the report ID and accordingly call the appropriate CFU request handlers
 * \note User must ensure LPM transition is avoided during CFU
 * \param pUsbdCtxt
 * \param pUsbApp
 * \param reportID
 * \param wLength
 * \retval status
 */
uint8_t Cy_USB_HandleCFUOfferNContentRequest(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t reportID, uint16_t wLength);

/**
 * \name Cy_CFU_SPI_Start
 * \brief starts SPI with SPI_FLASH_0, to be used for CFU on the FPGA add-on board
 * \param pAppCtxt
 * \retval status Indicate SPI start success or failure
 */
uint32_t Cy_CFU_SPI_Start(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_CFU_ExtSPIFlash_Prepare
 * \brief Initialize and erase the external flash, in preparation for CFU writing
 * \param componentID Unutilised in this application, can be used if selection required amongst several target components for CFU is required
 * \retval status
 */
uint32_t Cy_CFU_ExtSPIFlash_Prepare(uint8_t componentId);

/**
 * \name Cy_CFU_ExtSPIFlash_Write
 * \param offset address to write to
 * \param pData pointer to data array that needs to be written
 * \param length len of data to be considered from location pData
 * \param componentID Can be used if selection required amongst multiple target components for CFU (Unutilised in this application)
 * \retval status
 */
uint32_t Cy_CFU_ExtSPIFlash_Write(uint32_t offset, uint8_t* pData, uint8_t length, uint8_t componentId);

/**
 * \name Cy_CFU_ExtSPIFlash_Read
 * \brief Function to read a data chunk from flash
 * \param offset address on the SPI flash to be read from
 * \param pData pointer to a buffer into which read data is stored
 * \param length number of bytes of data
 * \param flashIndex cy_en_flash_index_t flash index identifier typecasted to uint8_t
 * \retval Read operation exit status
 */
uint32_t Cy_CFU_ExtSPIFlash_Read(uint32_t offset, uint8_t* pData, uint16_t length, uint8_t flashIndex);

/**
 * \name calculateOneByteChecksum
 * \note Perform addition of 8b of data repeatedly to a 32b store, to be used for checksum
 * \param data Pointer to start of data to be considered for checksum addition
 * \param len Number of bytes of data to be added to the checksum
 * \retval Checksum 32b resultant value of repeated 8b additions
 */
uint32_t calculateOneByteChecksum(uint8_t *data, uint32_t len);

typedef struct spi_meta_newdata{
    char start_sig[4];
    uint32_t fw_offset;
    uint32_t fw_checksum;
    uint32_t validity;
    uint32_t reserved;
    char end_sig[4];
} spi_meta_t;
void Cy_CFU_SPI_ConvertMetadataBytes(spi_meta_t* restrict metadata, uint8_t* buffer, bool mode);

#if defined(__cplusplus)
}
#endif

#endif /* _CFU_H_ */

/* [] END OF FILE */
