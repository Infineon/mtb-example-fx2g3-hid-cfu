/***************************************************************************//**
* \file cfu.c
* \version 1.0
*
* \brief C source file with CFU handlers and related resources.
*
*******************************************************************************
* \copyright
* (c) (2025), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* Changes: Implemented functions to perform CFU on the EZ-USB FX2G3 device
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

/* Application includes */
#include "cfu.h"
#include "cy_debug.h"

/* CFU External Flash dependency */
#include "cy_smif.h"
#include "spi.h"
#include <string.h>

/* SPI Flash Metadata */
spi_meta_t spi_meta_new;
uint8_t spi_meta_w_buf[24];

/* Persistent Variables */
CY_NOINIT char vIgnore_status[8];

/* Global Variables */
cy_stc_cfu_status_t cfu_status;                                             /* Hold device update status, version info, etc. */
bool cfu_last_block = false;                                                /* Indicates starting CFU data block */
bool cfu_first_block = false;                                               /* Indicates last CFU data block */
bool isDriver = false;                                                      /* Indicates if incoming CFU data is from host-side driver */
uint32_t checksum = 0;                                                      /* To calculate 1 Byte checksum of CFU data */
uint32_t recdB = 0;                                                         /* Track count of bytes received until a page worth is available */
uint32_t cfu_extspiflash_addr = 0;                                          /* Address to write to on the external SPI Flash */
uint32_t pageCount = 0;                                                     /* To keep track of how many pages are written so far */
uint32_t contentPacketNum = 0;                                              /* Keep track of how many times the write function was called */
uint32_t hwVariantMask;                                                     /* Host-provided checksum of incoming firmware image */
uint8_t cfuBuffer[CFU_OFFER_RESPONSE_BUFFER_SIZE] = {0};                    /* Buffer to populate CFU response */
uint8_t tempbuf[32] __attribute__((__aligned__(32)));                       /* Buffer to hold metadata and content, preventing data overflow */
uint8_t metadata_buffer[SPI_META_SIZE] __attribute__((__aligned__(32)));    /* Buffer to hold raw metadata bytes in an array */
uint8_t overflow = 0;                                                       /* Bytes of data overflown after filling tempbuf */
uint8_t difference = 1;                                                     /* Bytes of pending data after penultimate overflow is loaded into buffer */

/* Static Variables */
static uint8_t sfBuffer[SPI_FLASH_PAGE_SIZE];                               /* Buffer to hold data until we have page size worth of Bytes */
static cy_cfu_current_offer_info_t s_currentOffer;                          /* CFU-spec defined struct to hold CFU offer */
static bool s_bankSwapPending = false;                                      /* Indicates if bank swap is pending (unutilised in this application) */

/* Functions */
/**
 * \name Cy_CFU_ExtSPIFlash_SignalUpdateComplete
 * \brief Callback when component is done handling firmware image transfer
 * \retval None
 */
 static void Cy_CFU_ExtSPIFlash_SignalUpdateComplete(void){
    /* Update of image has been completed successfully. */
    s_currentOffer.updateInProgress = false;
}

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
bool Cy_USB_HandleHidRequests(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    bool isHandled = false;
    uint8_t reportID = wValue & 0xff;
    
    if(bRequest == GET_HID_REPORT)
    {
        CFU_LOG_TRACE_MSG("Report ID: 0d%d", reportID);
        switch (reportID)
        {
        case CFU_VERSION:
            Cy_CFU_HandleCFUVersionRequest(pUsbdCtxt,pUsbApp,CFU_VERSION);
            isHandled = true;
            break;
        default:
            break;
        }
    }
    else if (bRequest == SET_HID_REPORT)
    {
        switch (reportID)
        {
        case CFU_OFFER_COMMAND:
        case CFU_SET_REPORT_ID_OFFER_CONTENT:
            Cy_USB_HandleCFUOfferNContentRequest(pUsbdCtxt,pUsbApp,reportID,wLength);
            isHandled = true;
            break;
        default:
            break;
        }
    }
    return isHandled;
}

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
void Cy_CFU_ProcessCFWUContent(cy_cfu_content_command_t* pCommand, cy_cfu_content_response_t* pResponse){
    uint8_t status = FIRMWARE_UPDATE_STATUS_SUCCESS;                                        /* CFU content processing status */
    uint16_t sequenceNumber = isDriver?pCommand->sequenceNumber--:pCommand->sequenceNumber; /* Data packet sequence number */
    uint8_t componentId = s_currentOffer.activeComponentId;                                 /* Unimportant for this application */

    /* First block */
    if (pCommand->flags & FIRMWARE_UPDATE_FLAG_FIRST_BLOCK){

        cfu_first_block = true;             /* Set flag to true */
        if(pCommand->sequenceNumber==1){    /* If the sequence number start at 1, host interacts via driver */
            isDriver = true;
            pCommand->sequenceNumber--;     /* Perform appropriate sequence number correction */
        }

        if(!Cy_CFU_ExtSPIFlash_Prepare(componentId)){                                   /* Prepare for CFU */

            checksum += calculateOneByteChecksum(pCommand->pData, pCommand->length);    /* Update checksum */

            /* Write the incoming data onto the external SPI flash */
            if (Cy_CFU_ExtSPIFlash_Write(pCommand->address, pCommand->pData, pCommand->length, componentId) != 0){
                status = FIRMWARE_UPDATE_STATUS_ERROR_WRITE;
            }
            cfu_first_block = false;                                                    /* First block operations complete, update flag */
        }
        else{
            status = FIRMWARE_UPDATE_STATUS_ERROR_PREPARE;
        }
    }
    /* Last block */
    else if (pCommand->flags & FIRMWARE_UPDATE_FLAG_LAST_BLOCK){

        checksum += calculateOneByteChecksum(pCommand->pData, pCommand->length);        /* Update checksum */

        cfu_last_block = true;                                                  /* Update flag */

        /* Write last block data onto the external SPI flash */
        if (Cy_CFU_ExtSPIFlash_Write(pCommand->address, pCommand->pData, pCommand->length, componentId)){
            status = FIRMWARE_UPDATE_STATUS_ERROR_WRITE;
        }
        CFU_LOG_MESSAGE("Data Checksum: 0x%x", checksum);                       /* Log the 1 Byte checksum value */
        CFU_LOG_MESSAGE("Checksum Verification: %s", hwVariantMask==checksum?"SUCCESS":"FAILURE");  /* Log checksum verification status */
        Cy_CFU_ExtSPIFlash_SignalUpdateComplete();                /* End of CFU writes */
        checksum = 0;                                                           /* Reset checksum */
        cfu_last_block = false;                                                 /* Reset last block flag */
    }
    /* Any data block between first and last block */
    else{
        checksum += calculateOneByteChecksum(pCommand->pData, pCommand->length);    /* Update checksum */

        /* Write block onto external SPI flash */
        if (Cy_CFU_ExtSPIFlash_Write(pCommand->address, pCommand->pData, pCommand->length, componentId) != 0){
            status = FIRMWARE_UPDATE_STATUS_ERROR_WRITE;
        }
    }

    /* In case of any error */
    if (status != FIRMWARE_UPDATE_STATUS_SUCCESS)
    {
        CFU_LOG_ERROR("FIRMWARE_UPDATE_STATUS_NOT_SUCCESS\r\n");
        s_currentOffer.updateInProgress = false;    /* Terminate update */
    }

    /* Show current segment number and status */
    if(!status){
        CFU_LOG_TRACE_MSG("Cy_CFU_ProcessCFWUContent-status: %d\r\n", status);
        CFU_LOG_TRACE_MSG("Cy_CFU_ProcessCFWUContent-seg. no.: %d\r\n", sequenceNumber);
    } else {
        CFU_LOG_ERROR("Cy_CFU_ProcessCFWUContent-status: %d\r\n", status);
        CFU_LOG_ERROR("Cy_CFU_ProcessCFWUContent-seg. no.: %d\r\n", sequenceNumber);
    }

    /* Provide appropriate response */
    memset((uint8_t *)pResponse, 0, sizeof(cy_cfu_content_response_t));
    pResponse->sequenceNumber = sequenceNumber;
    pResponse->status = status;
}

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
void Cy_CFU_ProcessCFWUOffer(cy_cfu_offer_command_t* pCommand,
                    cy_cfu_offer_response_t* pResponse)
{

    /* A token is a user-software defined byte.  It's a signature
     * that disambiguates one user-software conducting a CFU from
     * another user-software programming conducting a CFU.  Elect
     * a unique token byte in the user-software design.  The CFU
     * implementation does not define tokens.
     */
    uint8_t token = pCommand->componentInfo.token;
    uint8_t componentId = pCommand->componentInfo.componentId;
    CFU_LOG_MESSAGE("Token: %d", token);
    CFU_LOG_MESSAGE("Component ID: %d", componentId);
    
    bool return_flag = false;

    /* CFU Protocol document
     * 5.2.1.3 Vendor Specific : These four bytes may be used to encode any custom information
     * in the offer that is specific to vendor implementation.
     *
     * We use hwVariantMask to identify the 1 Byte checksum of the binary as calculated and provided by the host
     */
    hwVariantMask = pCommand->hwVariantMask;
    CFU_LOG_MESSAGE("[Provided by Host via CFU Offer]\r\n       Upcoming Firmware Checksum (hwVariantMask): 0x%x", hwVariantMask);

    /* The last offer isn't completely processed.
     * When the update from the last offer is pending
     * completion, the state is FIRMWARE_UPDATE_OFFER_BUSY.
     * If this condition is detected, return immediately.
     */
    if (s_currentOffer.updateInProgress)
    {
        memset((uint8_t *)pResponse, 0, sizeof (cy_cfu_offer_response_t));
        CFU_LOG_MESSAGE("Busy! [Update in Progress]");
        pResponse->status = FIRMWARE_UPDATE_OFFER_BUSY;
        pResponse->rejectReasonCode = FIRMWARE_UPDATE_OFFER_BUSY;
        pResponse->token = token;
        return_flag=true;
    }

    /* Or, if intent is to retrieve the status of a special offer
     * If this condition is detected, return immediately.
     * This is basically a Offer Update Extended Command-Response
     */
    else if (componentId == CFU_SPECIAL_OFFER_CMD)
    {
        cy_cfu_special_offer_command_t* pSpecialCommand = (cy_cfu_special_offer_command_t*)pCommand;
        if (pSpecialCommand->componentInfo.commandCode == CFU_SPECIAL_OFFER_GET_STATUS)
        {
            memset((uint8_t *)pResponse, 0, sizeof (cy_cfu_offer_response_t));

            pResponse->status = FIRMWARE_UPDATE_OFFER_COMMAND_READY;
            pResponse->token = token;
            return_flag=true;
        }
        else if(pSpecialCommand->componentInfo.commandCode == CFU_SPECIAL_OFFER_NOTIFY_ON_READY)
        {
            memset((uint8_t *)pResponse, 0, sizeof (cy_cfu_offer_response_t));

            pResponse->status = FIRMWARE_UPDATE_OFFER_ACCEPT;
            pResponse->token = token;
            return_flag=true;
        }
    }
    /* Offer Update Information Command-Response */
    else if(componentId == CFU_OFFER_METADATA_INFO_CMD)
    {
        cy_cfu_offer_info_only_command_t* pSpecialCommand = (cy_cfu_offer_info_only_command_t*)pCommand;
        if (pSpecialCommand->componentInfo.infoCode == OFFER_INFO_START_ENTIRE_TRANSACTION)
        {

            memset((uint8_t *)pResponse, 0, sizeof (cy_cfu_offer_response_t));

            pResponse->status = FIRMWARE_UPDATE_OFFER_ACCEPT;
            pResponse->token = token;
            return_flag=true;
        }
        else if (pSpecialCommand->componentInfo.infoCode == OFFER_INFO_START_OFFER_LIST)
        {
            memset((uint8_t *)pResponse, 0, sizeof (cy_cfu_offer_response_t));

            pResponse->status = FIRMWARE_UPDATE_OFFER_ACCEPT;
            pResponse->token = token;
            return_flag=true;
        }
        else if (pSpecialCommand->componentInfo.infoCode == OFFER_INFO_END_OFFER_LIST)
        {
            memset((uint8_t *)pResponse, 0, sizeof (cy_cfu_offer_response_t));

            pResponse->status = FIRMWARE_UPDATE_OFFER_ACCEPT;
            pResponse->token = token;
            return_flag=true;
        }
    }

    /* Or, if the MCU is in progress of swapping Bank 0 for Bank 1 (or vice versa)
     * so the offer is rejected
     * If this condition is detected, return immediately.
     */
    else if (s_bankSwapPending)
    {
        memset((uint8_t *)pResponse, 0, sizeof (cy_cfu_offer_response_t));
        CFU_LOG_MESSAGE("Offer Rejected [Bank Swap Pending]");
        pResponse->status = FIRMWARE_UPDATE_OFFER_REJECT;
        pResponse->rejectReasonCode = FIRMWARE_UPDATE_OFFER_SWAP_PENDING;
        pResponse->token = token;
        return_flag=true;
    }

    if(return_flag) return;

    /* Else, continue processing the offer by inspecting the registration list.
     * Each offer will specify whether or not to:
     *        A) force an MCU reset after processing the offer content
     *        B) ignore the version of the MCU Component Firmware when
     *           analyzing this offers version.
     *
     *    Unless there was overriding flags via "force" or "ignore"
     *    Then the offer is accepted and this notifies the other
     *    user software to move towards the action of submitting Content.
     *
     */
    bool forceReset = pCommand->componentInfo.forceImmediateReset;
    bool forceIgnoreVersion = pCommand->componentInfo.forceIgnoreVersion;

    /* Default Offer Processing for Component with ID 0 (external SPI flash) */
    if(!componentId) pResponse->status = FIRMWARE_UPDATE_OFFER_ACCEPT;

    /* This code shows how to send offer information that includes
     * the request to ignore checking any version info.
     *
     * If we're ignoring FWVersion check, and that was the reason
     * the offer was rejected, reverse the decision
     * NOTE:  in released/final Firmware, this ability to accept
     *        any version is usually disabled in FW (this is a TODO
     *        for implementers of CFU)
     */
    CFU_LOG_TRACE_MSG("[FW Image Versions] Current: v%d, Offered: v%d", cfu_status.version, pCommand->version);
    uint8_t major_version = (pCommand->version & 0xFF000000) >> 24;
    uint8_t minor_version_h = (pCommand->version & 0x00FF0000) >> 16;
    uint8_t minor_version_l = (pCommand->version & 0x0000FF00) >> 8;
    uint8_t patch_version = pCommand->version & 0x000000FF;
    CFU_LOG_MESSAGE("Offered Version: v%d.%d.%d.%d", major_version, minor_version_h, minor_version_l, patch_version);
    if (forceIgnoreVersion)
    {
        if ((pResponse->status == FIRMWARE_UPDATE_OFFER_REJECT)
                && (pResponse->rejectReasonCode == FIRMWARE_OFFER_REJECT_OLD_FW))
        {
            if(!strcmp(vIgnore_status, "REJECT")){
                CFU_LOG_MESSAGE("Offer Rejected [Duplicated Version Ignore Request]");
                strcpy(vIgnore_status, "ACCEPT");      /* Not the first time ver ignore flag was recd */
                pResponse->status = FIRMWARE_UPDATE_OFFER_REJECT;
                pResponse->rejectReasonCode = FIRMWARE_OFFER_REJECT_OLD_FW;
            } else {
                CFU_LOG_MESSAGE("Offer Accepted [Version Ignored]");
                if(isDriver) CFU_LOG_MESSAGE("NOTE: Driver Being Used for CFU");
                isDriver?strcpy(vIgnore_status, "REJECT"):strcpy(vIgnore_status, "ACCEPT");
                pResponse->status = FIRMWARE_UPDATE_OFFER_ACCEPT;
                cfu_status.version = pCommand->version;
            }
        }
    }
    else
    {
        memset((uint8_t *)pResponse, 0, sizeof (cy_cfu_offer_response_t));
        if(pCommand->version <= cfu_status.version)
        {
            CFU_LOG_MESSAGE("Offer Rejected [Older Version Offered]");
            pResponse->status = FIRMWARE_UPDATE_OFFER_REJECT;
            pResponse->rejectReasonCode = FIRMWARE_OFFER_REJECT_OLD_FW;
            pResponse->token = token;
            return;
        }
        else
        {
            CFU_LOG_MESSAGE("Offer Accepted [Newer Version Offered]");
            pResponse->status = FIRMWARE_UPDATE_OFFER_ACCEPT;
            pResponse->token = token;
            cfu_status.version = pCommand->version;
        }
    }
    CFU_LOG_MESSAGE("[Version Ignore Information]\
        \r\n       Skip Version Check: %s\
        \r\n       Allow Ignore Version on Device Reset: %s",
        forceIgnoreVersion?"YES":"NO", strcmp(vIgnore_status, "ACCEPT")?"YES":"NO");

    /* This is the point detecting that the offer is accepted
     * This implementation starts a timer to ensure the FW
     * does not wait forever for the update process to complete.
     */
    if (pResponse->status == FIRMWARE_UPDATE_OFFER_ACCEPT)
    {
        cfu_status.updateStatus= CY_CFU_UPDATE_IN_PROGRESS;
        s_currentOffer.updateInProgress = true;
        s_currentOffer.forceReset = forceReset;
        s_currentOffer.activeComponentId = componentId;
    }
}

/**
 * \name Cy_CFU_ProcessCFWUGetFWVersion
 * \brief Process the get firmware version CFU command.
 * \param pResponse The response to populate and return to host
 * \retval None
 */
void Cy_CFU_ProcessCFWUGetFWVersion(cy_cfu_get_version_response_t* pResponse)
{

    memset((uint8_t *)pResponse, 0, sizeof(cy_cfu_get_version_response_t));

    /* CFU Protocol version */
    pResponse->header.fwUpdateRevision = CPFWU_REVISION;

    pResponse->header.componentCount = cfu_status.deviceCount;
    
    pResponse->versionAndProductInfoBlob[5] = COMPOSITE_DEVICE;
    pResponse->versionAndProductInfoBlob[3] = CY_FX_DWORD_GET_BYTE3(cfu_status.version);
    pResponse->versionAndProductInfoBlob[2] = CY_FX_DWORD_GET_BYTE2(cfu_status.version);
    pResponse->versionAndProductInfoBlob[1] = CY_FX_DWORD_GET_BYTE1(cfu_status.version);
    pResponse->versionAndProductInfoBlob[0] = CY_FX_DWORD_GET_BYTE0(cfu_status.version);

    /* This is a vendor specific section in the property which
     * can be used for retrieving the device update status
     */
    pResponse->versionAndProductInfoBlob[6] = cfu_status.updateStatus;
}

/**
 * \name Cy_USB_SendStatusBulkEp
 * \param pUsbApp Pointer to application context
 * \param data Pointer to data to send to interrupt endpoint
 * \param len length of data from pointer
 * \retval SUCCESS on reaching function end
 */
cy_en_usbd_ret_code_t Cy_USB_SendStatusBulkEp(cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *data,uint16_t len){

    Cy_USBD_GetUSBLinkActive(pUsbApp->pUsbdCtxt);

    uint8_t InEndpt = BULK_IN_ENDPOINT_2;

    Cy_USB_AppQueueWrite(pUsbApp, InEndpt, (uint8_t *)(data), len);
    return CY_USBD_STATUS_SUCCESS;
}

/**
 * \name Cy_CFU_HandleCFUVersionRequest
 * \brief Send CFU version and product info to host
 * \param pUsbdCtxt
 * \param pUsbApp
 * \param reportID
 * \retval status Indicate if info is successfully sent to host
 */
uint8_t Cy_CFU_HandleCFUVersionRequest(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t reportID)
{
    uint8_t retStatus = 0;
    uint8_t cfuBuffer[CFU_VERSION_RESPONSE_SIZE] __attribute__((aligned(64))) = {0};

    cy_cfu_get_version_response_t pResponse;

    /* Populate response.
     * Response should have the Report ID at index 0.
     */
    cfuBuffer[REPORT_ID_INDEX] = reportID;                      /* Supply same reportID as input (ref CFU spec) */
    cfu_status.deviceCount = NUM_SUBCOMPONENT;                    /* For CFU, this device has no subcomponents */
    cfu_status.updateStatus = CY_CFU_IDLE;                        /* At this time, no other update is taking place */
    uint8_t fpga_metadata_array[32] __attribute__ ((aligned(32)));          /* to store version info string from FPGA */
    uint8_t version_ascii_codes_array[16] __attribute__ ((aligned(32)));    /* Hold version info string made of ascii codes */
    uint32_t version_from_fpga = 0;                                         /* Hold ascii to raw converted version number */
    uint8_t vSection = 0;                                                   /* Track version section (major, minor_h, minor_l, patch)*/
    spi_meta_t fpga_metadata;                                               /* Hold metadata extracted from the FPGA add-on board's SPI flash */

    /* Read the first 24 Bytes of data from SPI flash. This is supposed to contain metadata */
    Cy_CFU_ExtSPIFlash_Read(0x0, fpga_metadata_array, 24, (uint8_t)SPI_FLASH_0);

    /* Convert the raw Bytes into meaningful metadata */
    Cy_CFU_SPI_ConvertMetadataBytes(&fpga_metadata, fpga_metadata_array, false);

    /* Check if the metadata indicates a valid image */
    if((fpga_metadata.validity & VALID_IMG_Msk) == VALID_IMG){
        CFU_LOG_MESSAGE("SPI Flash Contains a Valid Firmware Image");

        /* Just after the metadata, check if the bitstream contains "Version" string preceeding version information */
        if((fpga_metadata.validity & V_STRING_PRESENT_Msk) == V_STRING_PRESENT){
            Cy_CFU_ExtSPIFlash_Read(0x26, version_ascii_codes_array, 16, (uint8_t)SPI_FLASH_0);
        
        /* If there is no "Version" string present, version info is expected to start 8 Bytes prior */
        } else {
            Cy_CFU_ExtSPIFlash_Read(0x18, version_ascii_codes_array, 16, (uint8_t)SPI_FLASH_0);
        }

        uint8_t iteration = 0;
        version_ascii_codes_array[15] = '\0';
        for(int i=0; i<15 && iteration<4; i++){
            if(version_ascii_codes_array[i]==0x2E || version_ascii_codes_array[i]==0x0A){
                /* Delimiter reached, end section */
                version_from_fpga = version_from_fpga << 8;
                version_from_fpga += vSection;
                vSection = 0;
                iteration++;
            }
            else if(version_ascii_codes_array[i]<0x40 && 0x29<version_ascii_codes_array[i]){ 
                vSection *= 10;
                vSection += version_ascii_codes_array[i]-0x30;
            } else {
                version_from_fpga = version_from_fpga << 8;
                version_from_fpga += vSection;
                break;
            }
        }

        uint8_t fpga_major_version = (version_from_fpga & 0xFF000000) >> 24;
        uint8_t fpga_minor_version_h = (version_from_fpga & 0x00FF0000) >> 16;
        uint8_t fpga_minor_version_l = (version_from_fpga & 0x0000FF00) >> 8;
        uint8_t fpga_patch_version = version_from_fpga & 0x000000FF;
        CFU_LOG_MESSAGE("[from FPGA add-on board]\
            \r\n       Version: %d.%d.%d.%d",
            fpga_major_version, fpga_minor_version_h, fpga_minor_version_l, fpga_patch_version
        );

    } else {
        if(fpga_metadata.validity==INVALID_IMG){
            CFU_LOG_ERROR("Invalid Image - SPI Flash Programming Aborted During Previous Session");
        } else {
            CFU_LOG_ERROR("Invalid Image - SPI Flash Does NOT Contain a Valid Firmware Image");
        }

        version_from_fpga = CY_FX_MAKEDWORD(MAJOR_VERSION, MINOR_VERSION_H, MINOR_VERSION_L, PATCH_VERSION); 
        uint8_t fpga_major_version = (version_from_fpga & 0xFF000000) >> 24;
        uint8_t fpga_minor_version_h = (version_from_fpga & 0x00FF0000) >> 16;
        uint8_t fpga_minor_version_l = (version_from_fpga & 0x0000FF00) >> 8;
        uint8_t fpga_patch_version = version_from_fpga & 0x000000FF;
        CFU_LOG_MESSAGE("Assumed Current Version: %d.%d.%d.%d",
            fpga_major_version, fpga_minor_version_h, fpga_minor_version_l, fpga_patch_version
        );
    }

    cfu_status.version = version_from_fpga;

    /* Pass to MSFT-based CFU API to complete response population */
    Cy_CFU_ProcessCFWUGetFWVersion(&pResponse);

    /* Prepare CFU response */
    memcpy((cfuBuffer+SIZE_REPORT_ID), (uint8_t *)&pResponse, sizeof(cy_cfu_get_version_response_t));

    /* Send response back to host */
    retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt,(uint8_t *)cfuBuffer, sizeof(cy_cfu_get_version_response_t) + SIZE_REPORT_ID);

    return retStatus;
}

/**
 * \name Cy_CFU_HandleCFUOfferOpUsageReq
 * \brief Handler to interface with the MSFT-based CFU offer processing API
 * \param pUsbdCtxt
 * \param pUsbApp
 * \param buf
 * \retval status Indicates successful response transfer after CFU offer processing
 */
uint8_t Cy_CFU_HandleCFUOfferOpUsageReq(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *buf)
{
    cy_cfu_offer_command_t pCommand;
    cy_cfu_offer_response_t pResponse;
    uint8_t reportID = OFFER_OUTPUT_USAGE_REPORT_ID;
    uint16_t responseSize = 0;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    if(buf[0] == OFFER_OUTPUT_USAGE_REPORT_ID){
        memcpy((uint8_t*)&pCommand, (buf + SIZE_REPORT_ID), sizeof(cy_cfu_offer_command_t));

        /* Pass received offer to MSFT API and collect response */
        Cy_CFU_ProcessCFWUOffer(&pCommand, &pResponse);
        responseSize = sizeof(cy_cfu_offer_response_t)+ SIZE_REPORT_ID;
        memset((cfuBuffer+SIZE_REPORT_ID), 0, sizeof(cy_cfu_offer_response_t));

        /* Copy response to response buffer and send to host */
        cfuBuffer[REPORT_ID_INDEX] = reportID;
        memcpy((cfuBuffer+SIZE_REPORT_ID), (uint8_t*)&pResponse, sizeof(cy_cfu_offer_response_t));

        retStatus = Cy_USB_SendStatusBulkEp(pUsbApp,(uint8_t*)buf,responseSize);
    }
    return retStatus;
}

/**
 * \name Cy_CFU_HandleCFUContentOpUsage
 * \brief Handler to interface with the MSFT-based CFU content processing API
 * \param pUsbdCtxt
 * \param pUsbApp
 * \param buf
 * \retval status Indicates successful response transfer after CFU content processing
 */
uint8_t Cy_CFU_HandleCFUContentOpUsage(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *buf)
{
    cy_cfu_content_command_t p1Command;
    cy_cfu_content_response_t p1Response;

    uint16_t responseSize = 0;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    if(buf[0] == CONTENT_OUTPUT_USAGE_REPORT_ID){

        memcpy((uint8_t*)&p1Command, (uint8_t *)(buf + SIZE_REPORT_ID), sizeof(cy_cfu_content_command_t));

        /* Pass received content to MSFT API and collect response */
        Cy_CFU_ProcessCFWUContent(&p1Command, &p1Response);

        /* Send response to host - note that the report ID type of this response is different from the received report ID */
        memset(buf, 0, CFU_OFFER_RESPONSE_BUFFER_SIZE);

        buf[0] = CONTENT_RESPONSE_INPUT_USAGE_REPORT_ID;
        responseSize = sizeof(cy_cfu_content_response_t) + 1;
        memcpy((buf + 1), (uint8_t *)&p1Response, sizeof(cy_cfu_content_response_t));
        retStatus = Cy_USB_SendStatusBulkEp(pUsbApp,(uint8_t*)buf,responseSize);
    }

    return retStatus;

}

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
uint8_t Cy_USB_HandleCFUOfferNContentRequest(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t reportID, uint16_t wLength)
{
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    switch(reportID)
    {
        case OFFER_OUTPUT_USAGE_REPORT_ID:
        {
            /* Copy received offer from Host to buffer */
            memset(cfuBuffer, 0, wLength);
            retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, (uint8_t *)cfuBuffer, wLength);

            while (!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt))
            {
                Cy_SysLib_DelayUs(10);
            }
            if(retStatus == CY_USBD_STATUS_SUCCESS)
            {
                if(cfuBuffer[0] == OFFER_OUTPUT_USAGE_REPORT_ID){
                    Cy_CFU_HandleCFUOfferOpUsageReq(pUsbdCtxt, pUsbApp, cfuBuffer);
                }
                CFU_LOG_TRACE_MSG("Offer Output Usage Report ID: %d\r\n", cfuBuffer[0]);
            }
            else{
                CFU_LOG_ERROR("EP0 - Recv FAIL, Status: %d\r\n", retStatus);
            }
            break;
        }

        case CONTENT_OUTPUT_USAGE_REPORT_ID:
        {
            /** \note User should ensure prevention of transition to LPM during firmware update */

            /* Copy received content to buffer */
            memset(cfuBuffer, 0, wLength);

            retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, (uint8_t *)cfuBuffer, wLength);
            if (retStatus == CY_USBD_STATUS_SUCCESS)
            {
                /* Wait until receive DMA transfer has been completed. */
                while (!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) {
                    Cy_SysLib_DelayUs(10);
                }
                if (!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) {
                    DBG_APP_ERR("EP0 receive failed\r\n");
                    Cy_USB_USBD_RetireRecvEndp0Data(pUsbdCtxt);
                    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
                }
            }

            while(!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) Cy_SysLib_DelayUs(10);
            if(retStatus == CY_USBD_STATUS_SUCCESS)
            {
                if(cfuBuffer[0] == CONTENT_OUTPUT_USAGE_REPORT_ID)
                {
                    Cy_CFU_HandleCFUContentOpUsage(pUsbApp->pUsbdCtxt,pUsbApp,cfuBuffer);
                }
                CFU_LOG_TRACE_MSG("Content Output Usage Report ID: %d\n\r", cfuBuffer[0]);
            }
            else{
                CFU_LOG_ERROR("EP0 - Recv FAIL, Status: %d\r\n", retStatus);
            }

            /** \note If user disabled LPM for update, LPM can be re-enabled here */
            break;
        }

        default:
        {
            break;
        }

    }

    return retStatus;
}

/**
 * \name Cy_CFU_SPI_Start
 * \brief starts SPI with SPI_FLASH_0, to be used for CFU on the FPGA add-on board
 * \param pAppCtxt
 * \retval status Indicate SPI start success or failure
 */
uint32_t Cy_CFU_SPI_Start(cy_stc_usb_app_ctxt_t *pAppCtxt){
    if((Cy_SPI_Start(pAppCtxt, SPI_FLASH_0)==CY_SMIF_SUCCESS)){
        DBG_APP_INFO("SPI Start Success\r\n");
        return 0u;
    } else {
        DBG_APP_ERR("SPI Start Failed\r\n");
        return 1u;
    }
}

/**
 * \name Cy_CFU_ExtSPIFlash_Prepare
 * \brief Initialize and erase the external flash, in preparation for CFU writing
 * \param componentID Unutilised in this application, can be used if selection required amongst several target components for CFU is required
 * \retval status
 */
uint32_t Cy_CFU_ExtSPIFlash_Prepare(uint8_t componentId){
    CFU_LOG_MESSAGE("Preparing for FW Update")
    uint8_t status = 0;
    
    /* Initialize Flash */
    if((Cy_CFU_SPI_FlashInit(SPI_FLASH_0, false, false)==CY_SMIF_SUCCESS)){
        CFU_LOG_MESSAGE("Flash Init Success");
    } else {
        CFU_LOG_ERROR("Flash Init Fail");
    }
    
    /* Erase Sectors from addresses 0x00000 through 0xF0000 */
    uint16_t local_wIndex = 0x0;
    while(local_wIndex<=0xF){
        uint32_t sector = local_wIndex & 0xFF;
        uint32_t spiAddress = sector * CY_APP_SPI_FLASH_ERASE_SIZE;
        if(!(Cy_SPI_SectorErase(SPI_FLASH_0, spiAddress) == CY_SMIF_SUCCESS)){
            CFU_LOG_ERROR("Sector from 0x%x, Erase failed!", spiAddress);
        }
        local_wIndex+=0x1;
    }
    CFU_LOG_MESSAGE("Flash Erase Success");

    CFU_LOG_MESSAGE("Completed Preparing for FW Update");
    return (uint32_t)status;
}

/**
 * \name Cy_CFU_SPI_ConvertMetadataBytes
 * \param metadata Metadata struct for metdata info
 * \param buffer Array for raw Bytes
 * \param mode  true for Metadata to Bytes
 *              false for Bytes to Metadata
 * \retval None
 */
void Cy_CFU_SPI_ConvertMetadataBytes(spi_meta_t* restrict metadata, uint8_t* buffer, bool mode){
    uint8_t* address = buffer;
    if(mode){ /* Metadata to Bytes */
        CFU_LOG_TRACE_MSG("Metadata Start");

        /* start signature */
        memcpy(address, &(metadata->start_sig), sizeof(metadata->start_sig));
        address += sizeof(metadata->start_sig);
        
        /* fw_checksum */
        memcpy(address, &(metadata->fw_checksum), sizeof(metadata->fw_checksum));
        address += sizeof(metadata->fw_checksum);
        
        /* fw_offset */
        memcpy(address, &(metadata->fw_offset), sizeof(metadata->fw_offset));
        address += sizeof(metadata->fw_offset);
        
        /* validity */
        memcpy(address, &(metadata->validity), sizeof(metadata->validity));
        address += sizeof(metadata->validity);

        /* reserved */
        memcpy(address, &(metadata->reserved), sizeof(metadata->reserved));
        address += sizeof(metadata->reserved);
        
        /* end signature */
        memcpy(address, &(metadata->end_sig), sizeof(metadata->end_sig));
        address += sizeof(metadata->end_sig);

        CFU_LOG_TRACE_MSG("Start signature: %c%c%c%c", buffer[0], buffer[1], buffer[2], buffer[3]);
        CFU_LOG_TRACE_MSG("FW size: 0x%x", (buffer[4]<<0) + (buffer[5]<<8) + (buffer[6]<<16) + (buffer[7]<<24));
        CFU_LOG_TRACE_MSG("FW offset: 0x%x", (buffer[8]<<0) + (buffer[9]<<8) + (buffer[10]<<16) + (buffer[11]<<24));
        CFU_LOG_TRACE_MSG("Validity: 0x%x", (buffer[12]<<0) + (buffer[13]<<8) + (buffer[14]<<16) + (buffer[15]<<24));
        CFU_LOG_TRACE_MSG("End signature: %c%c%c%c", buffer[20], buffer[21], buffer[22], buffer[23]);
        CFU_LOG_TRACE_MSG("Metadata End");
    } else {
        CFU_LOG_TRACE_MSG("Metadata Start");

        /* Start signature */
        metadata->start_sig[0] = buffer[0];
        metadata->start_sig[1] = buffer[1];
        metadata->start_sig[2] = buffer[2];
        metadata->start_sig[3] = buffer[3];

        metadata->fw_checksum = (buffer[4]<<0) + (buffer[5]<<8) + (buffer[6]<<16) + (buffer[7]<<24);
        metadata->fw_offset = (buffer[8]<<0) + (buffer[9]<<8) + (buffer[10]<<16) + (buffer[11]<<24);
        metadata->validity = (buffer[12]<<0) + (buffer[13]<<8) + (buffer[14]<<16) + (buffer[15]<<24);
        metadata->end_sig[0] = buffer[20];
        metadata->end_sig[1] = buffer[21];
        metadata->end_sig[2] = buffer[22];
        metadata->end_sig[3] = buffer[23];

        CFU_LOG_TRACE_MSG("Start signature: %c%c%c%c", metadata->start_sig[0], metadata->start_sig[1], metadata->start_sig[2], metadata->start_sig[3]);
        CFU_LOG_TRACE_MSG("FW size: 0x%x", metadata->fw_checksum);
        CFU_LOG_TRACE_MSG("FW offset: 0x%x", metadata->fw_offset);
        CFU_LOG_TRACE_MSG("Validity: 0x%x", metadata->validity);
        CFU_LOG_TRACE_MSG("End signature: %c%c%c%c", metadata->end_sig[0], metadata->end_sig[1], metadata->end_sig[2], metadata->end_sig[3]);
        CFU_LOG_TRACE_MSG("Metadata End");
    }
}

void gen_metadata(uint8_t* buffer){
    /* Prepare metadata to write onto SPI flash */
    spi_meta_new.start_sig[0] = 'I';                /* Start Signature "IFX#" */
    spi_meta_new.start_sig[1] = 'F';
    spi_meta_new.start_sig[2] = 'X';
    spi_meta_new.start_sig[3] = '#';

    spi_meta_new.fw_checksum = hwVariantMask;       /* Size of incoming firmware */
    spi_meta_new.fw_offset = SPI_META_SIZE;         /* Metadata size = 24B */
    spi_meta_new.validity = INVALID_IMG;            /* Mark as invalid until completion of CFU */
    spi_meta_new.reserved = 0xFFFFFFFF;             /* reserved, fill with 1s */

    spi_meta_new.end_sig[0] = 'I';                  /* End Signature "IFX$" */
    spi_meta_new.end_sig[1] = 'F';
    spi_meta_new.end_sig[2] = 'X';
    spi_meta_new.end_sig[3] = '$';

    Cy_CFU_SPI_ConvertMetadataBytes(&spi_meta_new, buffer, true);
    CFU_LOG_MESSAGE("Prepared Metadata");

}

/**
 * \name Cy_CFU_ExtSPIFlash_Write
 * \param offset address to write to
 * \param pData pointer to data array that needs to be written
 * \param length len of data to be considered from location pData
 * \param componentID Can be used if selection required amongst multiple target components for CFU (Unutilised in this application)
 * \retval status
 */
uint32_t Cy_CFU_ExtSPIFlash_Write(uint32_t offset, uint8_t* pData, uint8_t length, uint8_t componentId){

    uint8_t status = 0;                 /* Return status */
    contentPacketNum++;                 /* Increment func call counter - represents data packet count */

    /* If this is the first data pkt, corresponding addr will be start addr for the upcoming group of data pkts */
    if (recdB==0) {
        cfu_extspiflash_addr=offset;    /* Set the write address to the offset given */
    }

    /* First block */
    if(cfu_first_block){
        uint8_t version_string_check_str[8];
        memcpy(version_string_check_str, pData, sizeof(uint8_t)*8);
        version_string_check_str[7]='\0';

        /* Generate metadata */
        gen_metadata(metadata_buffer);
        if(!strcmp("Version", (char *)version_string_check_str)){
            /* String present */
            (&spi_meta_new)->validity = (uint32_t)V_STRING_PRESENT;
        } else {
            /* String absent */
            (&spi_meta_new)->validity = (uint32_t)VALID_IMG;
        }

        /* Copy the contents of metadata into the temp buffer */
        memcpy(tempbuf, metadata_buffer, SPI_META_SIZE);

        /* Indicate the overflow size caused by metadata */
        overflow = SPI_META_SIZE;

        CFU_LOG_MESSAGE("START");
    }

    memcpy(sfBuffer+recdB, tempbuf, overflow);
    recdB += overflow;

    if(length>overflow){
        /* Copy the contents of pData into the buffer */
        memcpy(sfBuffer+recdB, pData, length-overflow);
        
        /* Increment the received byte count */
        recdB += length-overflow;

        memcpy(tempbuf, pData+(length-overflow), overflow);

        difference = length-overflow;

    } else {
        /* Copy the contents of pData into the buffer */
        memcpy(sfBuffer+recdB, pData, length);
        
        /* Increment the received byte count */
        recdB += length;

        memcpy(tempbuf, pData+(length%difference), length%difference);
    }

    /* Last block */
    if(cfu_last_block && recdB<SPI_FLASH_PAGE_SIZE){

        /* Fill the rest of the array with zeroes */
        recdB+=length;
        memset((sfBuffer+recdB), 0, SPI_FLASH_PAGE_SIZE-recdB);
        recdB=SPI_FLASH_PAGE_SIZE;
    }
 
    /* If page size (SPI_FLASH_PAGE_SIZE) worth of data is ready in buffer, write page */
    if(recdB==SPI_FLASH_PAGE_SIZE) {
        if(!Cy_SPI_WritePage(cfu_extspiflash_addr, sfBuffer, SPI_FLASH_0)){

            pageCount++;    /* SPI_FLASH_PAGE_SIZE worth data has been written */
            recdB=0;        /* Reset the number of bytes received */
            CFU_LOG_TRACE_MSG("> Current Page: %d, Total blocks written: %d", pageCount, contentPacketNum);
        } else {
            CFU_LOG_ERROR("Current Page: %d, Total bytes written: %d", pageCount, contentPacketNum);
        }
    }

    /* Last block */
    if(cfu_last_block){
        /* Reset variables */
        recdB = 0;
        pageCount = 0;
        contentPacketNum = 0;

        vTaskDelay(3000);
        memcpy(sfBuffer+recdB, tempbuf, overflow);
        memset(sfBuffer+recdB+overflow, 0, SPI_FLASH_PAGE_SIZE-overflow);
        {
            if(!Cy_SPI_WritePage(cfu_extspiflash_addr+SPI_FLASH_PAGE_SIZE, sfBuffer, SPI_FLASH_0)){
                pageCount++;    /* SPI_FLASH_PAGE_SIZE worth data has been written */
                recdB=0;        /* Reset the number of bytes received */
                CFU_LOG_TRACE_MSG("Current Page: %d, Total blocks written: %d", pageCount, contentPacketNum);
            } else {
                CFU_LOG_ERROR("Current Page: %d, Total blocks written: %d", pageCount, contentPacketNum);
            }
        }

        /* Update metadata to declare valid image */
        Cy_CFU_SPI_ConvertMetadataBytes(&spi_meta_new, metadata_buffer, true);

        /* Re-write partial region in first page */
        cy_en_smif_status_t rewrite_status = Cy_SPI_WritePage_Partial(0x0, metadata_buffer, SPI_FLASH_0, 24);
        if(rewrite_status!=CY_SMIF_SUCCESS){
            CFU_LOG_ERROR("FAILED to rewrite first page, status: 0x%x", rewrite_status);
        } else {
            CFU_LOG_MESSAGE("Updated Image Validity: %s",
                (&spi_meta_new)->validity==VALID_IMG?"VALID_IMG":
                ((&spi_meta_new)->validity==V_STRING_PRESENT?"V_STRING_PRESENT":"INVALID_IMAGE"));
        }

        CFU_LOG_MESSAGE("END");
        /* (Optionally) Reset the device (NOT IMPLEMENTED) */
    }

    return (uint32_t)status;
}

/**
 * \name Cy_CFU_ExtSPIFlash_Read
 * \brief Function to read a data chunk from flash
 * \param offset address on the SPI flash to be read from
 * \param pData pointer to a buffer into which read data is stored
 * \param length number of bytes of data
 * \param flashIndex cy_en_flash_index_t flash index identifier typecasted to uint8_t
 * \retval Read operation exit status
 */
uint32_t Cy_CFU_ExtSPIFlash_Read(uint32_t offset, uint8_t* pData, uint16_t length, uint8_t flashIndex){
    return (uint32_t)Cy_SPI_ReadOperation(offset, pData, (uint32_t)length, (cy_en_flash_index_t)flashIndex);
}

/**
 * \name calculateOneByteChecksum
 * \note Perform addition of 8b of data repeatedly to a 32b store, to be used for checksum
 * \param data Pointer to start of data to be considered for checksum addition
 * \param len Number of bytes of data to be added to the checksum
 * \retval Checksum 32b resultant value of repeated 8b additions
 */
uint32_t calculateOneByteChecksum(uint8_t *data, uint32_t len) {
    uint32_t checksum = 0;
    uint32_t i = 0;
    for (i = 0; i < len; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

/* [] END OF FILE */
