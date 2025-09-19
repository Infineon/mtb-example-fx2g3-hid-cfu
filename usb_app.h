/***************************************************************************//**
* \file usb_app.h
* \version 1.0
*
* \brief Provides definition used by the FX2G3 USB Echo Device application.
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

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

#ifdef FX_CFU

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

#define RED                             "\033[0;31m"
#define CYAN                            "\033[0;36m"
#define COLOR_RESET                     "\033[0m"

#define ASSERT(condition, value)           Cy_CheckStatus(__func__, __LINE__, condition, value, true);
#define ASSERT_NON_BLOCK(condition, value) Cy_CheckStatus(__func__, __LINE__, condition, value, false);
#define ASSERT_AND_HANDLE(condition, value, failureHandler) Cy_CheckStatusHandleFailure(__func__, __LINE__, condition, value, false, failureHandler);

/* P4.0 is used for VBus detect functionality. */
#define VBUS_DETECT_GPIO_PORT           (P4_0_PORT)
#define VBUS_DETECT_GPIO_PIN            (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR           (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE               (0u)

/*Related to WINUSB*/
extern uint8_t glOsString[];
extern uint8_t glOsCompatibilityId[];
extern uint8_t glOsFeature[];

/* USBD layer return code shared between USBD layer and Application layer. */
typedef enum cy_en_usb_app_ret_code_ {
    CY_USB_APP_STATUS_SUCCESS=0,
    CY_USB_APP_STATUS_FAILURE,
}cy_en_usb_app_ret_code_t;

#define RAM_BUF_SZ_WORDS        (15360)         /* Total size of the RAM based DMA buffers: 15 KB. */

/* 
 * USB application data structure which is bridge between USB system and device
 * functionality.
 * It maintains some usb system information which comes from USBD and it also
 * maintains information about device functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    uint8_t firstInitDone;
    uint8_t devAddr;
    uint8_t activeCfgNum;
    uint8_t currentAltSetting;
    cy_en_usb_device_state_t devState;
    cy_en_usb_device_state_t prevDevState;
    cy_en_usb_speed_t devSpeed;
    cy_en_usb_enum_method_t enumMethod;

    cy_stc_app_endp_dma_set_t endpInDma[16];
    cy_stc_app_endp_dma_set_t endpOutDma[16];
    uint8_t intfAltSetEndp[4][4];    /* Max 4INTF AND EACH INTERFACE 4 alt setting */ 
    /* Next three are related to central DMA */
    DMAC_Type *pCpuDmacBase;
    DW_Type *pCpuDw0Base;
    DW_Type *pCpuDw1Base;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;
    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt;

    /* Global Task handles */
    TaskHandle_t taskHandle;
    QueueHandle_t xQueue;
    void *pDevFuncCtxt;
    /* Timer functionality for endp0Rcvd */
    TimerHandle_t endp0OutTimerHandle;
    uint32_t endp0OuttimerExpiry;
    bool dataXferIntrEnabled;
    /* VBus detect status */
    bool vbusPresent;
    bool usbConnected;                                              /** Whether USB connection is enabled. */
    /* USB connection status */
    bool usbConnectDone;
    uint8_t *qspiWriteBuffer;
    uint8_t *qspiReadBuffer;
    uint8_t glpassiveSerialMode;
};
typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;

void  *Cy_USB_EchoDeviceInit(cy_stc_usb_app_ctxt_t *pAppCtxt);

void Cy_USB_AppSetupEndpDmaParamsHs(cy_stc_usb_app_ctxt_t *pUsbApp,
                                  uint8_t *pEndpDscr);

void Cy_USB_AppQueueRead(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                         uint8_t *pBuffer, uint16_t dataSize);

uint16_t Cy_USB_AppReadShortPacket(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                   uint8_t endpNumber, uint16_t pktSize);

void Cy_USB_AppQueueWrite(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                          uint8_t *pBuffer, uint16_t dataSize);

void Cy_USB_AppInit (cy_stc_usb_app_ctxt_t *pAppCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, DMAC_Type *pCpuDmacBase,
        DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base, cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);

void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);

void Cy_USB_RecvEndp0TimerCallback(TimerHandle_t xTimer);

void Cy_USB_AppSetCfgCallback(void *pAppCtxt,
                              cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetCallback(void *pAppCtxt, 
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt, 
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, 
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetupCallback(void *pAppCtxt, 
                             cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSuspendCallback(void *pAppCtxt, 
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppResumeCallback (void *pAppCtxt, 
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, 
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1SleepCallback(void *pUsbApp,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1ResumeCallback(void *pUsbApp,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetFeatureCallback(void *pUsbApp,
                                  cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppClearFeatureCallback(void *pUsbApp,
                                  cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg);

/* Functions to be provided at the application level to do USB connect/disconnect. */
bool Cy_USB_ConnectionEnable(cy_stc_usb_app_ctxt_t *pAppCtxt);

void Cy_USB_ConnectionDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);

void Cy_USB_AppLightDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);


void Cy_USB_AppInitEndpCpuDmaDscrConfig(cy_stc_dma_descriptor_config_t *pEndpCpuDmaDscrConfig,
                                        uint32_t *pSrcAddr, uint32_t *pDstAddr, uint32_t endpSize,
                                        cy_en_usb_endp_dir_t endpDirection);

void Cy_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking);

void Cy_USB_SendEndp0DataFailHandler(void);

void Cy_CheckStatusHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)());

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */

