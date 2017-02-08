/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"
/* USER CODE BEGIN INCLUDE */
#include "bsp.h"
/* USER CODE END INCLUDE */
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CUSTOM_HID 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_Defines
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_DEFINES */
#define HID_PACKET_SIZE 64
/* USER CODE END PRIVATE_DEFINES */
  
/**
  * @}
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 

/** @defgroup USBD_AUDIO_IF_Private_Variables
 * @{
 */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */ 
  0x06, 0xd0, 0xf1,// USAGE_PAGE (FIDO Alliance)
  0x09, 0x01,// USAGE (Keyboard)
  0xa1, 0x01,// COLLECTION (Application)

        0x09, 0x20,                   //   USAGE (Input Report Data)
        0x15, 0x00,                   //   LOGICAL_MINIMUM (0)
        0x26, 0xff, 0x00,             //   LOGICAL_MAXIMUM (255)
        0x75, 0x08,                   //   REPORT_SIZE (8)
        0x95, HID_PACKET_SIZE,                   //   REPORT_COUNT (64)
        0x81, 0x02,                   //   INPUT (Data,Var,Abs)
        0x09, 0x21,                   //   USAGE(Output Report Data)
        0x15, 0x00,                   //   LOGICAL_MINIMUM (0)
        0x26, 0xff, 0x00,             //   LOGICAL_MAXIMUM (255)
        0x75, 0x08,                   //   REPORT_SIZE (8)
        0x95, HID_PACKET_SIZE,                   //   REPORT_COUNT (64)
        0x91, 0x02,                   //   OUTPUT (Data,Var,Abs)

  /* USER CODE END 0 */ 
  0xC0    /*     END_COLLECTION	             */
   
}; 

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */
/**
  * @}
  */ 
  
/** @defgroup USBD_CUSTOM_HID_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @{
  */
static int8_t CUSTOM_HID_Init_FS     (void);
static int8_t CUSTOM_HID_DeInit_FS   (void);
static int8_t CUSTOM_HID_OutEvent_FS (uint8_t event_idx, uint8_t state);
 

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS = 
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CUSTOM_HID_Init_FS
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{ 
  /* USER CODE BEGIN 4 */ 
  return (0);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CUSTOM_HID_DeInit_FS
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */ 
  return (0);
  /* USER CODE END 5 */ 
}

/**
  * @brief  CUSTOM_HID_OutEvent_FS
  *         Manage the CUSTOM HID class events       
  * @param  event_idx: event index
  * @param  state: event state
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS  (uint8_t event_idx, uint8_t state)
{ 
  /* USER CODE BEGIN 6 */ 
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData; 
  set_app_u2f_hid_msg((struct u2f_hid_msg *) hhid->Report_buf );
  return (0);
  /* USER CODE END 6 */ 
}

/* USER CODE BEGIN 7 */ 

void usb_write(uint8_t* buf, uint8_t len)
{
    uint8_t errors = 0;
    while (USBD_OK != (USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, buf, len)))
    {
        u2f_delay(2);
        if (errors++ > 30)
        {
            set_app_error(ERROR_USB_WRITE);
            break;
        }
    }
}
/**
  * @brief  USBD_CUSTOM_HID_SendReport_FS
  *         Send the report to the Host       
  * @param  report: the report to be sent
  * @param  len: the report length
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
/*  
static int8_t USBD_CUSTOM_HID_SendReport_FS ( uint8_t *report,uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len); 
}
*/
/* USER CODE END 7 */ 

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
