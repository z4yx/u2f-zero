/*******************************************************************************
 * @file callback.c
 * @brief USB Callbacks.
 *******************************************************************************/

//=============================================================================
// src/callback.c: generated by Hardware Configurator
//
// This file is only generated if it does not exist. Modifications in this file
// will persist even if Configurator generates code. To refresh this file,
// you must first delete it and then regenerate code.
//=============================================================================
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <SI_EFM8UB1_Register_Enums.h>
#include <efm8_usb.h>
#include <stdio.h>
#include "idle.h"
#include "app.h"
#include "bsp.h"
#include "descriptors.h"
#include "u2f_hid.h"

#define HID_INTERFACE_INDEX 0
extern SI_SEGMENT_VARIABLE(appdata, struct APP_DATA, SI_SEG_DATA);

uint8_t tmpBuffer;


void USBD_ResetCb(void) {
	u2f_print("USBD_ResetCb\r\n");
}


void USBD_DeviceStateChangeCb(USBD_State_TypeDef oldState,
		USBD_State_TypeDef newState) {
	u2f_print("USBD_DeviceStateChangeCb\r\n");
}

bool USBD_IsSelfPoweredCb(void) {
	//u2f_print("USBD_IsSelfPoweredCb\r\n");
	return false;
}


USB_Status_TypeDef USBD_SetupCmdCb(
		SI_VARIABLE_SEGMENT_POINTER(setup, USB_Setup_TypeDef, MEM_MODEL_SEG)) {

	USB_Status_TypeDef retVal = USB_STATUS_REQ_UNHANDLED;


	if ((setup->bmRequestType.Type == USB_SETUP_TYPE_STANDARD)
			&& (setup->bmRequestType.Direction == USB_SETUP_DIR_IN)
			&& (setup->bmRequestType.Recipient == USB_SETUP_RECIPIENT_INTERFACE)) {
		// A HID device must extend the standard GET_DESCRIPTOR command
		// with support for HID descriptors.

		switch (setup->bRequest) {
		case GET_DESCRIPTOR:
			if ((setup->wValue >> 8) == USB_HID_REPORT_DESCRIPTOR) {
				switch (setup->wIndex) {
				case 0: // Interface 0
					USBD_Write(EP0, ReportDescriptor0,
							EFM8_MIN(sizeof(ReportDescriptor0), setup->wLength),
							false);
					retVal = USB_STATUS_OK;
					break;

				default: // Unhandled Interface
					break;
				}
			} else if ((setup->wValue >> 8) == USB_HID_DESCRIPTOR) {
				switch (setup->wIndex) {
				case 0: // Interface 0
					USBD_Write(EP0, (&configDesc[18]),
							EFM8_MIN(USB_HID_DESCSIZE, setup->wLength), false);
					retVal = USB_STATUS_OK;
					break;

				default: // Unhandled Interface
					break;
				}
			}
			break;
		}
	}
	else if ((setup->bmRequestType.Type == USB_SETUP_TYPE_CLASS)
	           && (setup->bmRequestType.Recipient == USB_SETUP_RECIPIENT_INTERFACE)
	           && (setup->wIndex == HID_INTERFACE_INDEX))
	  {
	    // Implement the necessary HID class specific commands.
	    switch (setup->bRequest)
	    {
	      case USB_HID_SET_REPORT:
	    	  u2f_print("output report\r\n");
	        break;

	      case USB_HID_GET_REPORT:
	    	  //u2f_print("input report\r\n");

	        break;

	      case USB_HID_SET_IDLE:
	        if (((setup->wValue & 0xFF) == 0)             // Report ID
	            && (setup->wLength == 0)
	            && (setup->bmRequestType.Direction != USB_SETUP_DIR_IN))
	        {
	        	// u2f_print("set idle\r\n");
	          idleTimerSet(setup->wValue >> 8);
	          retVal = USB_STATUS_OK;
	        }
	        else u2f_print("unhandled USB_HID_SET_IDLE\r\n");
	        break;

	      case USB_HID_GET_IDLE:
	        if ((setup->wValue == 0)                      // Report ID
	            && (setup->wLength == 1)
	            && (setup->bmRequestType.Direction == USB_SETUP_DIR_IN))
	        {
	        	u2f_print("get idle\r\n");
	          tmpBuffer = idleGetRate();
	          USBD_Write(EP0, &tmpBuffer, 1, false);
	          retVal = USB_STATUS_OK;
	        }
	        else u2f_print("unhandled USB_HID_GET_IDLE\r\n");
	        break;
	      default:
	    	  u2f_print("unhandled setup->bRequest\r\n");
	    }
	  }
	  else
	  {
		  if (setup->bmRequestType.Recipient == USB_SETUP_RECIPIENT_ENDPOINT)
			  u2f_print("endpoint called!\n");
	  }

	return retVal;
}


uint16_t USBD_XferCompleteCb(uint8_t epAddr, USB_Status_TypeDef status,
		uint16_t xferred, uint16_t remaining) {

	int i = 0;
	char buf[6];
	struct u2f_hid_msg res;
	uint8_t* resbuf = (uint8_t*)&res;

	if (epAddr == EP1OUT)
	{

#ifdef U2F_PRINT
		for (i=0; i < sizeof(appdata.hidmsgbuf); i++)
		{
			uint16_t l = (uint8_t)appdata.hidmsgbuf[i];
			sprintf(buf,"%x",l);
			u2f_write_s(buf);
		}
		u2f_write_s("\r\n");
#endif

		do {
			i = hid_u2f_request((struct u2f_hid_msg*)appdata.hidmsgbuf,
							&res);
			if (i == U2FHID_REPLY || i == U2FHID_INCOMPLETE)
			{
				USBD_Write(EP1IN, resbuf, 64, false);
			}
		} while (i == U2FHID_INCOMPLETE);

	}
	return 0;
}
