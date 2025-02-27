/*
             LUFA Library
     Copyright (C) Dean Camera, 2015.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2015  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for USBtoSerial.c.
 */

#ifndef _USB_SERIAL_H_
#define _USB_SERIAL_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/interrupt.h>
		#include <avr/power.h>

		#include "Descriptors.h"
		//#include "AVRISPDescriptors.h"
		#include "Lib/V2Protocol.h"
		#include "Config/AppConfig.h"

		//#include <LUFA/Drivers/Board/LEDs.h>
		#include <LUFA/Drivers/Peripheral/Serial.h>
		#include <LUFA/Drivers/Misc/RingBuffer.h>
		#include <LUFA/Drivers/USB/USB.h>
		#include <LUFA/Platform/Platform.h>

	/* Macros: */
		#define AVR_RESET_LINE_MASK			(1<<PD7)
		#define AVR_RESET_LINE_PORT			PORTD
		#define AVR_RESET_LINE_DDR			DDRD
		
		//#define BOARD BOARD_GSCHEIDUINO
		//#define BOARD BOARD_ATEVAL
		
		#if (BOARD == BOARD_GSCHEIDUINO)
			#define LEDS_PORT	PORTD
			#define LEDS_DDR	DDRD
			#define LEDS_LED1        (1 << 6)
			#define LEDS_LED2        (1 << 5)
			#define LEDS_LED3        (1 << 4)
			#define LEDS_ALL_LEDS    (LEDS_LED1 | LEDS_LED2 | LEDS_LED3)
			#define LEDS_NO_LEDS     0
		#else
			#define LEDS_PORT	PORTB
			#define LEDS_DDR	DDRB
			#define LEDS_LED1        (1 << 6)
			#define LEDS_LED2        (1 << 6)
			#define LEDS_LED3        (1 << 6)
			#define LEDS_ALL_LEDS    (LEDS_LED1 | LEDS_LED2 | LEDS_LED3)
			#define LEDS_NO_LEDS     0
		#endif
		
		/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
		#define LEDMASK_USB_NOTREADY      	LEDS_LED1

		/** LED mask for the library LED driver, to indicate that the USB interface is enumerating. */
		#define LEDMASK_USB_ENUMERATING   	LEDS_LED1

		/** LED mask for the library LED driver, to indicate that the USB interface is ready. */
		#define LEDMASK_USB_READY         	LEDS_LED1

		/** LED mask for the library LED driver, to indicate that an error has occurred in the USB interface. */
		#define LEDMASK_USB_ERROR         	LEDS_LED1
		
		#define LEDMASK_TX					LEDS_LED2
		#define LEDMASK_RX					LEDS_LED3
		
		/** Firmware mode define for the USART Bridge mode. */
		#define MODE_USART_BRIDGE        false

		/** Firmware mode define for the AVRISP Programmer mode. */
		#define MODE_PDI_PROGRAMMER      true	
		
		/* External Variables: */
		extern bool         CurrentFirmwareMode;

	/* Function Prototypes: */
		void SetupHardware(void);
		void AVRISP_Task(void);
		void UARTBridge_Task(void);

		void EVENT_USB_Device_Connect(void);
		void EVENT_USB_Device_Disconnect(void);
		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_ControlRequest(void);

		void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
		void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
		
		uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
									const uint8_t wIndex,
									const void** const DescriptorAddress,
		                                    uint8_t* const DescriptorMemorySpace)
		                                    ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3) ATTR_NON_NULL_PTR_ARG(4);
		

#endif

