/* debug.c */
/* Beacon debug interface. {{{
 *
 * Copyright (C) 2012 Florent Duchon
 *
 * APBTeam:
 *        Web: http://apbteam.org/
 *      Email: team AT apbteam DOT org
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * }}} */
#include <stdarg.h>
#include "configuration.h"
#include "debug.h"
#include "servo.h"

HAL_UsartDescriptor_t appUsartDescriptor;          			// USART descriptor (required by stack)
uint8_t usartRxBuffer[APP_USART_RX_BUFFER_SIZE];   	// USART Rx buffer
uint8_t usartTxBuffer[APP_USART_TX_BUFFER_SIZE];   	// USART Tx buffer

/* This function initializes the USART interface for debugging on avr */
 void initSerialInterface(void)
 {
	appUsartDescriptor.tty             	= USART_CHANNEL;
	appUsartDescriptor.mode            	= USART_MODE_ASYNC;
	appUsartDescriptor.baudrate        	= USART_BAUDRATE_38400;
	appUsartDescriptor.dataLength     	= USART_DATA8;
	appUsartDescriptor.parity          	= USART_PARITY_EVEN;
	appUsartDescriptor.stopbits        	= USART_STOPBIT_1;
	appUsartDescriptor.rxBuffer        	= usartRxBuffer;
	appUsartDescriptor.rxBufferLength	= sizeof(usartRxBuffer);
	appUsartDescriptor.txBuffer        	= NULL; // use callback mode
	appUsartDescriptor.txBufferLength	= 0;
	appUsartDescriptor.rxCallback      	= usartRXCallback;
	appUsartDescriptor.txCallback      	= NULL;
	appUsartDescriptor.flowControl     = USART_FLOW_CONTROL_NONE;
	OPEN_USART(&appUsartDescriptor);
 }


/* RX USART Callback */
void usartRXCallback(uint16_t bytesToRead)
{
	uint8_t rxBuffer;
 	READ_USART(&appUsartDescriptor,&rxBuffer,bytesToRead);
	
	switch(rxBuffer)
	{
		case 'o':
			/* Increase servo 1 angle */
			uprintf("SERVO_1 = %d\r\n",servo_angle_increase(SERVO_1));
			break;
		case 'l':
			/* Decrease servo 1 angle */
			uprintf("SERVO_1 = %d\r\n",servo_angle_decrease(SERVO_1));
			break;
		case 'p':
			/* Increase servo 2 angle */
			uprintf("SERVO_2 = %d\r\n",servo_angle_increase(SERVO_2));
			break;
		case 'm':
			/* Decrease servo 2 angle */
			uprintf("SERVO_2 = %d\r\n",servo_angle_decrease(SERVO_2));
			break;
		/* Default */
		default :
			uprintf(" ?? Unknown command ??\r\n");
	}
}

/* This function sends data string via the USART interface */
void uprintf(char *format, ...)
{
	va_list args;
	va_start(args,format);
	vsprintf(usartTxBuffer,format,args);
	WRITE_USART(&appUsartDescriptor,usartTxBuffer,strlen(usartTxBuffer));
	va_end(args);
}