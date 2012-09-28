/*
* Copyright (c) 2012 All Right Reserved, Gustavo Litovsky
*
* You may use this file for any purpose, provided this copyright notice and
* the attribution remains.
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
* Author: Gustavo Litovsky
* gustavo@glitovsky.com
*
* File:    main.c
* Summary: Main UART driver for MSP430F5438 Experimenter Board
* 		   This simple application shows how to configure the UART driver
* 		   to 115200 baud 8N1 and uses it to send the string Hello continuously.
* 		   If it receives the letter 'a', it will toggle an LED on the board.
*
*/

#include <msp430.h>
#include <string.h>
#include "uart.h"

void setSMCLK8MHz();

// UART Port Configuration parameters and registers
UARTConfig cnf;
USCIUARTRegs uartUsciRegs;
USARTUARTRegs uartUsartRegs;

// Buffers to be used by UART Driver
unsigned char uartTxBuf[200];
unsigned char uartRxBuf[200];

void main(void)
{
	// Disable MSP430 Watchdog Timer
	WDTCTL = WDTPW | WDTHOLD;

	// Configure Experimenter Board LED on P1.0 - Active High
	P1SEL &= ~BIT0;
	P1DIR |= BIT0;
	P1OUT |= BIT0;

	// Configure SMCLK to 8MHz
	setSMCLK8MHz();


	/********************************
	 *
	 * UART Specific Configuration
	 *
	 ********************************/
	initUartDriver();

	// Configure UART Module on UCA0
	cnf.moduleName = UCA0;

	// Use UART Pins P5.7 and P5.6
	cnf.portNum = PORT_3;
	cnf.RxPinNum = PIN5;
	cnf.TxPinNum = PIN4;

	// 115200 Baud from 8MHz SMCLK
	cnf.clkRate = 8000000L;
	cnf.baudRate = 9600L;            // EZ430 allows only up to 9600 baud
	cnf.clkSrc = UART_CLK_SRC_SMCLK;

	// 8N1
	cnf.databits = 8;
	cnf.parity = UART_PARITY_NONE;
	cnf.stopbits = 1;

	int res = configUSCIUart(&cnf,&uartUsciRegs);
	if(res != UART_SUCCESS)
	{
		// Failed to initialize UART for some reason
		__no_operation();
	}

	// Configure the buffers that will be used by the UART Driver.
	// These buffers are exclusively for the UART driver's use and should not be touched
	// by the application itself. Note that they may affect performance if they're too
	// small.
	setUartTxBuffer(&cnf, uartTxBuf, 200);
	setUartRxBuffer(&cnf, uartRxBuf, 200);

	enableUartRx(&cnf);

	/*********************************/

	__enable_interrupt(); // Enable Global Interrupts
	while(1)
	{
		// Send the string hello using interrupt driven
		uartSendDataInt(&cnf,(unsigned char *)"Hello\r\n", strlen("Hello\r\n"));
		_delay_cycles(100000);

		int bytesAvailable = numUartBytesReceived(&cnf);
		if(bytesAvailable > 0)
		{
			unsigned char tempBuf[100];
			memset(tempBuf,0,100);

			volatile int bytesRead = readRxBytes(&cnf, tempBuf, bytesAvailable, 0);
			if(bytesRead == bytesAvailable)
			{
				// All requested bytes read. Do something with it.

				// If we receive the letter a, we toggle the LED
				if(tempBuf[0] == 'a')
				{
					P1OUT ^= BIT0;
				}
			}
			else
			{
				// Couldn't read all the bytes we requested
				__no_operation();
			}
		}
	}
}

/*!
 * \brief Initializes SMCLK to 8MHz
 *
 *
 * @param None
 * \return None
 *
 */
void setSMCLK8MHz()
{
	BCSCTL1 = CALBC1_8MHZ;                    // Set DCO to 8MHz
	DCOCTL = CALDCO_8MHZ;
}
