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

	// Configure UART Module on USCIA1
	cnf.moduleName = USCI_A1;

	// Use UART Pins P5.7 and P5.6
	cnf.portNum = PORT_5;
	cnf.RxPinNum = PIN7;
	cnf.TxPinNum = PIN6;

	// 115200 Baud from 8MHz SMCLK
	cnf.clkRate = 8000000L;
	cnf.baudRate = 115200L;
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
	// Set clock to 8MHz
	UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
	UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

	__bis_SR_register(SCG0);                  // Disable the FLL control loop
	UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
	UCSCTL1 = DCORSEL_5;                      // Select DCO range 16MHz operation
	UCSCTL2 = FLLD_1 + 244;                   // Set DCO Multiplier for 8MHz
											// (N + 1) * FLLRef = Fdco
											// (244 + 1) * 32768 = 8MHz
											// Set FLL Div = fDCOCLK/2
	__bic_SR_register(SCG0);                  // Enable the FLL control loop

	// Worst-case settling time for the DCO when the DCO range bits have been
	// changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
	// UG for optimization.
	// 32 x 32 x 8 MHz / 32,768 Hz = 250000 = MCLK cycles for DCO to settle
	__delay_cycles(250000);

	// Loop until XT1,XT2 & DCO fault flag is cleared
	do
	{
	UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
											// Clear XT2,XT1,DCO fault flags
	SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	}while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
}
