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
* File:    uart.c
* Summary: Implementation of generic MSP430 UART Driver
*
*/

#include <msp430.h>
#include <math.h>
#include <string.h>
#include "uart.h"

// Port Information List so user isn't forced to pass information all the time
UARTConfig * prtInfList[5];

/*!
 * \brief Initializes the UART Driver
 *
 *
 * @param None
 * \return None
 *
 */
void initUartDriver()
{
	int i = 0;
	for(i = 0; i < 5; i++)
	{
		prtInfList[i] = NULL;
	}
}

/*!
 * \brief Configures the MSP430 pins for UART module
 *
 *
 * @param prtInf is UARTConfig instance with the configuration settings
 * \return Success or errors as defined by UART_ERR_CODES
 *
 */
int initUartPort(UARTConfig * prtInf)
{
	unsigned char * prtSelReg = NULL;
	switch(prtInf->portNum)
	{
#ifdef __MSP430_HAS_PORT1_R__
		case 1:
			prtSelReg = (unsigned char *)&P1SEL;
			break;
#endif
#ifdef __MSP430_HAS_PORT2_R__
		case 2:
			prtSelReg = (unsigned char *)&P2SEL;
			break;
#endif
#ifdef __MSP430_HAS_PORT3_R__
		case 3:
			prtSelReg = (unsigned char *)&P3SEL;
			break;
#endif
#ifdef __MSP430_HAS_PORT4_R__
		case 4:
			prtSelReg = (unsigned char *)&P4SEL;
			break;
#endif
#ifdef __MSP430_HAS_PORT5_R__
		case 5:
			prtSelReg = (unsigned char *)&P5SEL;
			break;
#endif
#ifdef __MSP430_HAS_PORT6_R__
		case 6:
			prtSelReg = (unsigned char *)&P6SEL;
			break;
#endif
#ifdef __MSP430_HAS_PORT7_R__
		case 7:
			prtSelReg = (unsigned char *)&P7SEL;
			break;
#endif
#ifdef __MSP430_HAS_PORT8_R__
		case 8:
			prtSelReg = (unsigned char *)&P8SEL;
			break;
#endif
#ifdef __MSP430_HAS_PORT9_R__
		case 9:
			prtSelReg = (unsigned char *)&P9SEL;
			break;
#endif
		default:
			prtSelReg = NULL;
			break;
	}

	if(prtSelReg == NULL)
	{
		return UART_BAD_PORT_SELECTED;
	}

	// Configure Port to use UART Module by setting the corresponding bits on the PxSEL
	// register.
	*prtSelReg |=  (BIT0 << prtInf->RxPinNum) | (BIT0 << prtInf->TxPinNum);

	return UART_SUCCESS;
}


/*!
 * \brief Configures the UART Pins and Module for communications
 *
 * This function accepts a UARTConfig instance and initializes
 * the UART module appropriately. See UARTConfig for more info.
 *
 *
 * @param prtInf is UARTConfig instance with the configuration settings
 * @param confRegs is a pointer to a struct holding the configuration register
 * \return Success or errors as defined by UART_ERR_CODES
 *
 */
#if defined(__MSP430_HAS_USCI__) || defined(__MSP430_HAS_USCI_A0__) || defined(__MSP430_HAS_USCI_A1__) || defined(__MSP430_HAS_USCI_A2__)
int configUSCIUart(UARTConfig * prtInf,USCIUARTRegs * confRegs)
{
	initUartPort(prtInf);

	// Configure the pointers to the right registers
	switch(prtInf->moduleName)
	{
#if defined(__MSP430_HAS_USCI_A0__) || defined(__MSP430_HAS_USCI__)
		case USCI_A0:
			confRegs->CTL0_REG = (unsigned char *)&UCA0CTL0;
			confRegs->CTL1_REG = (unsigned char *)&UCA0CTL1;
			confRegs->MCTL_REG = (unsigned char *)&UCA0MCTL;
			confRegs->BR0_REG  = (unsigned char *)&UCA0BR0;
			confRegs->BR1_REG  = (unsigned char *)&UCA0BR1;
			confRegs->IE_REG  = (unsigned char *)&UCA0IE;
			confRegs->RX_BUF = (unsigned char *)&UCA0RXBUF;
			confRegs->TX_BUF = (unsigned char *)&UCA0TXBUF;
			confRegs->IFG_REG = (unsigned char *)&UCA0IFG;
			break;
#endif
#ifdef __MSP430_HAS_USCI_A1__
		case USCI_A1:
			confRegs->CTL0_REG = (unsigned char *)&UCA1CTL0;
			confRegs->CTL1_REG = (unsigned char *)&UCA1CTL1;
			confRegs->MCTL_REG = (unsigned char *)&UCA1MCTL;
			confRegs->BR0_REG  = (unsigned char *)&UCA1BR0;
			confRegs->BR1_REG  = (unsigned char *)&UCA1BR1;
			confRegs->IE_REG  =  (unsigned char *)&UCA1IE;
			confRegs->RX_BUF =   (unsigned char *)&UCA1RXBUF;
			confRegs->TX_BUF =   (unsigned char *)&UCA1TXBUF;
			confRegs->IFG_REG =  (unsigned char *)&UCA1IFG;
			break;
#endif
#ifdef __MSP430_HAS_USCI_A2__
		case USCI_A2:
			confRegs->CTL0_REG = (unsigned char *)&UCA2CTL0;
			confRegs->CTL1_REG = (unsigned char *)&UCA2CTL1;
			confRegs->MCTL_REG = (unsigned char *)&UCA2MCTL;
			confRegs->BR0_REG  = (unsigned char *)&UCA2BR0;
			confRegs->BR1_REG  = (unsigned char *)&UCA2BR1;
			confRegs->IE_REG  = (unsigned char *)&UCA2IE;
			confRegs->RX_BUF = (unsigned char *)&UCA2RXBUF;
			confRegs->TX_BUF = (unsigned char *)&UCA2TXBUF;
			confRegs->IFG_REG = (unsigned char *)&UCA2IFG;
			break;
#endif
	}

	// Place Module in reset to allow us to modify its bits
	*confRegs->CTL1_REG |= UCSWRST;

	// Configure UART Settings
	if(prtInf->parity == UART_PARITY_EVEN)
	{
		*confRegs->CTL0_REG |= UCPEN | UCPAR;
	}
	else if(prtInf->parity == UART_PARITY_ODD)
	{
		*confRegs->CTL0_REG |= UCPEN;
	}

	if(prtInf->databits == 7)
	{
		*confRegs->CTL0_REG |= UC7BIT;
	}

	if(prtInf->stopbits == 2)
	{
		*confRegs->CTL0_REG |= UCSPB;
	}

	// Configure clock source

	// Clear clock source bits, then set the proper ones;
	*confRegs->CTL1_REG &= ~(UCSSEL1 | UCSSEL0);

	switch(prtInf->clkSrc)
	{
		case UART_CLK_SRC_ACLK:
			*confRegs->CTL1_REG |= (UCSSEL0);
			break;
		case UART_CLK_SRC_SMCLK:
			*confRegs->CTL1_REG |= (UCSSEL1);
			break;
	}

	// Set the baudrate dividers and modulation
	unsigned int N_div;
	N_div = prtInf->clkRate / prtInf->baudRate;

	float N_div_f;
	N_div_f = (float)prtInf->clkRate / (float)prtInf->baudRate;

	if(N_div >= 16)
	{
		// We can use Oversampling mode
		N_div /= 16;
		*confRegs->BR0_REG = (N_div & 0x00FF);
		*confRegs->BR1_REG = ((N_div & 0xFF00) >> 8);

		N_div_f /= 16.0;
		*confRegs->MCTL_REG = (unsigned char)(((N_div_f) - round(N_div_f))*16.0f) << 4; // Set BRF
		*confRegs->MCTL_REG |= UCOS16; // Enable Oversampling Mode
	}
	else
	{
		// We must use the Low Frequency mode
		*confRegs->BR0_REG = (N_div & 0x00FF);
		*confRegs->BR1_REG = ((N_div & 0xFF00) >> 8);

		*confRegs->MCTL_REG = (unsigned char)((N_div_f - round(N_div_f))*8.0f) << 1; // Set BRS
	}

	// Take Module out of reset
	*confRegs->CTL1_REG &= ~UCSWRST;

	// Sets the pointer to the register configuration so we don't
	// have to keep passing it around and can pass only the UART configuration
	prtInf->usciRegs = confRegs;

	initBufferDefaults(prtInf);

	// Assign pointer to port information to the array so it can be accessed later
	prtInfList[prtInf->moduleName] = prtInf;

	return UART_SUCCESS;
}
#endif

/*!
 * \brief Initializes RX and TX buffers pointers
 *
 *
 * @param None.
 * \return None.
 *
 */
void initBufferDefaults(UARTConfig * prtInf)
{
	prtInf->txBuf = NULL;
	prtInf->txBufLen = 0;

	prtInf->rxBuf = NULL;
	prtInf->rxBufLen = 0;

	prtInf->rxBytesReceived = 0;
	prtInf->txBytesToSend = 0;
	prtInf->txBufCtr = 0;
}

/*!
 * \brief Configures the UART Pins and Module for communications
 *
 * This function accepts a UARTConfig instance and initializes
 * the UART module appropriately. See UARTConfig for more info.
 *
 *
 * @param prtInf is UARTConfig instance with the configuration settings
 *
 * \return Success or errors as defined by UART_ERR_CODES
 *
 */
#if defined(__MSP430_HAS_UART0__) || defined(__MSP430_HAS_UART1__)
int configUSARTUart(UARTConfig * prtInf, USARTUARTRegs * confRegs)
{
	initUartPort(prtInf);

	// Configure the pointers to the right registers
	switch(prtInf->moduleName)
	{
#ifdef __MSP430_HAS_UART0__
		case USART_0:
			*confRegs->ME_REG =     ME1;
			*confRegs->U0CTL_REG =  UCTL0;
			*confRegs->UTCLT0_REG = UTCTL0;
			*confRegs->UBR0_REG =   UBR00;
			*confRegs->UBR1_REG =   UBR10;
			*confRegs->UMCTL_REG = UMCTL0;
			*confRegs->IE_REG =     IE1;
			*confRegs->RX_BUF =     RXBUF0;
			*confRegs->TX_BUF =     TXBUF0;
			*confRegs->IFG_REG =    IFG1 ;
			confRegs->TXIFGFlag =  UTXIFG0;
			confRegs->RXIFGFlag =  URXIFG0;
			confRegs->TXIE = UTXIE0;
			confRegs->RXIE = URXIE0;
			break;
#endif

#ifdef __MSP430_HAS_UART1__
		case USART_1:
			*confRegs->ME_REG =     ME2;
			*confRegs->U0CTL_REG =  UCTL1;
			*confRegs->UTCLT0_REG = UTCTL1;
			*confRegs->UBR0_REG =   UBR01;
			*confRegs->UBR1_REG =   UBR11;
			*confRegs->UMCTL_REG = UMCTL1;
			*confRegs->IE_REG =     IE2;
			*confRegs->RX_BUF =     RXBUF1;
			*confRegs->TX_BUF =     TXBUF1;
			confRegs->TXIFGFlag =  UTXIFG1;
			confRegs->RXIFGFlag =  URXIFG1;
			confRegs->TXIE = UTXIE1;
			confRegs->RXIE = URXIE1;

			break;
#endif
	}

	// Place Module in reset to allow us to modify its bits
	*confRegs->U0CTL_REG |= SWRST;

	// Configure UART Settings
	if(prtInf->parity == UART_PARITY_EVEN)
	{
		*confRegs->U0CTL_REG |= PENA;
		*confRegs->U0CTL_REG |= PEV;
	}
	else if(prtInf->parity == UART_PARITY_ODD)
	{
		*confRegs->U0CTL_REG |= PENA;
		*confRegs->U0CTL_REG &= ~PEV;
	}
	else if(prtInf->parity == UART_PARITY_NONE)
	{
		*confRegs->U0CTL_REG &= ~PENA;
		*confRegs->U0CTL_REG &= ~PEV;
	}

	if(prtInf->databits == 7)
	{
		*confRegs->U0CTL_REG &= ~CHAR;
	}
	else if(prtInf->databits == 8)
	{
		*confRegs->U0CTL_REG |= CHAR;
	}

	if(prtInf->stopbits == 1)
	{
		*confRegs->U0CTL_REG &= ~SPB;
	}
	else if(prtInf->stopbits == 2)
	{
		*confRegs->U0CTL_REG |= SPB;
	}

	// Configure clock source

	// Clear clock source bits, then set the proper ones;
	*confRegs->UTCLT0_REG &= ~(SSEL1 | SSEL0);

	switch(prtInf->clkSrc)
	{
		case UART_CLK_SRC_ACLK:
			*confRegs->UTCLT0_REG |= (SSEL0);
			break;
		case UART_CLK_SRC_SMCLK:
			*confRegs->UTCLT0_REG |= (SSEL1);
			break;
	}

	// Set the baudrate dividers and modulation
	unsigned int N_div;
	N_div = prtInf->clkRate / prtInf->baudRate;


	*confRegs->UBR0_REG = (N_div & 0x00FF);
	*confRegs->UBR1_REG = ((N_div & 0xFF00) >> 8);

	// Modulation currently set to 0. Needs proper handling
	*confRegs->UMCTL_REG = 0;

	// Take Module out of reset
	*confRegs->U0CTL_REG &= ~SWRST;


	prtInf->usartRegs = confRegs;

	initBufferDefaults(prtInf);

	// Assign pointer to port information to the array so it can be accessed later
	prtInfList[prtInf->moduleName] = prtInf;

	return UART_SUCCESS;
}

#endif

/*!
 * \brief Returns the number of bytes in the RX Buffer
 *
 *
 * @param prtInf is a pointer to the UART configuration
 *
 * \return Number of bytes in RX Buffer
 *
 */
int numUartBytesReceived(UARTConfig * prtInf)
{
	return prtInf->rxBytesReceived;
}

/*!
 * \brief Returns a pointer to the RX Buffer
 *
 *
 * @param prtInf is a pointer to the UART configuration
 *
 * \return pointer to the RX buffer of the UART
 *
 */
unsigned char * getUartRxBufferData(UARTConfig * prtInf)
{
	return prtInf->rxBuf;
}


/*!
 * \brief Sends len number of bytes from the buffer using the specified
 * UART, but does so by blocking.
 *
 * This function is blocking, although interrupts may trigger during its
 * execution.
 *
 * @param prtInf is a pointer to the UART configuration
 * @param buf is a pointer to the buffer containing the bytes to be sent.
 * @param len is an integer containing the number of bytes to send.
 *
 * \return Success or errors as defined by UART_ERR_CODES
 *
 */
int uartSendDataBlocking(UARTConfig * prtInf,unsigned char * buf, int len)
{
	int i = 0;
	for(i = 0; i < len; i++)
	{
		if(prtInf->moduleName == USCI_A0|| prtInf->moduleName == USCI_A1 || prtInf->moduleName == USCI_A2)
		{
#if defined(__MSP430_HAS_USCI__) || defined(__MSP430_HAS_USCI_A0__) || defined(__MSP430_HAS_USCI_A1__) || defined(__MSP430_HAS_USCI_A2__)
			while(!( *prtInf->usciRegs->IFG_REG & UCTXIFG));
			*prtInf->usciRegs->TX_BUF = buf[i];
#endif
		}

		if(prtInf->moduleName == USART_0|| prtInf->moduleName == USART_1)
		{
#if defined(__MSP430_HAS_UART0__) || defined(__MSP430_HAS_UART1__)
			while(!(*prtInf->usartRegs->IFG_REG & prtInf->usartRegs->TXIFGFlag));
			*prtInf->usciRegs->TX_BUF = buf[i];
#endif
		}
	}

	return UART_SUCCESS;
}


/*!
 * \brief Sends a string over the specified UART
 *
 * This function is blocking, although interrupts may trigger during its
 * execution.
 *
 * @param prtInf is a pointer to the UART configuration
 * @param uartId is the unique UART identifier, usually provided by the call to configUart.
 * @param string is a pointer to the string to be sent
 *
 * \par The string provided to the function must be null terminated.
 *
 * \return 0 if successful , -1 if failed
 *
 */
int uartSendStringBlocking(UARTConfig * prtInf,char * string)
{
	int res = 0;
	res = uartSendDataBlocking(prtInf,(unsigned char*)string, strlen(string));

	return res;
}

/*!
 * \brief Sets the UART TX Buffer
 *
 *
 * @param prtInf is UARTConfig instance with the configuration settings
 * @param buf is a pointer to a user provided buffer to be used by the UART driver
 * @param bufLen the length of the buffer provided to the UART driver
 * \return Success or errors as defined by UART_ERR_CODES
 *
 */
void setUartTxBuffer(UARTConfig * prtInf, unsigned char * buf, int bufLen)
{
	prtInf->txBuf = buf;
	prtInf->txBufLen = bufLen;

	int i = 0;
	for(i = 0; i < bufLen; i++)
	{
		buf[i] = 0;
	}
}

/*!
 * \brief Sets the UART RX Buffer
 *
 *
 * @param prtInf is UARTConfig instance with the configuration settings
 * @param buf is a pointer to a user provided buffer to be used by the UART driver
 * @param bufLen the length of the buffer provided to the UART driver
 * \return Success or errors as defined by UART_ERR_CODES
 *
 */
void setUartRxBuffer(UARTConfig * prtInf, unsigned char * buf, int bufLen)
{
	prtInf->rxBuf = buf;
	prtInf->rxBufLen = bufLen;

	int i = 0;
	for(i = 0; i < bufLen; i++)
	{
		buf[i] = 0;
	}
}

/*!
 * \brief Sends len number of bytes from the buffer using the specified
 * UART using interrupt driven.
 *
 * TX Interrupts are enabled and each time that the UART TX Buffer is empty
 * and there is more data to send, data is sent. Once the byte is sent, another
 * interrupt is triggered, until all bytes in the buffer sent.
 *
 * @param prtInf is a pointer to the UART configuration
 * @param buf is a pointer to the buffer containing the bytes to be sent.
 * @param len is an integer containing the number of bytes to send.
 *
 * \return Success or errors as defined by UART_ERR_CODES
 *
 */
int uartSendDataInt(UARTConfig * prtInf,unsigned char * buf, int len)
{
	if(len > prtInf->txBufLen )
	{
		return UART_INSUFFICIENT_TX_BUF;
	}

	int i = 0;
	for(i = 0; i < len; i++)
	{
		prtInf->txBuf[i] = buf[i];
	}

#if defined(__MSP430_HAS_USCI__) || defined(__MSP430_HAS_USCI_A0__) || defined(__MSP430_HAS_USCI_A1__) || defined(__MSP430_HAS_USCI_A2__)
	// Send the first byte. Since UART interrupt is enabled, it will be called once the byte is sent and will
	// send the rest of the bytes
	if(prtInf->moduleName == USCI_A0 || prtInf->moduleName == USCI_A1 || prtInf->moduleName == USCI_A2)
	{
		prtInf->txBytesToSend = len;
		prtInf->txBufCtr = 0;

		// Enable TX IE
		*prtInf->usciRegs->IFG_REG &= ~UCTXIFG;
		*prtInf->usciRegs->IE_REG |= UCTXIE;

		// Trigger the TX IFG. This will cause the Interrupt Vector to be called
		// which will send the data one byte at a time at each interrupt trigger.
		*prtInf->usciRegs->IFG_REG |= UCTXIFG;
	}
#endif

#if defined(__MSP430_HAS_UART0__) || defined(__MSP430_HAS_UART1__)
	if(prtInf->moduleName == USART_0|| prtInf->moduleName == USART_1)
	{
		prtInf->txBytesToSend = len;
		prtInf->txBufCtr = 0;

		// Clear TX IFG and Enable TX IE
		*prtInf->usartRegs->IFG_REG &= ~ prtInf->usartRegs->TXIFGFlag;
		*prtInf->usartRegs->IE_REG |= prtInf->usartRegs->TXIE;

		// Trigger the TX IFG. This will cause the Interrupt Vector to be called
		// which will send the data one byte at a time at each interrupt trigger.
		*prtInf->usartRegs->IFG_REG |= prtInf->usartRegs->TXIFGFlag;

	}
#endif

	return UART_SUCCESS;
}

void enableUartRx(UARTConfig * prtInf)
{
#if defined(__MSP430_HAS_USCI__) || defined(__MSP430_HAS_USCI_A0__) || defined(__MSP430_HAS_USCI_A1__) || defined(__MSP430_HAS_USCI_A2__)
	if(prtInf->moduleName == USCI_A0|| prtInf->moduleName == USCI_A1 || prtInf->moduleName == USCI_A2)
	{
		// Enable RX IE
		*prtInf->usciRegs->IFG_REG &= ~UCRXIFG;
		*prtInf->usciRegs->IE_REG |= UCRXIE;
	}
#endif

#if defined(__MSP430_HAS_UART0__) || defined(__MSP430_HAS_UART1__)
	if(prtInf->moduleName == USART_0|| prtInf->moduleName == USART_1)
	{
		// Enable RX IE
		*prtInf->usartRegs->IFG_REG &= ~   prtInf->usartRegs->RXIFGFlag;
		*prtInf->usartRegs->IE_REG |= prtInf->usartRegs->RXIE;
	}
#endif
}

#if defined(__MSP430_HAS_UART0__)
// UART0 TX ISR
#pragma vector=USART0TX_VECTOR
__interrupt void usart0_tx (void)
{
	// Send data if the buffer has bytes to send
	if(prtInfList[USART_0]->txBytesToSend > 0)
	{
		*prtInfList[USART_0]->usciRegs->TX_BUF = prtInfList[USART_0]->txBuf[prtInfList[USART_0]->txBufCtr];
		prtInfList[USART_0]->txBufCtr++;

		// If we've sent all the bytes, set counter to 0 to stop the sending
		if(prtInfList[USART_0]->txBufCtr == prtInfList[USART_0]->txBytesToSend)
		{
		  prtInfList[USART_0]->txBufCtr = 0;
		}
	}
}


// UART0 RX ISR
#pragma vector=USART0RX_VECTOR
__interrupt void usart0_rx (void)
{
	 // Store received byte in RX Buffer
	prtInfList[USART_0]->rxBuf[prtInfList[USART_0]->rxBytesReceived] = *prtInfList[USART_0]->usciRegs->RX_BUF;
	prtInfList[USART_0]->rxBytesReceived++;

	// If the received bytes filled up the buffer, go back to beginning
	if(prtInfList[USART_0]->rxBytesReceived > prtInfList[USART_0]->rxBufLen)
	{
	  prtInfList[USART_0]->rxBytesReceived = 0;
	}
}

#endif

#if defined(__MSP430_HAS_UART1__)
// UART1 TX ISR
#pragma vector=USART1TX_VECTOR
__interrupt void usart1_tx (void)
{
	// Send data if the buffer has bytes to send
	if(prtInfList[USART_1]->txBytesToSend > 0)
	{
		*prtInfList[USART_1]->usciRegs->TX_BUF = prtInfList[USART_1]->txBuf[prtInfList[USART_1]->txBufCtr];
		prtInfList[USART_1]->txBufCtr++;

		// If we've sent all the bytes, set counter to 0 to stop the sending
		if(prtInfList[USART_1]->txBufCtr == prtInfList[USART_1]->txBytesToSend)
		{
		  prtInfList[USART_1]->txBufCtr = 0;
		}
	}
}


// UART1 RX ISR
#pragma vector=USART1RX_VECTOR
__interrupt void usart1_rx (void)
{
	 // Store received byte in RX Buffer
	prtInfList[USART_1]->rxBuf[prtInfList[USART_1]->rxBytesReceived] = *prtInfList[USART_1]->usciRegs->RX_BUF;
	prtInfList[USART_1]->rxBytesReceived++;

	// If the received bytes filled up the buffer, go back to beginning
	if(prtInfList[USART_1]->rxBytesReceived > prtInfList[USART_1]->rxBufLen)
	{
	  prtInfList[USART_1]->rxBytesReceived = 0;
	}
}

#endif

#if defined(__MSP430_HAS_USCI__) || defined(__MSP430_HAS_USCI_A0__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
	switch(__even_in_range(UCA0IV,4))
	{
	  case 0:break;                             // Vector 0 - no interrupt
	  case 2:                                   // Vector 2 - RXIFG
		  // Store received byte in RX Buffer
		  prtInfList[USCI_A0]->rxBuf[prtInfList[USCI_A0]->rxBytesReceived] = *prtInfList[USCI_A0]->usciRegs->RX_BUF;
		  prtInfList[USCI_A0]->rxBytesReceived++;

		  // If the received bytes filled up the buffer, go back to beginning
		  if(prtInfList[USCI_A0]->rxBytesReceived > prtInfList[USCI_A0]->rxBufLen)
		  {
			  prtInfList[USCI_A0]->rxBytesReceived = 0;
		  }
		break;
	  case 4:                                   // Vector 4 - TXIFG
		  // Send data if the buffer has bytes to send
		  if(prtInfList[USCI_A0]->txBytesToSend > 0)
		  {
			  *prtInfList[USCI_A0]->usciRegs->TX_BUF = prtInfList[USCI_A0]->txBuf[prtInfList[USCI_A0]->txBufCtr];
			  prtInfList[USCI_A0]->txBufCtr++;

			  // If we've sent all the bytes, set counter to 0 to stop the sending
			  if(prtInfList[USCI_A0]->txBufCtr == prtInfList[USCI_A0]->txBytesToSend)
			  {
				  prtInfList[USCI_A0]->txBufCtr = 0;

				  // Disable TX IE
				  *prtInfList[USCI_A0]->usciRegs->IE_REG &= ~UCTXIE;

				  // Clear TX IFG
				  *prtInfList[USCI_A0]->usciRegs->IFG_REG &= ~UCTXIFG;
			  }
		  }
		  break;
	  default: break;
	}
}
#endif

#if defined(__MSP430_HAS_USCI__) || defined(__MSP430_HAS_USCI_A1__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
	switch(__even_in_range(UCA1IV,4))
	{
	  case 0:break;                             // Vector 0 - no interrupt
	  case 2:                                   // Vector 2 - RXIFG
		  // Store received byte in RX Buffer
		  prtInfList[USCI_A1]->rxBuf[prtInfList[USCI_A1]->rxBytesReceived] = *prtInfList[USCI_A1]->usciRegs->RX_BUF;
		  prtInfList[USCI_A1]->rxBytesReceived++;

		  // If the received bytes filled up the buffer, go back to beginning
		  if(prtInfList[USCI_A1]->rxBytesReceived > prtInfList[USCI_A1]->rxBufLen)
		  {
			  prtInfList[USCI_A1]->rxBytesReceived = 0;
		  }
		break;
	  case 4:
		  // Send data if the buffer has bytes to send
		  if(prtInfList[USCI_A1]->txBytesToSend > 0)
		  {
			  *prtInfList[USCI_A1]->usciRegs->TX_BUF = prtInfList[USCI_A1]->txBuf[prtInfList[USCI_A1]->txBufCtr];
			  prtInfList[USCI_A1]->txBufCtr++;

			  // If we've sent all the bytes, set counter to 0 to stop the sending
			  if(prtInfList[USCI_A1]->txBufCtr == prtInfList[USCI_A1]->txBytesToSend)
			  {
				  prtInfList[USCI_A1]->txBufCtr = 0;

				  // Disable TX IE
				  *prtInfList[USCI_A1]->usciRegs->IE_REG &= ~UCTXIE;

				  // Clear TX IFG
				  *prtInfList[USCI_A1]->usciRegs->IFG_REG &= ~UCTXIFG;
			  }
		  }
		  break;                             // Vector 4 - TXIFG
	  default: break;
	}
}
#endif

#if defined(__MSP430_HAS_USCI__) || defined(__MSP430_HAS_USCI_A2__)
#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
{
	switch(__even_in_range(UCA2IV,4))
	{
	  case 0:break;                             // Vector 0 - no interrupt
	  case 2:                                   // Vector 2 - RXIFG
		  // Store received byte in RX Buffer
		  prtInfList[USCI_A2]->rxBuf[prtInfList[USCI_A2]->rxBytesReceived] = *prtInfList[USCI_A2]->usciRegs->RX_BUF;
		  prtInfList[USCI_A2]->rxBytesReceived++;

		  // If the received bytes filled up the buffer, go back to beginning
		  if(prtInfList[USCI_A2]->rxBytesReceived > prtInfList[USCI_A2]->rxBufLen)
		  {
			  prtInfList[USCI_A2]->rxBytesReceived = 0;
		  }
		break;
	  case 4:
		  if(prtInfList[USCI_A2]->txBytesToSend > 0)
		  {
			  *prtInfList[USCI_A2]->usciRegs->TX_BUF = prtInfList[USCI_A2]->txBuf[prtInfList[USCI_A2]->txBufCtr];
			  prtInfList[USCI_A2]->txBufCtr++;

			  // If we've sent all the bytes, set counter to 0 to stop the sending
			  if(prtInfList[USCI_A2]->txBufCtr == prtInfList[USCI_A2]->txBytesToSend)
			  {
				  prtInfList[USCI_A2]->txBufCtr = 0;

				  // Disable TX IE
				  *prtInfList[USCI_A2]->usciRegs->IE_REG &= ~UCTXIE;

				  // Clear TX IFG
				  *prtInfList[USCI_A2]->usciRegs->IFG_REG &= ~UCTXIFG;
			  }
		  }
		  break;                             // Vector 4 - TXIFG
	  default: break;
	}
}
#endif

/*!
 * \brief Reads bytes from the Rx buffer
 *
 *	Note that when bytes are read using this function, their count will be reset.
 *	Therefore, it is recommended to read all bytes of interest at once and buffer on
 *	the user side as necessary.
 *
 * @param prtInf is UARTConfig instance with the configuration settings
 * @param data is a pointer to a user provided buffer to which the data will be written
 * @param numBytesToRead is the number of bytes to read from the buffer
 * @param offset is the offset in the buffer to read. Default is 0 to start at beginning of RX buffer
 * \return number of bytes placed in the data buffer
 *
 */
int readRxBytes(UARTConfig * prtInf, unsigned char * data, int numBytesToRead, int offset)
{
	int bytes = 0;

	// Ensure we don't read past what we have in the buffer
	if(numBytesToRead+offset <= prtInf->rxBytesReceived )
	{
		bytes = numBytesToRead;
	}
	else if(offset < prtInf->rxBufLen)
	{
		// Since offset is valid, we provide all possible bytes until the end of the buffer
		bytes = prtInf->rxBytesReceived - offset;
	}
	else
	{
		return 0;
	}

	int i = 0;
	for(i = 0; i < bytes; i++)
	{
		data[i] = prtInf->rxBuf[offset+i];
	}

	// reset number of bytes available, regardless of how many bytes are left
	prtInf->rxBytesReceived = 0;

	return i;

}
