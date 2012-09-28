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
* File:    uart.h
* Summary: Definitions for MSP430 UART Driver
*
*/

#ifndef UART_H_
#define UART_H_

#ifndef NULL
#define NULL 0
#endif

#define PORT_1 1
#define PORT_2 2
#define PORT_3 3
#define PORT_4 4
#define PORT_5 5
#define PORT_6 6
#define PORT_7 7
#define PORT_8 8
#define PORT_9 9

#define PIN0 0
#define PIN1 1
#define PIN2 2
#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7

enum UART_ERR_CODES
{
	UART_SUCCESS = 0,
	UART_BAD_MODULE_NAME,
	UART_BAD_CLK_SOURCE,
	UART_INSUFFICIENT_TX_BUF,
	UART_INSUFFICIENT_RX_BUF,
	UART_BAD_PORT_SELECTED
};

typedef enum
{
	USCI_A0,  /**< USCI_A0 Module  */
	USCI_A1,  /**< USCI_A1 Module  */
	USCI_A2,  /**< USCI_A2 Module  */
	USART_0,  /**< USART_0 Module  */
	USART_1  /**< USART_1 Module  */

}UART_MODULE_NAMES;

typedef enum
{
	UART_CLK_SRC_ACLK,  /**< ACLK as source for UART baud rate generator  */
	UART_CLK_SRC_SMCLK  /**< SMCLK as source for UART baud rate generator  */

}UART_CLK_SRCS;

typedef enum
{
	UART_PARITY_NONE,  /**< No Parity  */
	UART_PARITY_EVEN,  /**< Even Parity  */
	UART_PARITY_ODD    /**< Odd Parity  */

}UART_PARITY;

/** @struct USCIUARTRegs
 *  @brief This structure contains pointers to the relevant
 *  		registers necessary to configure and use a USCI UART module.
 *
 */
typedef struct
{
	unsigned char * CTL0_REG;
	unsigned char * CTL1_REG;
	unsigned char * MCTL_REG;
	unsigned char * BR0_REG;
	unsigned char * BR1_REG;
	unsigned char * IE_REG;
	unsigned char * RX_BUF;
	unsigned char * TX_BUF;
	unsigned char * IFG_REG;
} USCIUARTRegs;

/** @struct USARTUARTRegs
 *  @brief This structure contains pointers to the relevant
 *  		registers necessary to configure and use a USART UART module.
 *
 */
typedef struct
{
	unsigned char * ME_REG;
	unsigned char * U0CTL_REG;
	unsigned char * UTCLT0_REG;
	unsigned char * UBR0_REG;
	unsigned char * UBR1_REG;
	unsigned char * UMCTL_REG;
	unsigned char * IE_REG;
	unsigned char * RX_BUF;
	unsigned char * TX_BUF;
	unsigned char * IFG_REG;
	unsigned char TXIFGFlag;
	unsigned char RXIFGFlag;
	unsigned char TXIE;
	unsigned char RXIE;
} USARTUARTRegs;

/** @struct UARTConfig
 *  @brief This struct contains all the configuration needed for
 *  	   UART operation.
 *
 */
typedef struct
{
	UART_MODULE_NAMES moduleName; /**< Module Name that specifies which module to use  */
	char portNum;                 /**< GPIO Port Number  */
	char TxPinNum;                /**< GPIO TX Pin Number  */
	char RxPinNum;                /**< GPIO RX Pin Number  */
	unsigned long clkRate;        /**< Clock rate of the clock used as source for module  */
	unsigned long baudRate;       /**< UART baud rate desired  */
	UART_CLK_SRCS clkSrc;         /**< Clock source used for UART module  */
	char databits;                /**< Number of data bits used for communications  */
	char stopbits;                /**< Number of stop bits used for communications  */
	UART_PARITY parity;           /**< Parity used for communications  */
	USCIUARTRegs * usciRegs;
	USARTUARTRegs * usartRegs;
	unsigned char * txBuf;
	unsigned char * rxBuf;
	int txBufLen;
	int rxBufLen;
	int rxBytesReceived;
	int txBytesToSend;
	int txBufCtr;
} UARTConfig;



/* Function Declarations */
int configUSCIUart(UARTConfig * prtInf,USCIUARTRegs * confRegs);
int configUSARTUart(UARTConfig * prtInf, USARTUARTRegs * confRegs);
int uartSendDataBlocking(UARTConfig * prtInf,unsigned char * buf, int len);
int uartSendStringBlocking(UARTConfig * prtInf,char * string);
int initUSCIUart(USCIUARTRegs * confRegs, UARTConfig * prtInf);
int initUartPort(UARTConfig * prtInf);
void initBufferDefaults(UARTConfig * prtInf);
void setUartTxBuffer(UARTConfig * prtInf, unsigned char * buf, int bufLen);
void setUartRxBuffer(UARTConfig * prtInf, unsigned char * buf, int bufLen);
void initUartDriver();
int uartSendDataInt(UARTConfig * prtInf,unsigned char * buf, int len);
void enableUartRx(UARTConfig * prtInf);
int numUartBytesReceived(UARTConfig * prtInf);
unsigned char * getUartRxBufferData(UARTConfig * prtInf);
int readRxBytes(UARTConfig * prtInf, unsigned char * data, int numBytesToRead, int offset);

#endif /* UART_H_ */
