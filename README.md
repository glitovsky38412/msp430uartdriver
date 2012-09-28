msp430uartdriver
================

MSP430 UART Driver

This is a generic MSP430 UART Driver

===============================
Gustavo Litovsky
www.glitovsky.com
gustavo@glitovsky.com
===============================

9/26/2012

Summary:
This folder contains a UART Driver that allows simple
configuration and use of the UART of an MSP430. At the moment
only the MSP430F5438 and similar devices are supported, but the code
can be easily augmented to support other devices.

List of tested platforms:
MSP-EXP430F5438A Experimenter Board with MSP430F5438

Files:
main.c - main example file running a simple UART application
uart.h - definitions for UART driver
uart.c - implementation of UART Driver


How to use in your own application:
1) Copy uart.c and uart.h to your own application
2) Call the initialization and configuration functions in the same order as in main.c,
changing parameters as needed. uart.c is heavily commented to make its use simple.
3) call the send and read functions as needed for your application

