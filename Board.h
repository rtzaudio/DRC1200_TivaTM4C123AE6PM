/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "DRC1200_TivaTM4C123AE6PMI.h"

#define Board_initGeneral           DRC1200_initGeneral
#define Board_initGPIO              DRC1200_initGPIO
#define Board_initI2C               DRC1200_initI2C
#define Board_initSPI               DRC1200_initSPI
#define Board_initUART              DRC1200_initUART
#define Board_initQEI				DRC1200_initQEI
#define Board_initWatchdog          DRC1200_initWatchdog

#define Board_readDIPSwitch			DRC1200_readDIPSwitch

#define Board_LED_ON                DRC1200_LED_ON
#define Board_LED_OFF               DRC1200_LED_OFF

#define Board_I2C0					DRC1200_I2C0			// I2C CAT24C16WI EPROM
#define Board_I2C1					DRC1200_I2C1			// I2C AT24CS01 EPROM/Serial#

#define Board_SPI_U7                DRC1200_SPI2			// SSI-2 : U7  SPI MCP23S17SO Remote Control Switches
#define Board_SPI_U10               DRC1200_SPI1			// SSI-1 : U10 SPI MCP23S17SO LED Driver and Transport Switches
#define Board_SPI_U11               DRC1200_SPI0			// SSI-0 : U11 SPI MCP23S17SO LED Drivers

#define Board_UART_RS422            DRC1200_UART0			// RS-422 driver on UART0

#define Board_GPIO_LED1				DRC1200_GPIO_LED1		// LED1 on display card
#define Board_GPIO_LED2				DRC1200_GPIO_LED2		// LED2 on display card
#define Board_GPIO_JOGSW			DRC1200_GPIO_JOGSW		// Jog wheel encoder push switch
#define Board_GPIO_RS422_RE			DRC1200_GPIO_RS422_RE	// RS422 Receiver Enable
#define Board_GPIO_RS422_DE			DRC1200_GPIO_RS422_DE	// RS422 Transmitter Enable

#define Board_GPIO_U7_INTA			DRC1200_GPIO_U7_INTA	// MCP23S17T I/O Expander Interrupt
#define Board_GPIO_U7_INTB			DRC1200_GPIO_U7_INTB	// MCP23S17T I/O Expander Interrupt
#define Board_GPIO_U10_INTA			DRC1200_GPIO_U10_INTA	// MCP23S17T I/O Expander Interrupt
#define Board_GPIO_U10_INTB			DRC1200_GPIO_U10_INTB	// MCP23S17T I/O Expander Interrupt

#define Board_GPIO_U7_CS            DRC1200_GPIO_U7_CS      // MCP23S17T U7 CS
#define Board_GPIO_U10_CS           DRC1200_GPIO_U10_CS     // MCP23S17T U10 CS
#define Board_GPIO_U11_CS           DRC1200_GPIO_U11_CS     // MCP23S17T U11 CS

#define Board_WATCHDOG0             DRC1200_WATCHDOG0

/* Board specific I2C EPROM addresses */
#define Board_AT24CS01_EPROM_ADDR   DRC1200_AT24CS01_EPROM_ADDR
#define Board_AT24CS01_SERIAL_ADDR	DRC1200_AT24CS01_SERIAL_ADDR

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
