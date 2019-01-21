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
/** ============================================================================
 *  @file       DRC1200.h
 *
 *  @brief      DRC1200 Board Specific APIs
 *
 *  The DRC1200 header file should be included in an application as follows:
 *  @code
 *  #include <DRC1200.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef __DRC1200_TM4C123AE6PMI_H
#define __DRC1200_TM4C123AE6PMI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/GPIO.h>

/* LEDs on DRC1200 are active high. */
#define DRC1200_LED_OFF ( 0)
#define DRC1200_LED_ON  (~0)

#define PIN_LOW			( 0)
#define PIN_HIGH		(~0)

/*******************************************************************************
 * MCP23S17 Register Addresses (IOCON.BANK = 0)
 ******************************************************************************/

#define MCP_IODIRA      0x00
#define MCP_IODIRB      0x01
#define MCP_IOPOLA      0x02
#define MCP_IOPOLB      0x03
#define MCP_GPINTENA    0x04
#define MCP_GPINTENB    0x05
#define MCP_DEFVALA     0x06
#define MCP_DEFVALB     0x07
#define MCP_INTCONA     0x08
#define MCP_INTCONB     0x09
#define MCP_IOCONA      0x0A
#define MCP_IOCONB      0x0B
#define MCP_GPPUA       0x0C
#define MCP_GPPUB       0x0D
#define MCP_INTFA       0x0E
#define MCP_INTFB       0x0F
#define MCP_INTCAPA     0x10
#define MCP_INTCAPB     0x11
#define MCP_GPIOA       0x12
#define MCP_GPIOB       0x13
#define MCP_OLATA       0x14
#define MCP_OLATB       0x15

/* IOCON Configuration Register Bits */
#define C_INTPOL        0x02	/* INT output 1=Active-high, 0=Active-low. */
#define C_ODR           0x04	/* INT pin as an open-drain output         */
#define C_HAEN          0x08	/* Hardware address enable (N/A for I2C)   */
#define C_DISSLW        0x10	/* Slew rate disable bit                   */
#define C_SEQOP         0x20	/* Disable address pointer auto-increment  */
#define C_MIRROR        0x40	/* INT A/B pins mirrored                   */
#define C_BANK          0x80	/* port registers are in different banks   */

/*******************************************************************************
 * MCP23S17 I/O Expander Bit Definitions
 ******************************************************************************/

/*
 * U7 is for transport control switches and button LEDs.
 * 5-bits for output LED's and 5-bits for pushbutton switch inputs.
 */

/* U7 PORT-A (Output) Transport Control Button LED's */
#define L_REC			0x01			// REC button LED
#define L_PLAY			0x02			// PLAY button LED
#define L_REW			0x04			// REW button LED
#define L_FWD			0x08			// FWD button LED
#define L_STOP			0x10			// STOP button LED

/* U7 PORT-B (Input) Transport Push Button Switch Bits */
#define SW_REC			0x01			// REC button switch
#define SW_PLAY			0x02			// PLAY button switch
#define SW_REW			0x04			// REW button switch
#define SW_FWD			0x08			// FWD button switch
#define SW_STOP			0x10			// STOP button switch

/*
 * U11 is two 8-bit OUTPUT ports for driving button LED's.
 */

/* U11 PORT-A (Output) LED Bits */
#define L_LOC1			(0x01 << 0)		// LOC1 button LED
#define L_LOC2			(0x02 << 0)		// LOC2 button LED
#define L_LOC3			(0x04 << 0)		// LOC3 button LED
#define L_LOC4			(0x08 << 0)		// LOC4 button LED
#define L_LOC5			(0x10 << 0)		// LOC5 button LED
#define L_LOC6			(0x20 << 0)		// LOC6 button LED
#define L_LOC7			(0x40 << 0)		// LOC7 button LED
#define L_LOC8			(0x80 << 0)		// LOC8 button LED

/* U11 PORT-B (Output) LED Bits */
#define L_LOC0			(0x01 << 8)		// LOC0 button LED
#define L_LOC9			(0x02 << 8)		// LOC9 button LED
#define L_MENU			(0x04 << 8)		// SET button LED
#define L_EDIT			(0x08 << 8)		// ESC button LED
#define L_STORE			(0x10 << 8)		// PREV button LED
#define L_ALT			(0x20 << 8)		// MENU button LED
#define L_AUTO			(0x40 << 8)		// NEXT button LED
#define L_CUE			(0x80 << 8)		// EDIT button LED

#define L_LOC_MASK      (L_LOC1|L_LOC2|L_LOC3| L_LOC4| L_LOC5| \
                         L_LOC6|L_LOC7| L_LOC8|L_LOC9|L_LOC0)

/*
 * U10 is two 8-bit INPUT ports for reading button switches.
 */

/* U10 PORT-A (Input) Pushbutton Switch Bits */
#define SW_LOC1			(0x01 << 0)		// LOC1 button switch
#define SW_LOC2			(0x02 << 0)		// LOC2 button switch
#define SW_LOC3			(0x04 << 0)		// LOC3 button switch
#define SW_LOC4			(0x08 << 0)		// LOC4 button switch
#define SW_LOC5			(0x10 << 0)		// LOC5 button switch
#define SW_LOC6			(0x20 << 0)		// LOC6 button switch
#define SW_LOC7			(0x40 << 0)		// LOC7 button switch
#define SW_LOC8			(0x80 << 0)		// LOC8 button switch

/* U10 PORT-B (Input) Pushbutton Switch Bits */
#define SW_LOC0		    (0x01 << 8)		// LOC0 button switch
#define SW_LOC9			(0x02 << 8)		// LOC9 button switch
#define SW_MENU			(0x04 << 8)		// MENU button switch
#define SW_EDIT			(0x08 << 8)		// EDIT button switch
#define SW_STORE		(0x10 << 8)		// STORE button switch
#define SW_ALT 			(0x20 << 8)		// ALT button switch
#define SW_AUTO			(0x40 << 8)		// AUTO button switch
#define SW_CUE			(0x80 << 8)		// CUE button switch

#define SW_LOC_MASK     (SW_LOC1|SW_LOC2|SW_LOC3|SW_LOC4|SW_LOC5| \
                         SW_LOC6|SW_LOC7|SW_LOC8|SW_LOC9|SW_LOC0)

/* Processor GPIO Mapped Config DIP Switches. The lower
 * 4-bits of PORT-G are inputs from config DIP switch.
 */

#define DIP_SW1			0x01        	// config DIP switch 1
#define DIP_SW2        	0x02        	// config DIP switch 2
#define DIP_SW3        	0x04        	// config DIP switch 3
#define DIP_SW4        	0x08        	// config DIP switch 4

#define DIP_SW_MASK		(DIP_SW1|DIP_SW2|DIP_SW3|DIP_SW4)

/*******************************************************************************
 * I2C EPROM Address Defines
 ******************************************************************************/

#define DRC1200_AT24CS01_EPROM_ADDR		(0xA0 >> 1)
#define DRC1200_AT24CS01_SERIAL_ADDR	(0xB0 >> 1)

/*******************************************************************************
 * Tiva Quadrature Encoder for Shuttle Jog Wheel
 ******************************************************************************/

/* Encoder is C14N32P-A3 from CUI Stack with 32 CPR */
#define JOGWHEEL_BASE           QEI1_BASE
#define JOGWHEEL_PPR            32
#define JOGWHEEL_EDGES_PER_REV  (JOGWHEEL_PPR * 4)
#define JOGWHEEL_TIMER_PERIOD   80000000

/*!
 *  @def    DRC1200_GPIOName
 *  @brief  Enum of LED names on the DRC1200 dev board
 */
typedef enum DRC1200_GPIOName {
	DRC1200_GPIO_U10_INTA,		/* 00 MCP23S17T I/O Expander Interrupt */
    DRC1200_GPIO_U10_INTB,		/* 01 MCP23S17T I/O Expander Interrupt */
    DRC1200_GPIO_U7_INTA,		/* 02 MCP23S17T I/O Expander Interrupt */
    DRC1200_GPIO_U7_INTB,		/* 03 MCP23S17T I/O Expander Interrupt */
	DRC1200_GPIO_JOGSW,			/* 04 Jog wheel encoder push switch */
	DRC1200_GPIO_DIP_SW1,       /* 05 config DIP switch 1 */
	DRC1200_GPIO_DIP_SW2,       /* 06 config DIP switch 2 */
	DRC1200_GPIO_DIP_SW3,       /* 07 config DIP switch 3 */
	DRC1200_GPIO_DIP_SW4,       /* 08 config DIP switch 4 */
	DRC1200_GPIO_LED1,			/* 09 LED1 on display card */
	DRC1200_GPIO_LED2,			/* 10 LED2 on display card */
	DRC1200_GPIO_RS422_RE,		/* 11 RS422 Receiver Enable */
	DRC1200_GPIO_RS422_DE,		/* 12 RS422 Transmitter Enable */
    DRC1200_GPIO_U11_CS,        /* 13 MCP23S17T U11 CS (SSI0FSS) */
    DRC1200_GPIO_U10_CS,        /* 14 MCP23S17T U10 CS (SSI1FSS) */
    DRC1200_GPIO_U7_CS,         /* 15 MCP23S17T U7 CS  (SSI2FSS) */
            
    DRC1200_GPIOCOUNT
} DRC1200_GPIOName;

/*!
 *  @def    DRC1200_I2CName
 *  @brief  Enum of I2C names on the DRC1200 dev board
 */
typedef enum DRC1200_I2CName {
    DRC1200_I2C0 = 0,
    DRC1200_I2C1,

    DRC1200_I2CCOUNT
} DRC1200_I2CName;

/*!
 *  @def    DRC1200_SPIName
 *  @brief  Enum of SPI names on the DRC1200 dev board
 */
typedef enum DRC1200_SPIName {
    DRC1200_SPI0 = 0,
    DRC1200_SPI1,
    DRC1200_SPI2,

    DRC1200_SPICOUNT
} DRC1200_SPIName;

/*!
 *  @def    DRC1200_UARTName
 *  @brief  Enum of UARTs on the DRC1200 dev board
 */
typedef enum DRC1200_UARTName {
    DRC1200_UART0 = 0,

    DRC1200_UARTCOUNT
} DRC1200_UARTName;

/*
 *  @def    DRC1200_WatchdogName
 *  @brief  Enum of Watchdogs on the DRC1200 dev board
 */
typedef enum DRC1200_WatchdogName {
    DRC1200_WATCHDOG0 = 0,

    DRC1200_WATCHDOGCOUNT
} DRC1200_WatchdogName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings. This include
 *     - Enable clock sources for peripherals
 */
extern void DRC1200_initGeneral(void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_config
 *  variable.
 */
extern void DRC1200_initGPIO(void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern void DRC1200_initI2C(void);

/*!
 *  @brief  Initialize board specific PWM settings
 *
 *  This function initializes the board specific PWM settings and then calls
 *  the PWM_init API to initialize the PWM module.
 *
 *  The PWM peripherals controlled by the PWM module are determined by the
 *  PWM_config variable.
 */
extern void DRC1200_initPWM(void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern void DRC1200_initSPI(void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern void DRC1200_initUART(void);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern void DRC1200_initWatchdog(void);

extern void DRC1200_initQEI(void);
extern uint32_t DRC1200_readDIPSwitch(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRC1200_TM4C123AE6PMI_H */
