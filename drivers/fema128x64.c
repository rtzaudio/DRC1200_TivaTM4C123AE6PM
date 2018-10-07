//*****************************************************************************
//
// fema128x64x1.c - Display driver for the FEMA GM12864F-24-O3CY
//                  128 x 64 mono OLED display with a SSD1309 controller.
//                  This driver uses the SSI interface to the controller.
//
// Port to TI RTOS and modifications by Robert E. Starr, Jr.
//
//
// Copyright (c) 2011-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup display_api
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>
#include <inc/hw_types.h>
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"

#include <grlib/grlib.h>

#include "fema128x64.h"

//*****************************************************************************
// This really should be in grlib, but it's our custom GR driver and we
// format the grlib offscreen data as the display memory expects to see it.
// Since it's only used by this display driver only, we just declare
// the offscreen init function here.
//*****************************************************************************
extern void GrOffScreenMonoInit(tDisplay *psDisplay, uint8_t *pui8Image,
                                int32_t i32Width, int32_t i32Height);

//*****************************************************************************
// Defines the SSI and GPIO peripherals that are used for this display.
//*****************************************************************************
#define DISPLAY_SSI_PERIPH          SYSCTL_PERIPH_SSI3
#define DISPLAY_SSI_GPIO_PERIPH     SYSCTL_PERIPH_GPIOD
#define DISPLAY_RST_GPIO_PERIPH     SYSCTL_PERIPH_GPIOD

//*****************************************************************************
// Defines the GPIO pin configuration macros for the pins that are used for
// the SSI function.
//*****************************************************************************
#define DISPLAY_PINCFG_SSICLK       GPIO_PD0_SSI3CLK
#define DISPLAY_PINCFG_SSIFSS       GPIO_PD1_SSI3FSS
#define DISPLAY_PINCFG_SSITX        GPIO_PD3_SSI3TX

//*****************************************************************************
// Defines the port and pins for the SSI peripheral.
//*****************************************************************************
#define DISPLAY_SSI_PORT            GPIO_PORTD_BASE
#define DISPLAY_SSI_PINS            (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3)

//*****************************************************************************
// Defines the port and pins for the display voltage enable signal.
//*****************************************************************************
#define DISPLAY_PWR_PORT            GPIO_PORTD_BASE
#define DISPLAY_PWR_PIN             GPIO_PIN_7

//*****************************************************************************
// Defines the port and pins for the display reset signal.
//*****************************************************************************
#define DISPLAY_RST_PORT            GPIO_PORTD_BASE
#define DISPLAY_RST_PIN             GPIO_PIN_6

//*****************************************************************************
// Defines the port and pins for the display Data/Command (D/C) signal.
//*****************************************************************************
#define DISPLAY_D_C_PORT            GPIO_PORTD_BASE
#define DISPLAY_D_C_PIN             GPIO_PIN_5

//*****************************************************************************
// Defines the SSI peripheral used and the data speed.
//*****************************************************************************
#define DISPLAY_SSI_BASE            SSI3_BASE
#define DISPLAY_SSI_CLOCK           800000			// 800kHz clock speed

//*****************************************************************************
// OLED Controller Constants
//*****************************************************************************

#define	BRIGHTNESS	    0xBF

//*****************************************************************************
// The graphics library display structure for the FEMA OLED display.
//*****************************************************************************

tDisplay g_FEMA128x64;

static uint32_t g_ulClockMS = 0;

//*****************************************************************************
// The actual offscreen display buffer memory. The screen buffer is followed
// by two extra 32 bit words the contain the button LED state flags.
//*****************************************************************************

unsigned char g_ucScreenBuffer[SCREEN_BUFSIZE];

//*****************************************************************************
//
//! Get the size of the offscreen video buffer memory.
//!
//! \return Buffer size in bytes.
//
//*****************************************************************************

unsigned int SSD1309GetScreenBufferSize(void)
{
	return OLED_BUFSIZE;
}

//*****************************************************************************
//
//! Get the size of the offscreen video buffer memory.
//!
//! \return Buffer size in bytes.
//
//*****************************************************************************

unsigned char* SSD1309GetScreenBuffer(void)
{
	return &g_ucScreenBuffer[0];
}

//*****************************************************************************
//
//! Write a set of command bytes to the display controller.
//
//! \param pi8Cmd is a pointer to a set of command bytes.
//! \param ui32Count is the count of command bytes.
//!
//! This function provides a way to send multiple command bytes to the display
//! controller.  It can be used for single commands, or multiple commands
//! chained together in a buffer.  It will wait for any previous operation to
//! finish, and then copy all the command bytes to the controller.  It will
//! not return until the last command byte has been written to the SSI FIFO,
//! but data could still be shifting out to the display controller when this
//! function returns.
//!
//! \return None.
//
//*****************************************************************************

static void
SSD1309WriteCommandBuf(const uint8_t *pi8Cmd, uint32_t ui32Count)
{
    //
    // Wait for any previous SSI operation to finish.
    //
    while(SSIBusy(DISPLAY_SSI_BASE))
    {
    }

    //
    // Set the D/C pin low to indicate command
    //
    GPIOPinWrite(DISPLAY_D_C_PORT, DISPLAY_D_C_PIN, 0);
    SysCtlDelay(100);

    //
    // Send all the command bytes to the display
    //
    while(ui32Count--)
    {
        SSIDataPut(DISPLAY_SSI_BASE, *pi8Cmd);
        pi8Cmd++;
    }
}

static void
SSD1309WriteCommand(const uint8_t i8Cmd)
{
	SSD1309WriteCommandBuf(&i8Cmd, 1);
}

//*****************************************************************************
//
//! Write a set of data bytes to the display controller.
//
//! \param pi8Data is a pointer to a set of data bytes, containing pixel data.
//! \param ui32Count is the count of command bytes.
//!
//! This function provides a way to send a set of pixel data to the display.
//! The data will draw pixels according to whatever the most recent col, row
//! settings are for the display.  It will wait for any previous operation to
//! finish, and then copy all the data bytes to the controller.  It will
//! not return until the last data byte has been written to the SSI FIFO,
//! but data could still be shifting out to the display controller when this
//! function returns.
//!
//! \return None.
//
//*****************************************************************************

static void
SSD1309WriteDataBuf(const uint8_t *pi8Data, uint32_t ui32Count)
{
    //
    // Wait for any previous SSI operation to finish.
    //
    while(SSIBusy(DISPLAY_SSI_BASE))
    {
    }

    //
    // Set the D/C pin high to indicate data
    //
    GPIOPinWrite(DISPLAY_D_C_PORT, DISPLAY_D_C_PIN, DISPLAY_D_C_PIN);
    SysCtlDelay(100);

    //
    // Send all the data bytes to the display
    //
    while(ui32Count--)
    {
        SSIDataPut(DISPLAY_SSI_BASE, *pi8Data);
        pi8Data++;
    }
}

//*****************************************************************************
//
// Inline SSD1309 Display Controller Commands
//
//*****************************************************************************

inline void SSD1309SetStartColumn(unsigned char d)
{
   	// Set Lower Column Start Address for Page Addressing Mode
	//   Default => 0x00
	// Set Higher Column Start Address for Page Addressing Mode
	//   Default => 0x10
    SSD1309WriteCommand(0x00 + d % 16);
	SSD1309WriteCommand(0x10 + d / 16);
}

inline void SSD1309SetAddressingMode(unsigned char d)
{
	// Set Memory Addressing Mode
	//	Default => 0x02
    //     0x00 => Horizontal Addressing Mode
    //     0x01 => Vertical Addressing Mode
    //     0x02 => Page Addressing Mode
	SSD1309WriteCommand(0x20);
	SSD1309WriteCommand(d);
}

inline void SSD1309SetColumnAddress(unsigned char a, unsigned char b)
{
    // Set Column Address
	//   Default => 0x00 (Column Start Address)
	//   Default => 0x7F (Column End Address)
	SSD1309WriteCommand(0x21);
	SSD1309WriteCommand(a);
	SSD1309WriteCommand(b);
}

inline void SSD1309SetPageAddress(unsigned char a, unsigned char b)
{
    // Set Page Address
	//   Default => 0x00 (Page Start Address)
	//   Default => 0x07 (Page End Address)
	SSD1309WriteCommand(0x22);
	SSD1309WriteCommand(a);
	SSD1309WriteCommand(b);
}

inline void SSD1309SetStartLine(unsigned char d)
{
    // Set Display Start Line
    //   Default => 0x40 (0x00)
	SSD1309WriteCommand(0x40|d);
}

inline void SSD1309SetContrastControl(unsigned char d)
{
	// Set Contrast Control for Bank 0
	//   Default => 0x7F
	SSD1309WriteCommand(0x81);
	SSD1309WriteCommand(d);
}

inline void SSD1309SetSegmentRemap(unsigned char d)
{
	// Set Segment Re-Map
    //   Default => 0xA0
    //     0xA0 => Column Address 0 Mapped to SEG0
    //     0xA1 => Column Address 0 Mapped to SEG127
	SSD1309WriteCommand(d);
}

inline void SSD1309SetEntireDisplay(unsigned char d)
{
	// Set Entire Display On / Off
    //   Default => 0xA4
    //     0xA4 => Normal Display
    //     0xA5 => Entire Display On
	SSD1309WriteCommand(d);
}

inline void SSD1309SetInverseDisplay(unsigned char d)
{
	// Set Inverse Display On/Off
    //   Default => 0xA6
    //     0xA6 => Normal Display
    //     0xA7 => Inverse Display On
	SSD1309WriteCommand(d);
}

inline void SSD1309SetMultiplexRatio(unsigned char d)
{
	SSD1309WriteCommand(0xA8);		// Set Multiplex Ratio
	SSD1309WriteCommand(d);			//   Default => 0x3F (1/64 Duty)
}

inline void SSD1309SetDisplayOnOff(unsigned char d)
{
	// Set Display On/Off
    //   Default => 0xAE
    //     0xAE => Display Off
    //     0xAF => Display On
	SSD1309WriteCommand(d);
}

inline void SSD1309SetStartPage(unsigned char d)
{
	// Set Page Start Address for Page Addressing Mode
    //   Default => 0xB0 (0x00)
	SSD1309WriteCommand(0xB0|d);
}

inline void SSD1309SetCommonRemap(unsigned char d)
{
	// Set COM Output Scan Direction
    //   Default => 0xC0
    //     0xC0 => Scan from COM0 to 63
    //     0xC8 => Scan from COM63 to 0
	SSD1309WriteCommand(d);
}

inline void SSD1309SetDisplayOffset(unsigned char d)
{
	// Set Display Offset
	//   Default => 0x00
	SSD1309WriteCommand(0xD3);
	SSD1309WriteCommand(d);
}

inline void SSD1309SetDisplayClock(unsigned char d)
{
	// Set Display Clock Divide Ratio / Oscillator Frequency
	//   Default => 0x70
    //     D[3:0] => Display Clock Divider
    //     D[7:4] => Oscillator Frequency
	SSD1309WriteCommand(0xD5);
	SSD1309WriteCommand(d);
}

inline void SSD1309SetLowPower(unsigned char d)
{
	// Set Low Power Display Mode
	//   Default => 0x04 (Normal Power Mode)
	SSD1309WriteCommand(0xD8);
	SSD1309WriteCommand(d);
}

inline void SSD1309SetPrechargePeriod(unsigned char d)
{
	// Set Pre-Charge Period
	//   Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
    //     D[3:0] => Phase 1 Period in 1~15 Display Clocks
    //     D[7:4] => Phase 2 Period in 1~15 Display Clocks
	SSD1309WriteCommand(0xD9);
	SSD1309WriteCommand(d);
}

inline void SSD1309SetCommonConfig(unsigned char d)
{
	// Set COM Pins Hardware Configuration
	//   Default => 0x12
    //     Alternative COM Pin Configuration
    //     Disable COM Left/Right Re-Map
	SSD1309WriteCommand(0xDA);
	SSD1309WriteCommand(d);
}

inline void SSD1309SetVCOMH(unsigned char d)
{
	// Set VCOMH Deselect Level
	//   Default => 0x34 (0.78*VCC)
	SSD1309WriteCommand(0xDB);
	SSD1309WriteCommand(d);
}

inline void SSD1309SetNOP()
{
	// Command for No Operation
	SSD1309WriteCommand(0xE3);
}

inline void SSD1309SetCommandLock(unsigned char d)
{
	// Set Command Lock
	//   Default => 0x12
    //     0x12 => Driver IC interface is unlocked from entering command.
    //     0x16 => All Commands are locked except 0xFD.
	SSD1309WriteCommand(0xFD);
	SSD1309WriteCommand(d);
}

//*****************************************************************************
//
//! Flushes any cached drawing operations.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//!
//! This functions flushes any cached drawing operations to the display.  This
//! is useful when a local frame buffer is used for drawing operations, and the
//! flush would copy the local frame buffer to the display.  Since no memory
//! based frame buffer is used for this driver, the flush is a no operation.
//!
//! \return None.
//
//*****************************************************************************
static void
SSD1309Flush(void *pvDisplayData)
{
    unsigned char i;
    unsigned char *pui8Image;

    pui8Image = &g_ucScreenBuffer[5];

	for(i=0; i < 8; i++)
	{
		SSD1309SetStartPage(i);
		SSD1309SetStartColumn(0x00);
		SSD1309WriteDataBuf(pui8Image, 128);
		pui8Image += 128;
	}
}



//*****************************************************************************
//
//! Initializes the display driver.
//!
//! This function initializes the SSD1332 display controller on the panel,
//! preparing it to display data.
//!
//! \return None.
//
//*****************************************************************************

void
FEMA128x64Init(void)
{
    // Get the value to pass to SysCtlDelay() in order to delay for 1 ms.
    g_ulClockMS = SysCtlClockGet() / (3 * 1000);

    //
    // Enable the peripherals used by this driver
    //
    SysCtlPeripheralEnable(DISPLAY_SSI_PERIPH);
    SysCtlPeripheralEnable(DISPLAY_SSI_GPIO_PERIPH);
    SysCtlPeripheralEnable(DISPLAY_RST_GPIO_PERIPH);

    //
    // Select the SSI function for the appropriate pins
    //
    GPIOPinConfigure(DISPLAY_PINCFG_SSICLK);
    GPIOPinConfigure(DISPLAY_PINCFG_SSIFSS);
    GPIOPinConfigure(DISPLAY_PINCFG_SSITX);

    //
    // Configure the pins for the SSI function
    //
    GPIOPinTypeSSI(DISPLAY_SSI_PORT, DISPLAY_SSI_PINS);

    //
    // Configure display control pins as GPIO output
    //
    GPIOPinTypeGPIOOutput(DISPLAY_RST_PORT, DISPLAY_RST_PIN);

    //GPIOPinTypeGPIOOutput(DISPLAY_PWR_PORT, DISPLAY_PWR_PIN);
    // Enable pin PD7 for GPIOOutput
    //First open the lock and select the bits we want to modify in the GPIO commit register.
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    //Now modify the configuration of the pins that we unlocked.
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);

    GPIOPinTypeGPIOOutput(DISPLAY_D_C_PORT, DISPLAY_D_C_PIN);

    //
    // Reset pin high, power off
    //
    GPIOPinWrite(DISPLAY_RST_PORT, DISPLAY_RST_PIN, DISPLAY_RST_PIN);
    GPIOPinWrite(DISPLAY_PWR_PORT, DISPLAY_PWR_PIN, 0);
    SysCtlDelay(g_ulClockMS * 100);

    GPIOPinWrite(DISPLAY_D_C_PORT, DISPLAY_D_C_PIN, 0);

    //
    // Drive the reset pin low while we do other stuff
    //
    GPIOPinWrite(DISPLAY_RST_PORT, DISPLAY_RST_PIN, 0);

    //
    // Configure the SSI port
    //
    SSIDisable(DISPLAY_SSI_BASE);
    SSIConfigSetExpClk(DISPLAY_SSI_BASE, SysCtlClockGet(),
                           SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER,
                           DISPLAY_SSI_CLOCK, 8);
    SSIEnable(DISPLAY_SSI_BASE);

    //
    // Take the display out of reset
    //

    SysCtlDelay(g_ulClockMS * 100);
    GPIOPinWrite(DISPLAY_RST_PORT, DISPLAY_RST_PIN, DISPLAY_RST_PIN);
    SysCtlDelay(g_ulClockMS * 100);

    //
    // Send the initial configuration command bytes to the display
    //
	SSD1309SetCommandLock(0x12);			// Unlock Driver IC (0x12/0x16)
	SSD1309SetDisplayOnOff(0xAE);		    // Display Off (0xAE/0xAF)
	SSD1309SetDisplayClock(0xA0);		    // Set Clock as 116 Frames/Sec
	SSD1309SetMultiplexRatio(0x3F);		    // 1/64 Duty (0x0F~0x3F)
	SSD1309SetDisplayOffset(0x00);		    // Shift Mapping RAM Counter (0x00~0x3F)
	SSD1309SetStartLine(0x00);			    // Set Mapping RAM Display Start Line (0x00~0x3F)
	SSD1309SetLowPower(0x04);			    // Set Normal Power Mode (0x04/0x05)

	SSD1309SetAddressingMode(0x02);		    // Set Page Addressing Mode (0x00/0x01/0x02)
	SSD1309SetSegmentRemap(0xA1);		    // Set SEG/Column Mapping (0xA0/0xA1)
	SSD1309SetCommonRemap(0xC8);			// Set COM/Row Scan Direction (0xC0/0xC8)
	SSD1309SetCommonConfig(0x12);		    // Set Alternative Configuration (0x02/0x12)

	SSD1309SetContrastControl(BRIGHTNESS);  // Set SEG Output Current
	SSD1309SetPrechargePeriod(0x82);		// Set Pre-Charge as 8 Clocks & Discharge as 2 Clocks
	SSD1309SetVCOMH(0x34);			        // Set VCOM Deselect Level
	SSD1309SetEntireDisplay(0xA4);		    // Disable Entire Display On (0xA4/0xA5)
	SSD1309SetInverseDisplay(0xA6);		    // Disable Inverse Display On (0xA6/0xA7)

	/* Flush all zeros to controller memory to clear screen */
    SSD1309Flush(g_ucScreenBuffer);

    /* Enable Vcc power to the display */
    GPIOPinWrite(DISPLAY_PWR_PORT, DISPLAY_PWR_PIN, DISPLAY_PWR_PIN);
    SysCtlDelay(g_ulClockMS * 100);
    
    /* Set the display on (0xAF) */
	SSD1309SetDisplayOnOff(0xAF);           
    SysCtlDelay(g_ulClockMS * 100);

    /* Initialize the 1 BPP off-screen buffer. */
    GrOffScreenMonoInit(&g_FEMA128x64, g_ucScreenBuffer, 128, 64);
    g_FEMA128x64.pfnFlush = SSD1309Flush;
    
    //DrawBorderFrame();
    //DrawCheckerboard();
}

//*****************************************************************************
//
//! Enter display sleep mode.
//!
//! This function sets the display off and powers down 12V Vcc to enter 
//! sleep mode.
//!
//! \return None.
//
//*****************************************************************************
void
FEMA128x64Sleep(void)
{
    /* Set the display off (0xAE) */
	SSD1309SetDisplayOnOff(0xAE);           
    SysCtlDelay(g_ulClockMS * 100);
    
   /* Disable Vcc power to the display */
    GPIOPinWrite(DISPLAY_PWR_PORT, DISPLAY_PWR_PIN, 0);
    SysCtlDelay(g_ulClockMS * 100);
}

//*****************************************************************************
//
//! Exit display sleep mode.
//!
//! This function sets the display on and powers up 12V Vcc to exit
//! sleep mode.
//!
//! \return None.
//
//*****************************************************************************
void
FEMA128x64Wake(void)
{  
   /* Enable Vcc power to the display */
    GPIOPinWrite(DISPLAY_PWR_PORT, DISPLAY_PWR_PIN, DISPLAY_PWR_PIN);
    SysCtlDelay(g_ulClockMS * 100);
    
    /* Set the display on (0xAF) */
	SSD1309SetDisplayOnOff(0xAF);           
    SysCtlDelay(g_ulClockMS * 100);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  DEBUG HARDWARE FUNCTIONS
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#if 0
void DrawBorderFrame()
{
	unsigned char i,j,xl;

	SSD1309SetStartPage(0);
	SSD1309SetStartColumn(0);

	for(i=0;i<SCREEN_WIDTH;i++)
		SSD1309WriteData(0x01);

	SSD1309SetStartPage(0x07);
	SSD1309SetStartColumn(0);

	for(i=0;i<SCREEN_WIDTH;i++)
		SSD1309WriteData(0x80);

	for(i=0;i<8;i++)
	{
		SSD1309SetStartPage(i);

		for(j=0;j<SCREEN_WIDTH;j+=(SCREEN_WIDTH-1))
		{
			SSD1309SetStartColumn(j);
			SSD1309WriteData(0xFF);
		}
	}
}

static void DrawCheckerboard()
{
	unsigned char i,j;

	for(i=0; i < 8; i++)
	{
		SSD1309SetStartPage(i);
		SSD1309SetStartColumn(0x00);

		for(j=0;j<64;j++)
		{
			SSD1309WriteData(0x55);
			SSD1309WriteData(0xAA);
		}
	}
}
#endif

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
