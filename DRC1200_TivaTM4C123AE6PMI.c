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

/*
 *  ======== DRC1200.c ========
 *  This file is responsible for setting up the board specific items for the
 *  DRC1200 board.
 */

#include <stdint.h>
#include <stdbool.h>

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ssi.h>
#include <inc/hw_i2c.h>

#include <driverlib/gpio.h>
#include <driverlib/flash.h>
#include <driverlib/eeprom.h>
#include <driverlib/sysctl.h>
#include <driverlib/i2c.h>
#include <driverlib/ssi.h>

#include <driverlib/gpio.h>
#include <driverlib/i2c.h>
#include <driverlib/pin_map.h>
#include <driverlib/pwm.h>
#include <driverlib/ssi.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/udma.h>
#include <driverlib/adc.h>
#include <driverlib/qei.h>

#include "DRC1200_TivaTM4C123AE6PMI.h"

#ifndef TI_DRIVERS_UART_DMA
#define TI_DRIVERS_UART_DMA 0
#endif

/*
 *  =============================== DMA ===============================
 */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#endif
static tDMAControlTable dmaControlTable[32];
static bool dmaInitialized = false;

/* Hwi_Struct used in the initDMA Hwi_construct call */
static Hwi_Struct dmaHwiStruct;

/*
 *  ======== dmaErrorHwi ========
 */
static Void dmaErrorHwi(UArg arg)
{
    System_printf("DMA error code: %d\n", uDMAErrorStatusGet());
    uDMAErrorStatusClear();
    System_abort("DMA error!!");
}

/*
 *  ======== DRC1200_initDMA ========
 */
void DRC1200_initDMA(void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    if (!dmaInitialized) {
        Error_init(&eb);
        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(dmaHwiStruct), INT_UDMAERR, dmaErrorHwi, &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't construct DMA error hwi");
        }

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        uDMAEnable();
        uDMAControlBaseSet(dmaControlTable);

        dmaInitialized = true;
    }
}

/*
 *  =============================== General ===============================
 */
 
/*
 *  ======== DRC1200_initGeneral ========
 */
void DRC1200_initGeneral(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

	// Initialize the internal EEPROM so we can access it later
    if (EEPROMInit() != EEPROM_INIT_OK)
        System_abort("EEPROMInit() failed");

    uint32_t size = EEPROMSizeGet();
}

/*
 *  =============================== GPIO ===============================
 */

/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(GPIOTiva_config, ".const:GPIOTiva_config")
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOTiva.h>

/* GPIO configuration structure */

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in DTC1200_TM4C123AE6PMI.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[DRC1200_GPIOCOUNT] = {
	/*=== Input pins ===*/
    /* (0) PE0 : U10_INTA input */
    GPIOTiva_PE_0 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_RISING,
    /* (1) PE1 : U10_INTB input */
    GPIOTiva_PE_1 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_RISING,
	/* (2) PE3 : U7_INTA input */
    GPIOTiva_PE_3 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_RISING,
	/* (3) PE3 : U7_INTB input */
    GPIOTiva_PE_3 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_RISING,
	/* (4) PG4 : JOGSW input */
    GPIOTiva_PG_4 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_RISING,
    /* (5) PG0 : DIP_SW1 input */
    GPIOTiva_PG_0 | GPIO_CFG_IN_PU,
    /* (6) PG1 : DIP_SW2 input */
    GPIOTiva_PG_1 | GPIO_CFG_IN_PU,
    /* (7) PG2 : DIP_SW3 input */
    GPIOTiva_PG_2 | GPIO_CFG_IN_PU,
    /* (8) PG3 : DIP_SW4 input */
    GPIOTiva_PG_3 | GPIO_CFG_IN_PU,
    /*=== Output pins ===*/
    /* (9) PC7 : LED1 output */
    GPIOTiva_PC_7 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* (10) PG5 : LED2 output */
    GPIOTiva_PG_5 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* (11) PB0 : RS422 #RE output */
    GPIOTiva_PB_0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* (12) PB1 : RS422 DE output */
    GPIOTiva_PB_1 | GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW,
	/* (13) PA3 : U11 CS */
	GPIOTiva_PA_3 | GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH,
	/* (14) PF3 : U10 CS */
	GPIOTiva_PF_3 | GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH,
	/* (15) PB5 : U7 CS */
	GPIOTiva_PB_5 | GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH,
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in DTC1200_TivaTM4C123AE6PMI.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
	NULL,	/* DRC1200_GPIO_U10_INTA */
	NULL, 	/* DRC1200_GPIO_U10_INTB */
	NULL,	/* DRC1200_GPIO_U7_INTA  */
	NULL,	/* DRC1200_GPIO_U7_INTB  */
	NULL	/* DRC1200_GPIO_JOGSW    */
};

/* The device-specific GPIO_config structure */
const GPIOTiva_Config GPIOTiva_config = {
    .pinConfigs         = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks          = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks  = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority        = (~0)
};

/*
 *  ======== DRC1200_initGPIO ========
 */
void DRC1200_initGPIO(void)
{
    // Enable pin PA3 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
    
    // Enable pin PB0 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
    // Enable pin PB1 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);
    // Enable pin PB5 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);

    // Enable pin PC7 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);

    // Enable pin PD6 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);
    // Enable pin PD7 for GPIOOutput
    //First open the lock and select the bits we want to modify in the GPIO commit register.
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    //Now modify the configuration of the pins that we unlocked.
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);
    // Enable pin PD5 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_5);

    // Enable pin PE3 for GPIOInput
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Enable pin PE1 for GPIOInput
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
    // Enable pin PE2 for GPIOInput
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2);
    // Enable pin PE0 for GPIOInput
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);

    // Enable pin PF3 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    
    // Enable PG0-PG3 for GPIOInput with weak pullup for config DIP switch
    GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, DIP_SW_MASK);
    GPIOPadConfigSet(GPIO_PORTG_BASE, DIP_SW_MASK, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    // Enable pin PG4 for GPIOInput
    GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_4);
    // Enable pin PG5 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_5);

    /* Once GPIO_init is called, GPIO_config cannot be changed */
    GPIO_init();
}
    
/*
 *  =============================== I2C ===============================
 */

/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(I2C_config, ".const:I2C_config")
#pragma DATA_SECTION(i2cTivaHWAttrs, ".const:i2cTivaHWAttrs")
#endif

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CTiva.h>

/* I2C objects */
I2CTiva_Object i2cTivaObjects[DRC1200_I2CCOUNT];

/* I2C configuration structure, describing which pins are to be used */

const I2CTiva_HWAttrs i2cTivaHWAttrs[DRC1200_I2CCOUNT] = {
    {
        .baseAddr    = I2C0_BASE,
        .intNum      = INT_I2C0,
        .intPriority = (~0)
    },
    {
        .baseAddr    = I2C1_BASE,
        .intNum      = INT_I2C1,
        .intPriority = (~0)
    },
};

const I2C_Config I2C_config[] = {
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[0],
        .hwAttrs     = &i2cTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[1],
        .hwAttrs     = &i2cTivaHWAttrs[1]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== DRC1200_initI2C ========
 */
void DRC1200_initI2C(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    // Enable pin PB3 for I2C0 I2C0SDA
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    // Enable pin PB2 for I2C0 I2C0SCL
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);

    // Enable pin PA7 for I2C1 I2C1SDA
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    // Enable pin PA6 for I2C1 I2C1SCL
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);

    I2C_init();
}

/*
 *  =============================== SPI ===============================
 */

/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SPI_config, ".const:SPI_config")
#pragma DATA_SECTION(spiTivaDMAHWAttrs, ".const:spiTivaDMAHWAttrs")
#endif

#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPITivaDMA.h>

SPITivaDMA_Object spiTivaDMAObjects[DRC1200_SPICOUNT];

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(spiTivaDMAscratchBuf, 32)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=32
#elif defined(__GNUC__)
__attribute__ ((aligned (32)))
#endif
uint32_t spiTivaDMAscratchBuf[DRC1200_SPICOUNT];

const SPITivaDMA_HWAttrs spiTivaDMAHWAttrs[DRC1200_SPICOUNT] = {
    {
        .baseAddr               = SSI0_BASE,
        .intNum                 = INT_SSI0,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[0],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_CHANNEL_SSI0RX,
        .txChannelIndex         = UDMA_CHANNEL_SSI0TX,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH10_SSI0RX,
        .txChannelMappingFxnArg = UDMA_CH11_SSI0TX
    },
    {
        .baseAddr               = SSI1_BASE,
        .intNum                 = INT_SSI1,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[1],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_CHANNEL_SSI1RX,
        .txChannelIndex         = UDMA_CHANNEL_SSI1TX,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH24_SSI1RX,
        .txChannelMappingFxnArg = UDMA_CH25_SSI1TX
    },
	{
        .baseAddr               = SSI2_BASE,
        .intNum                 = INT_SSI2,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[2],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_SEC_CHANNEL_UART2RX_12,
        .txChannelIndex         = UDMA_SEC_CHANNEL_UART2TX_13,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH12_SSI2RX,
        .txChannelMappingFxnArg = UDMA_CH13_SSI2TX
    }
};

const SPI_Config SPI_config[] = {
    {&SPITivaDMA_fxnTable, &spiTivaDMAObjects[0], &spiTivaDMAHWAttrs[0]},
    {&SPITivaDMA_fxnTable, &spiTivaDMAObjects[1], &spiTivaDMAHWAttrs[1]},
    {&SPITivaDMA_fxnTable, &spiTivaDMAObjects[2], &spiTivaDMAHWAttrs[2]},
    {NULL, NULL, NULL},
};

/*
 *  ======== DRC1200_initSPI ========
 */
void DRC1200_initSPI(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);

    // Enable pin PA5 for SSI0 SSI0TX
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5);
    // Enable pin PA4 for SSI0 SSI0RX
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_4);
    // Enable pin PA3 for SSI0 SSI0FSS
    //GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    //GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_3);
    // Enable pin PA2 for SSI0 SSI0CLK
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2);

    // Enable pin PF0 for SSI1 SSI1RX
    // First open the lock and select the bits we want to modify in the GPIO commit register.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
    // Now modify the configuration of the pins that we unlocked.
    GPIOPinConfigure(GPIO_PF0_SSI1RX);
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0);
    // Enable pin PF2 for SSI1 SSI1CLK
    GPIOPinConfigure(GPIO_PF2_SSI1CLK);
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2);
    // Enable pin PF3 for SSI1 SSI1FSS
    //GPIOPinConfigure(GPIO_PF3_SSI1FSS);
    //GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_3);
    // Enable pin PF1 for SSI1 SSI1TX
    GPIOPinConfigure(GPIO_PF1_SSI1TX);
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_1);

    // Enable pin PB6 for SSI2 SSI2RX
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_6);
    // Enable pin PB4 for SSI2 SSI2CLK
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4);
    // Enable pin PB5 for SSI2 SSI2FSS
    //GPIOPinConfigure(GPIO_PB5_SSI2FSS);
    //GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5);
    // Enable pin PB7 for SSI2 SSI2TX
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_7);

    DRC1200_initDMA();
    SPI_init();
}

/*
 *  =============================== UART ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartTivaHWAttrs, ".const:uartTivaHWAttrs")
#endif

#include <ti/drivers/UART.h>
#if TI_DRIVERS_UART_DMA
#include <ti/drivers/uart/UARTTivaDMA.h>

UARTTivaDMA_Object uartTivaObjects[DRC1200_UARTCOUNT];

const UARTTivaDMA_HWAttrs uartTivaHWAttrs[DRC1200_UARTCOUNT] = {
    {
        .baseAddr       = UART0_BASE,
        .intNum         = INT_UART0,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH8_UART0RX,
        .txChannelIndex = UDMA_CH9_UART0TX,
    }
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[0],
        .hwAttrs     = &uartTivaHWAttrs[0]
    },
    {NULL, NULL, NULL}
};
#else
#include <ti/drivers/uart/UARTTiva.h>

UARTTiva_Object uartTivaObjects[DRC1200_UARTCOUNT];
unsigned char uartTivaRingBuffer[DRC1200_UARTCOUNT][32];

/* UART configuration structure */
const UARTTiva_HWAttrs uartTivaHWAttrs[DRC1200_UARTCOUNT] = {
    {
        .baseAddr    = UART0_BASE,
        .intNum      = INT_UART0,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer[0],
        .ringBufSize = sizeof(uartTivaRingBuffer[0])
    }
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[0],
        .hwAttrs     = &uartTivaHWAttrs[0]
    },
    {NULL, NULL, NULL}
};
#endif /* TI_DRIVERS_UART_DMA */

/*
 *  ======== DRC1200_initUART ========
 */
void DRC1200_initUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /*
     * Enable and configure UART0
     */

    // Enable pin PA1 for UART0 U0TX
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

    // Enable pin PA0 for UART0 U0RX
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);

    /* Initialize the UART driver */
#if TI_DRIVERS_UART_DMA
    DRC1200_initDMA();
#endif

    UART_init();
}

/*
 *  =============================== Watchdog ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(Watchdog_config, ".const:Watchdog_config")
#pragma DATA_SECTION(watchdogTivaHWAttrs, ".const:watchdogTivaHWAttrs")
#endif

#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogTiva.h>

/* Watchdog objects */
WatchdogTiva_Object watchdogTivaObjects[DRC1200_WATCHDOGCOUNT];

/* Watchdog configuration structure */
const WatchdogTiva_HWAttrs watchdogTivaHWAttrs[DRC1200_WATCHDOGCOUNT] = {
    /* EK_LM4F120XL_WATCHDOG0 with 1 sec period at default CPU clock freq */
    {WATCHDOG0_BASE, INT_WATCHDOG, 80000000},
};

const Watchdog_Config Watchdog_config[] = {
    {&WatchdogTiva_fxnTable, &watchdogTivaObjects[0], &watchdogTivaHWAttrs[0]},
    {NULL, NULL, NULL},
};

/*
 *  ======== DRC1200_initWatchdog ========
 *
 * NOTE: To use the other watchdog timer with base address WATCHDOG1_BASE,
 *       an additional function call may need be made to enable PIOSC. Enabling
 *       WDOG1 does not do this. Enabling another peripheral that uses PIOSC
 *       such as ADC0 or SSI0, however, will do so. Example:
 *
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);
 *
 *       See the following forum post for more information:
 *       http://e2e.ti.com/support/microcontrollers/stellaris_arm_cortex-m3_microcontroller/f/471/p/176487/654390.aspx#654390
 */
void DRC1200_initWatchdog(void)
{
    /* Enable peripherals used by Watchdog */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    /* Initialize the Watchdog driver */
    Watchdog_init();
}


/*
 *  ==================== DRC1200 Custom Board Specific ====================
 */

/* Read four DIP switch settings */
uint32_t DRC1200_readDIPSwitch(void)
{
	uint32_t bits;
	/* Read the four lower bits of the DIP switch and invert */
	bits = GPIOPinRead(GPIO_PORTG_BASE, DIP_SW_MASK);
	return bits ^ DIP_SW_MASK;
}

/* End-Of-File */
