/****************************************************************************
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2016, RTZ Professional Audio LLC
 *
 * The source code contained in this file and associated files is copyright
 * protected material. This material contains the confidential, proprietary
 * and trade secret information of Sigma Software Research. No disclosure or
 * use of any portions of this material may be made without the express
 * written consent of RTZ Professional Audio.
 *****************************************************************************/

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* Generic Includes */
#include <file.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

//#include <driverlib/sysctl.h>

/* XDCtools Header files */
#include "Board.h"
#include "DRC1200.h"
#include "IOExpander.h"

/* Semaphore timeout 100ms */
#define TIMEOUT_SPI		(100)

static bool MCP23S17_write(
	IOExpander_Handle	handle,
    uint8_t				ucRegAddr,
    uint8_t				ucData
    );

static bool MCP23S17_read(
	IOExpander_Handle	hSPI,
    uint8_t   			ucRegAddr,
    uint8_t*  			pucData
    );

/*****************************************************************************
 * I/O handle and configuration data
 *****************************************************************************/

#define NUM_OBJ	3

static IOExpander_InitData initData_U11[] = {
    { MCP_IOCONA, C_SEQOP },	/* Configure for byte mode */
    { MCP_IODIRA, 0x00 },	    /* Port A - all outputs to LED drivers */
    { MCP_IODIRB, 0x00 },		/* Port B - all outputs to LED drivers */
};

static IOExpander_InitData initData_U10[] = {
    { MCP_IOCONA, C_SEQOP },	/* Configure for byte mode */
    { MCP_IODIRA, 0xFF },	    /* Port A - all inputs from push button switches */
    { MCP_IODIRB, 0xFF },		/* Port B - all inputs from push button switches */
};

static IOExpander_InitData initData_U7[] = {
    { MCP_IOCONA, C_SEQOP },	/* Configure for byte mode */
    { MCP_IODIRA, 0x00 },	    /* Port A - all outputs to button LED's */
    { MCP_IODIRB, 0xFF },		/* Port B - all inputs from button switches */
};

#define IDSIZE(d)	( sizeof(d)/sizeof(IOExpander_InitData) )

/* I/O Expander Handle Data */

static IOExpander_Object IOExpanderObjects[NUM_OBJ] = {
	{ NULL, Board_SPI_U11, Board_GPIO_U11_CS, initData_U11, IDSIZE(initData_U11) },
	{ NULL, Board_SPI_U10, Board_GPIO_U10_CS, initData_U10, IDSIZE(initData_U10) },
	{ NULL, Board_SPI_U7,  Board_GPIO_U7_CS,  initData_U7,  IDSIZE(initData_U7)  },
};

/*****************************************************************************
 * Open the I/O expander and initialize it
 *****************************************************************************/

IOExpander_Handle IOExpander_open(unsigned int index)
{
	unsigned int i, key;
	IOExpander_Handle handle;
	IOExpander_InitData* initData;
	SPI_Params	spiParams;

	handle = &(IOExpanderObjects[index]);

	/* Determine if the device index was already opened */
	key = Hwi_disable();
	if (handle->spiHandle) {
		Hwi_restore(key);
		return NULL;
	}
	Hwi_restore(key);

	SPI_Params_init(&spiParams);

	spiParams.transferMode	= SPI_MODE_BLOCKING;
	spiParams.mode 			= SPI_MASTER;
	spiParams.frameFormat 	= SPI_POL0_PHA0;
	spiParams.bitRate 		= 1000000;
	spiParams.dataSize 		= 8;

	/* Open SPI driver to the IO Expander */
	handle->spiHandle = SPI_open(handle->boardSPI, &spiParams);

	if (handle->spiHandle == NULL) {
		System_printf("Error opening I/O Expander SPI port\n");
		return NULL;
	}

	/* Intialize the I/O expander */

	initData = IOExpanderObjects[index].initData;

	for (i=0; i < IOExpanderObjects[index].initDataCount; i++)
	{
	     MCP23S17_write(handle, initData->addr, initData->data);
	     ++initData;
	}

	return handle;
}

/* Close an I/O Expander Port */

void IOExpander_close(IOExpander_Handle handle)
{
    SPI_close(handle->spiHandle);
}

/*****************************************************************************
 * Write a register command byte to MCP23S17 expansion I/O controller.
 *****************************************************************************/

bool MCP23S17_write(
	IOExpander_Handle	handle,
    uint8_t   			ucRegAddr,
    uint8_t   			ucData
    )
{
	SPI_Transaction opcodeTransaction;
	SPI_Transaction dataTransaction;
	uint8_t txBuffer[2];
	uint8_t rxBuffer[2];
	uint8_t dummyBuffer = 0;

	txBuffer[0] = 0x40;			/* write opcode */
	txBuffer[1] = ucRegAddr;	/* register address */

	/* Initialize opcode transaction structure */
	opcodeTransaction.count = 2;
	opcodeTransaction.txBuf = (Ptr)&txBuffer;
	opcodeTransaction.rxBuf = (Ptr)&rxBuffer;

	/* Initialize data transaction structure */
	dataTransaction.count = 1;
	dataTransaction.txBuf = (Ptr)&ucData;
	dataTransaction.rxBuf = (Ptr)&dummyBuffer;

	/* Hold SPI chip select low */
	GPIO_write(handle->boardCS, PIN_LOW);

	/* Initiate SPI transfer of opcode */
	if(!SPI_transfer(handle->spiHandle, &opcodeTransaction)) {
	    System_printf("Unsuccessful master SPI transfer to MCP23S17");
	    return false;
	}
	
	/* Initiate SPI transfer of data */
	if(!SPI_transfer(handle->spiHandle, &dataTransaction)) {
	    System_printf("Unsuccessful master SPI transfer to MCP23S17");
	    return false;
	}
	 
	/* Release SPI chip select */
	GPIO_write(handle->boardCS, PIN_HIGH);

	return true;
}

/*****************************************************************************
 * Read a register command byte from MCP23S17 expansion I/O controller.
 *****************************************************************************/

bool MCP23S17_read(
	IOExpander_Handle	handle,
    uint8_t				ucRegAddr,
    uint8_t*			pucData
    )
{
	SPI_Transaction opcodeTransaction;
	SPI_Transaction dataTransaction;
	uint8_t txBuffer[2];
	uint8_t rxBuffer[2];
	uint8_t dummyBuffer = 0;

	txBuffer[0] = 0x41;			/* read opcode */
	txBuffer[1] = ucRegAddr;	/* register address */

	/* Initialize opcode transaction structure */
	opcodeTransaction.count = 2;
	opcodeTransaction.txBuf = (Ptr)&txBuffer;
	opcodeTransaction.rxBuf = (Ptr)&rxBuffer;

	/* Initialize data transaction structure */
	dataTransaction.count = 1;
	dataTransaction.txBuf = (Ptr)&dummyBuffer;
	dataTransaction.rxBuf = (Ptr)pucData;

	/* Hold SPI chip select low */
	GPIO_write(handle->boardCS, PIN_LOW);

	/* Initiate SPI transfer of opcode */
	if(!SPI_transfer(handle->spiHandle, &opcodeTransaction)) {
	    System_printf("Unsuccessful master SPI transfer to MCP23S17");
	    return false;
	}
	
	/* Initiate SPI transfer of data */
	if(!SPI_transfer(handle->spiHandle, &dataTransaction)) {
	    System_printf("Unsuccessful master SPI transfer to MCP23S17");
	    return false;
	}
	 
	/* Release SPI chip select */
	GPIO_write(handle->boardCS, PIN_HIGH);

	return true;
}

/*****************************************************************************
 * Hwi Interrupt Handlers (interrupts are currently not used)
 *****************************************************************************/

void gpioU7IntA(void)
{
	GPIO_clearInt(Board_GPIO_U7_INTA);
}

void gpioU7IntB(void)
{
	GPIO_clearInt(Board_GPIO_U7_INTB);
}

void gpioU10IntA(void)
{
	GPIO_clearInt(Board_GPIO_U10_INTA);
}

void gpioU10IntB(void)
{
	GPIO_clearInt(Board_GPIO_U10_INTB);
}

/*****************************************************************************
 * PUBLIC FUNCTIONS - Mutex synchronized and may be called by any task
 *****************************************************************************/

static IOExpander_Handle handleU7;
static IOExpander_Handle handleU10;
static IOExpander_Handle handleU11;

void IOExpander_initialize(void)
{
	/* Open the SPI port to U11 */

	handleU11 = IOExpander_open(Board_SPI_U11);

	if (handleU11 == NULL)
		System_abort("Error opening SPI to U11\n");

	/* Open the SPI port to U10 */

	handleU10 = IOExpander_open(Board_SPI_U10);

	if (handleU10 == NULL)
		System_abort("Error opening SPI to U10\n");

	/* Open the SPI port to U7 */

	handleU7  = IOExpander_open(Board_SPI_U7);

	if (handleU7 == NULL)
		System_abort("Error opening SPI to U7\n");
}

/*****************************************************************************
 * U7 INPUT/OUTPUT: Transport Control Buttons & LED indicators.
 *****************************************************************************/

/* Current transport LED bit state */
static uint8_t s_maskTransportLED = 0;

bool SetTransportLEDMask(uint8_t maskSet, uint8_t maskClear)
{
	/* Clear any bits in the clear mask */
	s_maskTransportLED &= ~(maskClear);

	/* Set any bits in the set mask */
	s_maskTransportLED |= maskSet;

	return MCP23S17_write(handleU7, MCP_GPIOA, s_maskTransportLED);
}

uint8_t GetTransportLEDMask(void)
{
	return s_maskTransportLED;
}

/* Read the current transport switch button states */
bool ReadTransportSwitches(uint8_t* pSwitchBits)
{
	return MCP23S17_read(handleU7, MCP_GPIOA, pSwitchBits);
}

/*****************************************************************************
  * U11 OUTPUT : Remote Control Button LED indicators.
 *****************************************************************************/

/* Current button LED bit state */
static uint16_t s_maskButtonLED = 0;

bool SetButtonLEDMask(uint16_t maskSet, uint16_t maskClear)
{
	/* Clear any bits in the clear mask */
	s_maskButtonLED &= ~(maskClear);

	/* Set any bits in the set mask */
	s_maskButtonLED |= maskSet;

	/* Update Port-A & Port-B outputs */
	MCP23S17_write(handleU11, MCP_GPIOA, (uint8_t)(s_maskButtonLED & 0xFF));
	MCP23S17_write(handleU11, MCP_GPIOB, (uint8_t)((s_maskButtonLED >> 8) & 0xFF));

	return true;
}

uint16_t GetButtonLEDMask(void)
{
	return s_maskButtonLED;
}

/*****************************************************************************
 * U10 INPUT : Remote Control Button Switch Inputs
 *****************************************************************************/

/* Read the current transport switch button states */
bool ReadButtonSwitches(uint16_t* pSwitchBits)
{
	uint16_t portA;
	uint16_t portB;

	MCP23S17_read(handleU10, MCP_GPIOA, (uint8_t*)&portA);
	MCP23S17_read(handleU10, MCP_GPIOB, (uint8_t*)&portB);

	*pSwitchBits = portA | (portB << 8);

	return true;
}

// End-Of-File
