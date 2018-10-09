/*****************************************************************************
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
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Tivaware Driver files */
#include <driverlib/eeprom.h>

/* Generic Includes */
#include <file.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

/* Graphiclib Header file */
#include <grlib/grlib.h>
#include <RemoteTask.h>
#include "drivers/fema128x64.h"

//#include <driverlib/sysctl.h>

/* XDCtools Header files */
#include "Board.h"
#include "DRC1200.h"
#include "IOExpander.h"
#include "RAMPServer.h"

/* Mailbox Handles created dynamically */

Mailbox_Handle g_mailboxRemote = NULL;

/* Global context for drawing */
tContext g_context;

/* Global system parameters */
SYSPARMS g_sysParams;
SYSDATA  g_sysData;

/* Static Function Prototypes */
Int main();
Void MainButtonTask(UArg a0, UArg a1);
bool ReadSerialNumber(uint8_t ui8SerialNumber[16]);

//*****************************************************************************
// Main Program Entry Point
//*****************************************************************************

Int main()
{ 
    Error_Block eb;
	Task_Handle task;
    Task_Params taskParams;
    Mailbox_Params mboxParams;

    System_printf("enter main()\n");

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
	Board_initSPI();
	Board_initI2C();
	Board_initUART();
	//Board_initQEI();
	
    /* Initialize the OLED display driver */
    FEMA128x64Init();
    /* Initialize the graphics context */
    GrContextInit(&g_context, &g_FEMA128x64);

    /* Read the DIP switch settings */
    g_sysData.dipSwitch = Board_readDIPSwitch();
    System_printf("Config DIP Switch: %x\n", g_sysData.dipSwitch);

    /* Disable the LED's during startup up */
    GPIO_write(Board_GPIO_LED1, Board_LED_OFF);
    GPIO_write(Board_GPIO_LED2, Board_LED_OFF);

    /* Dessert RS-422 DE & RE pins */
    GPIO_write(DRC1200_GPIO_RS422_RE, PIN_HIGH);
    GPIO_write(DRC1200_GPIO_RS422_DE, PIN_LOW);
    
    /* Disable I/O expander chip selects */    
    GPIO_write(Board_GPIO_U10_CS, PIN_HIGH);
    GPIO_write(Board_GPIO_U11_CS, PIN_HIGH);
    GPIO_write(Board_GPIO_U7_CS, PIN_HIGH);

	/* Initialize the default servo and program data values */
    memset(&g_sysParams, 0, sizeof(SYSPARMS));
    InitSysDefaults(&g_sysParams);

	/* Create display task mailbox */

    Mailbox_Params_init(&mboxParams);
    Error_init(&eb);
    g_mailboxRemote = Mailbox_create(sizeof(RemoteMessage), 16, &mboxParams, &eb);

    if (g_mailboxRemote == NULL)
        System_abort("Mailbox create failed");

    /*
     * Now start the main application button polling task
     */

    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 1024;
    taskParams.priority  = 5;

    task = Task_create(MainButtonTask, &taskParams, &eb);

    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

    BIOS_start();    /* does not return */

    return(0);
}

//*****************************************************************************
//
//*****************************************************************************

#define DEBOUNCE    6

Void MainButtonTask(UArg a0, UArg a1)
{
    uint8_t bits;
    uint16_t switches;
    uint32_t debounce_xport = 0;
    uint32_t debounce_switches = 0;
    Task_Params taskParams;
    Error_Block eb;
    RAMP_MSG msg;

    /* Initialize the MCP23S17 I/O expanders */
    IOExpander_initialize();

    /* Read the globally unique serial number from EPROM */
    if (!ReadSerialNumber(g_sysData.ui8SerialNumber)) {
    	System_printf("Read Serial Number Failed!\n");
    	System_flush();
    }

    /* Start the remote communications service tasks */
    if (!RAMP_Server_init())
    {
        System_abort("RAMP Init Failed!\n");
    }

    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 1524;
    taskParams.priority  = 5;

    Task_create(RemoteTaskFxn, &taskParams, &eb);

    /* Ensure all button LED's are off */
    SetTransportLEDMask(0, 0xFF);
    SetButtonLEDMask(0, 0xFF);

    /****************************************************************
     * Enter the main application button processing loop forever.
     ****************************************************************/

    for(;;)
	{
        /*
         *  Read the TRANSPORT button bits
         */

        ReadTransportSwitches(&bits);

        uint8_t temp = bits & ~(SW_REC);

        if (temp)
        {
            if (++debounce_xport >= DEBOUNCE)
            {
                debounce_xport = 0;

                /* Debounced button press, send it to STC */
                msg.type     = MSG_TYPE_SWITCH;
                msg.opcode   = OP_SWITCH_TRANSPORT;
                msg.param1.U = (uint32_t)bits;
                msg.param2.U = 0;

                /* Send the button press to to STC controller */
                RAMP_Send_Message(&msg, 0);
            }
        }

        /*
         *  Read the LOCATE button bits
         */

        ReadButtonSwitches(&switches);

        if (switches)
        {
            if (++debounce_switches >= DEBOUNCE)
            {
                debounce_switches = 0;

                /* Debounced button press, send it to STC */
                msg.type     = MSG_TYPE_SWITCH;
                msg.opcode   = OP_SWITCH_REMOTE;
                msg.param1.U = (uint32_t)switches;
                msg.param2.U = (uint32_t)bits;

                /* Send the button press to to STC controller */
                RAMP_Send_Message(&msg, 0);
            }
        }

        Task_sleep(10);
	}
}

//*****************************************************************************
// Read system parameters into our global settings buffer from EEPROM.
//
// Returns:  0 = Sucess
//          -1 = Error reading flash
//
//*****************************************************************************

int SysParamsRead(SYSPARMS* sp)
{
    InitSysDefaults(sp);

    EEPROMRead((uint32_t *)sp, 0, sizeof(SYSPARMS));

    if (sp->magic != MAGIC)
    {
        System_printf("Initializing Default System Parameters\n");
    	InitSysDefaults(sp);
    	return -1;
    }

	return 0;
}

//*****************************************************************************
// Write system parameters from our global settings buffer to EEPROM.
//
// Returns:  0 = Sucess
//          -1 = Error writing EEPROM data
//*****************************************************************************

int SysParamsWrite(SYSPARMS* sp)
{
    int32_t rc;

    sp->version = MAKEREV(FIRMWARE_VER, FIRMWARE_REV);
    sp->magic   = MAGIC;

    rc = EEPROMProgram((uint32_t *)sp, 0, sizeof(SYSPARMS));

    return rc;
}

//*****************************************************************************
// Set default runtime values
//*****************************************************************************

void InitSysDefaults(SYSPARMS* p)
{
    /* default servo parameters */
    p->magic        = MAGIC;
    p->version      = MAKEREV(FIRMWARE_VER, FIRMWARE_REV);
}

//*****************************************************************************
// This function reads the globally unique serial number
// from the I2C eprom/serial# part.
//*****************************************************************************

bool ReadSerialNumber(uint8_t ui8SerialNumber[16])
{
    bool            ret = false;
    uint8_t         txByte;
    I2C_Handle      handle;
    I2C_Params      params;
    I2C_Transaction i2cTransaction;

    /* default invalid serial number is all FF's */
    memset(ui8SerialNumber, 0xFF, sizeof(ui8SerialNumber));

    I2C_Params_init(&params);

    params.transferCallbackFxn = NULL;
    params.transferMode        = I2C_MODE_BLOCKING;
    params.bitRate             = I2C_100kHz;

    if ((handle = I2C_open(Board_I2C1, &params)) != NULL)
    {
        /* Note the Upper bit of the word address must be set
         * in order to read the serial number. Thus 80H sets
         * the starting address to zero prior to reading
         * the sixteen bytes of serial number data.
         */
        txByte = 0x80;

        i2cTransaction.slaveAddress = Board_AT24CS01_SERIAL_ADDR;
        i2cTransaction.writeBuf     = &txByte;
        i2cTransaction.writeCount   = 1;
        i2cTransaction.readBuf      = ui8SerialNumber;
        i2cTransaction.readCount    = 16;

        ret = I2C_transfer(handle, &i2cTransaction);

        I2C_close(handle);
    }

    return ret;
}

/* End-Of-File */
