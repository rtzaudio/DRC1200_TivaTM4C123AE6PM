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
 *    ======== tcpEcho.c ========
 *    Contains BSD sockets code.
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/I2C.h>

/* Generic Includes */
#include <file.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdbool.h>

/* Graphiclib Header file */
#include <grlib/grlib.h>
#include "drivers/fema128x64.h"

/* PMX42 Board Header file */
#include "Board.h"
#include "DRC1200.h"
#include "DisplayTask.h"

/* Global context for drawing */
extern tContext g_context;

/* Handles created dynamically */
extern Mailbox_Handle g_mailboxRemote;

extern SYSDATA g_sysData;

extern tFont *g_psFontWDseg7bold24pt;

/* Static Module Globals */
uint32_t s_uScreenNum = 0;

/* Static Function Prototypes */
static int GetHexStr(char* pTextBuf, uint8_t* pDataBuf, int len);

//*****************************************************************************
// OLED Display Drawing task
//*****************************************************************************

Void DisplayTask(UArg arg0, UArg arg1)
{
    uint32_t secs = 0;
    bool screensave = FALSE;
    DisplayMessage msg;

    ClearDisplay();

    DisplayWelcome();

    Task_sleep(2000);

    while (true)
    {
    	/* Wait for a message up to 1 second */
        if (!Mailbox_pend(g_mailboxRemote, &msg, 1000))
        {
        	/* No message, blink the LED */
    		GPIO_toggle(Board_GPIO_LED1);

    		/* Check for display sleep timeout */
    		if (++secs >= 60)
    		{
    			/* power down and put the display in sleep mode */
    			FEMA128x64Sleep();
    			secs = 0;
    			screensave = TRUE;
    		}

        	continue;
        }

        /* Reset the screen saver timeout */
		secs = 0;

        /* Check if screen saver is active */
		if (screensave)
		{
			screensave = FALSE;
			/* Wakeup the screen and power it up */
			FEMA128x64Wake();
		}

		switch(msg.dispCommand)
		{
		case REFRESH:
		    GrFlush(&g_context);
		    break;

		case WAKE:
            secs = 0;
            screensave = TRUE;
            FEMA128x64Wake();
		    break;

        case SLEEP:
            secs = 0;
            screensave = FALSE;
            FEMA128x64Sleep();
            break;

        default:
            break;
        }
    }
}

//*****************************************************************************
// Format a data buffer into an ascii hex string.
//*****************************************************************************

int GetHexStr(char* pTextBuf, uint8_t* pDataBuf, int len)
{
    char fmt[8];
    uint32_t i;
    int32_t l;

    *pTextBuf = 0;
    strcpy(fmt, "%02X");

    for (i=0; i < len; i++)
    {
        l = sprintf(pTextBuf, fmt, *pDataBuf++);
        pTextBuf += l;

        if (((i % 2) == 1) && (i != (len-1)))
        {
            l = sprintf(pTextBuf, "-");
            pTextBuf += l;
        }
    }

    return strlen(pTextBuf);
}

//*****************************************************************************
//
//*****************************************************************************

void ClearDisplay(void)
{
    tRectangle rect = {0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1};
    GrContextForegroundSetTranslated(&g_context, 0);
    GrContextBackgroundSetTranslated(&g_context, 0);
    GrRectFill(&g_context, &rect);
}

//*****************************************************************************
//
//*****************************************************************************

void DisplayWelcome(void)
{
    int len;
    char buf[64];

    /* Set foreground pixel color on to 0x01 */
    GrContextForegroundSetTranslated(&g_context, 1);
    GrContextBackgroundSetTranslated(&g_context, 0);

    //tRectangle rect = {0, 0, SCREEN_WIDTH-1, SCREEN_HEIGHT-1};
    //GrRectDraw(&g_context, &rect);

    /* Setup font */
    uint32_t y;
    uint32_t height;
    uint32_t spacing = 2;

    /* Display the program version/revision */
    GrContextFontSet(&g_context, g_psFontCm28b);
    height = GrStringHeightGet(&g_context);
    y = 12;
    len = sprintf(buf, "DRC-1200");
    GrStringDrawCentered(&g_context, buf, len, SCREEN_WIDTH/2, y, FALSE);
    y += (height/2) + 4;

    /* Switch to fixed system font */
    GrContextFontSet(&g_context, g_psFontFixed6x8);
    height = GrStringHeightGet(&g_context);

    sprintf(buf, "Firmware v%d.%02d", FIRMWARE_VER, FIRMWARE_REV);
    GrStringDraw(&g_context, buf, -1, 25, y, 0);
    y += height + spacing + 4;

    /* Get the serial number string and display it */

    GetHexStr(buf, &g_sysData.ui8SerialNumber[0], 8);
    GrStringDraw(&g_context, buf, -1, 8, y, 0);
    y += height + spacing;

    GetHexStr(buf, &g_sysData.ui8SerialNumber[8], 8);
    GrStringDraw(&g_context, buf, -1, 8, y, 0);
    y += height + spacing;

    GrFlush(&g_context);
}

// End-Of-File
