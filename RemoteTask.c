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
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Gate.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

#include <file.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include <driverlib/sysctl.h>

/* Graphiclib Header file */
#include <grlib/grlib.h>
#include "drivers/fema128x64.h"

/* PMX42 Board Header file */
#include "Board.h"
#include "DRC1200.h"
#include "RAMP.h"
#include "RemoteTask.h"

#define MAX_MSGS	8

/* External Data Items */

extern tContext g_context;
extern SYSDATA g_sysData;

/* Global Data Items */

UART_Handle g_handleUart422;

static Queue_Handle g_queTx;
static Queue_Handle g_queFree;

static Semaphore_Handle g_semTx;
static Semaphore_Handle g_semFree;

/* RAMP frame control blocks */
static FCB g_RxFcb;
static FCB g_TxFcb;

/* Static Function Prototypes */

void DispatchRxMessage(uint8_t *textbuf, size_t textlen);
void AckReceivedPacket(uint8_t type, uint8_t acknak);

//*****************************************************************************
// Dispatch received messages
//*****************************************************************************

void DispatchRxMessage(uint8_t *textbuf, size_t textlen)
{
	/* Refresh OLED display */
    GrFlush(&g_context);
}

//*****************************************************************************
// Initalize the remote control RS-422 serial message packet service.
//*****************************************************************************

Void RemoteInit()
{
    Int i;
	Task_Handle task;
    Error_Block eb;
	Task_Params taskParams;
	UART_Params uartParams;
	Semaphore_Params semParams;
    RemoteMsgObj *msg;

    /* Create a binary Semaphore for the tx task */
    Error_init(&eb);
    g_semTx = Semaphore_create(0, NULL, &eb);

    /* Create a counting Semaphore for the free message buffer queue */
    Error_init(&eb);
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_COUNTING;
    g_semFree = Semaphore_create(MAX_MSGS, NULL, &eb);

    Error_init(&eb);
    g_queFree = Queue_create(NULL, &eb);

    Error_init(&eb);
    g_queTx = Queue_create(NULL, &eb);

    /* Allocate queue message buffer for packet tx task */
    Error_init(&eb);
    msg = (RemoteMsgObj*)Memory_alloc(NULL, sizeof(RemoteMsgObj) * MAX_MSGS, 0, &eb);
    if (msg == NULL) {
    	System_abort("Memory allocation failed");
    }

    /* Put all messages on freeQueue */
    for (i = 0; i < MAX_MSGS; msg++, i++) {
    	Queue_put(g_queFree, (Queue_Elem*)msg);
    }

    /*
     * Open the UART for RS-422 communications
     */

	UART_Params_init(&uartParams);

	uartParams.readMode       = UART_MODE_BLOCKING;
	uartParams.writeMode      = UART_MODE_BLOCKING;
	uartParams.readTimeout    = 1000;					// 1 second read timeout
	uartParams.writeTimeout   = BIOS_WAIT_FOREVER;
	uartParams.readCallback   = NULL;
	uartParams.writeCallback  = NULL;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.writeDataMode  = UART_DATA_BINARY;
	uartParams.readDataMode   = UART_DATA_BINARY;
	uartParams.readEcho       = UART_ECHO_OFF;
	uartParams.baudRate       = 115200;
	uartParams.stopBits       = UART_STOP_ONE;
	uartParams.parityType     = UART_PAR_NONE;

	g_handleUart422 = UART_open(Board_UART_RS422, &uartParams);

	if (g_handleUart422 == NULL)
	    System_abort("Error initializing UART\n");

	/* Assert RS-422 DE & RE pins to enable */
    GPIO_write(DRC1200_GPIO_RS422_RE, PIN_LOW);
    GPIO_write(DRC1200_GPIO_RS422_DE, PIN_HIGH);

    /* Create the RAMP packet receive tasks */
    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.priority = 10;
    task = Task_create(RemoteRxTask, &taskParams, &eb);
  	if (task == NULL)
     	System_abort("Error creating RAMP RX task\n");

    /* Create the RAMP packet transmit task */
  	Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.priority = 11;
    task = Task_create(RemoteTxTask, &taskParams, &eb);
  	if (task == NULL)
     	System_abort("Error creating RAMP TX task\n");
}

//*****************************************************************************
// This function receives incoming packets and dispatches accordingly.
//*****************************************************************************

Void RemoteRxTask(UArg arg0, UArg arg1)
{
	int rc;

	/* Initialize the tx FCB structure */
	RAMP_InitFcb(&g_RxFcb);

	/* Begin the packet receive task loop */

    while (true)
    {
    	/* Attempt to read a valid RAMP packet frame from the serial port.
    	 * If we get a packet, then check it's TYPE and handle accordingly.
    	 */
    	rc = RAMP_FrameRx(g_handleUart422, &g_RxFcb, SSD1309GetScreenBuffer(), SSD1309GetScreenBufferSize());

    	if (rc != 0)
    	{
        	if (rc != ERR_TIMEOUT)
        	{
        		System_printf("RAMP RX ERROR %d\n", rc);
        		System_flush();
        	}
        	continue;
    	}

    	/* DEBUG - Rx end, turn the LED off */
    	GPIO_write(Board_GPIO_LED1, Board_LED_OFF);

    	/* We've received a valid RAMP packet of some sort, check the message
    	 * type and handle accordingly.
    	 */

    	switch(g_RxFcb.type & FRAME_TYPE_MASK)
    	{
    	case TYPE_ACK_ONLY:		/* ACK message frame only      */
    		System_printf("ACK-MSG %d\n", g_RxFcb.acknak);
    		System_flush();
    		/* FIX: need to scan resend list and remove packet ACK'ed here */
    		break;

    	case TYPE_NAK_ONLY:		/* NAK message frame only      */
    		System_printf("NAK-MSG %d\n", g_RxFcb.acknak);
    		System_flush();
    		/* FIX: need to scan resend list and resend NAK'd packet */
    		break;

    	case TYPE_MSG_ACK:		/* message plus piggyback ACK  */
    		/* FIX: need to scan resend list and remove packet ACK'ed here */
    		/* ACK the message received */
    		AckReceivedPacket(TYPE_ACK_ONLY, g_RxFcb.seqnum);
    		/* Dispatch the message received */
    		DispatchRxMessage(g_RxFcb.textbuf, g_RxFcb.textlen);
    		System_printf("RX-MSG+ACK %d:%d bytes\n", g_RxFcb.seqnum, g_RxFcb.textlen);
    		System_flush();
    		break;

    	case TYPE_MSG_NAK:		/* message plus piggyback NAK  */
    		/* ACK the message received */
    		AckReceivedPacket(TYPE_ACK_ONLY, g_RxFcb.seqnum);
    		/* Dispatch the message received */
    		DispatchRxMessage(g_RxFcb.textbuf, g_RxFcb.textlen);
    		System_printf("RX-MSG+NAK %d:%d bytes\n", g_RxFcb.seqnum, g_RxFcb.textlen);
    		System_flush();
    		break;

    	case TYPE_MSG_ONLY:		/* message only frame          */
    		/* ACK the message received */
    		AckReceivedPacket(TYPE_ACK_ONLY, g_RxFcb.seqnum);
    		/* Dispatch the message received */
    		DispatchRxMessage(g_RxFcb.textbuf, g_RxFcb.textlen);
    		System_printf("RX-MSG %d:%d bytes\n", g_RxFcb.seqnum, g_RxFcb.textlen);
    		System_flush();
    		break;

    	case TYPE_DATAGRAM:		/* datagram, no ACK required   */
    		/* Dispatch the message received */
    		DispatchRxMessage(g_RxFcb.textbuf, g_RxFcb.textlen);
    		System_printf("RX-DGRAM %d:%d bytes\n", g_RxFcb.seqnum, g_RxFcb.textlen);
    		System_flush();
    		break;
    	}
    }
}

//*****************************************************************************
// This function queues an ACK or NAK only packet to the transmitter task.
//*****************************************************************************

void AckReceivedPacket(uint8_t type, uint8_t acknak)
{
	RemoteMsgObj *msg;

	if (Semaphore_pend(g_semFree, 100))
	{
		/* Get next free message buffer */
		msg = Queue_get(g_queFree);

		msg->type   = type;
		msg->acknak = acknak;

		/* Queue it to the transmitter task */
		Queue_put(g_queTx, (Queue_Elem*)msg);

		/* Post semaphore for tx to wake */
		Semaphore_post(g_semTx);
    }
}

//*****************************************************************************
// This function transmits packets and pending ACK/NAK
//*****************************************************************************

Void RemoteTxTask(UArg arg0, UArg arg1)
{
	uint8_t seqnum = 1;
	RemoteMsgObj *msg;

	/* Initialize the tx FCB structure */
	RAMP_InitFcb(&g_TxFcb);

	/* Begin the packet transmit task loop */

	for (;;)
	{
		/* Wait for semaphore to be posted to the writer task. */

		if (Semaphore_pend(g_semTx, 5000))
		{
			/* Get the next msg to transmit */
			msg = Queue_get(g_queTx);

			/* Set address byte from DIP switches */
			g_TxFcb.address = g_sysData.dipSwitch;

			/* Tx starting, turn the LED on */
			GPIO_write(Board_GPIO_LED2, Board_LED_ON);

			switch(msg->type)
			{
			case TYPE_ACK_ONLY:
				g_TxFcb.type     = F_ACKNAK | TYPE_ACK_ONLY;
				g_TxFcb.acknak   = msg->acknak;
				System_printf("TX-ACK %d\n", g_TxFcb.acknak);
				System_flush();
				/* Transmit the ACK packet out */
				RAMP_FrameTx(g_handleUart422, &g_TxFcb, NULL, 0);
				break;

			case TYPE_NAK_ONLY:
				g_TxFcb.type     = F_ACKNAK | TYPE_NAK_ONLY;
				g_TxFcb.acknak   = msg->acknak;
				System_printf("TX-NAK %d\n", g_TxFcb.acknak);
				System_flush();
				/* Transmit the NAK packet out */
				RAMP_FrameTx(g_handleUart422, &g_TxFcb, NULL, 0);
				break;

			default:
				/* Transmit the full packet out */
				g_TxFcb.type     = TYPE_MSG_ONLY;
				g_TxFcb.seqnum   = INC_SEQ_NUM(seqnum);
				System_printf("TX-MSG %d\n", g_TxFcb.type);
				System_flush();
				RAMP_FrameTx(g_handleUart422, &g_TxFcb, msg->text, msg->textlen);
				break;
			}

			/* Tx complete, turn the LED off */
			GPIO_write(Board_GPIO_LED2, Board_LED_OFF);

			/* Place the queue message back in the free list */
			Queue_put(g_queFree, (Queue_Elem*)msg);

			/* Post the free list semaphore */
			Semaphore_post(g_semFree);
		}
		else
		{
			/* Transmit queue is empty, check for any unacknowledged packets
			 * that may be pending. If packets are still in the pending list
			 * and have timed out, then we need to retransmit these until
			 * acknowledged or max retries has occurred.
			 */
		}
	}
}

// End-Of-File
