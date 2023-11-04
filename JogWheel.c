/* ============================================================================
 *
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2016, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * ============================================================================
 *
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
 * ============================================================================ */

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* TI-RTOS Driver files */
#include <ti/drivers/gpio.h>
#include <ti/drivers/spi.h>
#include <ti/drivers/i2c.h>
#include <ti/drivers/uart.h>

#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/qei.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>

/* Generic Includes */
#include <file.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

/* Project specific includes */
#include "DRC1200.h"
#include "JogWheel.h"

/* Hwi_Struct used in the init Hwi_construct call */
static Hwi_Struct qei0HwiStruct;

/* Interrupt Handlers */
static Void JogwheelHwi(UArg arg);

/*****************************************************************************
 * QEI Configuration and interrupt handling
 *****************************************************************************/

void Jogwheel_initialize(void)
{
	Error_Block eb;
	Hwi_Params  hwiParams;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    /* Initialize the QEI quadrature encoder interface for operation. We are
     * using QEI0 for the jog wheel. This is used for velocity and direction
     * information (not position). The QEI module supports capturing a measure
     * of the encoder velocity, which is simply a count of encoder pulses
     * during a fixed time period; the number of pulses is directly proportional
     * to the encoder speed. Note that the velocity capture can only operate when
     * the position capture is enabled.
     */

    /* Enable pin PC6 for QEI1 PHB1 */
    GPIOPinConfigure(GPIO_PC6_PHB1);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_6);

    /* Enable pin PC5 for QEI1 PHA1 */
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5);

    /* Configure the quadrature encoder to capture edges on both signals and
     * maintain an absolute position. Using a 32 CPR encoder at four edges
     * per line, there are 128 pulses per revolution; therefore set the
     * maximum position to 127 since the count is zero based.
     */
    QEIConfigure(JOGWHEEL_BASE,
                 QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_RESET | QEI_CONFIG_NO_SWAP,
                 JOGWHEEL_EDGES_PER_REV - 1);

    /* This function configures the operation of the velocity capture portion
     * of the quadrature encoder. The position increment signal is pre-divided
     * as specified by ulPreDiv before being accumulated by the velocity
     * capture. The divided signal is accumulated over ulPeriod system clock
     * before being saved and resetting the accumulator.
     */

    /* Configure the Velocity capture period - 80,000,000 is 1000ms at 80MHz */
    QEIVelocityConfigure(JOGWHEEL_BASE, QEI_VELDIV_1, 8000000);   //JOGWHEEL_TIMER_PERIOD);

    /* Enable both quadrature encoder interfaces */
    QEIEnable(JOGWHEEL_BASE);

    /* Enable both quadrature velocity capture interfaces. */
    QEIVelocityEnable(JOGWHEEL_BASE);

    /* Set initial position to zero */
    QEIPositionSet(JOGWHEEL_BASE, 0);

    /* Now we construct the interrupt handler objects for TI-RTOS */

    Error_init(&eb);
    Hwi_Params_init(&hwiParams);
    Hwi_construct(&(qei0HwiStruct), INT_QEI1, JogwheelHwi, &hwiParams, &eb);
    if (Error_check(&eb)) {
        System_abort("Couldn't construct DMA error hwi");
    }

    //QEIIntEnable(JOGWHEEL_BASE, QEI_INTERROR | QEI_INTDIR);
}

/*****************************************************************************
 * Read the jog wheel position or set new position.
 *****************************************************************************/

uint32_t Jogwhell_getPosition(void)
{
    return QEIPositionGet(JOGWHEEL_BASE);
}

void Jogwhell_setPosition(uint32_t position)
{
    return QEIPositionSet(JOGWHEEL_BASE, position);
}

/*****************************************************************************
 * Read the jog wheel direction and velocity information.
 *****************************************************************************/

void Jogwheel_read(uint32_t* velocity, int32_t* direction)
{
    if (velocity)
        *velocity = QEIVelocityGet(JOGWHEEL_BASE);

    if (direction)
        *direction = QEIDirectionGet(JOGWHEEL_BASE);
}

/*****************************************************************************
 * QEI Interrupt Handler
 *****************************************************************************/

Void JogwheelHwi(UArg arg)
{
	UInt key;
    unsigned long ulIntStat;

    /* Get and clear the current interrupt source(s) */
    ulIntStat = QEIIntStatus(JOGWHEEL_BASE, true);
    QEIIntClear(JOGWHEEL_BASE, ulIntStat);

    /* Determine which interrupt occurred */

    if (ulIntStat & QEI_INTERROR)       	/* phase error detected */
    {
    	key = Hwi_disable();
    	//g_servo.qei_supply_error_cnt++;
    	Hwi_restore(key);
    }
    else if (ulIntStat & QEI_INTTIMER)  	/* velocity timer expired */
    {

    }
    else if (ulIntStat & QEI_INTDIR)    	/* direction change */
    {

    }

    QEIIntEnable(JOGWHEEL_BASE, ulIntStat);
}

/* End-Of-File */
