/*
 * servo.h : created 7/30/99 10:50:41 AM
 *  
 * Copyright (C) 1999-2000, RTZ Audio. ALL RIGHTS RESERVED.
 *  
 * THIS MATERIAL CONTAINS  CONFIDENTIAL, PROPRIETARY AND TRADE
 * SECRET INFORMATION OF RTZ AUDIO. NO DISCLOSURE OR USE OF ANY
 * PORTIONS OF THIS MATERIAL MAY BE MADE WITHOUT THE EXPRESS
 * WRITTEN CONSENT OF RTZ AUDIO.
 */

#define DEBUG	1

/* Standard includes */
#include <file.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>

/* Standard Tivaware includes */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "inc/hw_i2c.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/i2c.h"
#include "driverlib/qei.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "Board.h"

/*** Global Constants ******************************************************/

/* version info */
#define FIRMWARE_VER        1           	/* firmware version */
#define FIRMWARE_REV        1          	/* firmware revision */

#define MAGIC               0xCEB0FACE  	/* magic number for EEPROM data */
#define MAKEREV(v, r)   	((v << 16) | (r & 0xFFFF))

#define MS(msec)  			( msec / portTICK_RATE_MS )

#define UNDEFINED   		(-1)

/*** System Structures *****************************************************/

/* Global Data */

typedef struct _SYSDATA
{
	uint32_t	dipSwitch;					/* 4 bit DIP switch settings */
	uint8_t		ui8SerialNumber[16];		/* 16 byte globally unique serial# */
} SYSDATA;

/* System Parameters stored in EPROM */

typedef struct _SYSPARMS
{
	unsigned long magic;
    unsigned long version;
    /*** GLOBAL PARAMETERS ***/
    long reserved10;
} SYSPARMS;

/* main.c */
void InitSysDefaults(SYSPARMS* p);
int SysParamsWrite(SYSPARMS* sp);
int SysParamsRead(SYSPARMS* sp);

/* End-Of-File */
