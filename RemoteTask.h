/*
 * PMX42.h : created 5/18/2015
 *
 * Copyright (C) 2015, Robert E. Starr. ALL RIGHTS RESERVED.
 *
 * THIS MATERIAL CONTAINS  CONFIDENTIAL, PROPRIETARY AND TRADE
 * SECRET INFORMATION. NO DISCLOSURE OR USE OF ANY
 * PORTIONS OF THIS MATERIAL MAY BE MADE WITHOUT THE EXPRESS
 * WRITTEN CONSENT OF THE AUTHOR.
 */

#ifndef _REMOTETASK_H_
#define _REMOTETASK_H_

/*** CONSTANTS AND CONFIGURATION *******************************************/


/*** DISPLAY MESSAGE STRUCTURES ********************************************/

typedef enum RemoteCommand{
    DISPLAY_REFRESH,
    DISPLAY_WAKE,
    DISPLAY_SLEEP,
} RemoteCommand;

typedef struct RemoteMessage{
    RemoteCommand	command;
    uint32_t		param1;
    uint32_t		param2;
} RemoteMessage;

/*** FUNCTION PROTOTYPES ***************************************************/

void ClearDisplay();
void DisplayWelcome();
Void RemoteTaskFxn(UArg arg0, UArg arg1);

#endif /* _REMOTETASK_H_ */
