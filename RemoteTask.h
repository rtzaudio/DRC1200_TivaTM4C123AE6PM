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

#ifndef __REMOTETASK_H
#define __REMOTETASK_H

/*** CONSTANTS AND CONFIGURATION *******************************************/

typedef struct RemoteMsgObj {
	Queue_Elem	elem;			/* first field for Queue     */
	uint8_t		type;			/* tx message type           */
	uint8_t		acknak;			/* tx ack/nak sequence num   */
	uint16_t	textlen;		/* text len for buffer below */
	uint8_t		text[16];		/* text data buffer to send  */
} RemoteMsgObj, *RemoteMsg;

/*** FUNCTION PROTOTYPES ***************************************************/

Void RemoteInit();
Void RemoteRxTask(UArg arg0, UArg arg1);
Void RemoteTxTask(UArg arg0, UArg arg1);

#endif /* __REMOTETASK_H */
