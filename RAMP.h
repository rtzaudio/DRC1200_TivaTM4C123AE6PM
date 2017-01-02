/*
 * Remote Asyncronous Message Protocol
 *
 * Copyright (C) 2016, RTZ Professional Audio, LLC. ALL RIGHTS RESERVED.
 *
 *              *** RAMP Protocol Message Frame Structure ***
 *
 *                    +------------------------------+   byte
 *                +-- |    SOF PREAMBLE (MSB=0x89)   |    0
 *                |   +------------------------------+
 *                |   |    SOF PREAMBLE (LSB=0xFC)   |    1
 *     Preamble --+   +------------------------------+
 *                |   |      FRAME LENGTH (MSB)      |    2
 *                |   +------------------------------+
 *                +-- |      FRAME LENGTH (LSB)      |    3
 *                    +---+---+---+---+--------------+
 *                +-- | E | R | P | A |    TYPE      |    4
 *                |   +---+---+---+---+--------------+
 *                |   |           ADDRESS            |    5
 *       Header --+   +------------------------------+
 *                |   |          SEQUENCE#           |    6
 *                |   +------------------------------+
 *                +-- |       ACK/NAK SEQUENCE#      |    7
 *                    +------------------------------+
 *                +-- |       TEXT LENGTH (MSB)      |    8
 *                |   +------------------------------+
 *                |   |       TEXT LENGTH (LSB)      |    9
 *                |   +------------------------------+
 *         Text --+   |              .               |    .
 *                |   |              .               |    .
 *                |   |          TEXT DATA           |    .
 *                |   |              .               |    .
 *                +-- |              .               |    .
 *                    +------------------------------+
 *                +-- |          CRC (MSB)           |  10 + textlen
 *          CRC --+   +------------------------------+
 *                +-- |          CRC (LSB)           |  11 + textlen
 *                    +------------------------------+
 * 
 *    RAMP Frame Contents Description:
 * 
 *      * SOF: start of frame preamble 0x89FC identifier.
 * 
 *      * Frame length: Total length less preamble (includes CRC bytes)
 * 
 *      * Flags: E=ERROR, R=RESYNC, P=PRIORITY, A=ACK/NAK
 *
 *      * Type: 1 = Frame contains ACK-only
 *              2 = Frame contains NAK-only
 *              3 = Frame contains text msg only
 *              4 = Frame contains msg + piggyback ACK
 *              5 = Frame contains msg + piggybackNAK
 *              6 = Frame is Datagram text data only, no ACK req'd
 * 
 *      * Address: Specifies the node address
 *
 *      * Sequence#: Transmit frame sequence number (1-24)
 *       
 *      * ACK/NAK Seq#: Receive piggyback ACK/NAK sequence# (0 if none)
 *       
 *      * Text Length: length of text data segment (0-1024)
 *       
 *      * Text Data: Segment text bytes (if text-length nonzero)
 *       
 *      * CRC value: CRC-16 value calculated from offset 2 to end of text data
 *       
 *       
 *                  *** RAMP ACK/NAK Frame Structure ***
 *
 *                    +------------------------------+   byte
 *                +-- |    SOF PREAMBLE (MSB=0x89)   |    0
 *                |   +------------------------------+
 *                |   |    SOF PREAMBLE (LSB=0xFC)   |    1
 *     Preamble --+   +------------------------------+
 *                |   |     FRAME LENGTH (MSB=0)     |    2
 *                |   +------------------------------+
 *                +-- |     FRAME LENGTH (LSB=5)     |    3
 *                    +---+---+---+---+--------------+
 *                +-- | 0 | 0 | 0 | 1 |     TYPE     |    4
 *                |   +---+---+---+---+--------------+
 *       Header --+   |           ADDRESS            |    5
 *                |   +------------------------------+
 *                +-- |      ACK/NAK SEQUENCE#       |    6
 *                    +------------------------------+
 *                +-- |          CRC (MSB)           |    7
 *          CRC --+   +------------------------------+
 *                +-- |          CRC (LSB)           |    8
 *                    +------------------------------+
 *
 *    ACK/NAK Frame Content Description:
 *
 *      * SOF: start of frame preamble 0x89FC identifier.
 * 
 *      * Frame length: Length in bytes (always 5 for ACK/NAK only)
 * 
 *      * Type: frame type 1=ACK/2=NAK (always 11H or 12H)
 *
 *      * Address: Specifies the node address
 *
 *      * ACK/NAK Sequence: ACK/NAK frame sequence# (1-21)
 *
 *      * CRC value: CRC-16 value calculate from offset 2 to 6
 */

/*** RAMP Contstants and Defines *******************************************/

#define PREAMBLE_MSB			0x89		/* first byte of preamble SOF  */
#define PREAMBLE_LSB			0xFC		/* second byte of preamble SOF */

#define MAX_WINDOW              8       	/* maximum window size         */

#define PREAMBLE_OVERHEAD       4       	/* preamble overhead (SOF+LEN) */
#define HEADER_OVERHEAD         4       	/* frame header overhead       */
#define TEXT_OVERHEAD           2       	/* text length overhead        */
#define CRC_OVERHEAD            2       	/* crc overhead (CRC lsb+msb)  */
#define FRAME_OVERHEAD          ( PREAMBLE_OVERHEAD + HEADER_OVERHEAD + TEXT_OVERHEAD + CRC_OVERHEAD )

#define CRC_PHANTOM_BYTE		0x80

#define MIN_SEQ_NUM             1       	/* min/max frame sequence num   */
#define MAX_SEQ_NUM             ( 3 * MAX_WINDOW )
#define NULL_SEQ_NUM            ( (uint8_t)0 )

#define ACK_FRAME_LEN           5
#define MAX_TEXT_LEN            2048
#define MIN_FRAME_LEN           ( FRAME_OVERHEAD - PREAMBLE_OVERHEAD )
#define MAX_FRAME_LEN           ( MIN_FRAME_LEN + MAX_TEXT_LEN )

#define INC_SEQ_NUM(n)		    ( (uint8_t)((n >= MAX_SEQ_NUM) ? MIN_SEQ_NUM : n+1) )

/* Frame Type Flag Bits (upper nibble) */
#define F_ACKNAK        		0x10		/* ACK/NAK only frame flag bit */
#define F_PRIORITY      		0x20    	/* high priority message frame */
#define F_RESYNC        		0x40		/* resyncronize flag bit       */
#define F_ERROR         		0x80		/* frame error flag bit        */

#define FRAME_FLAG_MASK    		0xF0		/* flag mask is upper 4 bits   */

/* Frame Type Code (lower nibble) */
#define TYPE_ACK_ONLY   		1			/* ACK message frame only      */
#define TYPE_NAK_ONLY   		2			/* NAK message frame only      */
#define TYPE_MSG_ONLY   		3			/* message only frame          */
#define TYPE_MSG_ACK    		4			/* piggyback message plus ACK  */
#define TYPE_MSG_NAK    		5			/* piggyback message plus NAK  */
#define TYPE_DATAGRAM    		8			/* datagram message - dont ACK */

#define FRAME_TYPE_MASK    		0x0F		/* type mask is lower 4 bits   */

/* Error Code Constants */
#define ERR_TIMEOUT             1           /* comm port timeout           */
#define ERR_NO_PREAMBLE         2           /* 1st preamble byte not found */
#define ERR_BAD_PREAMBLE        3           /* 2nd preamble byte not found */
#define ERR_SHORT_FRAME         4           /* short rx-frame error        */
#define ERR_RX_OVERFLOW         5           /* rx buffer overflow          */
#define ERR_SEQ_NUM             6           /* bad sequence number         */
#define ERR_FRAME_TYPE          7           /* invalid frame type          */
#define ERR_FRAME_LEN           8           /* bad rx-frame length         */
#define ERR_TEXT_LEN            9           /* bad rx-text length          */
#define ERR_ACKNAK_LEN          10          /* bad rx-text length          */
#define ERR_CRC                 11          /* rx-frame checksum bad       */
#define ERR_SYNC                12          /* frame sync error            */

/*** RAMP Structure Definitions ********************************************/

/* Frame Control Block Structure */

typedef struct fcb_t {
    uint8_t     type;               /* frame type bits       */
    uint8_t     seqnum;             /* frame tx/rx seq#      */
    uint8_t     acknak;             /* frame ACK/NAK seq#    */
    uint8_t     address;            /* source tx/rx address  */
    uint8_t*	textbuf;            /* pointer to text buf   */
    uint16_t    textlen;            /* ptr text in frame     */
    uint16_t    framelen;         	/* frame len specifier   */
    uint16_t	crc;				/* remotes CRC value     */
} FCB;

/* RAMP Function Prototypes */

void RAMP_InitFcb(FCB* fcb);
int RAMP_FrameTx(UART_Handle handle, FCB *fcb, uint8_t *textbuf, int textlen);
int RAMP_FrameRx(UART_Handle handle, FCB *fcb, uint8_t *textbuf, int maxlen);

/* end-of-file */
