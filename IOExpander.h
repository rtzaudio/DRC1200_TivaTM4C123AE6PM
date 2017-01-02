#ifndef DRC_1200_TIVATM4C123AE6PMI_IOEXPANDER_H_
#define DRC_1200_TIVATM4C123AE6PMI_IOEXPANDER_H_

typedef struct IOExpander_InitData {
    uint8_t	addr;
    uint8_t data;
} IOExpander_InitData;

typedef struct IOExpander_Object {
    SPI_Handle      		spiHandle;	/* Handle for SPI object */
    unsigned int    		boardSPI; 	/* Board SPI in Board.h */
    unsigned int    		boardCS;  	/* Board chip select in Board.h */
    IOExpander_InitData* 	initData;
    unsigned int			initDataCount;
} IOExpander_Object;

/*!
 *  @brief A handle that is returned from a AT45DB_open() call.
 */
typedef IOExpander_Object *IOExpander_Handle;

//*****************************************************************************
// Function Prototypes
//*****************************************************************************

void IOExpander_initialize(void);

bool SetTransportLEDMask(uint8_t maskSet, uint8_t maskClear);
uint8_t GetTransportLEDMask(void);
bool ReadTransportSwitches(uint8_t* pSwitchBits);

uint16_t GetButtonLEDMask(void);
bool SetButtonLEDMask(uint16_t maskSet, uint16_t maskClear);
bool ReadButtonSwitches(uint16_t* pSwitchBits);

#endif /*DRC_1200_TIVATM4C123AE6PMI_IOEXPANDER_H_*/
