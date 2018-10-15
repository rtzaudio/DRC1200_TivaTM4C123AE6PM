/* ============================================================================
 *
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2018, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * ============================================================================ */

#ifndef _JOGWHEEL_H_
#define _JOGWHEEL_H_

void Jogwheel_initialize(void);

void Jogwheel_read(uint32_t* velocity, int32_t* direction);

#endif /* _JOGWHEEL_H_ */
