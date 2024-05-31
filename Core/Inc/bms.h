/*
 * bms.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Nicolas
 */

#ifndef BMS_H
#define BMS_H

#include "stdlib.h"
#include "ltc.h"
#include "defines.h"

typedef struct Master {
	uint16_t maxCellVoltage;
	uint16_t minCellVoltage;
	uint16_t deltaVoltage;

	Slave slaves[NUM_SLAVES];
} Master;

void BMS_Init(Master **BMS);
void ElectricalManagement(Master *BMS);

#endif
