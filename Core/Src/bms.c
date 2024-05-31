/*
 * bms.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Nicolas
 */

#include "main.h"
#include "defines.h"
#include "bms.h"
#include "ltc.h"


void BMS_Init(Master **BMS) {
    *BMS = (Master*) calloc(1, sizeof(Master));


    LTC_config* config = (LTC_config*) calloc(1, sizeof(LTC_config));

    config->command = (LTC_command*) calloc(1, sizeof(LTC_command));

    LTC_Init(config);


    for (uint8_t i = 0; i < NUM_SLAVES; i++) {
        (*BMS)->slaves[i].config = config;
        (*BMS)->slaves[i].sensor.ADDR = i;
    }

    LTC_PEC_InitTable();
}


void ElectricalManagement(Master *BMS){
	LTC_SendBroadcastCommand(BMS->slaves[0].config, LTC_COMMAND_ADCV);
		uint16_t temp_minV = UINT16_MAX;
		uint16_t temp_maxV = 0;
		for(uint8_t i = 0; i < NUM_SLAVES; i++) {
			LTC_Read(LTC_READ_CELL, &(BMS->slaves[i]));
			if(BMS->slaves[i].sensor.V_MIN < temp_minV)
				temp_minV = BMS->slaves[i].sensor.V_MIN;
			if(BMS->slaves[i].sensor.V_MAX > temp_maxV)
				temp_maxV = BMS->slaves[i].sensor.V_MAX;
		}
		//BMS->maxCellVoltage = temp_maxV;
		BMS->maxCellVoltage = temp_maxV;
		BMS->minCellVoltage = temp_minV;
		BMS->deltaVoltage = BMS->maxCellVoltage - BMS->minCellVoltage;
}

