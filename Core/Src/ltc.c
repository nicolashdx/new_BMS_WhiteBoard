/*
 * ltc.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Nicolas
 */

#include "main.h"
#include "defines.h"
#include "ltc.h"

#define BYTESWAP(word) ((word >> 8) + (word << 8))

extern SPI_HandleTypeDef hspi1;
static uint16_t pec_table[LTC_PEC_TABLE_LENGTH];

void LTC_Init(LTC_config *config) {
	config->GPIO   = ALL_GPIOS_READ;
	config->REFON  = REFERENCE_SHUTS_DOWN_AFTER_CONVERSIONS;
	config->SWTRD  = SOFTWARE_TIMER_ENABLE_PIN_LOW;
	config->ADCOPT = SELECT_ADC_MODES_FAST;
	config->VUV    = DEFULT_VOLTAGE;
	config->VOV    = DEFULT_VOLTAGE;
	config->DCTO   = DISCHARGE_DISABLE;
	config->command->MD  = MD_FILTRED;
	config->command->DCP = DCP_PERMITED;
	LTC_SendBroadcastCommand(config, LTC_COMMAND_WRCOMM);
}

void LTC_PEC_InitTable() {
	uint16_t remainder;
	for(int i = 0; i < LTC_PEC_TABLE_LENGTH; i++) {
		remainder = i << 7;
		for(int bit = 8; bit > 0; --bit) {
			if(remainder & 0x4000) {
				remainder = remainder << 1;
				remainder = remainder ^ 0x4599;
			}
			else
				remainder = remainder << 1;
		}
		pec_table[i] = remainder&0xFFFF;
	}
}

void LTC_Wait(Slave *slave) {
	do{
		LTC_SendAddressedCommand(slave, LTC_COMMAND_PLADC);
	}while(!slave->config->ADC_READY);
}

void LTC_Read(uint8_t LTC_READ, Slave *slave){
	slave->config->command->BROADCAST = false;

}

void LTC_SendBroadcastCommand(LTC_config *config, uint16_t command_name) {
	uint16_t tx_data[4] = {0, 0, 0, 0};
	uint16_t rx_data[4] = {0, 0, 0, 0};
	config->command->NAME = command_name;
	LTC_Communication(config, tx_data, rx_data);
}

void LTC_SendAddressedCommand(Slave *slave, uint16_t command_name) {
	uint16_t tx_data[4] = {0, 0, 0, 0};
	uint16_t rx_data[4] = {0, 0, 0, 0};

	slave->config->command->NAME = command_name;
	LTC_ConfigCommandName(slave);

	if((slave->config->command->NAME & 0x07FF) == LTC_COMMAND_WRCFGA){
		LTC_WriteConfigRegister(slave, tx_data);
	}

	LTC_Communication(slave->config, tx_data, rx_data);
	LTC_ReceiveMessage(slave, rx_data);
}

void LTC_ReceiveMessage(Slave* slave, uint16_t rx_data[4]) {
	switch(slave->config->command->NAME & 0x07FF) {

	case LTC_COMMAND_RDCFGA:
		slave->config->ADCOPT = (rx_data[0] & 0x1);
		slave->config->SWTRD  = (rx_data[0] >> 1) & 0x1;
		slave->config->REFON  = (rx_data[0] >> 2) & 0x1;
		slave->config->GPIO   = (rx_data[0] >> 3) & 0x1F;
		slave->config->VUV    = (rx_data[0] >> 8) | (rx_data[1] & 0x000F);
		slave->config->VOV    = (rx_data[1] >> 4);
		slave->config->DCTO   = (rx_data[2] >> 12);
		break;

	case LTC_COMMAND_RDCVA:
		slave->sensor.CELL_VOLTAGES[0] = rx_data[0];
		slave->sensor.CELL_VOLTAGES[1] = rx_data[1];
		slave->sensor.CELL_VOLTAGES[2] = rx_data[2];
		break;

	case LTC_COMMAND_RDCVB:
		slave->sensor.CELL_VOLTAGES[3] = rx_data[0];
		slave->sensor.CELL_VOLTAGES[4] = rx_data[1];
		slave->sensor.CELL_VOLTAGES[5] = rx_data[2];
		break;

	case LTC_COMMAND_RDCVC:
		slave->sensor.CELL_VOLTAGES[6] = rx_data[0];
		slave->sensor.CELL_VOLTAGES[7] = rx_data[1];
		slave->sensor.CELL_VOLTAGES[8] = rx_data[2];
		break;

	case LTC_COMMAND_RDCVD:
		slave->sensor.CELL_VOLTAGES[9]  = rx_data[0];
		slave->sensor.CELL_VOLTAGES[10] = rx_data[1];
		slave->sensor.CELL_VOLTAGES[11] = rx_data[2];
		break;

	case LTC_COMMAND_RDAUXA:
		slave->sensor.GPIO_VOLTAGES[0] = rx_data[0];
		slave->sensor.GPIO_VOLTAGES[1] = rx_data[1];
		slave->sensor.GPIO_VOLTAGES[2] = rx_data[2];
		break;

	case LTC_COMMAND_RDAUXB:
		slave->sensor.GPIO_VOLTAGES[3] = rx_data[0];
		slave->sensor.GPIO_VOLTAGES[4] = rx_data[1];
		slave->sensor.REF_VOLTAGE = rx_data[2];
		break;

	case LTC_COMMAND_RDSTATA:
		slave->sensor.SOC  = rx_data[0] * 0.2;
		slave->sensor.INTERNAL_TEMP = rx_data[1] * 7.5;
		slave->sensor.ANALOG_VOLTAGE   = rx_data[2];
		break;

	case LTC_COMMAND_RDSTATB:
		slave->sensor.DIGITAL_VOLTAGE = rx_data[0];
		uint32_t flag = rx_data[1] | (uint32_t)rx_data[2] << 16;
		for (uint8_t j = 0; j < NUM_CELLS_PER_SLAVE; j++)
			slave->sensor.V_ERROR[j] = (flag >> (j * 2)) & 0x3;
		break;

	case LTC_COMMAND_ADCV	:
	case LTC_COMMAND_ADOW	:
	case LTC_COMMAND_CVST	:
	case LTC_COMMAND_ADAX	:
	case LTC_COMMAND_AXST	:
	case LTC_COMMAND_ADSTAT	:
	case LTC_COMMAND_STATST	:
	case LTC_COMMAND_ADCVAX	:
		slave->config->ADC_READY = false;
		break;

	case LTC_COMMAND_PLADC	:
		if(rx_data[0] == 0 || rx_data[1] == 0 || rx_data[2] == 0)
			slave->config->ADC_READY = false;
		else
			slave->config->ADC_READY = true;
		break;

	case LTC_COMMAND_DIAGN	:
		break;
	case LTC_COMMAND_WRCOMM	:
		break;
	case LTC_COMMAND_RDCOMM	:
		break;
	case LTC_COMMAND_STCOMM	:
		break;
	default:
		break;
	}
}

void LTC_StartTrasmission(){
	LTC_ChipSelect(RESET);
}

void LTC_EndTramission() {
	LTC_ChipSelect(SET);
}

void LTC_Communication(LTC_config *config, uint16_t* tx_data, uint16_t* rx_data) {
	uint16_t command = LTC_MakeCommand(config->command);
	LTC_WakeUp();
	LTC_StartTrasmission();
	LTC_TransmitCommand(command);
	LTC_TransmitReceive(command, tx_data, rx_data);
	LTC_EndTramission();
}

void LTC_ConfigCommandName(Slave* slave) {
	slave->config->command->NAME |= ((slave->sensor.ADDR & (0x1111 * ~slave->config->command->BROADCAST)) | ~slave->config->command->BROADCAST << 4) << 11;
}

void LTC_WriteConfigRegister(Slave* slave, uint16_t *tx_data) {
	tx_data[0] = (slave->config->ADCOPT << 8) | (slave->config->SWTRD << 9) | (slave->config->REFON << 10) | (slave->config->GPIO << 11) | (slave->config->VUV);
	tx_data[1] = (slave->config->VUV >> 8) | (slave->config->VOV << 4);
	tx_data[2] |= ((slave->sensor.CELL_TO_DISCHARGE & 0xff) << 8) | ((slave->sensor.CELL_TO_DISCHARGE & 0xf00) >> 8) | ((slave->config->DCTO & 0xf) << 4);
}

void LTC_ChipSelect(uint8_t level) {
	HAL_GPIO_WritePin(ISOSPI_CS_GPIO_Port, ISOSPI_CS_Pin , level);
}


uint16_t LTC_SPI(uint16_t Tx_data) {
	uint16_t Rx_data = 0;
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *) &Tx_data, (uint8_t *) &Rx_data, 1, 50);
	return(BYTESWAP(Rx_data));
}

void LTC_WakeUp() {
	LTC_StartTrasmission();
	LTC_SPI(0);
	LTC_EndTramission();
}

uint16_t LTC_MakeCommand(LTC_command *command) {
	switch(command->NAME) {
		case LTC_COMMAND_ADCV:
			return command->NAME | command->MD | command->DCP | command->CH;
			break;

		case LTC_COMMAND_ADOW:
			return command->NAME | command->MD | command->PUP | command->DCP | command->CH;
			break;

		case LTC_COMMAND_CVST:
		case LTC_COMMAND_AXST:
		case LTC_COMMAND_STATST:
			return command->NAME | command->MD | command->ST;
			break;

		case LTC_COMMAND_ADAX	:
			return command->NAME | command->MD | command->CHG;
			break;

		case LTC_COMMAND_ADSTAT	:
			return command->NAME | command->MD | command->CHST;
			break;

		case LTC_COMMAND_ADCVAX	:
			return command->NAME | command->MD | command->CHG;
			break;

		default:
			return command->NAME;
			break;
	}
}

uint16_t LTC_PEC(uint16_t *data , uint8_t len) {
	int32_t remainder, address;
	remainder = LTC_PEC_SEED;
	for (uint8_t i = 0; i < len; i++) {
		address   = ((remainder >> 7) ^ ((data[i] >> 8) & 0xFF)) & 0xFF; //calculate PEC table address
		remainder = (remainder << 8 ) ^ pec_table[address];
		address   = ((remainder >> 7) ^ (data[i] & 0xFF)) & 0xFF;    	 //calculate PEC table address
		remainder = (remainder << 8 ) ^ pec_table[address];
	}
	return (remainder * 2); //The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}

void LTC_TransmitCommand(uint16_t command) {
	uint16_t pec = LTC_PEC(&command, 1);
	LTC_SPI(command);
	LTC_SPI(pec);
}

void LTC_TransmitReceive(uint16_t command, uint16_t* tx_data, uint16_t* rx_data) {
	if((command & 0x07FF) == LTC_COMMAND_WRCFGA) {
		uint16_t pec = LTC_PEC(tx_data, 3);
		tx_data[3] = pec;
	}
	if((tx_data[0] & 0x07FF) < LTC_COMMAND_ADCV) {
		for (uint8_t i = 0; i < SPI_BUFFER_LENGTH; ++i) {
			rx_data[i] = LTC_SPI(tx_data[i]);
		}
	}
}


