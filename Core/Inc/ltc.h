/*
 * bms.h
 *
 *  Created on: Jan 20, 2024
 *      Author: Nicolas
 */

#ifndef LTC_H
#define LTC_H

#include "stdlib.h"
#include "defines.h"
#include "stdbool.h"

#define LTC_COMMAND_WRCFGA 		0b00000000001	// Write Configuration Register Group A
#define LTC_COMMAND_WRCFGB 		0b00000000100	// Write Configuration Register Group B

#define LTC_COMMAND_RDCFGA 		0b00000000010	// Read Configuration Register Group A
#define LTC_COMMAND_RDCFGB 		0b00000100110	// Read Configuration Register Group B

#define LTC_COMMAND_RDCVA 		0b00000000100	// Read Cell Voltage Register Group A
#define LTC_COMMAND_RDCVB 		0b00000000110	// Read Cell Voltage Register Group B
#define LTC_COMMAND_RDCVC 		0b00000001000	// Read Cell Voltage Register Group C
#define LTC_COMMAND_RDCVD 		0b00000001010	// Read Cell Voltage Register Group D
#define LTC_COMMAND_RDCVE 		0b00000001001	// Read Cell Voltage Register Group E
#define LTC_COMMAND_RDCVF 		0b00000001011	// Read Cell Voltage Register Group F

#define LTC_COMMAND_RDAUXA		0b00000001100	// Read Auxiliary Register Group A
#define LTC_COMMAND_RDAUXB		0b00000001110	// Read Auxiliary Register Group B
#define LTC_COMMAND_RDAUXC		0b00000001101	// Read Auxiliary Register Group C
#define LTC_COMMAND_RDAUXD		0b00000001111	// Read Auxiliary Register Group D

#define LTC_COMMAND_RDSTATA		0b00000010000	// Read Status Register Group A
#define LTC_COMMAND_RDSTATB		0b00000010010	// Read Status Register Group B

#define LTC_COMMAND_WRSCTRL		0b00000010100	// Write S Control Register Group
#define LTC_COMMAND_WRPWM		0b00000100000	// Write PWM Register Group
#define LTC_COMMAND_WRPSB		0b00000011100	// Write PWM/S Register Group B

#define LTC_COMMAND_RDSCTRL		0b00000010110	// Read S Control Register Group
#define LTC_COMMAND_RDPWM		0b00000100010	// Read PWM Register Group
#define LTC_COMMAND_RDPSB		0b00000011110	// Read PWM/S Register Group B

#define LTC_COMMAND_STSCTRL		0b00000011001	// Start S Control Pulsing and Poll Status
#define LTC_COMMAND_CLRSCTRL 	0b00000011000	// Clear S Control Register Group

#define LTC_COMMAND_ADCV        0b01001100000	// Start Cell Voltage ADC Conversion and Poll Status
#define LTC_COMMAND_ADOW        0b01000101000	// Start Open Wire ADC Conversion and Poll Status
#define LTC_COMMAND_CVST        0b01000000111	// Start Self-Test Cell Voltage Conversion and Poll Status
#define LTC_COMMAND_ADOL        0b01000000001	// Start Overlap Measurements of Cell 7 and Cell 13 Voltages
#define LTC_COMMAND_ADAX        0b10001100000	// Start GPIOs ADC Conversion and Poll Status
#define LTC_COMMAND_ADAXD		0b10000000000	//	Start GPIOs ADC Conversion and Poll Status
#define LTC_COMMAND_AXST        0b10000000111	// Start Self-Test GPIOs Conversion and Poll Status
#define LTC_COMMAND_ADSTAT      0b10001101000	// Start Status group ADC Conversion and Poll Status
#define LTC_COMMAND_ADSTATD		0b10000001000	// Start Status Group ADC Conversion with DR and Poll Status
#define LTC_COMMAND_STATST      0b10000001111	// Start Self-Test Status group Conversion and Poll Status
#define LTC_COMMAND_ADCVAX      0b10001101111	// Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
#define LTC_COMMAND_ADCVSC		0b10001100111	// Start Combined Cell Voltage and SC Conversion and Poll Status

#define LTC_COMMAND_CLRCELL     0b11100010001	// Clear Cell Voltage Register Group
#define LTC_COMMAND_CLRAUX      0b11100010010 	// Clear Auxiliary Register Group
#define LTC_COMMAND_CLRSTAT     0b11100010011	// Clear Status Register Group

#define LTC_COMMAND_PLADC       0b11100010100	// Poll ADC Conversion Status
#define LTC_COMMAND_DIAGN       0b11100010101	// Diagnose MUX and Poll Status

#define LTC_COMMAND_WRCOMM      0b11100100001	// Write COMM Register Group   ***** NOT IMPLEMENTED
#define LTC_COMMAND_RDCOMM      0b11100100010	// Read COMM Register Group    ***** NOT IMPLEMENTED
#define LTC_COMMAND_STCOMM      0b11100100011	// Start I2C/SPI Communication ***** NOT IMPLEMENTED

typedef enum {
	LTC_READ_CELL 	= 0b0001,
	LTC_READ_GPIO  	= 0b0010,
	LTC_READ_STATUS = 0b0100,
	LTC_READ_CONFIG = 0b1000,
}LTC_READ;

typedef enum {
	ALL_GPIOS_WRITE	= 0,
	ALL_GPIOS_READ 	= 0x1F,
}LTC_GPIO;

typedef enum {
	REFERENCE_REMAINS_UNTIL_WATCHDOG_TIMEOUT = 1,
	REFERENCE_SHUTS_DOWN_AFTER_CONVERSIONS 	 = 0,
}LTC_REFON;

typedef enum {
	SOFTWARE_TIMER_ENABLE_PIN_LOW  = 0,
	SOFTWARE_TIMER_ENABLE_PIN_HIGH = 1,
}LTC_SWTRD;

typedef enum {
	SELECT_ADC_MODES_FAST   = 0,
	SELECT_ADC_MODES_NORMAL = 1,
}LTC_ADCOPT;

typedef enum {
	DEFULT_VOLTAGE = 0x000,
}LTC_COMPARISON_VOLTAGE;


typedef enum {
	DISCHARGE_DISABLE = 0x0,
	DISCHARGE_30SEC   = 0x1,
	DISCHARGE_1MIN    = 0x2,
	DISCHARGE_2MIN    = 0x3,
	DISCHARGE_3MIN    = 0x4,
	DISCHARGE_4MIN    = 0x5,
	DISCHARGE_5MIN    = 0x6,
	DISCHARGE_10MIN   = 0x7,
	DISCHARGE_15MIN   = 0x8,
	DISCHARGE_20MIN   = 0x9,
	DISCHARGE_30MIN   = 0xA,
	DISCHARGE_40MIN   = 0xB,
	DISCHARGE_60MIN   = 0xC,
	DISCHARGE_75MIN   = 0xD,
	DISCHARGE_90MIN   = 0xE,
	DISCHARGE_120MIN  = 0xF,
}LTC_DCTO;

typedef enum {
	MD_FAST 	= 0b0010000000,	//	27kHz or 14kHz
	MD_NORMAL 	= 0b0100000000,	//	 7kHz or  3kHz
	MD_FILTRED  = 0b0110000000,	//	 26Hz or  2kHz
}LTC_MD;

typedef enum {
	DCP_PERMITED 	 = 0b00000010000,
	DCP_NOT_PERMITED = 0b00000000000,
}LTC_DCP;

typedef enum {
	PUP_PULL_UP 	= 0b00001000000,
	PUP_PULL_DOWN 	= 0b00000000000,
}LTC_PUP;

typedef enum {
	ST_01 	= 0b00000100000,
	ST_02 	= 0b00001000000,
}LTC_ST;

typedef enum {
	CH_ALL	= 0b00000000000,
	CH_1_7 	= 0b00000000001,
	CH_2_8  = 0b00000000010,
	CH_3_9  = 0b00000000011,
	CH_4_10 = 0b00000000100,
	CH_5_11 = 0b00000000101,
	CH_6_12	= 0b00000000110,
}LTC_CH;

typedef enum {
	CHG_ALL		= 0b00000000000,
	CHG_GPIO1	= 0b00000000001,
	CHG_GPIO2	= 0b00000000010,
	CHG_GPIO3 	= 0b00000000011,
	CHG_GPIO4  	= 0b00000000100,
	CHG_GPIO5 	= 0b00000000101,
	CHG_2REF	= 0b00000000110,
}LTC_CHG;

typedef enum {
	CHST_ALL 	= 0b00000000000,
	CHST_SOC 	= 0b00000000001,
	CHST_ITMP 	= 0b00000000010,
	CHST_VA 	= 0b00000000011,
	CHST_VD		= 0b00000000100,
}LTC_CHST;

typedef struct LTC_command{
	uint16_t NAME;
	uint8_t BROADCAST;

	//COMMAND SETTINGS
	uint16_t MD;		// set the ADC mode
	uint16_t DCP;		// set if discharge is permitted during discharge
	uint16_t CH;		// set CELL channels to convert
	uint16_t CHG;		// set GPIO channels to convert
	uint16_t CHST;		// set STAT channels to convert
	uint16_t PUP;		// set if pull up or pull down is enabled in ADOW
	uint16_t ST;		// set the self test mode
}LTC_command;

typedef struct LTC_config{
	LTC_command *command;
	uint8_t ORDER:1; 	// 1 bit - set printing order -> 0 = normal, 1 = lowest to highest

	//CONFIGURATION REGISTER
	uint8_t GPIO:5; 	// 5 bits - set individual GPIO modes
	uint8_t REFON:1; 	// 1 bit - set the reference configuration
	uint8_t SWTRD:1;	// 1 bit - set the under voltage limit
	uint8_t ADCOPT:1;	// 1 bit - set the ADC mode
	uint16_t VUV:12; 	// 12 bits - set the under voltage limit
	uint16_t VOV; 		// 12 bits - set the over  voltage limit
	uint8_t DCTO:4; 	// 4 bits - set the duration of discharge
	uint8_t ADC_READY;
} LTC_config;

typedef struct LTC_sensor{
    uint8_t ADDR;
    uint8_t V_ERROR[NUM_CELLS_PER_SLAVE];
    uint8_t T_ERROR[NUM_CELLS_PER_SLAVE];

    uint16_t CELL_VOLTAGES[NUM_CELLS_PER_SLAVE];
    uint16_t GPIO_VOLTAGES[NUM_TEMPS_PER_SLAVE];
    uint16_t REF_VOLTAGE;

    //STATUS REGISTER A & B
    uint16_t SOC; 		// 16 bits - get the sum of voltages
    uint16_t INTERNAL_TEMP; 		// 16 bits - get the internal temperature
    uint16_t ANALOG_VOLTAGE; 		// 16 bits - get the analog voltage
    uint16_t DIGITAL_VOLTAGE; 		// 16 bits - get the digital voltage
    uint16_t CELL_TO_DISCHARGE;  		// 12 bits - set which cell to discharge

    uint16_t V_MAX;
    uint16_t V_MIN;
    uint16_t V_DELTA;
} LTC_sensor;

typedef struct Slave {
	LTC_sensor sensor;
    LTC_config *config;
} Slave;

void LTC_Init(LTC_config *config);
void LTC_Wait(Slave *slave);
void LTC_Read(uint8_t LTC_READ, Slave *slave);

void LTC_PEC_InitTable();

void LTC_SendBroadcastCommand(LTC_config *config, uint16_t command_name);
void LTC_SendAddressedCommand(Slave *slave, uint16_t command_name);
void LTC_ReceiveMessage(Slave* slave, uint16_t rx_data[4]);
void LTC_Communication(LTC_config *config, uint16_t* tx_data, uint16_t* rx_data);
void LTC_ConfigCommandName(Slave* slave);
void LTC_WriteConfigRegister(Slave* slave, uint16_t *tx_data);

uint16_t LTC_SPI(uint16_t Tx_data);
void LTC_ChipSelect(uint8_t level);
void LTC_StartTrasmission();
void LTC_EndTrasmission();
void LTC_WakeUp();

uint16_t LTC_MakeCommand(LTC_command *command);
void LTC_TransmitCommand(uint16_t command);
void LTC_TransmitReceive(uint16_t command, uint16_t* tx_data, uint16_t* rx_data);

#endif
