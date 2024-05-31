/*
 * defines.h
 *
 *  Created on: Jan 20, 2024
 *      Author: Nicolas
 */

#ifndef DEFINES_H
#define DEFINES_H

#define 	NUM_CELLS_PER_SLAVE 	12 // MAX 18 CELL INPUTS - LTC6813
#define 	NUM_TEMPS_PER_SLAVE 	9 // MAX 9 GPIO PINS - LTC6813
#define 	NUM_SLAVES 				4

#define 	LTC_PEC_SEED 			16
#define 	LTC_PEC_TABLE_LENGTH	256
#define	 	SPI_BUFFER_LENGTH		4

#endif
