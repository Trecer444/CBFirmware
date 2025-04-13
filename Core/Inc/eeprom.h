/*
 * eeprom.h
 *
 *  Created on: Dec 1, 2021
 *      Author: Admin
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"



/* Declarations and definitions ----------------------------------------------*/

#define PAGE_DATA_OFFSET                                                8
#define PAGE_DATA_SIZE                                                  8

#define PARAM_1                                                         0x12121212
#define PARAM_2                                                         0x34343434
#define PARAM_3															0x56565656
#define PARAM_4															0x78787878
#define VAR_NUM                                                         4

#define PAGE_0_ADDRESS                                                  21U
#define PAGE_1_ADDRESS                                                  22U
#define PAGE_SIZE                                                       1024



typedef enum {
  PAGE_CLEARED = 0xFFFFFFFF,
  PAGE_ACTIVE = 0x00000000,
  PAGE_RECEIVING_DATA = 0x55555555,
} PageState;

typedef enum {
  PAGE_0 = 0,
  PAGE_1 = 1,
  PAGES_NUM = 2,
} PageIdx;

typedef enum {
  EEPROM_OK = 0,
  EEPROM_ERROR = 1,
} EepromResult;



/* Functions -----------------------------------------------------------------*/


extern EepromResult EEPROM_Init();
extern EepromResult EEPROM_Read(uint32_t varId, uint32_t *varValue);
extern EepromResult EEPROM_Write(uint32_t varId, uint32_t varValue);

#ifdef __cplusplus
}
#endif

#endif /* INC_EEPROM_H_ */
