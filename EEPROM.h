/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/*Includes*/
//#include "main.h"
#include "stm32f0xx_hal.h"

uint32_t FEE_Get_Version(void);
uint8_t FEE_Init(uint32_t start_address, uint32_t page_size);
HAL_StatusTypeDef FEE_Read_Data(uint8_t identifier, uint16_t *data);
uint8_t FEE_Write_Data(uint8_t identifier, uint16_t data);

//Record format: [8 bit|8 bit|16 bit] ~ [CRC/Hamming|address|value]

#endif /* __EEPROM_H */

