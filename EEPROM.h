/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/*Includes*/
#include "stm32f0xx_hal.h"

typedef struct eeprom_handle_s
{
	uint32_t start_address;
	uint32_t page_size;
	uint32_t active_page_address;
	uint32_t active_page_free_space;	//in records (32 bit)
	uint8_t lock;
} eeprom_handle_t;


uint32_t FEE_Get_Version(void);
uint8_t FEE_Init(eeprom_handle_t *heeprom);
HAL_StatusTypeDef FEE_Read_Data(eeprom_handle_t *heeprom, uint8_t identifier, uint16_t *data);
uint8_t FEE_Write_Data(eeprom_handle_t *heeprom, uint8_t identifier, uint16_t data);

//Record format: [8 bit|8 bit|16 bit] ~ [CRC/Hamming|address|value]

#endif /* __EEPROM_H */

