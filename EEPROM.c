/*EEPROM.c*/

/*Includes*/
#include "EEPROM.h"
#include <stdlib.h>
#include <string.h>


enum
{
	FEE_VERSION_MAJOR = 0x01,
	FEE_VERSION_MINOR = 0x03,
	FEE_VERSION_PATCH = 0x01
};

/* Page status definitions */
enum page_status_t
{
	page_status_erased		= 1<<0,
	page_status_receive		= 1<<1,
	page_status_active		= 1<<2,
	page_status_invalid		= 1<<3
};

enum {crc_poly = 0xE0};	//reversed CRC-8-CCITT, Hamming distance = 4 (up to 119 bits)
enum {payload_size = 3};	//3 bytes

typedef struct eeprom_handle_s
{
	uint32_t start_address;
	uint32_t page_size;
	uint32_t active_page_address;
	uint32_t active_page_free_space;	//in records (32 bit)
	uint8_t lock;
} eeprom_handle_t;


union record_t
{
	uint32_t word;

	struct
	{
		uint8_t crc;
		uint8_t id;
		uint16_t data;
	} field;
};

uint8_t crc_table[256] = {0};

static HAL_StatusTypeDef Format(eeprom_handle_t *heeprom);
static HAL_StatusTypeDef Add_Record(eeprom_handle_t *heeprom, uint8_t identifier, uint16_t data);
static HAL_StatusTypeDef Get_Record(eeprom_handle_t *heeprom, uint8_t identifier, uint16_t *data);
static HAL_StatusTypeDef Page_Transfer(eeprom_handle_t *heeprom);
static HAL_StatusTypeDef Calculate_Free_Space(eeprom_handle_t *heeprom, uint32_t page_address, uint32_t *free_space);
static HAL_StatusTypeDef Set_Page_Status(uint32_t page_address, uint8_t status);
static HAL_StatusTypeDef Get_Page_Status(uint32_t page_address, uint8_t *status);
static HAL_StatusTypeDef Check_Record_Integrity(union record_t record);
static void Compute_CRC_Table(uint8_t table[256], uint8_t polynomial);
static uint8_t Calculate_CRC(uint8_t *data, uint8_t length);


uint32_t FEE_Get_Version(void)
{
	return (FEE_VERSION_MAJOR<<16) | (FEE_VERSION_MINOR<<8) | (FEE_VERSION_PATCH);
}

/**
 * @brief  Restore the pages to a known good state in case of page's status corruption after a power loss.
 * @param  None.
 * @retval - Pointer to EEPROM handle
 */
eeprom_handle_t *FEE_Init(uint32_t start_address, uint32_t page_size)
{
	eeprom_handle_t *heeprom;
	uint32_t page_0_start_address, page_1_start_address;
	uint8_t page_status_0, page_status_1;
	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef erase_init;
	uint32_t free_space;
	uint32_t valid_page_address, receive_page_address, erased_page_address;
	HAL_StatusTypeDef status = HAL_ERROR;

	heeprom = (eeprom_handle_t*) malloc(sizeof(eeprom_handle_t));
	memset(heeprom, 0, sizeof(eeprom_handle_t));

	Compute_CRC_Table(crc_table, crc_poly);

	heeprom->start_address = start_address;
	heeprom->page_size = page_size;

	page_0_start_address = heeprom->start_address;
	page_1_start_address = heeprom->start_address + heeprom->page_size;

	Get_Page_Status(page_0_start_address, &page_status_0);
	Get_Page_Status(page_1_start_address, &page_status_1);


	erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init.NbPages     = 1;

	switch(page_status_0 | page_status_1)
	{
	case page_status_active | page_status_erased:
	if(page_status_0 == page_status_active)
	{
		valid_page_address = page_0_start_address;
		erased_page_address = page_1_start_address;
	}
	else
	{
		valid_page_address = page_1_start_address;
		erased_page_address = page_0_start_address;
	}

	Calculate_Free_Space(heeprom, erased_page_address, &free_space);	//stop here 07.08.2023 check calculation of free_space

	if(free_space != heeprom->page_size / 4)
	{
		erase_init.PageAddress = erased_page_address;
		HAL_FLASH_Unlock();
		status = HAL_FLASHEx_Erase(&erase_init, &page_error);
		HAL_FLASH_Lock();
		if (status != HAL_OK)
		{
			return NULL;
		}
	}

	heeprom->active_page_address = valid_page_address;
	break;

	case page_status_active | page_status_receive:
	if(page_status_0 == page_status_active)
	{
		valid_page_address = page_0_start_address;
		receive_page_address = page_1_start_address;
	}
	else
	{
		valid_page_address = page_1_start_address;
		receive_page_address = page_0_start_address;
	}

	heeprom->active_page_address = valid_page_address;
	Page_Transfer(heeprom);

	heeprom->active_page_address = receive_page_address;
	break;

	case page_status_receive | page_status_erased:
	if(page_status_0 == page_status_receive)
	{
		receive_page_address = page_0_start_address;
		erased_page_address = page_1_start_address;
	}
	else
	{
		receive_page_address = page_1_start_address;
		erased_page_address = page_0_start_address;
	}

	Calculate_Free_Space(heeprom, erased_page_address, &free_space);

	if(free_space != heeprom->page_size/4)
	{
		erase_init.PageAddress = erased_page_address;
		HAL_FLASH_Unlock();
		status = HAL_FLASHEx_Erase(&erase_init, &page_error);
		HAL_FLASH_Lock();
		if (status != HAL_OK)
		{
			return NULL;
		}
	}

	status = Set_Page_Status(receive_page_address, page_status_active);
	if (status != HAL_OK)
	{
		return NULL;
	}

	heeprom->active_page_address = receive_page_address;
	break;

	default:
		status = Format(heeprom);
		if (status != HAL_OK)
		{
			return NULL;
		}

		heeprom->active_page_address = heeprom->start_address;
		break;
	}

	Calculate_Free_Space(heeprom, heeprom->active_page_address, &heeprom->active_page_free_space);

	if (heeprom->active_page_free_space == 0)
	{
		Page_Transfer(heeprom);
	}

	return heeprom;
}

HAL_StatusTypeDef FEE_Terminate(eeprom_handle_t *heeprom)
{
	HAL_StatusTypeDef status;
	FLASH_EraseInitTypeDef erase_init;
	uint32_t page_error;

	erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init.PageAddress = heeprom->start_address;
	erase_init.NbPages     = 1;

	HAL_FLASH_Unlock();
	status = HAL_FLASHEx_Erase(&erase_init, &page_error);
	HAL_FLASH_Lock();

	free(heeprom);

	return status;
}

/**
 * @brief  Returns the last stored variable data, if found, which correspond to
 *   the passed virtual address
 * @param  VirtAddress: Variable virtual address
 * @param  Data: Global variable contains the read variable value
 * @retval Success or error status:
 *           - 0: if variable was found
 *           - 1: if the variable was not found
 *           - NO_VALID_PAGE: if no valid page was found.
 */
HAL_StatusTypeDef FEE_Read_Data(eeprom_handle_t *heeprom, uint8_t identifier, uint16_t *ptr_data)
{
	HAL_StatusTypeDef status;

	status = Get_Record(heeprom, identifier, ptr_data);

	return status;
}

/**
 * @brief  Writes/upadtes variable data in EEPROM.
 * @param  VirtAddress: Variable virtual address
 * @param  Data: 16 bit data to be written
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
uint8_t FEE_Write_Data(eeprom_handle_t *heeprom, uint8_t identifier, uint16_t data)
{
	enum {empty_id = 0xFF};

	uint8_t status;
	uint16_t old_data, stored_data;

	if(identifier == empty_id)
	{
		return HAL_ERROR;
	}

	status = Get_Record(heeprom, identifier, &old_data);

	if(data == old_data && status == HAL_OK)		//TODO optimize this workaround (buffer?)
	{
		return HAL_OK;
	}

	while(1)
	{
		if (heeprom->active_page_free_space == 0)
		{
			status = Page_Transfer(heeprom);
		}

		status = Add_Record(heeprom, identifier, data);

		status = Get_Record(heeprom, identifier, &stored_data);

		if(stored_data == data && status == HAL_OK)
		{
			return HAL_OK;
		}
	}

	return HAL_ERROR;
}

/**
 * @brief  Erases PAGE and PAGE1 and writes VALID_PAGE header to PAGE
 * @param  None
 * @retval Status of the last operation (Flash write or erase) done during
 *         EEPROM formating
 */
static HAL_StatusTypeDef Format(eeprom_handle_t *heeprom)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t page_error;
	FLASH_EraseInitTypeDef erase_init;
	uint32_t page_address;
	uint32_t free_space;

	for(uint8_t i=0; i<2; i++)
	{
		page_address = heeprom->start_address + heeprom->page_size * i;

		Calculate_Free_Space(heeprom, page_address, &free_space);

		if(free_space != heeprom->page_size/4)
		{
			erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
			erase_init.PageAddress = page_address;
			erase_init.NbPages     = 1;

			HAL_FLASH_Unlock();
			status = HAL_FLASHEx_Erase(&erase_init, &page_error);
			HAL_FLASH_Lock();
			if (status != HAL_OK)
			{
				return status;
			}
		}
	}

	status = Set_Page_Status(heeprom->start_address, page_status_active);

	heeprom->active_page_free_space = heeprom->page_size / 4 - 1;

	return status;
}

/**
 * @brief  Verify if active page is full and Writes variable in EEPROM.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static HAL_StatusTypeDef Add_Record(eeprom_handle_t *heeprom, uint8_t identifier, uint16_t data)	//TODO stopped here, add union record_t
{
	HAL_StatusTypeDef status;
	uint32_t address;
	uint8_t *payload;
	union record_t record;

	record.field.id = identifier;
	record.field.data = data;

	payload = (uint8_t*)&record.field.id;

	record.field.crc = Calculate_CRC(payload, payload_size);

	address = heeprom->active_page_address + heeprom->page_size - 4*heeprom->active_page_free_space;

	HAL_FLASH_Unlock();
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, record.word);
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 2, record.word>>16);
	HAL_FLASH_Lock();

	heeprom->active_page_free_space--;

	return status;
}

static HAL_StatusTypeDef Get_Record(eeprom_handle_t *heeprom, uint8_t identifier, /*restrict*/ uint16_t *data)	//TODO test restric with FLTO
{
	uint32_t address, page_start_address;
	uint16_t status = HAL_ERROR;
	union record_t record;

	page_start_address = heeprom->active_page_address;
	address = heeprom->active_page_address + heeprom->page_size - 4;

	while(address > page_start_address)
	{
		record.word = *(volatile uint32_t*)address;

		if(record.field.id == identifier)
		{
			status = Check_Record_Integrity(record);
			*data = record.field.data;	//data will be returned even it is corrupted
			break;
		}

		address -= 4;
	}

	return status;
}

/**
 * @brief  Transfers last updated variables data from the full Page to
 *   an empty one.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static HAL_StatusTypeDef Page_Transfer(eeprom_handle_t *heeprom)	//TODO add checking for already transfered variables
{
	uint32_t new_page_address, old_page_address, from_address, to_address;
	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef erase_init;
	HAL_StatusTypeDef status;
	uint8_t buf_data[256] = {0};	//'hash' table
	uint8_t identifier;
	uint32_t word, stored_data;
	uint8_t page_status;

	if (heeprom->active_page_address == heeprom->start_address)
	{
		new_page_address = heeprom->start_address + heeprom->page_size;
		old_page_address = heeprom->start_address;
	}
	else
	{
		new_page_address = heeprom->start_address;
		old_page_address = heeprom->start_address + heeprom->page_size;
	}

	Get_Page_Status(new_page_address, &page_status);
	if(page_status != page_status_receive)
	{
		status = Set_Page_Status(new_page_address, page_status_receive);
	}

	to_address = new_page_address + 4;
	from_address = old_page_address + heeprom->page_size - 4;

	HAL_FLASH_Unlock();

	while(from_address > old_page_address)
	{
		identifier = *((volatile uint8_t*)from_address+1);

		if(buf_data[identifier] == 0)
		{
			buf_data[identifier] = 1;

			word = *(volatile uint32_t*)from_address;
			stored_data = !word;

			while(stored_data != word)	//CRC will not be checked because the data is actually just moved and not added
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, to_address, word&0xFFFF);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, to_address+2, word>>16);

				stored_data = *(volatile uint32_t*)to_address;
				to_address += 4;
			}
		}

		from_address -= 4;
	}

	erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init.PageAddress = old_page_address;
	erase_init.NbPages     = 1;

	status = HAL_FLASHEx_Erase(&erase_init, &page_error);
	HAL_FLASH_Lock();
	if (status != HAL_OK)
	{
		return status;
	}

	status = Set_Page_Status(new_page_address, page_status_active);
	if (status != HAL_OK)
	{
		return status;
	}

	heeprom->active_page_address = new_page_address;
	Calculate_Free_Space(heeprom, new_page_address, &heeprom->active_page_free_space);

	return HAL_OK;
}

static HAL_StatusTypeDef Calculate_Free_Space(eeprom_handle_t *heeprom, uint32_t page_address, uint32_t *free_space)
{
	enum {empty_record = 0xFFFFFFFF};

	uint32_t address;

	address = page_address;

	while (address < page_address+heeprom->page_size)	//TODO parse whole page, empty_record can be before non-empty because of FLASH corruption
	{
		if ((*(volatile uint32_t*)address) == empty_record)
		{
			break;
		}

		address += 4;
	}

	*free_space = (heeprom->page_size - (address - page_address)) / 4;

	return HAL_OK;
}

static HAL_StatusTypeDef Set_Page_Status(uint32_t page_address, uint8_t status)
{
	HAL_FLASH_Unlock();

	switch(status)
	{
	case page_status_receive:
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page_address, 0x0000);
		break;
	case page_status_active:
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page_address+2, 0x0000);
		break;
	}

	HAL_FLASH_Lock();

	return HAL_OK;
}

static HAL_StatusTypeDef Get_Page_Status(uint32_t page_address, uint8_t *status)
{
	uint32_t word;
	uint16_t active_status, receive_status;

	word = *(volatile uint32_t*)page_address;
	receive_status = word & 0xFF;
	active_status = word>>16;

	if(active_status == 0x0000)	//TODO replace it with Count_Once()
	{
		*status = page_status_active;
	}
	else if(receive_status == 0x0000)
	{
		*status = page_status_receive;
	}
	else
	{
		*status = page_status_erased;
	}

	return HAL_OK;
}

static HAL_StatusTypeDef Check_Record_Integrity(union record_t record)
{
	uint8_t *payload;
	uint8_t crc;

	payload = &record.field.id;

	crc = Calculate_CRC(payload, payload_size);

	if(record.field.crc == crc)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
}

static uint8_t Calculate_CRC(uint8_t *data, uint8_t length)
{
	uint8_t index;
	uint16_t remainder = 0xFFFF;

	while(length--)
	{
		index = *data++ ^ remainder;
		remainder >>= 8;
		remainder ^= crc_table[index];
	}

	return remainder;
}

static void Compute_CRC_Table(uint8_t table[256], uint8_t polynomial)
{
	uint8_t remainder = 0x01;
	size_t i, j;

	for (i = 128; i > 0; i >>= 1)
	{
		if(remainder & 0x01)
		{
			remainder = (remainder >> 1) ^ polynomial;
		}
		else
		{
			remainder >>= 1;
		}

		for (j = 0; j < 256; j += 2*i)
		{
			table[i + j] = remainder ^ table[j];
		}
	}
}


