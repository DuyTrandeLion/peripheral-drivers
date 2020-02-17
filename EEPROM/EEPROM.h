/*
 * MIT License
 *
 * Copyright (c) 2020 <Duy Lion Tran>. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the
 * Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @file EEPROM.h
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#ifndef __EEPROM_H__
#define __EEPROM_H__


#include <stdint.h>
#include <stdbool.h>
#include "Miscellaneous.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Types off supported communications. */
#define I2C_COMM		0x00
#define SPI_COMM		0x01
#define MICROWIRE_COMM	0x03

/** Instruction sets for SPI EEPROM. */
#define READ_INST		0x03		/**< Read data from memory array beginning at selected address 	*/
#define WRITE_INST		0x02		/**< Write data to memory array beginning at selected address  	*/
#define WRDI_INST		0x04		/**< Write data to memory array beginning at selected address  	*/
#define WREN_INST		0x06		/**< Set the write enable latch (enable write operations)      	*/
#define RDSR_INST		0x05		/**< Read STATUS register										*/
#define WRSN_INST		0x01		/**< Write STATUS register										*/

/** EEPROM return codes. */
typedef enum
{
	EEPROM_OK 				= 0x00,
	EEPROM_DEVICE_BUSY 		= 0x01,
	EEPROM_NULL_PARAM 		= 0x02,
	EEPROM_COMM_ERROR		= 0x03,
	EEPROM_COMM_TIMEOUT		= 0x04,
	EEPROM_WRITE_FAIL		= 0x05,
	EEPROM_INVLD_PAGE_ADDR	= 0x06,
	EEPROM_UNKNOWN_ERROR	= 0x07
} EEPROM_State_t;


/** EEPROM event types of communication. */
typedef enum
{
	SPI_EEP_EVENT_TRANSMIT 					= 0x00,
	SPI_EEP_EVENT_RECEIVE  					= 0x01,
	SPI_EEP_EVENT_TRANSMIT_RECEIVE 			= 0x02,
	SPI_EEP_EVENT_ABORT_TRANSMIT			= 0x03,
	SPI_EEP_EVENT_ABORT_RECEIVE				= 0x04,
	SPI_EEP_EVENT_ABORT_TRANSMIT_RECEIVE 	= 0x05,
	SPI_EVENT_CS_LOW						= 0x06,
	SPI_EVENT_CS_HIGH						= 0x07,
	I2C_EEP_EVENT_TRANSMIT					= 0x08,
	I2C_EEP_EVENT_RECEIVE					= 0x09,
	I2C_EEP_EVENT_ABORT_TRANSMIT			= 0x0A,
	I2C_EEP_EVENT_ABORT_RECEIVE				= 0x0B
} EEPROM_Event_t;


/**
 * SPI event callback type.
 *
 * This function is called when there is SPI communication.
 *
 * @param[in] EEPROM_Event_t 	SPI Event type.
 * @param[in] uint16_t			Device address.
 * @param[in] uint16_t			Data address.
 * @param[in] uint8_t *			Pointer to data buffer.
 * @param[in] uint16_t			Size of data buffer.
 * @param[in] void *			Pointer to parameter of event handler
 *
 * @retval EEPROM_OK If the notification was sent successfully. Otherwise, an error code is returned.
 */
typedef EEPROM_State_t (*EEPROM_Comm_Handle_t)(EEPROM_Event_t, uint16_t, uint16_t, uint8_t *, uint16_t, void *);


/**
 * Busy time counting handle callback.
 *
 * @retval true 	If the device is busy
 */
typedef bool (*EEPROM_BusyWait_Handle_t)(void);


/** Memory map structure.  */
typedef struct
{
	uint16_t	indexAddress;
	uint16_t  	startAddress;
	uint16_t	currentAddress;
	uint16_t	dataSize;
} Memory_Map_t;

/** Structure to store any types of data.  */
typedef struct
{
	void *storeData;
} StoreData_t;

/** EEPROM configuration structure.  */
typedef struct
{
	uint8_t		deviceAddress;
	uint8_t     communication;
	uint16_t    blockSize;
	uint16_t    blocks;
	uint32_t	timeout;
} EEPROM_Config_t;

/** EEPROM definition structure.  */
typedef struct
{
	Memory_Map_t				memory;
	EEPROM_Config_t				config;
	EEPROM_Comm_Handle_t 		const commHandle;
	EEPROM_BusyWait_Handle_t	const busyHandle;
} EEPROM_Def_t;



/**@brief Function for initializing the EEPROM driver.
 *
 * @param[in] 	locEEPROM_p		Pointer to driver definition.
 *
 * @retval 		EEPROM_OK 		If the driver was initialized successfully.
 *
 * @note I2C maximum clock speed is 1MHz
 *
 * @note SPI full-duplex master with 8-bit word, MSB first
 * 		 SPI clock below 10MHz
 *       SPI mode 0 (CPOL = 0, CPHA = 0)
 *
 * @note Visit https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html
 * 		 and https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
 *       for more knowledge about SPI
 */
EEPROM_State_t EEPROM_Init(EEPROM_Def_t *locEEPROM_p);


/**@brief Function for de-initializing the EEPROM driver.
 *
 * @param[in] 	locEEPROM_p		Pointer to driver definition.
 *
 */
void EEPROM_DeInit(EEPROM_Def_t *locEEPROM_p);


/**@brief Function for reading random memory byte.
 *
 * @param[in] 	locEEPROM_p				Pointer to driver definition.
 * @param[in] 	locMemoryAddress_u16	Memory cell address to start reading.
 * @param[out] 	locReadData_p8			Buffer to store read data.
 * @param[in] 	locReadSize_u16			Data size to read.
 *
 * @retval 		EEPROM_OK 		If the driver was initialized successfully.
 *
 */
EEPROM_State_t EEPROM_ReadMemory(EEPROM_Def_t *locEEPROM_p,
		                         uint16_t locMemoryAddress_u16,
								 uint8_t *locReadData_p8,
								 uint16_t locReadSize_u16);


/**@brief Function for writing memory bytes.
 *
 * @param[in] 	locEEPROM_p				Pointer to driver definition.
 * @param[in] 	locMemoryAddress_u16	Memory cell address to start writing.
 * @param[in] 	locWriteData_p8			Data buffer to write.
 * @param[in] 	locWriteSize_u16		Data size to write.
 *
 * @retval 		EEPROM_OK 		If the driver was initialized successfully.
 *
 */
EEPROM_State_t EEPROM_WriteMemory(EEPROM_Def_t *locEEPROM_p,
		                          uint16_t locMemoryAddress_u16,
								  uint8_t *locWriteData_p8,
								  uint16_t locWriteSize_u16);


/**@brief Function for writing random memory page.
 *
 * @param[in] 	locEEPROM_p				Pointer to driver definition.
 * @param[in] 	locPageAddress_u16		Memory page address to write.
 * @param[in] 	locWriteData_p8			Data buffer to write.
 *
 * @retval 		EEPROM_OK 		If the driver was initialized successfully.
 *
 */
EEPROM_State_t EEPROM_WriteMemoryPage(EEPROM_Def_t *locEEPROM_p,
		                              uint16_t 	  locPageAddress_u16,
								      uint8_t     *locWriteData_p8);


/**@brief Function for writing/storing any types of data.
 *
 * @param[in] 	locEEPROM_p				Pointer to driver definition.
 * @param[in] 	locMemoryAddress_u16	Memory cell address to start writing.
 * @param[in] 	locStoreData_sp			Pointer to the data to tore.
 *
 * @retval 		EEPROM_OK 		If the driver was initialized successfully.
 *
 */
EEPROM_State_t EEPROM_MemoryPut(EEPROM_Def_t *locEEPROM_p,
		                        uint16_t 	  locMemoryAddress_u16,
								StoreData_t  *locStoreData_sp);


#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H__ */
