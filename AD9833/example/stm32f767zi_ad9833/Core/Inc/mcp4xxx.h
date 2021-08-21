/*
 * mcp4xxx.h
 *
 *  Created on: Aug 20, 2021
 *      Author: Admin
 */

#ifndef _MCP4XXX_H_
#define _MCP4XXX_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP4xxx_COMMAND_WRITE_DATA		(0x10)
#define MCP4xxx_COMMAND_SHUT_DOWN		(0x20)

#define MCP4xxx_SELECT_PTM_1			(0x01)
#define MCP4xxx_SELECT_PTM_2			(0x02)


/** MCP4xxx potentiometer return codes. */
typedef enum
{
    MCP4xxx_OK					= 0x00,
    MCP4xxx_DEVICE_BUSY			= 0x01,
    MCP4xxx_NULL_PARAM			= 0x02,
	MCP4xxx_INVALID_PARAM		= 0x03,
    MCP4xxx_COMM_ERROR			= 0x04,
    MCP4xxx_COMM_TIMEOUT		= 0x05,
    MCP4xxx_UNKNOWN_ERROR		= 0x06,
} MCP4xxx_State_t;

/** SPI event types of communication. */
typedef enum
{
    SPI_PTM_EVENT_TRANSMIT					= 0x00,
    SPI_PTM_EVENT_RECEIVE					= 0x01,
    SPI_PTM_EVENT_TRANSMIT_RECEIVE			= 0x02,
    SPI_PTM_EVENT_ABORT_TRANSMIT			= 0x03,
    SPI_PTM_EVENT_ABORT_RECEIVE				= 0x04,
    SPI_PTM_EVENT_ABORT_TRANSMIT_RECEIVE	= 0x05,
    SPI_PTM_EVENT_CLEAR_INPUT				= 0x06,
    SPI_PTM_EVENT_BUSY_WAIT					= 0x07,
} PTM_SPI_Event_t;

/** Supported devices. **/
typedef enum
{
	MCP41xx = 0,
	MCP42xx = 1,
} MCP4xxx_Device_t;

/** Device operation modes. **/
typedef enum
{
	RHEOSTAT_MODE		= 0,
	POTENTIOMETER_MODE	= 1,
} MCP4xxx_Mode_t;


/**
 * SPI event callback type.
 *
 * This function is called when there is SPI communication.
 *
 * @param[in] PTM_SPI_Event_t 	SPI Event type.
 * @param[in] uint8_t *			Pointer to data buffer.
 * @param[in] uint16_t			Size of data buffer.
 * @param[in] void *			Pointer to parameter of event handler
 *
 * @retval MCP4xxx_OK If the notification was sent successfully. Otherwise, an error code is returned.
 */
typedef MCP4xxx_State_t (*PTM_SPI_Handle_t)(PTM_SPI_Event_t locCommEvent_en, uint8_t *locData_p8, uint16_t locDataSize_u16, void *locContext_p);


/**
 * This function is called to shutdown the device
 *
 */
typedef void (*PTM_Shutdown_Handle_t)(void);


/** DAC definition structure.  */
typedef struct
{
	uint8_t					potentiometer[2];
	uint32_t				maxResistance;
	MCP4xxx_Device_t		device;
	MCP4xxx_Mode_t			mode;
	PTM_SPI_Handle_t 		spiHandle;
	PTM_Shutdown_Handle_t	shutdownHandle;
} PTM_Def_t;


/**
 * @brief Function to initialize and setup the digital potentiometer.
 *
 * @param[in] locMCP4xxx_p		Pointer to driver definition.
 *
 * @retval MCP4xxx_OK			If the driver was initialized successfully.
 *
 */
MCP4xxx_State_t MCP4xxx_Init(PTM_Def_t *locMCP4xxx_p);


/**
 * @brief Function to put the digital potentiometer to sleep mode.
 *
 * @param[in] locMCP4xxx_p				Pointer to driver definition.
 *
 * @retval MCP4xxx_OK					If the device is slept.
 *
 */
MCP4xxx_State_t MCP4xxx_Shutdown(PTM_Def_t *locMCP4xxx_p, uint8_t locPotentiometer_u8);


/**
 * @brief Function to set the resistance for the digital potentiometer.
 *
 * @param[in] locMCP4xxx_p					Pointer to driver definition.
 * @param[in] locPotentiometerValue_u32		Desired resistance.
 *
 * @retval MCP4xxx_OK						If the resistance was set successfully.
 *
 */
MCP4xxx_State_t MCP4xxx_WritePotentiometerValue(PTM_Def_t *locMCP4xxx_p, uint8_t locPotentiometer_u8, uint32_t locPotentiometerValue_u32);


/**
 * @brief Function to read the resistance value the digital potentiometer.
 *
 * @param[in] locMCP4xxx_p					Pointer to driver definition.
 * @param[out] locPotentiometerValue_u32	Resistance to be read.
 *
 * @retval MCP4xxx_OK						If the resistance was read successfully.
 *
 */
MCP4xxx_State_t MCP4xxx_GetPotentiometerValue(PTM_Def_t *locMCP4xxx_p, uint8_t locPotentiometer_u8, uint32_t *locPotentiometerValue_u32);


#ifdef __cplusplus
}
#endif

#endif /* _MCP4XXX_H_ */
