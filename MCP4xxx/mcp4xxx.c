#include "mcp4xxx.h"

#define NULL_CHECK_PARAM(MCP4xxx_Def)				if (NULL == MCP4xxx_Def->spiHandle) { return MCP4xxx_NULL_PARAM; }
#define VALIDATION_CHECK_PARAM(MCP4xxx_Def, PTM)	if ((MCP4xxx_SELECT_PTM_2 == PTM) && (MCP41xx == locMCP4xxx_p->device)) { return MCP4xxx_INVALID_PARAM; }


MCP4xxx_State_t MCP4xxx_Init(PTM_Def_t *locMCP4xxx_p)
{
	NULL_CHECK_PARAM(locMCP4xxx_p);

	if ((MCP42xx == locMCP4xxx_p->device) && (NULL == locMCP4xxx_p->shutdownHandle))
	{
		return MCP4xxx_NULL_PARAM;
	}

	if (((MCP41xx != locMCP4xxx_p->device) && (MCP42xx != locMCP4xxx_p->device)) || (0 == locMCP4xxx_p->maxResistance))
	{
		return MCP4xxx_INVALID_PARAM;
	}

	return MCP4xxx_OK;
}


MCP4xxx_State_t MCP4xxx_Shutdown(PTM_Def_t *locMCP4xxx_p, uint8_t locPotentiometer_u8)
{
	uint8_t locWriteData[2] = {0x00};
	MCP4xxx_State_t locRet;

	NULL_CHECK_PARAM(locMCP4xxx_p);

	if ((MCP42xx == locMCP4xxx_p->device) && (NULL == locMCP4xxx_p->shutdownHandle))
	{
		return MCP4xxx_NULL_PARAM;
	}

	VALIDATION_CHECK_PARAM(locMCP4xxx_p, locPotentiometer_u8);

	locWriteData[0] = MCP4xxx_COMMAND_SHUT_DOWN | locPotentiometer_u8;
	locWriteData[1] = 0xFF;

	locRet = locMCP4xxx_p->spiHandle(SPI_PTM_EVENT_TRANSMIT, locWriteData, 2, NULL);

	if (MCP42xx == locMCP4xxx_p->device)
	{
		locMCP4xxx_p->shutdownHandle();
	}

	return locRet;
}


MCP4xxx_State_t MCP4xxx_WritePotentiometerValue(PTM_Def_t *locMCP4xxx_p, uint8_t locPotentiometer_u8, uint32_t locPotentiometerValue_u32)
{
	uint8_t locWriteData[2] = {0x00};

	NULL_CHECK_PARAM(locMCP4xxx_p);
	VALIDATION_CHECK_PARAM(locMCP4xxx_p, locPotentiometer_u8);

	if (0 == locMCP4xxx_p->maxResistance)
	{
		return MCP4xxx_INVALID_PARAM;
	}

	locWriteData[0] = MCP4xxx_COMMAND_WRITE_DATA | locPotentiometer_u8;
	locWriteData[1] = ((float)locPotentiometerValue_u32 / (float)locMCP4xxx_p->maxResistance) * (float)255.0;

	locMCP4xxx_p->potentiometer[locPotentiometer_u8 - 1] = locWriteData[1];

	return locMCP4xxx_p->spiHandle(SPI_PTM_EVENT_TRANSMIT, locWriteData, 2, NULL);
}


MCP4xxx_State_t MCP4xxx_GetPotentiometerValue(PTM_Def_t *locMCP4xxx_p, uint8_t locPotentiometer_u8, uint32_t *locPotentiometerValue_u32)
{
	NULL_CHECK_PARAM(locMCP4xxx_p);
	VALIDATION_CHECK_PARAM(locMCP4xxx_p, locPotentiometer_u8);

	*locPotentiometerValue_u32 = ((float)locMCP4xxx_p->potentiometer[locPotentiometer_u8 - 1] / (float)255.0) * (float)locMCP4xxx_p->maxResistance;

	return MCP4xxx_OK;
}

