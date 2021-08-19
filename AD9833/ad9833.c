#include <math.h>
#include "ad9833.h"

#define NULL_CHECK_PARAM(AD983x_Def)	if ((NULL == AD983x_Def->spiHandle) || (NULL == AD983x_Def->delayHandle)) { return AD983x_NULL_PARAM; }


static AD983x_State_t AD983x_WriteRegister(DDS_Def_t *locAD983x_p, int16_t locValue_i16);
static AD983x_State_t AD983x_WriteControlRegister(DDS_Def_t *locAD983x_p);

static AD983x_State_t AD983x_WriteRegister(DDS_Def_t *locAD983x_p, int16_t locValue_i16)
{
	uint8_t locWriteData_au8[2];

	NULL_CHECK_PARAM(locAD983x_p);

	locWriteData_au8[0] = locValue_i16 >> 8;
	locWriteData_au8[1] = locValue_i16;

	return locAD983x_p->spiHandle(SPI_DDS_EVENT_TRANSMIT, locWriteData_au8, 2, NULL);
}


static AD983x_State_t AD983x_WriteControlRegister(DDS_Def_t *locAD983x_p)
{
	uint16_t locWaveForm_u16 = 0;

	NULL_CHECK_PARAM(locAD983x_p);

	if (REG0 == locAD983x_p->activeFrequency)
	{
		locWaveForm_u16 = locAD983x_p->waveForm0;
		locWaveForm_u16 &= ~FREQ1_OUTPUT_REG;
	}
	else
	{
		locWaveForm_u16 = locAD983x_p->waveForm1;
		locWaveForm_u16 |= FREQ1_OUTPUT_REG;
	}

	if (REG0 == locAD983x_p->activePhase)
	{
		locWaveForm_u16 &= ~PHASE1_OUTPUT_REG;
	}
	else
	{
		locWaveForm_u16 |= PHASE1_OUTPUT_REG;
	}

	if (locAD983x_p->outputEnabled)
	{
		locWaveForm_u16 &= ~RESET_CMD;
	}
	else
	{
		locWaveForm_u16 |= RESET_CMD;
	}

	if (locAD983x_p->dacEnabled)
	{
		locWaveForm_u16 &= ~DISABLE_DAC;
	}
	else
	{
		locWaveForm_u16 |= DISABLE_DAC;
	}

	if (locAD983x_p->internalClockEnabled)
	{
		locWaveForm_u16 &= ~DISABLE_INT_CLK;
	}
	else
	{
		locWaveForm_u16 |= DISABLE_INT_CLK;
	}

	return AD983x_WriteRegister(locAD983x_p, locWaveForm_u16);
}


AD983x_State_t AD983x_Init(DDS_Def_t *locAD983x_p)
{
	NULL_CHECK_PARAM(locAD983x_p);

	locAD983x_p->waveForm0 = locAD983x_p->waveForm1 = SINE_WAVE;
	locAD983x_p->phase0 = locAD983x_p->phase1 = 0.0;
	locAD983x_p->activeFrequency = REG0;
	locAD983x_p->activePhase = REG1;

	return AD983x_Reset(locAD983x_p);
}


AD983x_State_t AD983x_Reset(DDS_Def_t *locAD983x_p)
{
	AD983x_State_t locRet = AD983x_OK;

	NULL_CHECK_PARAM(locAD983x_p);

	locRet = AD983x_WriteRegister(locAD983x_p, RESET_CMD);
	locAD983x_p->delayHandle(20);

	return locRet;
}


AD983x_State_t AD983x_EnableOutput(DDS_Def_t *locAD983x_p, uint8_t locOutputState_u8)
{
	NULL_CHECK_PARAM(locAD983x_p);

	locAD983x_p->outputEnabled = locOutputState_u8;

	return AD983x_WriteControlRegister(locAD983x_p);
}


AD983x_State_t AD983x_SetOutputSource(DDS_Def_t *locAD983x_p, Registers_t locFrequencyRegister_en, Registers_t locPhaseRegister_en)
{
	NULL_CHECK_PARAM(locAD983x_p);

	locAD983x_p->activeFrequency = locFrequencyRegister_en;
	locAD983x_p->activePhase = locPhaseRegister_en;

	return AD983x_WriteControlRegister(locAD983x_p);
}


AD983x_State_t AD983x_EnableDAC(DDS_Def_t *locAD983x_p, uint8_t locDACState_u8)
{
	NULL_CHECK_PARAM(locAD983x_p);

	locAD983x_p->dacEnabled = locDACState_u8;

	return AD983x_WriteControlRegister(locAD983x_p);
}


AD983x_State_t AD983x_EnableInternalClock(DDS_Def_t *locAD983x_p, uint8_t locInternalClockState_u8)
{
	NULL_CHECK_PARAM(locAD983x_p);

	locAD983x_p->internalClockEnabled = locInternalClockState_u8;

	return AD983x_WriteControlRegister(locAD983x_p);
}


AD983x_State_t AD983x_SetFrequency(DDS_Def_t *locAD983x_p, Registers_t locFrequencyRegister_en, uint32_t locFrequencyInHz_u32)
{
	int16_t locFrequencyLower_u16 = 0;
	int16_t locFrequencyUpper_u16 = 0;
	uint16_t locOutputRegister_u16 = REG0;
	uint32_t locFrequency_u32 = 0;
	AD983x_State_t locRet = AD983x_OK;

	NULL_CHECK_PARAM(locAD983x_p);

	if (locFrequencyInHz_u32 > 12500000)
	{
		locFrequencyInHz_u32 = 12500000;
	}

	if (REG0 == locFrequencyRegister_en)
	{
		locAD983x_p->frequency0 = locFrequencyInHz_u32;
	}
	else if (REG1 == locFrequencyRegister_en)
	{
		locAD983x_p->frequency1 = locFrequencyInHz_u32;
	}

	locFrequency_u32 = (uint32_t)((double)((double)locFrequencyInHz_u32 / (double)locAD983x_p->referenceFrequency) * (double)pow2_28);
	locFrequencyUpper_u16 = (locFrequency_u32 & 0x0FFFC000) >> 14;
	locFrequencyLower_u16 = (locFrequency_u32 & 0x00003FFF);
	locOutputRegister_u16 = (REG0 == locFrequencyRegister_en)? FREQ0_WRITE_REG : FREQ1_WRITE_REG;
	locFrequencyUpper_u16 |= locOutputRegister_u16;
	locFrequencyLower_u16 |= locOutputRegister_u16;

	locRet |= AD983x_WriteControlRegister(locAD983x_p);
	locRet |= AD983x_WriteRegister(locAD983x_p, locFrequencyLower_u16);
	locRet |= AD983x_WriteRegister(locAD983x_p, locFrequencyUpper_u16);

	return locRet;
}


AD983x_State_t AD983x_SetPhase(DDS_Def_t *locAD983x_p, Registers_t locPhaseRegister_en, float locPhaseInDegree_f)
{
	uint16_t locPhase_u16 = 0;

	NULL_CHECK_PARAM(locAD983x_p);

	locPhaseInDegree_f = fmod(locPhaseInDegree_f, 360);

	if (locPhaseInDegree_f < 0)
	{
		locPhaseInDegree_f += 360.0;
	}

	locPhase_u16 = (uint16_t)(BITS_PER_DEG * locPhaseInDegree_f) & 0x0FFF;
	locPhase_u16 |= PHASE_WRITE_CMD;

	if (REG0 == locPhaseRegister_en)
	{
		locAD983x_p->phase0 = locPhaseInDegree_f;
	}
	else if (REG1 == locPhaseRegister_en)
	{
		locAD983x_p->phase1 = locPhaseInDegree_f;
		locPhase_u16 |= PHASE1_WRITE_REG;
	}

	return AD983x_WriteRegister(locAD983x_p, locPhase_u16);
}


AD983x_State_t AD983x_SetWaveForm(DDS_Def_t *locAD983x_p, Registers_t locWaveFormRegister_en, WaveformType_t locWaveFormType_en)
{
	NULL_CHECK_PARAM(locAD983x_p);

	if (REG0 == locWaveFormRegister_en)
	{
		locAD983x_p->waveForm0 = locWaveFormType_en;
	}
	else if (REG1 == locWaveFormRegister_en)
	{
		locAD983x_p->waveForm1 = locWaveFormType_en;
	}

	return AD983x_WriteControlRegister(locAD983x_p);
}


AD983x_State_t AD983x_ApplySignal(DDS_Def_t *locAD983x_p,
								  Registers_t locFrequencyRegister_en,
								  uint32_t locFrequencyInHz_u32,
								  Registers_t locPhaseRegister_en,
								  float locPhaseInDegree_f,
								  Registers_t locWaveFormRegister_en,
								  WaveformType_t locWaveFormType_en)
{
	AD983x_State_t locRet = AD983x_OK;

	NULL_CHECK_PARAM(locAD983x_p);

	locRet |= AD983x_SetOutputSource(locAD983x_p, locFrequencyRegister_en, locPhaseRegister_en);
	locRet |= AD983x_SetFrequency(locAD983x_p, locFrequencyRegister_en, locFrequencyInHz_u32);
	locRet |= AD983x_SetPhase(locAD983x_p, locPhaseRegister_en, locPhaseInDegree_f);
	locRet |= AD983x_SetWaveForm(locAD983x_p, locWaveFormRegister_en, locWaveFormType_en);

	return locRet;
}


AD983x_State_t AD983x_GetActualProgrammedFrequency(DDS_Def_t *locAD983x_p, Registers_t locFrequencyRegister_en, float *locFrequency_pf)
{
	uint32_t locFrequency_u32;
	float locFrequency_f;

	NULL_CHECK_PARAM(locAD983x_p);

	locFrequency_f = (REG0 == locFrequencyRegister_en)? locAD983x_p->frequency0 : locAD983x_p->frequency1;
	locFrequency_u32 = (uint32_t)((locFrequency_f * pow2_28) / (float)locAD983x_p->referenceFrequency) & 0x0FFFFFFFUL;
	*locFrequency_pf = ((float)locFrequency_u32 * (float)locAD983x_p->referenceFrequency) / (float)pow2_28;

	return AD983x_OK;
}


AD983x_State_t AD983x_GetActualProgrammedPhase(DDS_Def_t *locAD983x_p, Registers_t locPhaseRegister_en, float *locPhase_pf)
{
	uint16_t locPhase_u16;
	float locPhase_f;

	NULL_CHECK_PARAM(locAD983x_p);

	locPhase_f = (REG0 == locPhaseRegister_en)? locAD983x_p->phase0 : locAD983x_p->phase1;
	locPhase_u16 = (uint16_t)(BITS_PER_DEG * locPhase_f) & 0x0FFF;
	*locPhase_pf = (float)locPhase_u16 / BITS_PER_DEG;

	return AD983x_OK;
}


AD983x_State_t AD983x_GetResolution(DDS_Def_t *locAD983x_p, float *locResolution_pf)
{
	NULL_CHECK_PARAM(locAD983x_p);

	*locResolution_pf = (float)locAD983x_p->referenceFrequency / (float)pow2_28;

	return AD983x_OK;
}


AD983x_State_t AD983x_Sleep(DDS_Def_t *locAD983x_p)
{
	NULL_CHECK_PARAM(locAD983x_p);

	locAD983x_p->dacEnabled = 0;
	locAD983x_p->internalClockEnabled = 0;

	return AD983x_WriteControlRegister(locAD983x_p);
}
