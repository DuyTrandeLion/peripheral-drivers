/*
 * ad9833.h
 *
 *  Created on: Aug 15, 2021
 *      Author: Admin
 */

#ifndef _AD9833_H_
#define _AD9833_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define pow2_28				268435456UL			/* 2^28 used in frequency word calculation	*/
#define BITS_PER_DEG		11.3777777777778	/* 4096 / 360 */

/* Control register */
#define DISABLE_DAC			(0x01 << 6)
#define	DISABLE_INT_CLK		(0x01 << 7)
#define SLEEP_MODE			(DISABLE_DAC | DISABLE_INT_CLK)	/* Both DAC and Internal Clock	*/
#define RESET_CMD			(0x01 << 8)			/* Reset enabled (also CMD RESET) 			*/
#define PHASE1_OUTPUT_REG	(0x01 << 10)		/* Defines whether the PHASE0 register or the PHASE1 register data is added to the output of the phase accumulator.	*/
#define FREQ1_OUTPUT_REG	(0x01 << 11)		/* Defines whether the FREQ0 register or the FREQ1 register is used in the phase accumulator.						*/

/* Frequency register */
#define FREQ0_WRITE_REG		(0x01 << 14)
#define FREQ1_WRITE_REG		(0x01 << 15)

#define PHASE_WRITE_CMD		0xC000				/* Setup for Phase write					*/
#define PHASE1_WRITE_REG	0x2000				/* Which phase register						*/

/** AD9833 DDS return codes. */
typedef enum
{
    AD983x_OK					= 0x00,
    AD983x_DEVICE_BUSY			= 0x01,
    AD983x_NULL_PARAM			= 0x02,
    AD983x_COMM_ERROR			= 0x03,
    AD983x_COMM_TIMEOUT			= 0x04,
    AD983x_UNKNOWN_ERROR		= 0x05,
} AD983x_State_t;

/** SPI event types of communication. */
typedef enum
{
    SPI_DDS_EVENT_TRANSMIT					= 0x00,
    SPI_DDS_EVENT_RECEIVE					= 0x01,
    SPI_DDS_EVENT_TRANSMIT_RECEIVE			= 0x02,
    SPI_DDS_EVENT_ABORT_TRANSMIT			= 0x03,
    SPI_DDS_EVENT_ABORT_RECEIVE				= 0x04,
    SPI_DDS_EVENT_ABORT_TRANSMIT_RECEIVE	= 0x05,
    SPI_DDS_EVENT_CLEAR_INPUT				= 0x06,
    SPI_DDS_EVENT_BUSY_WAIT					= 0x07,
} DDS_SPI_Event_t;

/** Supported waveform type. */
typedef enum
{
	SINE_WAVE			= 0x2000,
	TRIANGLE_WAVE		= 0x2002,
	SQUARE_WAVE			= 0x2028,
	HALF_SQUARE_WAVE	= 0x2020,
} WaveformType_t;

/** Data registers. */
typedef enum
{
	REG0,
	REG1,
} Registers_t;


/**
 * SPI event callback type.
 *
 * This function is called when there is SPI communication.
 *
 * @param[in] DDS_SPI_Event_t 	SPI Event type.
 * @param[in] uint8_t *		Pointer to data buffer.
 * @param[in] uint16_t		Size of data buffer.
 * @param[in] void *		Pointer to parameter of event handler
 *
 * @retval AD983x_OK If the notification was sent successfully. Otherwise, an error code is returned.
 */
typedef AD983x_State_t (*DDS_SPI_Handle_t)(DDS_SPI_Event_t locCommEvent_en, uint8_t *locData_p8, uint16_t locDataSize_u16, void *locContext_p);


/**
 * Busy time counting handle callback.
 *
 * @retval true 	If the device is busy
 */
typedef void (*DDS_Delay_Handle_t)(uint32_t locDelayTime_u32);


/** DAC definition structure.  */
typedef struct
{
	uint8_t					outputEnabled;
	uint8_t					dacEnabled;
	uint8_t					internalClockEnabled;
	uint16_t				waveForm0;
	uint16_t				waveForm1;
	uint32_t				referenceFrequency;
	float					frequency0;
	float					frequency1;
	float					phase0;
	float					phase1;
	Registers_t				activeFrequency;
	Registers_t				activePhase;
    DDS_SPI_Handle_t 		spiHandle;
    DDS_Delay_Handle_t		delayHandle;
} DDS_Def_t;


/**
 * @brief Function to reset the waveform generator.
 *
 * @param[in] locAD983x_p		Pointer to driver definition.
 *
 * @retval AD983x_OK			If the driver was reseted successfully.
 *
 */
AD983x_State_t AD983x_Reset(DDS_Def_t *locAD983x_p);


/**
 * @brief Function to initialize and setup the waveform generator.
 *
 * @param[in] locAD983x_p		Pointer to driver definition.
 *
 * @retval AD983x_OK			If the driver was initialized successfully.
 *
 */
AD983x_State_t AD983x_Init(DDS_Def_t *locAD983x_p);


/**
 * @brief Function to enable output of the waveform generator by disabling the reset bit.
 *
 * @param[in] locAD983x_p			Pointer to driver definition.
 * @param[in] locOutputState_u8		Output state, carries only 0/1 value.
 *
 * @retval AD983x_OK				If the output was set successfully.
 *
 */
AD983x_State_t AD983x_EnableOutput(DDS_Def_t *locAD983x_p, uint8_t locOutputState_u8);


/**
 * @brief Function to set output source of the waveform generator
 *
 * @param[in] locAD983x_p				Pointer to driver definition.
 * @param[in] locFrequencyRegister_en	Output source of Frequency register, carries values from Registers_t.
 * @param[in] locPhaseRegister_en		Output source of Phase register, carries values from Registers_t.
 *
 * @retval AD983x_OK					If the feature was set successfully.
 *
 */
AD983x_State_t AD983x_SetOutputSource(DDS_Def_t *locAD983x_p, Registers_t locFrequencyRegister_en, Registers_t locPhaseRegister_en);


/**
 * @brief Function to enable DAC of the waveform generator.
 *
 * @param[in] locAD983x_p			Pointer to driver definition.
 * @param[in] locDACState_u8		DAC state, carries only 0/1 value.
 *
 * @retval AD983x_OK				If the DAC was set successfully.
 *
 */
AD983x_State_t AD983x_EnableDAC(DDS_Def_t *locAD983x_p, uint8_t locDACState_u8);


/**
 * @brief Function to enable internal clock of the waveform generator.
 *
 * @param[in] locAD983x_p				Pointer to driver definition.
 * @param[in] locInternalClockState_u8	Internal clock state, carries only 0/1 value.
 *
 * @retval AD983x_OK					If the internal clock was set successfully.
 *
 */
AD983x_State_t AD983x_EnableInternalClock(DDS_Def_t *locAD983x_p, uint8_t locInternalClockState_u8);


/**
 * @brief Function to set the waveform frequency of the waveform generator.
 *
 * @param[in] locAD983x_p				Pointer to driver definition.
 * @param[in] locFrequencyRegister_en	Output source of Frequency register, carries values from Registers_t.
 * @param[in] locFrequencyInHz_u32		Desired frequency.
 *
 * @retval AD983x_OK					If the frequency was set successfully.
 *
 */
AD983x_State_t AD983x_SetFrequency(DDS_Def_t *locAD983x_p, Registers_t locFrequencyRegister_en, uint32_t locFrequencyInHz_u32);


/**
 * @brief Function to set the waveform phase of the waveform generator.
 *
 * @param[in] locAD983x_p				Pointer to driver definition.
 * @param[in] locPhaseRegister_en		Output source of Phase register, carries values from Registers_t.
 * @param[in] locPhaseInDegree_f		Desired phase shift.
 *
 * @retval AD983x_OK					If the phase shift was set successfully.
 *
 */
AD983x_State_t AD983x_SetPhase(DDS_Def_t *locAD983x_p, Registers_t locPhaseRegister_en, float locPhaseInDegree_f);


/**
 * @brief Function to set the waveform type of the waveform generator.
 *
 * @param[in] locAD983x_p				Pointer to driver definition.
 * @param[in] locWaveFormRegister_en	Waveform register, carries values from Registers_t.
 * @param[in] locWaveFormType_en		Waveform type, carries values from WaveformType_t.
 *
 * @retval AD983x_OK					If the waveform type was set successfully.
 *
 */
AD983x_State_t AD983x_SetWaveForm(DDS_Def_t *locAD983x_p, Registers_t locWaveFormRegister_en, WaveformType_t locWaveFormType_en);


/**
 * @brief Function to set a signal for the waveform generator.
 *
 * @param[in] locAD983x_p				Pointer to driver definition.
 * @param[in] locFrequencyRegister_en	Output source of Frequency register, carries values from Registers_t.
 * @param[in] locFrequencyInHz_u32		Desired frequency.
 * @param[in] locPhaseRegister_en		Output source of Phase register, carries values from Registers_t.
 * @param[in] locPhaseInDegree_f		Desired phase shift.
 * @param[in] locWaveFormRegister_en	Waveform register, carries values from Registers_t.
 * @param[in] locWaveFormType_en		Waveform type, carries values from WaveformType_t.
 *
 * @retval AD983x_OK					If the desired signal was set successfully.
 *
 */
AD983x_State_t AD983x_ApplySignal(DDS_Def_t *locAD983x_p,
								  Registers_t locFrequencyRegister_en,
								  uint32_t locFrequencyInHz_u32,
								  Registers_t locPhaseRegister_en,
								  float locPhaseInDegree_f,
								  Registers_t locWaveFormRegister_en,
								  WaveformType_t locWaveFormType_en);


/**
 * @brief Function to get the actual frequency programmed to the waveform generator.
 *
 * @param[in] locAD983x_p				Pointer to driver definition.
 * @param[in] locFrequencyRegister_en	Output source of Frequency register, carries values from Registers_t.
 * @param[out] locFrequency_pf			Waveform generator programmed frequency.
 *
 * @retval AD983x_OK					If the data is successfully read.
 *
 */
AD983x_State_t AD983x_GetActualProgrammedFrequency(DDS_Def_t *locAD983x_p, Registers_t locFrequencyRegister_en, float *locFrequency_pf);


/**
 * @brief Function to get the actual phase programmed to the waveform generator.
 *
 * @param[in] locAD983x_p				Pointer to driver definition.
 * @param[in] locPhaseRegister_en		Output source of Phase register, carries values from Registers_t.
 * @param[out] locPhase_pf				Waveform generator programmed phase.
 *
 * @retval AD983x_OK					If the data is successfully read.
 *
 */
AD983x_State_t AD983x_GetActualProgrammedPhase(DDS_Def_t *locAD983x_p, Registers_t locPhaseRegister_en, float *locPhase_pf);


/**
 * @brief Function to get the resolution of the frequency programmed to the waveform generator.
 *
 * @param[in] locAD983x_p				Pointer to driver definition.
 * @param[out] locResolution_pf			Frequency resolution.
 *
 * @retval AD983x_OK					If the data is successfully read.
 *
 */
AD983x_State_t AD983x_GetFrequencyResolution(DDS_Def_t *locAD983x_p, float *locResolution_pf);


/**
 * @brief Function to put the waveform generator to sleep mode.
 *
 * @param[in] locAD983x_p				Pointer to driver definition.
 *
 * @retval AD983x_OK					If the device is slept.
 *
 */
AD983x_State_t AD983x_Sleep(DDS_Def_t *locAD983x_p);


#ifdef __cplusplus
}
#endif

#endif /* _AD9833_H_ */
