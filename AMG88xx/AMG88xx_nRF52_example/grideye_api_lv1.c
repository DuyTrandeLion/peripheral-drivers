/*******************************************************************************
 Copyright (C) <2015>, <Panasonic Corporation>
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1.	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2.	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.	The name of copyright holders may not be used to endorse or promote products derived from this software without specific prior written permission.
4.	This software code may only be redistributed and used in connection with a grid-eye product.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR POFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND OR ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY; OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project. 

 ******************************************************************************/


/*******************************************************************************
	include file
*******************************************************************************/
#include	"AMG88xx_conf.h"
#include        "grideye_api_lv1.h"


/*******************************************************************************
	macro definition
*******************************************************************************/

/* Grid-EYE's number of pixels */
#define		SNR_SZ_X			(8)
#define		SNR_SZ_Y			(8)
#define		SNR_SZ				(SNR_SZ_X * SNR_SZ_Y)


/*******************************************************************************
	public method
 ******************************************************************************/

/*------------------------------------------------------------------------------
	Read data form I2C bus.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_I2C_Read( UCHAR ucI2cAddr, UCHAR ucRegAddr, UCHAR ucSize, UCHAR* ucDstAddr )
{
	#if			defined(MCU_TEST)
	{
		extern UCHAR I2C_Read( UCHAR, UCHAR, UCHAR*, UCHAR );
		return( I2C_Read( ucI2cAddr, ucRegAddr, ucDstAddr, ucSize ) );
	}
	#else	/* !defined(MCU_TEST) */
	{
		return( 1 );
	}
	#endif	/*  defined(MCU_TEST) */
}

/*------------------------------------------------------------------------------
	Convert thermistor register value.
------------------------------------------------------------------------------*/
SHORT shAMG_PUB_TMP_ConvThermistor( UCHAR aucRegVal[2] )
{
	SHORT shVal = ((SHORT)(aucRegVal[1] & 0x07) << 8) | aucRegVal[0];

	if( 0 != (0x08 & aucRegVal[1]) )
	{
		shVal *= -1;
	}

	shVal *= 16;

	return( shVal );
}

/*------------------------------------------------------------------------------
	Convert temperature register value for 1 pixel.
------------------------------------------------------------------------------*/
SHORT shAMG_PUB_TMP_ConvTemperature( UCHAR aucRegVal[2] )
{
	SHORT shVal = ((SHORT)(aucRegVal[1] & 0x07) << 8) | aucRegVal[0];

	if( 0 != (0x08 & aucRegVal[1]) )
	{
		shVal -= 2048;
	}

	shVal *= 64;

	return( shVal );
}

/*------------------------------------------------------------------------------
	Convert temperature register value for 64 pixel.
------------------------------------------------------------------------------*/
void vAMG_PUB_TMP_ConvTemperature64( UCHAR* pucRegVal, SHORT* pshVal )
{
	UCHAR ucCnt = 0;

	for( ucCnt = 0; ucCnt < SNR_SZ; ucCnt++ )
	{
		pshVal[ucCnt] = shAMG_PUB_TMP_ConvTemperature( pucRegVal + ucCnt * 2 );
	}
}

/*------------------------------------------------------------------------------
	Convert value.
------------------------------------------------------------------------------*/
SHORT shAMG_PUB_CMN_ConvFtoS( float fVal )
{
	return( ( fVal > 0 ) ? (SHORT)(fVal * 256 + 0.5) : (SHORT)(fVal * 256 - 0.5) );
}

/*------------------------------------------------------------------------------
	Convert value.
------------------------------------------------------------------------------*/
float fAMG_PUB_CMN_ConvStoF( SHORT shVal )
{
	return( (float)shVal / 256 );
}
