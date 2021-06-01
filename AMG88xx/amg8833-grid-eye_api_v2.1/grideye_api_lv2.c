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
#include	"grideye_api_lv1.h"
#include        "grideye_api_lv2.h"


/*******************************************************************************
	macro definition
*******************************************************************************/
#define		TRUE				(1)
#define		FALSE				(0)

#define		SHORT_MAX_VAL		( 32767)					/* 0x7FFF		*/
#define		SHORT_MIN_VAL		(-32768)					/* 0x8000		*/
#define		ULONG_MAX_VAL		( 4294967295)				/* 0xFFFFFFFF	*/

/* Grid-EYE's number of pixels */
#define		SNR_SZ_X			(8)
#define		SNR_SZ_Y			(8)
#define		SNR_SZ				(SNR_SZ_X * SNR_SZ_Y)


/*******************************************************************************
	public method
 ******************************************************************************/

/*------------------------------------------------------------------------------
	Calculate average.
------------------------------------------------------------------------------*/
SHORT shAMG_PUB_CMN_CalcAve( SHORT* pshArray, USHORT usSize, UCHAR ucSkip, UCHAR ucMedian, BOOL* pbMedianWork )
{
	SHORT shAve = 0;

	if( 1 >= usSize )
	{
		return( *pshArray );
	}

	/* Adjust parameter. */
	if( 1 > ucSkip )
	{
		ucSkip = 1;
	}
	if( ucMedian > ((usSize - 1) / 2) )
	{
		ucMedian = ((usSize - 1) / 2);
	}

	/* Calculate average. */
	if( 0 == ucMedian )
	{
		USHORT	usCnt = 0;
		long	loSum = 0;

		for( usCnt = 0; usCnt < usSize; usCnt++ )
		{
			SHORT	shCurData = pshArray[usCnt * ucSkip];
			loSum += shCurData;
		}
		shAve = (SHORT)(loSum / usSize);
	}
	else
	{
		USHORT	usCnt = 0;
		long	loSum = 0;
		UCHAR	ucMedianCnt = 0;

		for( usCnt = 0; usCnt < usSize; usCnt++ )
		{
			pbMedianWork[usCnt] = TRUE;
		}

		for( ucMedianCnt = 0; ucMedianCnt < ucMedian; ucMedianCnt++ )
		{
			SHORT	shMaxData = SHORT_MIN_VAL;
			SHORT	shMinData = SHORT_MAX_VAL;
			UCHAR	ucIndex = 0;

			for( usCnt = 0; usCnt < usSize; usCnt++ )
			{
				if( FALSE != pbMedianWork[usCnt] )
				{
					SHORT	shCurData = pshArray[usCnt * ucSkip];
					if( shMaxData < shCurData )
					{
						shMaxData = shCurData;
						ucIndex = usCnt;
					}
				}
			}
			pbMedianWork[ucIndex] = FALSE;

			for( usCnt = 0; usCnt < usSize; usCnt++ )
			{
				if( FALSE != pbMedianWork[usCnt] )
				{
					SHORT	shCurData = pshArray[usCnt * ucSkip];
					if( shMinData > shCurData )
					{
						shMinData = shCurData;
						ucIndex = usCnt;
					}
				}
			}
			pbMedianWork[ucIndex] = FALSE;
		}

		for( usCnt = 0; usCnt < usSize; usCnt++ )
		{
			SHORT	shCurData = pshArray[usCnt * ucSkip];
			if( FALSE != pbMedianWork[usCnt] )
			{
				loSum += shCurData;
			}
		}
		shAve = (SHORT)(loSum / ( usSize - ucMedian * 2 ));
	}

	return( shAve );
}

/*------------------------------------------------------------------------------
	Calculate average.
------------------------------------------------------------------------------*/
SHORT shAMG_PUB_CMN_CalcIIR( SHORT shVal1, SHORT shVal2, SHORT shTh )
{
	const SHORT c_shMinTh = 0;
	const SHORT c_shMaxTh = 256;
	long loAddVal = 0;

	/* Adjust parameter. */
	if( c_shMinTh > shTh )
	{
		shTh = c_shMinTh;
	}
	if( c_shMaxTh < shTh )
	{
		shTh = c_shMaxTh;
	}

	/* Calculate average. */
	loAddVal = (long)shTh * ( shVal2 - shVal1 );
	return( shVal1 + (SHORT)(loAddVal / c_shMaxTh) );
}

/*------------------------------------------------------------------------------
	Calculate average.
------------------------------------------------------------------------------*/
SHORT shAMG_PUB_CMN_CalcIIR_f( SHORT shVal1, SHORT shVal2, float fTh )
{
	const float c_fMinTh = 0;
	const float c_fMaxTh = 1;
	float fAddVal = 0;

	/* Adjust parameter. */
	if( c_fMinTh > fTh )
	{
		fTh = c_fMinTh;
	}
	if( c_fMaxTh < fTh )
	{
		fTh = c_fMaxTh;
	}
	/* Calculate average. */
	fAddVal = fTh * ( shVal2 - shVal1 );
	return( shVal1 + (SHORT)fAddVal );
}

/*------------------------------------------------------------------------------
	Convert image : ( x, y ) -> ( ucWidth-1-x , y )
------------------------------------------------------------------------------*/
void vAMG_PUB_IMG_ConvertFlipX( UCHAR ucWidth, UCHAR ucHeight, SHORT* pshInImg, SHORT* pshOutImg )
{
	{
		UCHAR ucX = 0;
		UCHAR ucY = 0;
		for( ucY = 0; ucY < ucHeight; ucY++ )
		{
			for( ucX = 0; ucX < (ucWidth+1)/2; ucX++ )
			{
				USHORT	usIndex1 = (USHORT)(              ucX) + ucY * ucWidth;
				USHORT	usIndex2 = (USHORT)(ucWidth - 1 - ucX) + ucY * ucWidth;
				SHORT	shVal = pshInImg[usIndex1];
				pshOutImg[usIndex1] = pshInImg[usIndex2];
				pshOutImg[usIndex2] = shVal;
			}
		}
	}
}

/*------------------------------------------------------------------------------
	Convert image : ( x, y ) -> ( x, ucHeight-1-y )
------------------------------------------------------------------------------*/
void vAMG_PUB_IMG_ConvertFlipY( UCHAR ucWidth, UCHAR ucHeight, SHORT* pshInImg, SHORT* pshOutImg )
{
	{
		UCHAR ucX = 0;
		UCHAR ucY = 0;
		for( ucY = 0; ucY < (ucHeight+1)/2; ucY++ )
		{
			for( ucX = 0; ucX < ucWidth; ucX++ )
			{
				USHORT	usIndex1 = (USHORT)ucX + (               ucY) * ucWidth;
				USHORT	usIndex2 = (USHORT)ucX + (ucHeight - 1 - ucY) * ucWidth;
				SHORT	shVal = pshInImg[usIndex1];
				pshOutImg[usIndex1] = pshInImg[usIndex2];
				pshOutImg[usIndex2] = shVal;
			}
		}
	}
}

/*------------------------------------------------------------------------------
	Convert image : ( x, y ) -> ( y, x )
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_IMG_ConvertFlipXY( UCHAR ucWidth, UCHAR ucHeight, SHORT* pshInImg, SHORT* pshOutImg )
{
	BOOL bRet = FALSE;

	if( ucWidth == ucHeight )
	{
		UCHAR ucX = 0;
		UCHAR ucY = 0;
		for( ucY = 0; ucY < ucHeight; ucY++ )
		{
			for( ucX = ucY; ucX < ucWidth; ucX++ )
			{
				USHORT	usIndex1 = (USHORT)ucX + ucY * ucWidth;
				USHORT	usIndex2 = (USHORT)ucY + ucX * ucWidth;
				SHORT	shVal = pshInImg[usIndex1];
				pshOutImg[usIndex1] = pshInImg[usIndex2];
				pshOutImg[usIndex2] = shVal;
			}
		}
		bRet = TRUE;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Convert image : ( x, y ) -> ( ucHeight-1-y, x )
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_IMG_ConvertRotate90( UCHAR ucWidth, UCHAR ucHeight, SHORT* pshInImg, SHORT* pshOutImg )
{
	BOOL bRet = FALSE;

	if( ucWidth == ucHeight )
	{
		UCHAR ucX = 0;
		UCHAR ucY = 0;
		for( ucY = 0; ucY < (ucHeight+1)/2; ucY++ )
		{
			for( ucX = ucY; ucX < ucWidth - ucY - 1; ucX++ )
			{
				USHORT	usIndex1 = (USHORT)(              ucX) + (              ucY) * ucWidth;
				USHORT	usIndex2 = (USHORT)(              ucY) + (ucWidth - 1 - ucX) * ucWidth;
				USHORT	usIndex3 = (USHORT)(ucWidth - 1 - ucX) + (ucWidth - 1 - ucY) * ucWidth;
				USHORT	usIndex4 = (USHORT)(ucWidth - 1 - ucY) + (              ucX) * ucWidth;
				SHORT	shVal = pshInImg[usIndex1];
				pshOutImg[usIndex1] = pshInImg[usIndex2];
				pshOutImg[usIndex2] = pshInImg[usIndex3];
				pshOutImg[usIndex3] = pshInImg[usIndex4];
				pshOutImg[usIndex4] = shVal;
			}
			if( ucX == ucY )
			{
				USHORT	usIndex = (USHORT)ucX + ucY * ucWidth;
				pshOutImg[usIndex] = pshInImg[usIndex];
			}
		}

		bRet = TRUE;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Convert image : ( x, y ) -> ( ucWidth-1-x, ucHeight-1-y )
------------------------------------------------------------------------------*/
void vAMG_PUB_IMG_ConvertRotate180( UCHAR ucWidth, UCHAR ucHeight, SHORT* pshInImg, SHORT* pshOutImg )
{
	USHORT usSize = ucWidth * ucHeight;
	USHORT usIndex1 = 0;

	for( usIndex1 = 0; usIndex1 < (usSize+1)/2; usIndex1++ )
	{
		USHORT	usIndex2 = usSize - 1 - usIndex1;
		SHORT	shVal = pshInImg[usIndex1];
		pshOutImg[usIndex1] = pshInImg[usIndex2];
		pshOutImg[usIndex2] = shVal;
	}
}

/*------------------------------------------------------------------------------
	Convert image : ( x, y ) -> ( y, ucWidth-1-x )
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_IMG_ConvertRotate270( UCHAR ucWidth, UCHAR ucHeight, SHORT* pshInImg, SHORT* pshOutImg )
{
	BOOL bRet = FALSE;

	if( ucWidth == ucHeight )
	{
		UCHAR ucX = 0;
		UCHAR ucY = 0;
		for( ucY = 0; ucY < (ucHeight+1)/2; ucY++ )
		{
			for( ucX = ucY; ucX < ucWidth - ucY - 1; ucX++ )
			{
				USHORT	usIndex1 = (USHORT)(              ucX) + (              ucY) * ucWidth;
				USHORT	usIndex2 = (USHORT)(ucWidth - 1 - ucY) + (              ucX) * ucWidth;
				USHORT	usIndex3 = (USHORT)(ucWidth - 1 - ucX) + (ucWidth - 1 - ucY) * ucWidth;
				USHORT	usIndex4 = (USHORT)(              ucY) + (ucWidth - 1 - ucX) * ucWidth;
				SHORT	shVal = pshInImg[usIndex1];
				pshOutImg[usIndex1] = pshInImg[usIndex2];
				pshOutImg[usIndex2] = pshInImg[usIndex3];
				pshOutImg[usIndex3] = pshInImg[usIndex4];
				pshOutImg[usIndex4] = shVal;
			}
			if( ucX == ucY )
			{
				USHORT	usIndex = (USHORT)ucX + ucY * ucWidth;
				pshOutImg[usIndex] = pshInImg[usIndex];
			}
		}

		bRet = TRUE;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Linear interpolation : 8 x 8 -> 15 x 15
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_IMG_LinearInterpolationSQ15( SHORT* pshInImg, SHORT* pshOutImg )
{
	const UCHAR c_ucImgWidth  = 15;
	const UCHAR c_ucImgHeight = 15;
	BOOL bRet = FALSE;

	if( pshInImg != pshOutImg )
	{
		UCHAR ucX = 0;
		UCHAR ucY = 0;
		for( ucY = 0; ucY < c_ucImgHeight; ucY += 2 )
		{
			for( ucX = 0; ucX < c_ucImgWidth; ucX += 2 )
			{
				UCHAR ucSnr = ucX / 2 + ucY / 2 * SNR_SZ_X;
				UCHAR ucImg = ucX + ucY * c_ucImgWidth;
				pshOutImg[ucImg] = pshInImg[ucSnr];
			}
			for( ucX = 1; ucX < c_ucImgWidth; ucX += 2 )
			{
				UCHAR ucImg = ucX + ucY * c_ucImgWidth;
				pshOutImg[ucImg] = ( pshOutImg[ucImg-1] + pshOutImg[ucImg+1] ) / 2;
			}
		}
		for( ucY = 1; ucY < c_ucImgHeight; ucY += 2 )
		{
			for( ucX = 0; ucX < c_ucImgWidth; ucX++ )
			{
				UCHAR ucImg = ucX + ucY * c_ucImgWidth;
				pshOutImg[ucImg] = ( pshOutImg[ucImg-c_ucImgWidth] + pshOutImg[ucImg+c_ucImgWidth] ) / 2;
			}
		}

		bRet = TRUE;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Linear interpolation : 8 x 8 -> ucWidth x ucHeight
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_IMG_LinearInterpolation( UCHAR ucWidth, UCHAR ucHeight, SHORT* pshInImg, SHORT* pshOutImg )
{
	const UCHAR c_ucRes = 16;
	BOOL bRet = FALSE;

	if( pshInImg != pshOutImg )
	{
		USHORT	usImg = 0;

		for( usImg = 0; usImg < ucWidth * ucHeight; usImg++ )
		{
			UCHAR	ucImgX		= usImg % ucWidth;
			UCHAR	ucImgY		= usImg / ucWidth;
			USHORT	usSnrXn		= (USHORT)( c_ucRes * ucImgX * (SNR_SZ_X-1) / (ucWidth -1) );
			USHORT	usSnrYn		= (USHORT)( c_ucRes * ucImgY * (SNR_SZ_Y-1) / (ucHeight-1) );
			UCHAR	ucSnrX		= (UCHAR)( usSnrXn / c_ucRes );
			UCHAR	ucSnrY		= (UCHAR)( usSnrYn / c_ucRes );
			UCHAR	ucRateX1	= (UCHAR)( usSnrXn % c_ucRes );
			UCHAR	ucRateY1	= (UCHAR)( usSnrYn % c_ucRes );
			UCHAR	ucRateX0	= c_ucRes - ucRateX1;
			UCHAR	ucRateY0	= c_ucRes - ucRateY1;
			UCHAR	ucSnr		= ucSnrX + ucSnrY * SNR_SZ_X;
			long	loWork		= 0;

			if( ucImgX == (ucWidth - 1) )
			{
				if( ucImgY == (ucHeight - 1) )
				{
					loWork += (long)pshInImg[ucSnr               ];
				}
				else
				{
					loWork += (long)pshInImg[ucSnr               ] * ucRateY0;
					loWork += (long)pshInImg[ucSnr     + SNR_SZ_X] * ucRateY1;
					loWork /= c_ucRes;
				}
			}
			else
			{
				if( ucImgY == (ucHeight - 1) )
				{
					loWork += (long)pshInImg[ucSnr               ] * ucRateX0;
					loWork += (long)pshInImg[ucSnr + 1           ] * ucRateX1;
					loWork /= c_ucRes;
				}
				else
				{
					loWork += (long)pshInImg[ucSnr               ] * ucRateX0 * ucRateY0;
					loWork += (long)pshInImg[ucSnr + 1           ] * ucRateX1 * ucRateY0;
					loWork += (long)pshInImg[ucSnr     + SNR_SZ_X] * ucRateX0 * ucRateY1;
					loWork += (long)pshInImg[ucSnr + 1 + SNR_SZ_X] * ucRateX1 * ucRateY1;
					loWork /= c_ucRes * c_ucRes;
				}
			}
			pshOutImg[usImg] = (SHORT)loWork;
		}

		bRet = TRUE;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Image dilation.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_IMG_ImageDilation1( UCHAR ucWidth, UCHAR ucHeight, UCHAR* pucInImg, UCHAR* pucOutImg )
{
	USHORT	usImg	= 0;
	USHORT	usSize	= ucWidth * ucHeight;
	BOOL	bRet	= FALSE;

	if( pucInImg != pucOutImg )
	{
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			pucOutImg[usImg] = 0;
		}
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			UCHAR	ucImgX = usImg % ucWidth;
			UCHAR	ucImgY = usImg / ucWidth;

			if( 0 != pucInImg[usImg] )
			{
				pucOutImg[usImg] = 1;
				if( 0 != ucImgX )
				{
					pucOutImg[usImg - 1] = 1;
				}
				if( (ucWidth - 1) != ucImgX )
				{
					pucOutImg[usImg + 1] = 1;
				}
				if( 0 != ucImgY )
				{
					pucOutImg[usImg - ucWidth] = 1;
				}
				if( (ucHeight - 1) != ucImgY )
				{
					pucOutImg[usImg + ucWidth] = 1;
				}
			}
		}

		bRet = TRUE;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Image dilation.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_IMG_ImageDilation2( UCHAR ucWidth, UCHAR ucHeight, UCHAR ucLabelNo, UCHAR* pucInImg, UCHAR* pucOutImg )
{
	USHORT	usImg	= 0;
	USHORT	usSize	= ucWidth * ucHeight;
	BOOL	bRet	= FALSE;

	if( pucInImg != pucOutImg )
	{
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			pucOutImg[usImg] = 0;
		}
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			UCHAR	ucImgX = usImg % ucWidth;
			UCHAR	ucImgY = usImg / ucWidth;

			if( ucLabelNo == pucInImg[usImg] )
			{
				pucOutImg[usImg] = 1;
				if( 0 != ucImgX )
				{
					pucOutImg[usImg - 1] = 1;
				}
				if( (ucWidth - 1) != ucImgX )
				{
					pucOutImg[usImg + 1] = 1;
				}
				if( 0 != ucImgY )
				{
					pucOutImg[usImg - ucWidth] = 1;
				}
				if( (ucHeight - 1) != ucImgY )
				{
					pucOutImg[usImg + ucWidth] = 1;
				}
			}
		}

		bRet = TRUE;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Object detection.
------------------------------------------------------------------------------*/
void vAMG_PUB_ODT_CalcDiffImage( USHORT usSize, SHORT* pshInImg1, SHORT* pshInImg2, SHORT* pshOutImg )
{
	USHORT usImg = 0;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		pshOutImg[usImg] = pshInImg1[usImg] - pshInImg2[usImg];
	}
}

/*------------------------------------------------------------------------------
	Object detection.
------------------------------------------------------------------------------*/
void vAMG_PUB_ODT_CalcDetectImage1( USHORT usSize, SHORT* pshInImg, SHORT shTh, UCHAR ucMark, UCHAR* pucOutImg )
{
	USHORT usImg = 0;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		pucOutImg[usImg] = ( shTh <= pshInImg[usImg] ) ? ucMark : 0;
	}
}

/*------------------------------------------------------------------------------
	Object detection.
------------------------------------------------------------------------------*/
void vAMG_PUB_ODT_CalcDetectImage2( USHORT usSize, SHORT* pshInImg, SHORT* pshTh, UCHAR ucMark, UCHAR* pucOutImg )
{
	USHORT usImg = 0;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		pucOutImg[usImg] = ( pshTh[usImg] <= pshInImg[usImg] ) ? ucMark : 0;
	}
}

/*------------------------------------------------------------------------------
	Labeling.
------------------------------------------------------------------------------*/
UCHAR ucAMG_PUB_ODT_CalcDataLabeling8( UCHAR ucWidth, UCHAR ucHeight, UCHAR ucMark, USHORT usArea, UCHAR* pucImg, USHORT* pusSearchList )
{
	USHORT	usImg	= 0;
	USHORT	usSize	= ucWidth * ucHeight;
	UCHAR	ucDetectNum = 0;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		UCHAR	ucLabelNum	= 0;
		USHORT	usIndex		= 0;
		USHORT	usIndexAdd	= 0;

		if( ucMark == pucImg[usImg] )
		{
			pucImg[usImg] = 0;
			pusSearchList[usIndex] = usImg;
			usIndexAdd = 1;
		}

		while( usIndex < usIndexAdd )
		{
			UCHAR	ucX = pusSearchList[usIndex] % ucWidth;
			UCHAR	ucY = pusSearchList[usIndex] / ucWidth;
			{
				if( 0 <= (ucY - 1) )
				{
					USHORT usCheckIndex = (ucX  ) + (ucY-1) * ucWidth;
					if( ucMark == pucImg[usCheckIndex] )
					{
						pucImg[usCheckIndex] = 0;
						pusSearchList[usIndexAdd++] = usCheckIndex;
					}
				}
				if( ucHeight > (ucY + 1) )
				{
					USHORT usCheckIndex = (ucX  ) + (ucY+1) * ucWidth;
					if( ucMark == pucImg[usCheckIndex] )
					{
						pucImg[usCheckIndex] = 0;
						pusSearchList[usIndexAdd++] = usCheckIndex;
					}
				}
			}
			if( 0 <= (ucX - 1) )
			{
				{
					USHORT usCheckIndex = (ucX-1) + (ucY  ) * ucWidth;
					if( ucMark == pucImg[usCheckIndex] )
					{
						pucImg[usCheckIndex] = 0;
						pusSearchList[usIndexAdd++] = usCheckIndex;
					}
				}
				if( 0 <= (ucY - 1) )
				{
					USHORT usCheckIndex = (ucX-1) + (ucY-1) * ucWidth;
					if( ucMark == pucImg[usCheckIndex] )
					{
						pucImg[usCheckIndex] = 0;
						pusSearchList[usIndexAdd++] = usCheckIndex;
					}
				}
				if( ucHeight > (ucY + 1) )
				{
					USHORT usCheckIndex = (ucX-1) + (ucY+1) * ucWidth;
					if( ucMark == pucImg[usCheckIndex] )
					{
						pucImg[usCheckIndex] = 0;
						pusSearchList[usIndexAdd++] = usCheckIndex;
					}
				}
			}
			if( ucWidth > (ucX + 1) )
			{
				{
					USHORT usCheckIndex = (ucX+1) + (ucY  ) * ucWidth;
					if( ucMark == pucImg[usCheckIndex] )
					{
						pucImg[usCheckIndex] = 0;
						pusSearchList[usIndexAdd++] = usCheckIndex;
					}
				}
				if( 0 <= (ucY - 1) )
				{
					USHORT usCheckIndex = (ucX+1) + (ucY-1) * ucWidth;
					if( ucMark == pucImg[usCheckIndex] )
					{
						pucImg[usCheckIndex] = 0;
						pusSearchList[usIndexAdd++] = usCheckIndex;
					}
				}
				if( ucHeight > (ucY + 1) )
				{
					USHORT usCheckIndex = (ucX+1) + (ucY+1) * ucWidth;
					if( ucMark == pucImg[usCheckIndex] )
					{
						pucImg[usCheckIndex] = 0;
						pusSearchList[usIndexAdd++] = usCheckIndex;
					}
				}
			}
			usIndex++;
		}

		if( usIndex <= usArea )
		{
			ucLabelNum = 0;
		}
		else
		{
			ucDetectNum++;
			ucLabelNum = ucDetectNum;
		}

		{
			USHORT usImg2 = 0;
			for( usImg2 = 0; usImg2 < usIndex; usImg2++ )
			{
				pucImg[ pusSearchList[usImg2] ] = ucLabelNum;
			}
		}
	}

	return( ucDetectNum );
}

/*------------------------------------------------------------------------------
	Calculate features.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_FEA_CalcArea( USHORT usSize, UCHAR ucLabelNo, UCHAR* pucImg, USHORT* pusRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	USHORT	usArea	= 0;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		if( ucLabelNo == pucImg[usImg] )
		{
			bRet = TRUE;
			usArea++;
		}
	}

	if( FALSE != bRet )
	{
		*pusRet = usArea;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Calculate features.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_FEA_CalcRectangle( UCHAR ucWidth, UCHAR ucHeight, UCHAR ucLabelNo, UCHAR* pucImg, UCHAR* pucRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	USHORT	usSize	= ucWidth * ucHeight;
	UCHAR	ucMinX	= ucWidth  - 1;
	UCHAR	ucMinY	= ucHeight - 1;
	UCHAR	ucMaxX	= 0;
	UCHAR	ucMaxY	= 0;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		UCHAR	ucImgX = usImg % ucWidth;
		UCHAR	ucImgY = usImg / ucWidth;

		if( ucLabelNo == pucImg[usImg] )
		{
			bRet = TRUE;
			if( ucMinX > ucImgX )
			{
				ucMinX = ucImgX;
			}
			if( ucMinY > ucImgY )
			{
				ucMinY = ucImgY;
			}
			if( ucMaxX < ucImgX )
			{
				ucMaxX = ucImgX;
			}
			if( ucMaxY < ucImgY )
			{
				ucMaxY = ucImgY;
			}
		}
	}

	if( FALSE != bRet )
	{
		pucRet[0] = ucMinX;
		pucRet[1] = ucMinY;
		pucRet[2] = ucMaxX;
		pucRet[3] = ucMaxY;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Calculate features.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_FEA_CalcMinTemp( USHORT usSize, UCHAR ucLabelNo, UCHAR* pucImg, SHORT* pshImg, SHORT* pshRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	SHORT	shMin	= SHORT_MAX_VAL;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		if( ucLabelNo == pucImg[usImg] )
		{
			bRet = TRUE;
			if( shMin > pshImg[usImg] )
			{
				shMin = pshImg[usImg];
			}
		}
	}

	if( FALSE != bRet )
	{
		*pshRet = shMin;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Calculate features.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_FEA_CalcMaxTemp( USHORT usSize, UCHAR ucLabelNo, UCHAR* pucImg, SHORT* pshImg, SHORT* pshRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	SHORT	shMax	= SHORT_MIN_VAL;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		if( ucLabelNo == pucImg[usImg] )
		{
			bRet = TRUE;
			if( shMax < pshImg[usImg] )
			{
				shMax = pshImg[usImg];
			}
		}
	}

	if( FALSE != bRet )
	{
		*pshRet = shMax;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Calculate features.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_FEA_CalcAveTemp( USHORT usSize, UCHAR ucLabelNo, UCHAR* pucImg, SHORT* pshImg, SHORT* pshRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	long	loSum	= 0;
	USHORT	usCnt	= 0;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		if( ucLabelNo == pucImg[usImg] )
		{
			bRet = TRUE;
			loSum += pshImg[usImg];
			usCnt++;
		}
	}

	if( FALSE != bRet )
	{
		*pshRet = (SHORT)( loSum / usCnt );
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Calculate features.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_FEA_CalcStdDevTemp( USHORT usSize, UCHAR ucLabelNo, UCHAR* pucImg, SHORT* pshImg, USHORT* pusRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	ULONG	ulS		= 0;
	USHORT	usCnt	= 0;
	SHORT	shAve	= 0;

	if( FALSE != bAMG_PUB_FEA_CalcAveTemp( usSize, ucLabelNo, pucImg, pshImg, &shAve ) )
	{
		bRet = TRUE;
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			if( ucLabelNo == pucImg[usImg] )
			{
				ULONG ulPow = (ULONG)( (long)(pshImg[usImg] - shAve) * (pshImg[usImg] - shAve) );
				if( (ULONG_MAX_VAL - ulS) < ulPow )
				{
					bRet = FALSE;
				}
				else
				{
					ulS += ulPow;
					usCnt++;
				}
			}
		}
	}

	if( FALSE != bRet )
	{
		ULONG ulData = ulS / usCnt;
		ULONG ulWork = ( 1 < ulData ) ? ulData : 1;
		ULONG ulSqrt = 0;
		if( 0 != ulData )
		{
			do
			{
				ulSqrt = ulWork;
				ulWork = (ulData / ulWork + ulWork) >> 1;
			}
			while( ulWork < ulSqrt );
		}
		*pusRet = (USHORT)ulSqrt;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Calculate features.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_FEA_CalcStdDevTemp_f( USHORT usSize, UCHAR ucLabelNo, UCHAR* pucImg, SHORT* pshImg, float* pfRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	float	fS		= 0;
	USHORT	usCnt	= 0;
	SHORT	shAve	= 0;

	if( FALSE != bAMG_PUB_FEA_CalcAveTemp( usSize, ucLabelNo, pucImg, pshImg, &shAve ) )
	{
		bRet = TRUE;
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			if( ucLabelNo == pucImg[usImg] )
			{
				float fDiff = fAMG_PUB_CMN_ConvStoF(pshImg[usImg] - shAve);
				fS += fDiff * fDiff;
				usCnt++;
			}
		}
	}

	if( FALSE != bRet )
	{
		float fData = fS / usCnt;
		float fWork = ( 1 < fData ) ? fData : 1;
		float fSqrt = 0;
		if( 0 < fData )
		{
			do
			{
				fSqrt = fWork;
				fWork = (fData / fWork + fWork) * 0.5f;
			}
			while( fWork < fSqrt );
		}
		*pfRet = fSqrt;
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Calculate features.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_FEA_CalcCenterTemp( UCHAR ucWidth, UCHAR ucHeight, UCHAR ucLabelNo, UCHAR* pucImg, SHORT* pshImg, SHORT* pshRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	USHORT	usSize	= ucWidth * ucHeight;
	SHORT	shMin	= SHORT_MAX_VAL;
	ULONG	ulGx	= 0;
	ULONG	ulGy	= 0;
	ULONG	ulGw	= 0;

	if( FALSE != bAMG_PUB_FEA_CalcMinTemp( usSize, ucLabelNo, pucImg, pshImg, &shMin ) )
	{
		shMin -= 1;
		bRet = TRUE;
	}
	if( FALSE != bRet )
	{
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			if( ucLabelNo == pucImg[usImg] )
			{
				UCHAR	ucImgX = usImg % ucWidth;
				UCHAR	ucImgY = usImg / ucWidth;
				ULONG	ulWeight = (ULONG)( pshImg[usImg] - shMin );
				ulGx += ulWeight * (ucImgX + 1);
				ulGy += ulWeight * (ucImgY + 1);
				ulGw += ulWeight;
			}
		}

		if( 0 < ulGw )
		{
			pshRet[0] = ulGx * 256 / ulGw - 256;
			pshRet[1] = ulGy * 256 / ulGw - 256;
		}
		else
		{
			pshRet[0] = 0;
			pshRet[1] = 0;
		}
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Calculate features.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_FEA_CalcCenterTemp_f( UCHAR ucWidth, UCHAR ucHeight, UCHAR ucLabelNo, UCHAR* pucImg, SHORT* pshImg, float* pfRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	USHORT	usSize	= ucWidth * ucHeight;
	SHORT	shMin	= SHORT_MAX_VAL;
	SHORT	shMax	= SHORT_MIN_VAL;
	SHORT	shDiff	= 0;
	float	fGx		= 0;
	float	fGy		= 0;
	float	fGw		= 0;

	if( FALSE != bAMG_PUB_FEA_CalcMinTemp( usSize, ucLabelNo, pucImg, pshImg, &shMin ) )
	{
		shMin -= 1;
		if( FALSE != bAMG_PUB_FEA_CalcMaxTemp( usSize, ucLabelNo, pucImg, pshImg, &shMax ) )
		{
			bRet = TRUE;
		}
	}
	if( FALSE != bRet )
	{
		shDiff = shMax - shMin;
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			if( ucLabelNo == pucImg[usImg] )
			{
				UCHAR	ucImgX = usImg % ucWidth;
				UCHAR	ucImgY = usImg / ucWidth;
				float	fWeight = (float)( pshImg[usImg] - shMin ) / shDiff;
				fGx += fWeight * (ucImgX + 1);
				fGy += fWeight * (ucImgY + 1);
				fGw += fWeight;
			}
		}

		if( 0 < fGw )
		{
			pfRet[0] = fGx / fGw - 1;
			pfRet[1] = fGy / fGw - 1;
		}
		else
		{
			pfRet[0] = 0;
			pfRet[1] = 0;
		}
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Update background temperature.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_BGT_UpdateBackTemp( USHORT usSize, UCHAR* pucImg, SHORT* pshDiffImg, SHORT shTh, SHORT* pshBackImg )
{
	const SHORT c_shMinTh = 0;
	const SHORT c_shMaxTh = 256;
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	SHORT	shAve	= 0;

	/* Adjust parameter. */
	if( c_shMinTh > shTh )
	{
		shTh = c_shMinTh;
	}
	if( c_shMaxTh < shTh )
	{
		shTh = c_shMaxTh;
	}

	if( FALSE != bAMG_PUB_FEA_CalcAveTemp( usSize, 0, pucImg, pshDiffImg, &shAve ) )
	{
		bRet = TRUE;
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			SHORT shTemp = 0;
			if( 0 == pucImg[usImg] )
			{
				shTemp = (SHORT)( (long)shTh * pshDiffImg[usImg] / c_shMaxTh );
			}
			else
			{
				shTemp = (SHORT)( (long)shTh * shAve             / c_shMaxTh );
			}
			pshBackImg[usImg] += shTemp;
		}
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Judge human.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_HDT_JudgeHuman( USHORT usSize, USHORT usTh )
{
	return( ( usTh < usSize ) ? TRUE : FALSE );
}

/*------------------------------------------------------------------------------
	Output.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_OUT_CalcOutImage( UCHAR ucImgWidth, UCHAR ucImgHeight, UCHAR ucOutWidth, UCHAR ucOutHeight, SHORT* pshCenter, UCHAR* pucCenter )
{
	BOOL	bRet	= FALSE;

	if( (0 <= pshCenter[0]) && ((ucImgWidth-1) >= (pshCenter[0]/256)) )
	{
		if( (0 <= pshCenter[1]) && ((ucImgHeight-1) >= (pshCenter[1]/256)) )
		{
			bRet = TRUE;
			pucCenter[0] = (UCHAR)( (long)pshCenter[0] * ucOutWidth  / ((long)ucImgWidth  * 256) );
			pucCenter[1] = (UCHAR)( (long)pshCenter[1] * ucOutHeight / ((long)ucImgHeight * 256) );
		}
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Output.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_OUT_CalcOutImage_f( UCHAR ucImgWidth, UCHAR ucImgHeight, UCHAR ucOutWidth, UCHAR ucOutHeight, float* pfCenter, UCHAR* pucCenter )
{
	BOOL	bRet	= FALSE;

	if( (0 <= pfCenter[0]) && ((ucImgWidth-1) >= pfCenter[0]) )
	{
		if( (0 <= pfCenter[1]) && ((ucImgHeight-1) >= pfCenter[1]) )
		{
			bRet = TRUE;
			pucCenter[0] = (UCHAR)( pfCenter[0] * ucOutWidth  / (float)ucImgWidth  );
			pucCenter[1] = (UCHAR)( pfCenter[1] * ucOutHeight / (float)ucImgHeight );
		}
	}

	return( bRet );
}
