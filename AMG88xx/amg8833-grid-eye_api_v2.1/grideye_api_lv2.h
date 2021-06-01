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

#ifndef	__GRIDEYE_API_LV2_H
#define	__GRIDEYE_API_LV2_H


/*******************************************************************************
	include file
*******************************************************************************/
#include	"AMG88xx_conf.h"


/*******************************************************************************
	public method definition
*******************************************************************************/
SHORT	shAMG_PUB_CMN_CalcAve( SHORT*, USHORT, UCHAR, UCHAR, BOOL* );
SHORT	shAMG_PUB_CMN_CalcIIR( SHORT, SHORT, SHORT );
SHORT	shAMG_PUB_CMN_CalcIIR_f( SHORT, SHORT, float );

void	vAMG_PUB_IMG_ConvertFlipX    ( UCHAR, UCHAR, SHORT*, SHORT* );
void	vAMG_PUB_IMG_ConvertFlipY    ( UCHAR, UCHAR, SHORT*, SHORT* );
BOOL	bAMG_PUB_IMG_ConvertFlipXY   ( UCHAR, UCHAR, SHORT*, SHORT* );
BOOL	bAMG_PUB_IMG_ConvertRotate90 ( UCHAR, UCHAR, SHORT*, SHORT* );
void	vAMG_PUB_IMG_ConvertRotate180( UCHAR, UCHAR, SHORT*, SHORT* );
BOOL	bAMG_PUB_IMG_ConvertRotate270( UCHAR, UCHAR, SHORT*, SHORT* );

BOOL	bAMG_PUB_IMG_LinearInterpolationSQ15( SHORT*, SHORT* );
BOOL	bAMG_PUB_IMG_LinearInterpolation( UCHAR, UCHAR, SHORT*, SHORT* );

BOOL	bAMG_PUB_IMG_ImageDilation1( UCHAR, UCHAR, UCHAR*, UCHAR* );
BOOL	bAMG_PUB_IMG_ImageDilation2( UCHAR, UCHAR, UCHAR, UCHAR*, UCHAR* );

void	vAMG_PUB_ODT_CalcDiffImage   ( USHORT, SHORT*, SHORT*, SHORT* );
void	vAMG_PUB_ODT_CalcDetectImage1( USHORT, SHORT*, SHORT,  UCHAR, UCHAR* );
void	vAMG_PUB_ODT_CalcDetectImage2( USHORT, SHORT*, SHORT*, UCHAR, UCHAR* );

UCHAR	ucAMG_PUB_ODT_CalcDataLabeling8( UCHAR, UCHAR, UCHAR, USHORT, UCHAR*, USHORT* );

BOOL	bAMG_PUB_FEA_CalcArea        ( USHORT,       UCHAR, UCHAR*,         USHORT* );
BOOL	bAMG_PUB_FEA_CalcRectangle   ( UCHAR, UCHAR, UCHAR, UCHAR*,         UCHAR*  );
BOOL	bAMG_PUB_FEA_CalcMinTemp     ( USHORT,       UCHAR, UCHAR*, SHORT*, SHORT*  );
BOOL	bAMG_PUB_FEA_CalcMaxTemp     ( USHORT,       UCHAR, UCHAR*, SHORT*, SHORT*  );
BOOL	bAMG_PUB_FEA_CalcAveTemp     ( USHORT,       UCHAR, UCHAR*, SHORT*, SHORT*  );
BOOL	bAMG_PUB_FEA_CalcStdDevTemp  ( USHORT,       UCHAR, UCHAR*, SHORT*, USHORT* );
BOOL	bAMG_PUB_FEA_CalcStdDevTemp_f( USHORT,       UCHAR, UCHAR*, SHORT*, float*  );
BOOL	bAMG_PUB_FEA_CalcCenterTemp  ( UCHAR, UCHAR, UCHAR, UCHAR*, SHORT*, SHORT*  );
BOOL	bAMG_PUB_FEA_CalcCenterTemp_f( UCHAR, UCHAR, UCHAR, UCHAR*, SHORT*, float*  );

BOOL	bAMG_PUB_BGT_UpdateBackTemp( USHORT, UCHAR*, SHORT*, SHORT, SHORT* );

BOOL	bAMG_PUB_HDT_JudgeHuman( USHORT, USHORT );

BOOL	bAMG_PUB_OUT_CalcOutImage  ( UCHAR, UCHAR, UCHAR, UCHAR, SHORT*, UCHAR* );
BOOL	bAMG_PUB_OUT_CalcOutImage_f( UCHAR, UCHAR, UCHAR, UCHAR, float*, UCHAR* );


#endif	/* __GRIDEYE_API_LV2_H */
