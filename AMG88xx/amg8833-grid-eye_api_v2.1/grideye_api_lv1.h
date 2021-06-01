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

#ifndef	__GRIDEYE_API_LV1_H
#define	__GRIDEYE_API_LV1_H


/*******************************************************************************
	include file
*******************************************************************************/
#include	"AMG88xx_conf.h"


/*******************************************************************************
	public method definition
*******************************************************************************/
BOOL	 bAMG_PUB_I2C_Read( UCHAR, UCHAR, UCHAR, UCHAR* );
SHORT	shAMG_PUB_TMP_ConvThermistor( UCHAR[2] );
SHORT	shAMG_PUB_TMP_ConvTemperature( UCHAR[2] );
void	 vAMG_PUB_TMP_ConvTemperature64( UCHAR*, SHORT* );
SHORT	shAMG_PUB_CMN_ConvFtoS( float );
float	 fAMG_PUB_CMN_ConvStoF( SHORT );


#endif	/* __GRIDEYE_API_LV1_H */
