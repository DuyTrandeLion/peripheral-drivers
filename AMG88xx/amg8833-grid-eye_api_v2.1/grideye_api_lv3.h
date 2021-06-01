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
#ifndef	__GRIDEYE_API_LV3_H
#define	__GRIDEYE_API_LV3_H

/*******************************************************************************
	include file
*******************************************************************************/
#include	"AMG88xx_conf.h"

/*******************************************************************************
	public method definition
*******************************************************************************/
BOOL	bAMG_PUB_ODT_Initialize( UCHAR, UCHAR, ULONG, USHORT );
BOOL	bAMG_PUB_ODT_SetPrm( UCHAR, UCHAR );
BOOL	bAMG_PUB_ODT_Execute( short*, short*, short*, UCHAR*, UCHAR*, USHORT* );

BOOL	bAMG_PUB_OTR_Initialize( UCHAR, UCHAR, UCHAR, UCHAR, UCHAR, ULONG, USHORT );
BOOL	bAMG_PUB_OTR_SetPrm( UCHAR, short );
BOOL	bAMG_PUB_OTR_Execute   ( short*, short*, short*, UCHAR, UCHAR*, USHORT* );
BOOL	bAMG_PUB_OTR_GetResultByLabel(UCHAR, UCHAR, short* );
BOOL	bAMG_PUB_OTR_GetOutput( UCHAR*, UCHAR* );
BOOL	bAMG_PUB_BGT_SetPrm(UCHAR, short );
void	vAMG_PUB_BGT_UpdateBackTemp( USHORT, UCHAR*, short*, short* );

#endif	/* __GRIDEYE_API_LV3_H */

/*******************************************************************************
 * Copyright(C) by 2015 Panasonic Corporation.
 ******************************************************************************/
