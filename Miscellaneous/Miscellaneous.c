/*
 * MIT License
 *
 * Copyright (c) 2022 <Duy Lion Tran>. All rights reserved.
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
 * @file Miscellaneous.h
 * @date Feb 14th 2020
 * @version  1.0.0
 *
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "Miscellaneous.h"

#ifdef __cplusplus
extern "C" {
#endif


int8_t hex2int(char *prt, uint8_t *value)
{
    if ((*prt >= '0') && (*prt <= '9'))
    {
        *value = *prt - '0';

        return 0;
    }
    else if ((*prt >= 'A') && (*prt <= 'F'))
    {
        *value = *prt - 'A' + 10;

        return 0;
    }
    else if ((*prt >= 'a') && (*prt <= 'f'))
    {
        *value = *prt - 'a' + 10;

        return 0;
    }

    return -1;
}


void degree2radian(float par_deg, float *par_rad)
{
    *par_rad = par_deg * 3.14 / 180.0;
}


void normalize2_0_360(uint16_t par_deg, uint16_t *par_angle)
{
    if (par_deg <= 360)
    {
        *par_angle = par_deg;
    }
    else
    {
        *par_angle = par_deg % 360;
        *par_angle = ((*par_angle != 0)? *par_angle : 360);
    }
}


#ifdef __cplusplus
}
#endif
