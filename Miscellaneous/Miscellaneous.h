/*
 * MIT License
 *
 * Copyright (c) 2020 <Duy Lion Tran>. All rights reserved.
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

#ifndef __MISCELLANEOUS_H__
#define __MISCELLANEOUS_H__

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


#define SIZE(value)                     (sizeof(value) / sizeof(value[0]))

#define ABSOLUTE(value)                 ((value) >= 0.0)? (value) : ((-1.0) * (value))

#ifndef MAX
#define MAX(value1, value2)             ((value1) >= (value2))? (value1) : (value2)
#endif

#ifndef MIN
#define MIN(value1, value2)             ((value1) < (value2))? (value1) : (value2)
#endif

#define CLEAR_BUFFER(buffer)		(memset(buffer, 0, SIZE(buffer)))

#define MERGE_2BYTES(buffer, value)     ((value) = (((buffer[0]) << 8) & 0xFFFF));  \
                                        ((value) |= ((buffer[1]) & 0xFFFF))

#define MERGE_4BYTES(buffer, value)     ((value) = (((buffer[0]) << 24) & 0xFFFFFFFF));   \
                                        ((value) |= (((buffer[1]) << 16) & 0xFFFFFFFF));  \
                                        ((value) |= (((buffer[2]) << 8) & 0xFFFFFFFF));   \
                                        ((value) |= ((buffer[3]) & 0xFFFFFFFF))

#define PARSE_2BYTES(value, high_byte, low_byte)    ((high_byte) = (((value) >> 8) & 0xFF));  \
                                                    ((low_byte) = ((value) & 0xFF))


//#define ERROR_CHECK(_function_)         while (HAL_OK != _function_) { }

int8_t hex2int(char *prt, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* __MISCELLANEOUS_H__ */
