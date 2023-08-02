/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2023 Eiren Rain & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "customConversions.h"

//Convert 2 bytes in f16 format to a float
//f16: SEEE EEDD DDDD DDDD
//f32: SEEE EEEE EDDD DDDD DDDD DDDD DDDD DDDD
float Conversions::convertBytesToFloat(uint8_t dataLow, uint8_t dataHigh) {
    uint16_t combinedData = ((uint16_t)dataHigh << 8 | dataLow);
    uint32_t dataHolder = ((uint32_t)(combinedData & DATA_MASK_F16) << 13);
    dataHolder |= ((uint32_t)(combinedData & SIGN_BIT_F16) << 16);
    uint8_t exponent = ((combinedData & EXPONENT_MASK_F16) >> 10);
    if (!exponent) //subnormal
        return 0.0F;
    if (exponent == 0b00011111) //nan
        exponent == 0xff;
    else
        exponent += 127 - 15;
    dataHolder |= (((uint32_t)exponent << 23));
    float dataFloat = 0.0F;
    memcpy(&dataFloat, &dataHolder, 4);
    // float dataFloat = reinterpret_cast<float &>(dataHolder);
    return dataFloat;
}