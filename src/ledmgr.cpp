/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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
#include "ledmgr.h"

namespace LEDMGR
{
    /*!
    *  @brief  Turn a LED on
    *  @param  pin
    *          LED pin
    */
    void On(uint8_t pin)
    {
        #if ENABLE_LEDS
            digitalWrite(pin, LOW);
        #endif
    }

    /*!
    *  @brief  Turn a LED off
    *  @param  pin
    *          LED pin
    */
    void Off(uint8_t pin)
    {
        #if ENABLE_LEDS
            digitalWrite(pin, HIGH);
        #endif
    }

    /*!
    *  @brief  Blink a LED for [time]ms
    *  @param  pin
    *          LED pin
    *  @param  time
    *          Amount of ms to turn the LED on
    *  @param  direction
    *          Direction turn the LED on, usually LOW
    */
    void Blink(uint8_t pin, unsigned long time, uint8_t direction)
    {
        #if ENABLE_LEDS
            digitalWrite(pin, direction);
            delay(time);
            digitalWrite(pin, direction ^ 1);
        #endif
    }

    /*!
    *  @brief  Show a pattern on a LED
    *  @param  pin
    *          LED pin
    *  @param  timeon
    *          Amount of ms to turn the LED on
    *  @param  timeoff
    *          Amount of ms to turn the LED off
    *  @param  times
    *          Amount of times to display the pattern
    *  @param  direction
    *          Direction turn the LED on, usually LOW
    */
    void Pattern(uint8_t pin, unsigned long timeon, unsigned long timeoff, int times, uint8_t direction)
    {
        #if ENABLE_LEDS
            for (int i = 0; i < times; i++)
            {
                digitalWrite(pin, direction);
                delay(timeon);
                digitalWrite(pin, direction ^ 1);
                delay(timeoff);
            }
        #endif
    }
}