/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

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
#ifndef SLIMEVR_GLOBALS_H_
#define SLIMEVR_GLOBALS_H_

#include <Arduino.h>
#include "consts.h"
#include "debug.h"
#include "defines.h"

#ifndef SECOND_IMU
#define SECOND_IMU IMU
#endif

#ifndef SECOND_IMU_ROTATION
#define SECOND_IMU_ROTATION IMU_ROTATION
#endif

#ifndef BATTERY_MONITOR
#define BATTERY_MONITOR BAT_INTERNAL
#endif

// If LED_PIN is not defined in "defines.h" take the default pin from "pins_arduino.h" framework.
// If there is no pin defined for the board, use LED_PIN 255 and disable LED
#if defined(LED_PIN)
    // LED_PIN is defined
    #if (LED_PIN < 0) || (LED_PIN >= LED_OFF)
        #define ENABLE_LEDS false    
    #else
        #define ENABLE_LEDS true
    #endif
#else
    // LED_PIN is not defined
    #if defined(LED_BUILTIN) && (LED_BUILTIN < LED_OFF) && (LED_BUILTIN >= 0)
        #define LED_PIN LED_BUILTIN
        #define ENABLE_LEDS true
    #else
        #define LED_PIN LED_OFF
        #define ENABLE_LEDS false
    #endif
#endif

#if !defined(LED_INVERTED)
    // default is inverted for SlimeVR / ESP-12E
    #define LED_INVERTED true
#endif

#if LED_INVERTED
#define LED__ON LOW
#define LED__OFF HIGH
#else
#define LED__ON HIGH
#define LED__OFF LOW
#endif

#endif // SLIMEVR_GLOBALS_H_
