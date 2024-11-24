/*
    SomaticVR Code is placed under the MIT license
    Copyright (c) 2023 Somatic VR, LLC

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

#pragma once

#include <Arduino.h>
#include "globals.h"
#include "logging/Logger.h"

namespace SlimeVR
{
    enum ButtonState
    {
        RELEASED,
        JUSTPRESSED,
        PRESSED,
        JUSTRELEASED
    };

    class ButtonMonitor
    {
    public:
        ButtonMonitor(uint8_t pin) : m_Pin(pin) {}

        void setup();

        /*!
         *  @brief Returns the state of the button on pin
         */
        bool isPressed();


        void update();

    private:
        uint8_t m_CurrentCount = 0;
        unsigned long m_Timer = 0;
        ButtonState m_CurrentState = RELEASED;
        unsigned long m_LastUpdate = millis();

        uint8_t m_Pin;

        Logging::Logger m_Logger = Logging::Logger("ButtonMonitor");
    };
}

