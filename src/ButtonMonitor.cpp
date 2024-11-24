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

#include "ButtonMonitor.h"
#include "GlobalVars.h"
#include "status/Status.h"

#define SHUTDOWN_COUNT 60
#define ENABLE_BUTTON true

namespace SlimeVR
{
    void ButtonMonitor::setup()
    {
#if ENABLE_BUTTON
        pinMode(m_Pin, INPUT);
#endif

        // Do the initial pull of the state
        update();
    }

    bool ButtonMonitor::isPressed()
    {
#if ENABLE_BUTTON
        return digitalRead(m_Pin) == HIGH;
#else
        return false;
#endif
    }

    void ButtonMonitor::update()
    {
        unsigned long time = millis();
        unsigned long diff = time - m_LastUpdate;

        // Don't tick the ButtonMonitor *too* often
        if (diff < 10)
        {
            return;
        }

        m_LastUpdate = time;

        unsigned int length = 0;
        unsigned int count = 0;

        if (!statusManager.hasStatus(Status::SHUTDOWN_INITIATED)) // ignore button presses after shutdown initiated
        {
            m_Timer = 0;
            // Advance stage
            switch (m_CurrentState)
            {
            case RELEASED:
                if (isPressed())
                {
                    m_CurrentState = JUSTPRESSED;
                    m_CurrentCount = 0;
                }
                break;
            case JUSTPRESSED:
                m_Logger.trace("Button Pressed");
                m_CurrentState = PRESSED;
            case PRESSED:
                if (isPressed())
                {
                    m_CurrentCount++;
                    // m_Logger.debug("Pressed count: %d", m_CurrentCount);
                    if (m_CurrentCount >= SHUTDOWN_COUNT)
                    {
                        statusManager.setStatus(Status::SHUTDOWN_INITIATED, true);
                        m_Logger.trace("Entering Shutdown");
                    }
                }
                else
                {
                    m_CurrentState = JUSTRELEASED;
                }
                break;
            case JUSTRELEASED:
                m_Logger.trace("Button Released");
                m_CurrentState = RELEASED;
                break;
            }
        }
    }
}
