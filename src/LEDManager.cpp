/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

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

#include "LEDManager.h"
#include "GlobalVars.h"
#include "status/Status.h"

namespace SlimeVR
{
    void LEDManager::setup()
    {
#if ENABLE_LEDS
        pinMode(m_Pin, OUTPUT);
#endif

        // Do the initial pull of the state
        update();
    }

    void LEDManager::on()
    {
#if ENABLE_LEDS
        digitalWrite(m_Pin, LED__ON);
#endif
    }

    void LEDManager::off()
    {
#if ENABLE_LEDS
        digitalWrite(m_Pin, LED__OFF);
#endif
    }

    void LEDManager::blink(unsigned long time)
    {
        on();
        delay(time);
        off();
    }

    void LEDManager::pattern(unsigned long timeon, unsigned long timeoff, int times)
    {
        for (int i = 0; i < times; i++)
        {
            blink(timeon);
            delay(timeoff);
        }
    }

    void LEDManager::update()
    {
        unsigned long time = millis();
        unsigned long diff = time - m_LastUpdate;

        // Don't tick the LEDManager *too* often
        if (diff < 10)
        {
            return;
        }

        m_LastUpdate = time;

        unsigned int length = 0;
        unsigned int count = 0;

        if (statusManager.hasStatus(Status::LOW_BATTERY))
        {
            count = LOW_BATTERY_COUNT;
            switch (m_CurrentStage)
            {
            case ON:
            case OFF:
                length = LOW_BATTERY_LENGTH;
                break;
            case GAP:
                length = DEFAULT_GAP;
                break;
            case INTERVAL:
                length = LOW_BATTERY_INTERVAL;
                break;
            }
        }
        else if (statusManager.hasStatus(Status::IMU_ERROR))
        {
            count = IMU_ERROR_COUNT;
            switch (m_CurrentStage)
            {
            case ON:
            case OFF:
                length = IMU_ERROR_LENGTH;
                break;
            case GAP:
                length = DEFAULT_GAP;
                break;
            case INTERVAL:
                length = IMU_ERROR_INTERVAL;
                break;
            }
        }
        else if (statusManager.hasStatus(Status::WIFI_CONNECTING))
        {
            count = WIFI_CONNECTING_COUNT;
            switch (m_CurrentStage)
            {
            case ON:
            case OFF:
                length = WIFI_CONNECTING_LENGTH;
                break;
            case GAP:
                length = DEFAULT_GAP;
                break;
            case INTERVAL:
                length = WIFI_CONNECTING_INTERVAL;
                break;
            }
        }
        else if (statusManager.hasStatus(Status::SERVER_CONNECTING))
        {
            count = SERVER_CONNECTING_COUNT;
            switch (m_CurrentStage)
            {
            case ON:
            case OFF:
                length = SERVER_CONNECTING_LENGTH;
                break;
            case GAP:
                length = DEFAULT_GAP;
                break;
            case INTERVAL:
                length = SERVER_CONNECTING_INTERVAL;
                break;
            }
        }
        else
        {
#if defined(LED_INTERVAL_STANDBY) && LED_INTERVAL_STANDBY > 0
            count = 1;
            switch (m_CurrentStage)
            {
            case ON:
            case OFF:
                length = STANDBUY_LENGTH;
                break;
            case GAP:
                length = DEFAULT_GAP;
                break;
            case INTERVAL:
                length = LED_INTERVAL_STANDBY;
                break;
            }
#else
            return;
#endif
        }

        if (m_CurrentStage == OFF || m_Timer + diff >= length)
        {
            m_Timer = 0;
            // Advance stage
            switch (m_CurrentStage)
            {
            case OFF:
                on();
                m_CurrentStage = ON;
                m_CurrentCount = 0;
                break;
            case ON:
                off();
                m_CurrentCount++;
                if (m_CurrentCount >= count)
                {
                    m_CurrentCount = 0;
                    m_CurrentStage = INTERVAL;
                }
                else
                {
                    m_CurrentStage = GAP;
                }
                break;
            case GAP:
            case INTERVAL:
                on();
                m_CurrentStage = ON;
                break;
                on();
                m_CurrentStage = ON;
                break;
            }
        }
        else
        {
            m_Timer += diff;
        }
    }
}
