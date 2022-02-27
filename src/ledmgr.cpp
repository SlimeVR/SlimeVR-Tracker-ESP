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

namespace LEDManager
{
    SlimeVR::Logging::Logger logger("LEDManager");

    /*!
    *  @brief  Turn a LED on
    *  @param  pin
    *          LED pin
    */
    void on(uint8_t pin)
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
    void off(uint8_t pin)
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
    void blink(uint8_t pin, unsigned long time, uint8_t direction)
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
    void pattern(uint8_t pin, unsigned long timeon, unsigned long timeoff, int times, uint8_t direction)
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

    uint32_t currentStatus = 0;
    uint8_t currentCount = 0;
    unsigned long timer = 0;
    Stage currentStage = OFF;
    unsigned long statusPrintInterval = 0;
    unsigned long lastUpdate = millis();

    void setLedStatus(uint32_t status) {
        currentStatus |= status;
    }

    void unsetLedStatus(uint32_t status) {
        currentStatus &= ~status;
    }

    void ledStatusUpdate() {
        unsigned long time = millis();
        unsigned long diff = time - lastUpdate;
        if(diff < 10)
            return;
        lastUpdate = time;

        unsigned int length;
        unsigned int count;
        bool printStatus = false;
        #if defined(STATUS_PRINT_INTERVAL) && STATUS_PRINT_INTERVAL > 0
            if(statusPrintInterval += diff > STATUS_PRINT_INTERVAL) {
                statusPrintInterval = 0;
                printStatus = true;
            }
        #endif
        if((currentStatus & LED_STATUS_LOW_BATTERY) > 0) {
            count = LOW_BATTERY_COUNT;
            switch(currentStage) {
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
            if(printStatus)
                logger.debug("LOW BATTERY");
        } else if((currentStatus & LED_STATUS_IMU_ERROR) > 0) {
            count = IMU_ERROR_COUNT;
            switch(currentStage) {
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
            if(printStatus)
                logger.debug("IMU ERROR");
        } else if((currentStatus & LED_STATUS_WIFI_CONNECTING) > 0) {
            count = WIFI_CONNECTING_COUNT;
            switch(currentStage) {
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
            if(printStatus)
                logger.debug("WIFI CONNECTING");
        } else if((currentStatus & LED_STATUS_SERVER_CONNECTING) > 0) {
            count = SERVER_CONNECTING_COUNT;
            switch(currentStage) {
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
            if(printStatus)
                logger.debug("SERVER CONNECTING");
        } else {
            if(printStatus)
                logger.debug("OK");
            #if defined(LED_INTERVAL_STANDBUY) && LED_INTERVAL_STANDBUY > 0
                count = 1;
                switch(currentStage) {
                case ON:
                case OFF:
                    length = STANDBUY_LENGTH;
                    break;
                case GAP:
                    length = DEFAULT_GAP;
                    break;
                case INTERVAL:
                    length = LED_INTERVAL_STANDBUY;
                    break;
                }
            #else
                return;
            #endif
        }

        if(currentStage == OFF || timer + diff >= length) {
            timer = 0;
            // Advance stage
            switch(currentStage) {
            case OFF:
                on(STATUS_LED);
                currentStage = ON;
                currentCount = 0;
                break;
            case ON:
                off(STATUS_LED);
                currentCount++;
                if(currentCount >= count) {
                    currentCount = 0;
                    currentStage = INTERVAL;
                } else {
                    currentStage = GAP;
                }
                break;
            case GAP:
            case INTERVAL:
                on(STATUS_LED);
                currentStage = ON;
                break;
                on(STATUS_LED);
                currentStage = ON;
                break;
            }
        } else {
            timer += diff;
        }
    }

    void signalAssert() {
        pattern(LOADING_LED, 50, 50, 200);
    }
}