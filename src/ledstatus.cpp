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

#include "ledstatus.h"

#define DEFAULT_LENGTH 300
#define DEFAULT_GAP 500
#define DEFAULT_INTERVAL 3000

#define STANDBUY_LENGTH DEFAULT_LENGTH
#define IMU_ERROR_LENGTH DEFAULT_LENGTH
#define IMU_ERROR_INTERVAL 1000
#define IMU_ERROR_COUNT 5
#define WIFI_CONNECTING_LENGTH DEFAULT_LENGTH
#define WIFI_CONNECTING_INTERVAL 3000
#define WIFI_CONNECTING_COUNT 3
#define SERVER_CONNECTING_LENGTH DEFAULT_LENGTH
#define SERVER_CONNECTING_INTERVAL 3000
#define SERVER_CONNECTING_COUNT 2

enum stage {
    OFF, ON, GAP, INTERVAL
};

uint32_t currentStatus = 0;
uint8_t currentCount = 0;
unsigned long timer = 0;
stage currentStage = OFF;
unsigned long statusPrintInterval = 0;
unsigned long lastUpdate = millis();

void ledOn() {
    digitalWrite(STATUS_LED, LOW);
}

void ledOff() {
    digitalWrite(STATUS_LED, HIGH);
}

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
    if((currentStatus & LED_STATUS_IMU_ERROR) > 0) {
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
            Serial.println("[STATUS] IMU ERROR");
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
            Serial.println("[STATUS] WIFI CONNECTING");
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
            Serial.println("[STATUS] SERVER CONNECTING");
    } else {
        if(printStatus)
            Serial.println("[STATUS] OK");
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
            ledOn();
            currentStage = ON;
            currentCount = 0;
            break;
        case ON:
            ledOff();
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
            ledOn();
            currentStage = ON;
            break;
            ledOn();
            currentStage = ON;
            break;
        }
    } else {
        timer += diff;
    }
}