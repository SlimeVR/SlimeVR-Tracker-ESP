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

#include "serialcommands.h"
#include "network/network.h"
#include "logging/Logger.h"
#include <CmdCallback.hpp>
#include "GlobalVars.h"
#include "batterymonitor.h"
#include "utils.h"

#if ESP32
    #include "nvs_flash.h"
#endif

namespace SerialCommands {
    SlimeVR::Logging::Logger logger("SerialCommands");

    CmdCallback<5> cmdCallbacks;
    CmdParser cmdParser;
    CmdBuffer<64> cmdBuffer;

    void cmdSet(CmdParser * parser) {
        if(parser->getParamCount() != 1 && parser->equalCmdParam(1, "WIFI")  ) {
            if(parser->getParamCount() < 3) {
                logger.error("CMD SET WIFI ERROR: Too few arguments");
                logger.info("Syntax: SET WIFI \"<SSID>\" \"<PASSWORD>\"");
            } else {
                WiFiNetwork::setWiFiCredentials(parser->getCmdParam(2), parser->getCmdParam(3));
                logger.info("CMD SET WIFI OK: New wifi credentials set, reconnecting");
            }
        } else {
            logger.error("CMD SET ERROR: Unrecognized variable to set");
        }
    }

    void printState() {
        logger.info(
            "SlimeVR Tracker, board: %d, hardware: %d, build: %d, firmware: %s, address: %s, mac: %s, status: %d, wifi state: %d",
            BOARD,
            HARDWARE_MCU,
            FIRMWARE_BUILD_NUMBER,
            FIRMWARE_VERSION,
            WiFiNetwork::getAddress().toString().c_str(),
            WiFi.macAddress().c_str(),
            statusManager.getStatus(),
            WiFiNetwork::getWiFiState()
        );
        Sensor* sensor1 = sensorManager.getFirst();
        Sensor* sensor2 = sensorManager.getSecond();
        logger.info(
            "Sensor 1: %s (%.3f %.3f %.3f %.3f) is working: %s, had data: %s",
            getIMUNameByType(sensor1->getSensorType()),
            UNPACK_QUATERNION(sensor1->getQuaternion()),
            sensor1->isWorking() ? "true" : "false",
            sensor1->hadData ? "true" : "false"
        );
        logger.info(
            "Sensor 2: %s (%.3f %.3f %.3f %.3f) is working: %s, had data: %s",
            getIMUNameByType(sensor2->getSensorType()),
            UNPACK_QUATERNION(sensor2->getQuaternion()),
            sensor2->isWorking() ? "true" : "false",
            sensor2->hadData ? "true" : "false"
        );
    }

    void cmdGet(CmdParser * parser) {
        if (parser->getParamCount() < 2) {
            return;
        }
        
        if (parser->equalCmdParam(1, "INFO")) {
            printState();
        }

        if (parser->equalCmdParam(1, "CONFIG")) {
            String str =
                "BOARD=%d\n" 
                "IMU=%d\n"
                "SECOND_IMU=%d\n"
                "IMU_ROTATION=%f\n"
                "SECOND_IMU_ROTATION=%f\n"
                "BATTERY_MONITOR=%d\n"
                "BATTERY_SHIELD_RESISTANCE=%d\n"
                "BATTERY_SHIELD_R1=%d\n"
                "BATTERY_SHIELD_R2=%d\n"
                "PIN_IMU_SDA=%d\n"
                "PIN_IMU_SCL=%d\n"
                "PIN_IMU_INT=%d\n"
                "PIN_IMU_INT_2=%d\n"
                "PIN_BATTERY_LEVEL=%d\n"
                "LED_PIN=%d\n"
                "LED_INVERTED=%d\n";

            Serial.printf(
                str.c_str(),
                BOARD,
                IMU,
                SECOND_IMU,
                IMU_ROTATION,
                SECOND_IMU_ROTATION,
                BATTERY_MONITOR,
                BATTERY_SHIELD_RESISTANCE,
                BATTERY_SHIELD_R1,
                BATTERY_SHIELD_R2,
                PIN_IMU_SDA,
                PIN_IMU_SCL,
                PIN_IMU_INT,
                PIN_IMU_INT_2,
                PIN_BATTERY_LEVEL,
                LED_PIN,
                LED_INVERTED
            );
        }

        if (parser->equalCmdParam(1, "TEST")) {
            logger.info(
                "[TEST] Board: %d, hardware: %d, build: %d, firmware: %s, address: %s, mac: %s, status: %d, wifi state: %d",
                BOARD,
                HARDWARE_MCU,
                FIRMWARE_BUILD_NUMBER,
                FIRMWARE_VERSION,
                WiFiNetwork::getAddress().toString().c_str(),
                WiFi.macAddress().c_str(),
                statusManager.getStatus(),
                WiFiNetwork::getWiFiState()
            );
            Sensor* sensor1 = sensorManager.getFirst();
            sensor1->motionLoop();
            logger.info(
                "[TEST] Sensor 1: %s (%.3f %.3f %.3f %.3f) is working: %s, had data: %s",
                getIMUNameByType(sensor1->getSensorType()),
                UNPACK_QUATERNION(sensor1->getQuaternion()),
                sensor1->isWorking() ? "true" : "false",
                sensor1->hadData ? "true" : "false"
            );
            if(!sensor1->hadData) {
                logger.error("[TEST] Sensor 1 didn't send any data yet!");
            } else {
                logger.info("[TEST] Sensor 1 sent some data, looks working.");
            }
        }
    }

    void cmdReboot(CmdParser * parser) {
        logger.info("REBOOT");
        ESP.restart();
    }

    void cmdFactoryReset(CmdParser * parser) {
        logger.info("FACTORY RESET");

        configuration.reset();

        WiFi.disconnect(true); // Clear WiFi credentials
        #if ESP8266
            ESP.eraseConfig(); // Clear ESP config
        #elif ESP32
            nvs_flash_erase();
        #else
            #warning SERIAL COMMAND FACTORY RESET NOT SUPPORTED
            logger.info("FACTORY RESET NOT SUPPORTED");
            return;
        #endif

        #if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
            #warning FACTORY RESET does not clear your hardcoded WiFi credentials!
            logger.warn("FACTORY RESET does not clear your hardcoded WiFi credentials!");
        #endif

        delay(3000);
        ESP.restart();
    }

    void setUp() {
        cmdCallbacks.addCmd("SET", &cmdSet);
        cmdCallbacks.addCmd("GET", &cmdGet);
        cmdCallbacks.addCmd("FRST", &cmdFactoryReset);
        cmdCallbacks.addCmd("REBOOT", &cmdReboot);
        
    }

    void update() {
        cmdCallbacks.updateCmdProcessing(&cmdParser, &cmdBuffer, &Serial);
    }
}
