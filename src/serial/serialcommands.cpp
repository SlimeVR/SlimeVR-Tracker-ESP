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
#include "logging/Logger.h"
#include <CmdCallback.hpp>
#include "base64.hpp"
#include "GlobalVars.h"
#include "batterymonitor.h"
#include "utils.h"

#if ESP32
    #include "nvs_flash.h"
#endif

namespace SerialCommands {
    SlimeVR::Logging::Logger logger("SerialCommands");

    CmdCallback<6> cmdCallbacks;
    CmdParser cmdParser;
    CmdBuffer<256> cmdBuffer;


	bool lengthCheck (const char* const text, unsigned int length, const char* const cmd, const char* const name)
	{
		size_t l = text != nullptr ? strlen(text) : 0;
		if ((l > length)) {
			logger.error("%s ERROR: %s is longer than %d bytes / Characters", cmd, name, length);
			return false;
		}
		return true;
	}

	unsigned int decode_base64_length_null(const char* const b64char, unsigned int* b64ssidlength)
	{
		if (b64char==NULL) {
			return 0;
		}
		*b64ssidlength = (unsigned int)strlen(b64char);
		return decode_base64_length((unsigned char *)b64char, *b64ssidlength);
	}

	void cmdSet(CmdParser * parser) {
		if(parser->getParamCount() != 1) {
			if (parser->equalCmdParam(1, "WIFI")) {
				if(parser->getParamCount() < 3) {
					logger.error("CMD SET WIFI ERROR: Too few arguments");
					logger.info("Syntax: SET WIFI \"<SSID>\" \"<PASSWORD>\"");
				} else {
					const char *sc_ssid = parser->getCmdParam(2);
					const char *sc_pw = parser->getCmdParam(3);

					if (!lengthCheck(sc_ssid, 32, "CMD SET WIFI", "SSID") &&
						!lengthCheck(sc_pw, 64, "CMD SET WIFI", "Password")) {
						return;
					}

					WiFiNetwork::setWiFiCredentials(sc_ssid, sc_pw);
					logger.info("CMD SET WIFI OK: New wifi credentials set, reconnecting");
				}
			} else if (parser->equalCmdParam(1, "BWIFI")) {
				if(parser->getParamCount() < 3) {
					logger.error("CMD SET BWIFI ERROR: Too few arguments");
					logger.info("Syntax: SET BWIFI <B64SSID> <B64PASSWORD>");
				} else {
					const char * b64ssid = parser->getCmdParam(2);
					const char * b64pass = parser->getCmdParam(3);
					unsigned int b64ssidlength = 0;
					unsigned int b64passlength = 0;
					unsigned int ssidlength = decode_base64_length_null(b64ssid, &b64ssidlength);
					unsigned int passlength = decode_base64_length_null(b64pass, &b64passlength);

					// alloc the strings and set them to 0 (null terminating)
					char ssid[ssidlength+1];
					memset(ssid, 0, ssidlength+1);
					char pass[passlength+1];
					memset(pass, 0, passlength+1);
					// make a pointer to pass
					char *ppass = pass;
					decode_base64((const unsigned char *)b64ssid, b64ssidlength, (unsigned char*)ssid);
					if (!lengthCheck(ssid, 32, "CMD SET BWIFI", "SSID")) {
						return;
					}

					if ((b64pass!=NULL) && (b64passlength > 0)) {
						decode_base64((const unsigned char *)b64pass, b64passlength, (unsigned char*)pass);
						if (!lengthCheck(pass, 64, "CMD SET BWIFI", "Password")) {
							return;
						}
					} else {
						// set the pointer for pass to null for no password
						ppass = NULL;
					}
					WiFiNetwork::setWiFiCredentials(ssid, ppass);
					logger.info("CMD SET BWIFI OK: New wifi credentials set, reconnecting");
				}
			} else {
				logger.error("CMD SET ERROR: Unrecognized variable to set");
			}
		} else {
			logger.error("CMD SET ERROR: No variable to set");
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
        for (auto sensor : sensorManager.getSensors()) {
            logger.info(
                "Sensor[%d]: %s (%.3f %.3f %.3f %.3f) is working: %s, had data: %s",
                sensor->getSensorId(),
                getIMUNameByType(sensor->getSensorType()),
                UNPACK_QUATERNION(sensor->getFusedRotation()),
                sensor->isWorking() ? "true" : "false",
                sensor->hadData ? "true" : "false"
            );
        }
    }

    void cmdGet(CmdParser * parser) {
        if (parser->getParamCount() < 2) {
            return;
        }

        if (parser->equalCmdParam(1, "INFO")) {
            printState();

            // We don't want to print this on every timed state output
            logger.info("Git commit: %s", GIT_REV);
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
            Sensor* sensor0 = sensorManager.getSensors()[0];
            sensor0->motionLoop();
            logger.info(
                "[TEST] Sensor[0]: %s (%.3f %.3f %.3f %.3f) is working: %s, had data: %s",
                getIMUNameByType(sensor0->getSensorType()),
                UNPACK_QUATERNION(sensor0->getFusedRotation()),
                sensor0->isWorking() ? "true" : "false",
                sensor0->hadData ? "true" : "false"
            );
            if(!sensor0->hadData) {
                logger.error("[TEST] Sensor[0] didn't send any data yet!");
            } else {
                logger.info("[TEST] Sensor[0] sent some data, looks working.");
            }
        }

        if (parser->equalCmdParam(1, "WIFISCAN")) {
			logger.info("[WSCAN] Scanning for WiFi networks...");

			// Scan would fail if connecting, stop connecting before scan
			if (WiFi.status() != WL_CONNECTED) {
				WiFi.disconnect();
			}
			if (WiFiNetwork::isProvisioning()) {
				WiFiNetwork::stopProvisioning();
			}

			WiFi.scanNetworks();

			int scanRes = WiFi.scanComplete();
			if (scanRes >= 0) {
				logger.info("[WSCAN] Found %d networks:", scanRes);
				for (int i = 0; i < scanRes; i++) {
					logger.info("[WSCAN] %d:\t%02d\t%s\t(%d)\t%s",
						i, WiFi.SSID(i).length(), WiFi.SSID(i).c_str(), WiFi.RSSI(i),
						((WiFi.encryptionType(i) == 0) ? "OPEN" : "PASS")
					);
				}
				WiFi.scanDelete();
			} else {
				logger.info("[WSCAN] Scan failed!");
			}

			// Restore conencting state
			if (WiFi.status() != WL_CONNECTED) {
				WiFi.begin();
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

    void cmdTemperatureCalibration(CmdParser* parser) {
        if (parser->getParamCount() > 1) {
            if (parser->equalCmdParam(1, "PRINT")) {
                for (auto sensor : sensorManager.getSensors()) {
                    sensor->printTemperatureCalibrationState();
                }
                return;
            } else if (parser->equalCmdParam(1, "DEBUG")) {
                for (auto sensor : sensorManager.getSensors()) {
                    sensor->printDebugTemperatureCalibrationState();
                }
                return;
            } else if (parser->equalCmdParam(1, "RESET")) {
                for (auto sensor : sensorManager.getSensors()) {
                    sensor->resetTemperatureCalibrationState();
                }
                return;
            } else if (parser->equalCmdParam(1, "SAVE")) {
                for (auto sensor : sensorManager.getSensors()) {
                    sensor->saveTemperatureCalibration();
                }
                return;
            }
        }
        logger.info("Usage:");
        logger.info("  TCAL PRINT: print current temperature calibration config");
        logger.info("  TCAL DEBUG: print debug values for the current temperature calibration profile");
        logger.info("  TCAL RESET: reset current temperature calibration in RAM (does not delete already saved)");
        logger.info("  TCAL SAVE: save current temperature calibration to persistent flash");
        logger.info("Note:");
        logger.info("  Temperature calibration config saves automatically when calibration percent is at 100%");
    }

    void setUp() {
        cmdCallbacks.addCmd("SET", &cmdSet);
        cmdCallbacks.addCmd("GET", &cmdGet);
        cmdCallbacks.addCmd("FRST", &cmdFactoryReset);
        cmdCallbacks.addCmd("REBOOT", &cmdReboot);
        cmdCallbacks.addCmd("TCAL", &cmdTemperatureCalibration);
    }

    void update() {
        cmdCallbacks.updateCmdProcessing(&cmdParser, &cmdBuffer, &Serial);
    }
}
