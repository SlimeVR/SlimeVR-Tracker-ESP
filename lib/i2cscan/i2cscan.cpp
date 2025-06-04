#include "i2cscan.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "../../src/globals.h"

namespace {
#ifdef ESP8266
	std::array<uint8_t, 7> portArray = {16, 5, 4, 2, 14, 12, 13};
	std::array<std::string, 7> portMap = {"D0", "D1", "D2", "D4", "D5", "D6", "D7"};
	std::array<uint8_t, 1> portExclude = {LED_PIN};
#elif defined(ESP32C3)
	std::array<uint8_t, 9> portArray = {2, 3, 4, 5, 6, 7, 8, 9, 10};
	std::array<std::string, 9> portMap = {"2", "3", "4", "5", "6", "7", "8", "9", "10"};
	std::array<uint8_t, 5> portExclude = {18, 19, 20, 21, LED_PIN};
#elif defined(ESP32C6)
	std::array<uint8_t, 20> portArray = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15, 18, 19, 20, 21, 22, 23};
	std::array<std::string, 20> portMap = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "14", "15", "18", "19", "20", "21", "22", "23"};
	std::array<uint8_t, 4> portExclude = {12, 13, 16, 17, LED_PIN};
#elif defined(ESP32)
	std::array<uint8_t, 16> portArray = {4, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
	std::array<std::string, 16> portMap = {"4", "13", "14", "15", "16", "17", "18", "19", "21", "22", "23", "25", "26", "27", "32", "33"};
	std::array<uint8_t, 1> portExclude = {LED_PIN};
#endif
}

namespace I2CSCAN {
    enum class ScanState : uint8_t {
        IDLE,
        SCANNING,
        DONE
    };

	namespace {
		ScanState scanState = ScanState::IDLE;
    	uint8_t currentSDA = 0;
    	uint8_t currentSCL = 0;
    	uint8_t currentAddress = 1;
    	bool found = false;
		uint8_t busyHits = 0;
    	std::vector<uint8_t> validPorts;

		auto selectNextPort() -> bool {
			currentSCL++;

			if(validPorts[currentSCL] == validPorts[currentSDA]) currentSCL++;

			if (currentSCL < validPorts.size()) {
				Wire.begin((int)validPorts[currentSDA], (int)validPorts[currentSCL]); //NOLINT
				return true;
			}

			currentSCL = 0;
			currentSDA++;

			if (currentSDA >= validPorts.size()) {
				if (!found) {
					Serial.println("[ERR] I2C: No I2C devices found"); //NOLINT
				}
	#ifdef ESP32
				Wire.end();
	#endif
				Wire.begin(static_cast<int>(PIN_IMU_SDA), static_cast<int>(PIN_IMU_SCL));
				scanState = ScanState::DONE;
				return false;
			}

			Wire.begin((int)validPorts[currentSDA], (int)validPorts[currentSCL]);
			return true;
		}
	}

    void scani2cports() {
        if (scanState != ScanState::IDLE) {
			if (scanState == ScanState::DONE) {
				Serial.println("[DBG] I2C scan finished previously, resetting and scanning again..."); //NOLINT
			} else {
				return; // Already scanning, do not start again
			}
        }

        // Filter out excluded ports
		validPorts.clear();
		validPorts.reserve(portArray.size() - portExclude.size()); // Reserve space to avoid reallocations

		for (const auto& port : portArray) {
			bool isExcluded = false;

			// Check if the port is in the excluded list
			for (const auto& excluded : portExclude) {
				if (port == excluded) {
					isExcluded = true; // Port is excluded, break out of the loop
					break;
				}
			}

			if (!isExcluded) {
				validPorts.push_back(port); // Port is valid, add it to the list
			}
		}

		// Reset scan variables and start scanning
        found = false;
        currentSDA = 0;
        currentSCL = 1;
        currentAddress = 1;
        scanState = ScanState::SCANNING;
	}

    void update() {
        if (scanState != ScanState::SCANNING) {
            return;
        }

#ifdef ESP32
		if (currentAddress == 1) {
            Wire.end();
		}
#endif

        Wire.beginTransmission(currentAddress);
        const uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("[DBG] I2C (@ %s(%d) : %s(%d)): I2C device found at address 0x%02x  !\n",
                            portMap[currentSDA].c_str(), validPorts[currentSDA], portMap[currentSCL].c_str(), validPorts[currentSCL], currentAddress);
            found = true;
        } else if (error == 4) { // Address was busy, log and warn
            Serial.printf("[WARN] I2C (@ %s(%d) : %s(%d)): Busy at address 0x%02x, skipping !\n",
                            portMap[currentSDA].c_str(), validPorts[currentSDA], portMap[currentSCL].c_str(), validPorts[currentSCL], currentAddress);
            busyHits++;
        }

        currentAddress++;

        if (currentAddress <= 127) {
			if (busyHits > 10) {
				Serial.printf("[ERR] I2C: Too many busy hits (%d), power down tracker and check connections !\n", busyHits);
			}

            return;
        }

        currentAddress = 1;
        selectNextPort();
    }

    auto inArray(uint8_t value, const uint8_t *array, size_t arraySize) -> bool {
        for (size_t i = 0; i < arraySize; i++) {
            if (value == array[i]) {
                return true;
            }
        }

        return false;
    }

    auto hasDevOnBus(uint8_t addr) -> bool {
        byte error;
#if ESP32C3
        int retries = 2;
        do {
#endif
            Wire.beginTransmission(addr);
            error = Wire.endTransmission(); // The return value of endTransmission is used to determine if a device is present
#if ESP32C3
        }
        while (error != 0 && retries--);
#endif
        if(error == 0)
            return true;
        return false;
    }

    /**
     * This routine turns off the I2C bus and clears it
     * on return SCA and SCL pins are tri-state inputs.
     * You need to call Wire.begin() after this to re-enable I2C
     * This routine does NOT use the Wire library at all.
     *
     * returns 0 if bus cleared
     *         1 if SCL held low.
     *         2 if SDA held low by slave clock stretch for > 2sec
     *         3 if SDA held low after 20 clocks.
     * From: http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
     * (c)2014 Forward Computing and Control Pty. Ltd.
     * NSW Australia, www.forward.com.au
     * This code may be freely used for both private and commerical use
     */

    auto clearBus(uint8_t SDA, uint8_t SCL) -> int {
#if defined(TWCR) && defined(TWEN)
        TWCR &= ~(_BV(TWEN)); // Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

        pinMode(SDA, INPUT_PULLUP);
        pinMode(SCL, INPUT_PULLUP);

        boolean SCL_LOW = (digitalRead(SCL) == LOW);
        if (SCL_LOW) {
            return 1; // I2C bus error. Could not clear SCL, clock line held low.
        }

        boolean SDA_LOW = (digitalRead(SDA) == LOW);
        int clockCount = 20; // > 2x9 clock

        while (SDA_LOW && (clockCount > 0)) {
            clockCount--;
            pinMode(SCL, INPUT);
            pinMode(SCL, OUTPUT);
            delayMicroseconds(10);
            pinMode(SCL, INPUT);
            pinMode(SCL, INPUT_PULLUP);
            delayMicroseconds(10);
            SCL_LOW = (digitalRead(SCL) == LOW);
            int counter = 20;
            while (SCL_LOW && (counter > 0)) {
                counter--;
                delay(100);
                SCL_LOW = (digitalRead(SCL) == LOW);
            }
            if (SCL_LOW) {
                return 2;
            }
            SDA_LOW = (digitalRead(SDA) == LOW);
        }
        if (SDA_LOW) {
            return 3;
        }

        pinMode(SDA, INPUT);
        pinMode(SDA, OUTPUT);
        delayMicroseconds(10);
        pinMode(SDA, INPUT);
        pinMode(SDA, INPUT_PULLUP);
        delayMicroseconds(10);
        pinMode(SDA, INPUT);
        pinMode(SCL, INPUT);
        return 0;
    }
}
