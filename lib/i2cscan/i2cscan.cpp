#include "i2cscan.h"

#include <array>
#include <cstdint>
#include <string>

#ifdef ESP32
#include "driver/i2c.h"
#endif

#include "../../src/globals.h"
#include "../../src/consts.h"

#define I2CSCAN_DEBUG false

namespace I2CSCAN {
    enum class ScanState : uint8_t {
        IDLE,
        SCANNING,
        DONE
    };

	namespace {
		static uint8_t defaultSDAPin =  static_cast<uint8_t>(PIN_IMU_SDA);
		static uint8_t defaultSCLPin =  static_cast<uint8_t>(PIN_IMU_SCL);
		uint8_t activeSDAPin = defaultSDAPin;
		uint8_t activeSCLPin = defaultSCLPin;
		ScanState scanState = ScanState::IDLE;
    	uint8_t currentSDA = 0;
    	uint8_t currentSCL = 0;
		uint8_t currentSDAPortIndex = 0;
		uint8_t currentSCLPortIndex = 0;
		uint8_t startSDAPortIndex = 255;
		uint8_t startSCLPortIndex = 255;
    	uint8_t currentAddress = 1;
    	bool found = false;
		uint8_t txFails = 0;
    	std::vector<uint8_t> validPortsIndex;


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
		std::array<uint8_t, 7> portExclude = {0, 9, 12, 13, 16, 17, LED_PIN};
#elif defined(ESP32)
		std::array<uint8_t, 16> portArray = {4, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
		std::array<std::string, 16> portMap = {"4", "13", "14", "15", "16", "17", "18", "19", "21", "22", "23", "25", "26", "27", "32", "33"};
		std::array<uint8_t, 1> portExclude = {LED_PIN};
#endif

		void switchPort(uint8_t sdaPortIndex, uint8_t sclPortIndex) {
#ifdef ESP32
			// code from I2CWireSensorInterface.cpp
			Wire.end();
			// Reset the Pins to not map
			gpio_set_direction((gpio_num_t)activeSCLPin, GPIO_MODE_INPUT);
			gpio_set_direction((gpio_num_t)activeSDAPin, GPIO_MODE_INPUT);
			// this line seems not to be needed
			//i2c_set_pin(I2C_NUM_0, (int)portArray[sdaPortIndex], (int)portArray[sclPortIndex], false, false, I2C_MODE_MASTER);
#endif
			Wire.begin((int)portArray[sdaPortIndex], (int)portArray[sclPortIndex]);

			activeSDAPin = portArray[sdaPortIndex];
			activeSCLPin = portArray[sclPortIndex];
#ifdef  I2CSCAN_DEBUG
			Serial.printf_P(PSTR("[DEBUG] [I2CSCAN] Change I2C to SDA: %d, SCL: %d\r\n"), (int)portArray[sdaPortIndex], (int)portArray[sclPortIndex]);
#endif
		}

		void incrementvalidPortsIndex(uint8_t &index) {
			index = (index+1) % validPortsIndex.size();
		}

		bool incrementSDA(){
			incrementvalidPortsIndex(currentSDAPortIndex);
			if (currentSDAPortIndex == startSDAPortIndex) {
				// scan finished
				switchPort(startSDAPortIndex, startSCLPortIndex);
				if (!found) {
					Serial.println(F("[ERROR] [I2CSCAN] I2C: No I2C devices found")); //NOLINT
				}
				scanState = ScanState::DONE;
				return false;
			}
			return true;
		}

		bool selectNextPort(){
			while (1) {
				incrementvalidPortsIndex(currentSCLPortIndex);
				if (currentSCLPortIndex == startSCLPortIndex) {
					// Point to increase SDA reached
					if (!incrementSDA()) {
						return false;
					}
				}
				if (currentSCLPortIndex != currentSDAPortIndex) {
					currentSCL = validPortsIndex[currentSCLPortIndex];
					currentSDA = validPortsIndex[currentSDAPortIndex];
					switchPort(currentSDA, currentSCL);
					return true;
				}
			}
		}

		template <uint8_t size1, uint8_t size2>
		uint8_t countCommonElements(
			const std::array<uint8_t, size1>& array1,
			const std::array<uint8_t, size2>& array2) {

			uint8_t count = 0;
			for (const auto& elem1 : array1) {
				for (const auto& elem2 : array2) {
					if (elem1 == elem2) {
						count++;
					}
				}
			}

			return count;
		}
	}  // anonymous namespace

    void scani2cports() {
        if (scanState != ScanState::IDLE) {
			if (scanState == ScanState::DONE) {
				Serial.println(F("[INFO ] [I2CSCAN] I2C scan finished previously, resetting and scanning again...")); //NOLINT
			} else {
				return; // Already scanning, do not start again
			}
        }

        // Filter out excluded ports
		validPortsIndex.clear();
		uint8_t excludes = countCommonElements<portArray.size(), portExclude.size()>(portArray, portExclude);
		validPortsIndex.reserve(portArray.size() - excludes); // Reserve space to avoid reallocations

		for (uint8_t i=0; i < portArray.size(); i++) {
			if (std::find(portExclude.begin(), portExclude.end(), portArray[i]) == portExclude.end()) {
				validPortsIndex.push_back(i); // Port is valid, add it to the list
			}
		}

		// Lets find out the configured Index for the Pins
		for (uint8_t i=0; i < validPortsIndex.size(); i++) {
			if (portArray[validPortsIndex[i]] == defaultSDAPin) {
				startSDAPortIndex = i;
			}
			if (portArray[validPortsIndex[i]] == defaultSCLPin) {
				startSCLPortIndex = i;
			}
		}
#ifdef  I2CSCAN_DEBUG
		Serial.printf_P(PSTR("[DEBUG] [I2CSCAN] Default I2C Ports SDA: %d SCL: %d\r\n"), defaultSDAPin, defaultSCLPin);
#endif
		if (startSDAPortIndex == 255 || startSCLPortIndex == 255) {
			Serial.printf_P(PSTR("[ERROR] [I2CSCAN] I2C Ports SDA: %d SCL: %d not found in Array. Abort the I2C Scan\r\n"), defaultSDAPin, defaultSCLPin);
			// What todo when the PIN is not in the Index? Abort the scan?
			// The current behavior will then just set index 0 and 1 as default.
			// This will lead to problematic behavior if the Arrays are not correctly defined.
			// As it will not able to reset back to the previous configured pins
			return;
		}

#ifdef  I2CSCAN_DEBUG
		for (const auto& portsIndex : validPortsIndex) {
			Serial.printf("[DEBUG] [I2CSCAN] validPortsIndex Pin Index: %2d PinNum: %2d PinName: %s\r\n", portsIndex, portArray[portsIndex], portMap[portsIndex].c_str());
		}
#endif
		// Reset scan variables and start scanning
        found = false;
		currentSDAPortIndex = startSDAPortIndex;
		currentSCLPortIndex = startSCLPortIndex;
		currentSDA = validPortsIndex[currentSDAPortIndex];
		currentSCL = validPortsIndex[currentSCLPortIndex];
		activeSDAPin = portArray[currentSDA];
		activeSCLPin = portArray[currentSCL];
        currentAddress = 1;
		txFails = 0;
        scanState = ScanState::SCANNING;
	}

    void update() {
        if (scanState != ScanState::SCANNING) {
            return;
        }

        Wire.beginTransmission(currentAddress);
        const uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf_P(PSTR("[INFO ] [I2CSCAN] I2C (SDA: %s(%d) SCL: %s(%d)): I2C device found at address 0x%02x!\n"),
                            portMap[currentSDA].c_str(), portArray[currentSDA], portMap[currentSCL].c_str(), portArray[currentSCL], currentAddress);
            found = true;
        } else if (error == 4) { // Unable to start transaction, log and warn
            Serial.printf_P(PSTR("[WARN ] [I2CSCAN] I2C (SDA: %s(%d) SCL: %s(%d)): Unable to start transaction at address 0x%02x!\n"),
                            portMap[currentSDA].c_str(), portArray[currentSDA], portMap[currentSCL].c_str(), portArray[currentSCL], currentAddress);
            txFails++;
        }

        currentAddress++;

        if (currentAddress <= 127) {
			if (txFails > 5) {
#if BOARD == BOARD_SLIMEVR_LEGACY || BOARD == BOARD_SLIMEVR_DEV || BOARD == BOARD_SLIMEVR || BOARD == BOARD_SLIMEVR_V1_2
				Serial.printf_P(PSTR("[ERROR] [I2CSCAN] I2C: Too many transaction errors (%d), please power off the tracker and contact SlimeVR support!\n"), txFails);
#else
				Serial.printf_P(PSTR("[ERROR] [I2CSCAN] I2C: Too many transaction errors (%d), please power off the tracker and check the IMU connections!\n"), txFails);
#endif
			}

            return;
        }

        currentAddress = 1;
        selectNextPort();
    }

    bool hasDevOnBus(uint8_t addr) {
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

    int clearBus(uint8_t SDA, uint8_t SCL) {
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
