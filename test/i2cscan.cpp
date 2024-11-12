/*
 * i2c_port_address_scanner
 * Scans ports D0 to D7 on an ESP8266 and searches for I2C device. based on the original
 * code available on Arduino.cc and later improved by user Krodal and Nick Gammon
 * (www.gammon.com.au/forum/?id=10896) D8 throws exceptions thus it has been left out
 *
 */
#include <Arduino.h>
#include <Wire.h>

#include "configuration/Configuration.h"
#include "credentials.h"
#include "helper_3dmath.h"
#include "ota.h"
#include "udpclient.h"

SlimeVR::Configuration::DeviceConfig config;

uint8_t portArray[] = {16, 5, 4, 0, 2, 14, 12, 13};
// String portMap[] = {"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7"}; //for Wemos
String portMap[]
	= {"GPIO16", "GPIO5", "GPIO4", "GPIO0", "GPIO2", "GPIO14", "GPIO12", "GPIO13"};

void check_if_exist_I2C() {
	byte error, address;
	int nDevices;
	nDevices = 0;
	for (address = 1; address < 127; address++) {
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0) {
			Serial.print("I2C device found at address 0x");
			if (address < 16) {
				Serial.print("0");
			}
			Serial.print(address, HEX);
			Serial.println("  !");

			nDevices++;
		} else if (error == 4) {
			Serial.print("Unknow error at address 0x");
			if (address < 16) {
				Serial.print("0");
			}
			Serial.println(address, HEX);
		}
	}  // for loop
	if (nDevices == 0) {
		Serial.println("No I2C devices found");
	} else {
		Serial.println("Scan finished\n");
	}
}

void scanPorts() {
	for (uint8_t i = 0; i < sizeof(portArray); i++) {
		for (uint8_t j = 0; j < sizeof(portArray); j++) {
			if (i != j) {
				Serial.print(
					"Scanning (SDA : SCL) - " + portMap[i] + " : " + portMap[j] + " - "
				);
				Wire.begin(portArray[i], portArray[j]);
				check_if_exist_I2C();
			}
		}
	}
}

void I2C_recovery() {
	Serial.println("Starting I2C bus recovery");
	delay(2000);
	int SDAPIN = 70;
	int CLKPIN = 71;
	// try i2c bus recovery at 100kHz = 5uS high, 5uS low
	pinMode(SDAPIN, OUTPUT);  // keeping SDA high during recovery
	digitalWrite(SDAPIN, HIGH);
	pinMode(CLKPIN, OUTPUT);
	for (int i = 0; i < 10; i++) {  // 9nth cycle acts as NACK
		digitalWrite(CLKPIN, HIGH);
		delayMicroseconds(5);
		digitalWrite(CLKPIN, LOW);
		delayMicroseconds(5);
	}

	// a STOP signal (SDA from low to high while CLK is high)
	digitalWrite(SDAPIN, LOW);
	delayMicroseconds(5);
	digitalWrite(CLKPIN, HIGH);
	delayMicroseconds(2);
	digitalWrite(SDAPIN, HIGH);
	delayMicroseconds(2);
	// bus status is now : FREE

	Serial.println("bus recovery done, starting scan in 2 secs");
	// return to power up mode
	pinMode(SDAPIN, INPUT);
	pinMode(CLKPIN, INPUT);
	delay(2000);
	// pins + begin advised in https://github.com/esp8266/Arduino/issues/452
	// Wire1.pins(SDAPIN, CLKPIN); //this changes default values for sda and clock as
	// well
	Wire.begin();
	// only pins: no signal on clk and sda
	// only begin: no signal on clk, no signal on sda
}

void loop() { otaUpdate(); }

void setup() {
	Serial.begin(115200);
	while (!Serial)
		;  // Leonardo: wait for serial monitor
	Serial.println("\n\nI2C Scanner to scan for devices on each port pair D0 to D7");
	scanPorts();
	I2C_recovery();
	scanPorts();

	setUpWiFi(&config);
	otaSetup(otaPassword);
}
