/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 unlogisch04 & SlimeVR contributors

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

// #include "globals.h"
#include <Arduino.h>

#if defined(CONFIG_IDF_TARGET_ESP32C3)
#include "soc/rtc_cntl_reg.h"
#endif

#define PREINIT_CONSOLEMAXLENGTH 30

static const char conshelptxt[] PROGMEM
	= "Avaiable Commands\r\n"
	  "HELP          - Prints this dialog\r\n"
	  "SET FLASHMODE - Sets the tracker into flashmode\r\n"
	  "FRST          - Factory reset\r\n"
	  "REBOOT        - Reboots the tracker into normal mode\r\n"
	  "CONTINUE      - Continue to normal mode\r\n";

static const char consinittxt[] PROGMEM
	= "\r\n===== Recovery Console =====\r\n"
	  "The tracker rebootet due to a crash\r\n"
	  "With the recovery console you're able to do some basic recovery operations.\r\n";

static const char consflashmode[] PROGMEM
	= "Entering flashing mode.\r\n"
	  "You can now close the serial monitor\r\n"
	  "and go to the firmware flasher to flash\r\n"
	  "your tracker over USB.\r\n"
	  "If you entered the flashing mode by accident,\r\n"
	  "turn your tracker off and on.";

static const char consfrst[] PROGMEM = "Init Factory Reset\r\n";

static const char consnormal[] PROGMEM = "Continue to normal mode\r\n";

static const char consna[] PROGMEM = "Error: Unknown command";

static const char consnimpl[] PROGMEM = "Error: Command is not implemented";

static const char constrlong[] PROGMEM = "Error: Too long input string";

bool initFullreset = false;

// Functions that need to be writen to the HW Layer
#if ESP8266
bool preinit_detectconsole() {
	rst_info* rstreason = ESP.getResetInfoPtr();
	// false: continue boot true: recovery console
	return (
		rstreason->reason == REASON_WDT_RST || rstreason->reason == REASON_EXCEPTION_RST
		|| rstreason->reason == REASON_SOFT_WDT_RST
	);
}

int8_t preinit_reboot() {
	// ESP.restart()) is asynchronous, this does not work at this point for ESP8266
	ESP.reset();
	return 1;
}

int8_t preinit_frst() { return 3; }

int8_t preinit_flashmode() {
	ESP.rebootIntoUartDownloadMode();
	return 1;
}
#else
bool preinit_detectconsole() {
	esp_reset_reason_t rstreason = esp_reset_reason();
	// false: continue boot true: recovery console
	return (
		rstreason == ESP_RST_PANIC || rstreason == ESP_RST_INT_WDT
		|| rstreason == ESP_RST_TASK_WDT || rstreason == ESP_RST_WDT
		|| rstreason == ESP_RST_CPU_LOCKUP
	);
}

int8_t preinit_reboot() {
	ESP.restart();
	return 1;
}

int8_t preinit_frst() { return 3; }

int8_t preinit_flashmode() {
	// It seems that the different ESP Chips have differnt Registerf if they even
	// avaliable.
	// On ESP Chip with a direct USB Interface the Flashmode is not as importandt, only
	// that it is not rebooting all the time and makes it hard to set it into download
	// mode over USB
#ifdef CONFIG_IDF_TARGET_ESP32C3
	// from https://esp32.com/viewtopic.php?t=33180
	REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
	esp_restart();
	return 1;
#else
	return -2;
#endif
}

#endif

void printSlimeVRConfig(Stream& Serial) { Serial.printf(sSlVRPrInfo); }

// not HW depending functions
int8_t preinit_select(String& cmd, Stream& Serial) {
	int8_t ret = -1;
	cmd.toUpperCase();
	Serial.printf("Command: %s\r\n", cmd.c_str());
	if (cmd.startsWith(F("HELP"))) {
		Serial.println(conshelptxt);
		ret = 1;
	} else if (cmd.startsWith("SET FLASHMODE")) {
		Serial.println(consflashmode);
		ret = preinit_flashmode();
	} else if (cmd.startsWith("FRST")) {
		Serial.println(consfrst);
		ret = preinit_frst();
	} else if (cmd.startsWith("REBOOT")) {
		ret = preinit_reboot();
	} else if (cmd.startsWith("CONTINUE")) {
		ret = 2;
	}
	return ret;
}

void preinit_console(Stream& Serial) {
	Serial.println(consinittxt);
	Serial.println(conshelptxt);

	String cmdbuffer;

	while (1) {
		if (Serial.available()) {
			int ichr = Serial.read();
			if (ichr < 0 || ichr > 255) {
				// not valid char
				continue;
			}
			char chr = (char)ichr;
			if (chr == '\r') {
				continue;
			}
			if (chr == '\n') {
				int8_t i = preinit_select(cmdbuffer, Serial);
				switch (i) {
					case -2:
						// Command not implemented (valid command but it does not exist)
						Serial.println(consnimpl);
						break;
					case -1:
						// Command not found
						Serial.println(consna);
						Serial.println(conshelptxt);
						break;
					case 2:
						// Command requets to go to normal mode
						Serial.println(consnormal);
						return;
					case 3:
						// Command requests a factory reset
						initFullreset = true;
						return;
					default:
						// No action
						break;
				}
				cmdbuffer = "";
			} else {
				cmdbuffer += chr;
				if (cmdbuffer.length() > PREINIT_CONSOLEMAXLENGTH) {
					cmdbuffer = "";
					Serial.println(constrlong);
					Serial.println(conshelptxt);
				}
			}
		}
#ifdef ESP8266
		ESP.wdtFeed();
#endif
		delay(1);
	}
}

// Register the Framework functions call that are called before
// the c++ constructor
#ifdef ESP8266
#include <user_interface.h>
extern "C" void preinit(void) {
	rst_info* rstreason = ESP.getResetInfoPtr();
	if (rstreason->reason == REASON_DEEP_SLEEP_AWAKE) {
		// Even if currently there is no deep sleep implemented skip for faster wakeup
		return;
	}
	HardwareSerial Serial(0);

	Serial.begin(115200);
	Serial.println();
	Serial.println();
	Serial.println(F("==== SlimeVR Tracker ESP Booting ===="));
	Serial.printf_P(PSTR("getBootVersion(): %d"), ESP.getBootVersion());
	Serial.println();
	Serial.printf_P(PSTR("Arduino Core Version: %s"), ESP.getCoreVersion().c_str());
	Serial.println();
	Serial.printf_P(PSTR("FullVersion: %s"), ESP.getFullVersion().c_str());
	Serial.println();
	Serial.printf_P(PSTR("Core Frequency: %d MHz"), ESP.getCpuFreqMHz());
	Serial.println();
	Serial.printf_P(
		PSTR("Checking FlashCRC: %s"),
		ESP.checkFlashCRC() ? F("OK") : F("Failed")
	);
	Serial.println();
	// MD5 takes quit a while to calculate maybe not make it?
	Serial.printf_P(PSTR("Sketch MD5: %s"), ESP.getSketchMD5().c_str());
	Serial.println();
	Serial.printf_P(PSTR("Reset reason: %s"), ESP.getResetInfo().c_str());
	Serial.println();
	Serial.printf_P(PSTR("Flash VendorId: %d"), ESP.getFlashChipVendorId());
	Serial.println();
	Serial.printf_P(PSTR("Flash size: %d"), ESP.getFlashChipSize());
	Serial.println();
	Serial.printf_P(PSTR("Flash size real: %d"), ESP.getFlashChipRealSize());
	Serial.println();
	Serial.printf_P(PSTR("Flash size by ChipId: %d"), ESP.getFlashChipSizeByChipId());
	Serial.println();
	Serial.println();

	printSlimeVRConfig(Serial);
	delay(1);

	if (preinit_detectconsole()) {
		preinit_console(Serial);
	}

	Serial.flush();
	Serial.end();
}
#elif defined(ESP32)
// On ESP32 the best place to inject code seems to be init() as it is not used and
// defined as weak
// it runs before initVariant(void) wich is a place for board variant code to initialize

const char* init_resetreason(esp_reset_reason_t rstreason) {
	switch (rstreason) {
		case ESP_RST_UNKNOWN:
			return "ESP_RST_UNKNOWN";
		case ESP_RST_POWERON:
			return "ESP_RST_POWERON";
		case ESP_RST_EXT:
			return "ESP_RST_EXT";
		case ESP_RST_SW:
			return "ESP_RST_SW";
		case ESP_RST_PANIC:
			return "ESP_RST_PANIC";
		case ESP_RST_INT_WDT:
			return "ESP_RST_INT_WDT";
		case ESP_RST_TASK_WDT:
			return "ESP_RST_TASK_WDT";
		case ESP_RST_WDT:
			return "ESP_RST_WDT";
		case ESP_RST_DEEPSLEEP:
			return "ESP_RST_DEEPSLEEP";
		case ESP_RST_BROWNOUT:
			return "ESP_RST_BROWNOUT";
		case ESP_RST_SDIO:
			return "ESP_RST_SDIO";
		case ESP_RST_USB:
			return "ESP_RST_USB";
		case ESP_RST_JTAG:
			return "ESP_RST_JTAG";
		case ESP_RST_EFUSE:
			return "ESP_RST_EFUSE";
		case ESP_RST_PWR_GLITCH:
			return "ESP_RST_PWR_GLITCH";
		case ESP_RST_CPU_LOCKUP:
			return "ESP_RST_CPU_LOCKUP";
		default:
			return "UNKNOWN";
	}
}

extern "C" void init(void) {
	esp_reset_reason_t rstreason = esp_reset_reason();
	if (rstreason == ESP_RST_DEEPSLEEP) {
		// Even if currently there is no deep sleep implemented skip for faster wakeup
		return;
	}

	Serial.begin(115200);
	Serial.println();
	Serial.println();
	Serial.println(F("==== SlimeVR Tracker ESP Booting ===="));
	Serial.printf_P(PSTR("Arduino Core Version: %s"), ESP.getCoreVersion());
	Serial.println();
	Serial.printf_P(PSTR("MCU: %s"), ESP.getChipModel());
	Serial.println();
	Serial.printf_P(PSTR("Core Frequency: %d MHz"), ESP.getCpuFreqMHz());
	Serial.println();
	// MD5 takes quit a while to calculate maybe not make it?
	Serial.printf_P(PSTR("Sketch MD5: %s"), ESP.getSketchMD5().c_str());
	Serial.println();
	Serial.printf_P(PSTR("Reset reason: %s"), init_resetreason(rstreason));
	Serial.println();
	Serial.printf_P(PSTR("Flash VendorId: %d"), ESP.getChipModel());
	Serial.println();
	Serial.printf_P(PSTR("Flash size: %d"), ESP.getFlashChipSize());
	Serial.println();
	Serial.println();

	printSlimeVRConfig(Serial);
	delay(1);

	if (preinit_detectconsole()) {
		preinit_console(Serial);
	}
}
#endif
