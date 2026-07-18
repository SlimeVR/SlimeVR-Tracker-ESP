#pragma once

// From the SH-2 interface that BNO08x use.
enum class SensorErrorCode : uint8_t {
	NOT_APPLICABLE = 0,
	POWER_ON_RESET = 1,
	INTERNAL_SYSTEM_RESET = 2,
	WATCHDOG_TIMEOUT = 3,
	EXTERNAL_RESET = 4,
	OTHER = 5,
};
