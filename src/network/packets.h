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

#ifndef SLIMEVR_PACKETS_H_
#define SLIMEVR_PACKETS_H_

#include <cstdint>

#include "../consts.h"
#include "../sensors/sensor.h"

enum class SendPacketType : uint8_t {
	HeartBeat = 0,
	//  Rotation = 1,
	//  Gyro = 2,
	Handshake = 3,
	Accel = 4,
	// Mag = 5,
	// RawCalibrationData = 6,
	// CalibrationFinished = 7,
	// RawMagnetometer = 9,
	Serial = 11,
	BatteryLevel = 12,
	Tap = 13,
	Error = 14,
	SensorInfo = 15,
	// Rotation2 = 16,
	RotationData = 17,
	MagnetometerAccuracy = 18,
	SignalStrength = 19,
	Temperature = 20,
	// UserAction = 21,
	FeatureFlags = 22,
	// RotationAcceleration = 23,
	AcknowledgeConfigChange = 24,
	FlexData = 26,
	Bundle = 100,
	Inspection = 105,
};

enum class ReceivePacketType : uint8_t {
	HeartBeat = 1,
	Vibrate = 2,
	Handshake = 3,
	Command = 4,
	Config = 8,
	PingPong = 10,
	SensorInfo = 15,
	FeatureFlags = 22,
	SetConfigFlag = 25,
};

enum class InspectionPacketType : uint8_t {
	RawImuData = 1,
	FusedImuData = 2,
	CorrectionData = 3,
};

enum class InspectionDataType : uint8_t {
	Int = 1,
	Float = 2,
};

// From the SH-2 interface that BNO08x use.
enum class PacketErrorCode : uint8_t {
	NOT_APPLICABLE = 0,
	POWER_ON_RESET = 1,
	INTERNAL_SYSTEM_RESET = 2,
	WATCHDOG_TIMEOUT = 3,
	EXTERNAL_RESET = 4,
	OTHER = 5,
};

#pragma pack(push, 1)

struct AccelPacket {
	float x;
	float y;
	float z;
	uint8_t sensorId;
};

struct BatteryLevelPacket {
	float batteryVoltage;
	float batteryPercentage;
};

struct TapPacket {
	uint8_t sensorId;
	uint8_t value;
};

struct ErrorPacket {
	uint8_t sensorId;
	uint8_t error;
};

struct SensorInfoPacket {
	uint8_t sensorId;
	SensorStatus sensorState;
	SensorTypeID sensorType;
	uint16_t sensorConfigData;
	bool hasCompletedRestCalibration;
	SensorPosition sensorPosition;
	SensorDataType sensorDataType;
	// ADD NEW FIELDS ABOVE THIS COMMENT ^^^^^^^^
	// WARNING! Only for debug purposes and SHOULD ALWAYS BE LAST IN THE PACKET.
	// It WILL BE REMOVED IN THE FUTURE
	// Send TPS
	float tpsCounterAveragedTps;
	float dataCounterAveragedTps;
};

struct RotationDataPacket {
	uint8_t sensorId;
	uint8_t dataType;
	float x;
	float y;
	float z;
	float w;
	uint8_t accuracyInfo;
};

struct MagnetometerAccuracyPacket {
	uint8_t sensorId;
	float accuracyInfo;
};

struct SignalStrengthPacket {
	uint8_t sensorId;
	uint8_t signalStrength;
};

struct TemperaturePacket {
	uint8_t sensorId;
	float temperature;
};

struct AcknowledgeConfigChangePacket {
	uint8_t sensorId;
	uint16_t configType;
};

struct FlexDataPacket {
	uint8_t sensorId;
	float flexLevel;
};

struct IntRawImuDataInspectionPacket {
	InspectionPacketType inspectionPacketType;
	uint8_t sensorId;
	InspectionDataType inspectionDataType;

	uint32_t rX;
	uint32_t rY;
	uint32_t rZ;
	uint8_t rA;

	uint32_t aX;
	uint32_t aY;
	uint32_t aZ;
	uint8_t aA;

	uint32_t mX;
	uint32_t mY;
	uint32_t mZ;
	uint8_t mA;
};

struct FloatRawImuDataInspectionPacket {
	InspectionPacketType inspectionPacketType;
	uint8_t sensorId;
	InspectionDataType inspectionDataType;

	float rX;
	float rY;
	float rZ;
	uint8_t rA;

	float aX;
	float aY;
	float aZ;
	uint8_t aA;

	float mX;
	float mY;
	float mZ;
	uint8_t mA;
};

struct SetConfigFlagPacket {
	uint8_t sensorId;
	uint16_t flagId;
	bool newState;
};

#pragma pack(pop)

#endif  // SLIMEVR_PACKETS_H_
