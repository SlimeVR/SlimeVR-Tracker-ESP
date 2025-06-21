/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2023 SlimeVR Contributors

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

#include "connection.h"

#include "GlobalVars.h"
#include "logging/Logger.h"
#include "packets.h"

#define TIMEOUT 3000UL

template <typename T>
uint8_t* convert_to_chars(T src, uint8_t* target) {
	auto* rawBytes = reinterpret_cast<uint8_t*>(&src);
	std::memcpy(target, rawBytes, sizeof(T));
	std::reverse(target, target + sizeof(T));
	return target;
}

namespace SlimeVR::Network {

bool Connection::beginPacket() {
	if (m_IsBundle) {
		m_BundlePacketPosition = 0;
		return true;
	}

	int r = m_UDP.beginPacket(m_ServerHost, m_ServerPort);
	if (r == 0) {
		// This *technically* should *never* fail, since the underlying UDP
		// library just returns 1.

		m_Logger.warn("UDP beginPacket() failed");
	}

	return r > 0;
}

bool Connection::endPacket() {
	if (m_IsBundle) {
		uint32_t innerPacketSize = m_BundlePacketPosition;

		MUST_TRANSFER_BOOL((innerPacketSize > 0));

		m_IsBundle = false;

		if (m_BundlePacketInnerCount == 0) {
			sendPacketType(SendPacketType::Bundle);
			sendPacketNumber();
		}
		sendShort(innerPacketSize);
		sendBytes(m_Packet, innerPacketSize);

		m_BundlePacketInnerCount++;
		m_IsBundle = true;
		return true;
	}

	int r = m_UDP.endPacket();
	if (r == 0) {
		// This is usually just `ERR_ABRT` but the UDP client doesn't expose
		// the full error code to us, so we just have to live with it.

		// m_Logger.warn("UDP endPacket() failed");
	}

	return r > 0;
}

bool Connection::beginBundle() {
	MUST_TRANSFER_BOOL(m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT));
	MUST_TRANSFER_BOOL(m_Connected);
	MUST_TRANSFER_BOOL(!m_IsBundle);
	MUST_TRANSFER_BOOL(beginPacket());

	m_IsBundle = true;
	m_BundlePacketInnerCount = 0;
	return true;
}

bool Connection::endBundle() {
	MUST_TRANSFER_BOOL(m_IsBundle);

	m_IsBundle = false;

	MUST_TRANSFER_BOOL((m_BundlePacketInnerCount > 0));

	return endPacket();
}

size_t Connection::write(const uint8_t* buffer, size_t size) {
	if (m_IsBundle) {
		if (m_BundlePacketPosition + size > sizeof(m_Packet)) {
			return 0;
		}
		memcpy(m_Packet + m_BundlePacketPosition, buffer, size);
		m_BundlePacketPosition += size;
		return size;
	}
	return m_UDP.write(buffer, size);
}

size_t Connection::write(uint8_t byte) { return write(&byte, 1); }

bool Connection::sendFloat(float f) {
	convert_to_chars(f, m_Buf);

	return write(m_Buf, sizeof(f)) != 0;
}

bool Connection::sendByte(uint8_t c) { return write(&c, 1) != 0; }

bool Connection::sendShort(uint16_t i) {
	convert_to_chars(i, m_Buf);

	return write(m_Buf, sizeof(i)) != 0;
}

bool Connection::sendInt(uint32_t i) {
	convert_to_chars(i, m_Buf);

	return write(m_Buf, sizeof(i)) != 0;
}

bool Connection::sendLong(uint64_t l) {
	convert_to_chars(l, m_Buf);

	return write(m_Buf, sizeof(l)) != 0;
}

bool Connection::sendBytes(const uint8_t* c, size_t length) {
	return write(c, length) != 0;
}

bool Connection::sendPacketNumber() {
	if (m_IsBundle) {
		return true;
	}

	uint64_t pn = m_PacketNumber++;

	return sendLong(pn);
}

bool Connection::sendShortString(const char* str) {
	uint8_t size = strlen(str);

	MUST_TRANSFER_BOOL(sendByte(size));
	MUST_TRANSFER_BOOL(sendBytes((const uint8_t*)str, size));

	return true;
}

bool Connection::sendPacketType(SendPacketType type) {
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));

	return sendByte(static_cast<uint8_t>(type));
}

bool Connection::sendLongString(const char* str) {
	int size = strlen(str);

	MUST_TRANSFER_BOOL(sendInt(size));

	return sendBytes((const uint8_t*)str, size);
}

int Connection::getWriteError() { return m_UDP.getWriteError(); }

// PACKET_HEARTBEAT 0
void Connection::sendHeartbeat() {
	MUST(m_Connected);
	MUST(sendPacketCallback(SendPacketType::HeartBeat, []() { return true; }));
}

// PACKET_ACCEL 4
void Connection::sendSensorAcceleration(uint8_t sensorId, Vector3 vector) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::Accel,
		AccelPacket{
			.x = vector.x,
			.y = vector.y,
			.z = vector.z,
			.sensorId = sensorId,
		}
	));
}

// PACKET_BATTERY_LEVEL 12
void Connection::sendBatteryLevel(float batteryVoltage, float batteryPercentage) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::BatteryLevel,
		BatteryLevelPacket{
			.batteryVoltage = batteryVoltage,
			.batteryPercentage = batteryPercentage,
		}
	));
}

// PACKET_TAP 13
void Connection::sendSensorTap(uint8_t sensorId, uint8_t value) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::Tap,
		TapPacket{
			.sensorId = sensorId,
			.value = value,
		}
	));
}

// PACKET_ERROR 14
void Connection::sendSensorError(uint8_t sensorId, uint8_t error) {
	MUST(m_Connected);
	sendPacket(
		SendPacketType::Error,
		ErrorPacket{
			.sensorId = sensorId,
			.error = error,
		}
	);
}

// PACKET_SENSOR_INFO 15
void Connection::sendSensorInfo(Sensor& sensor) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::SensorInfo,
		SensorInfoPacket{
			.sensorId = sensor.getSensorId(),
			.sensorState = sensor.getSensorState(),
			.sensorType = sensor.getSensorType(),
			.sensorConfigData = sensor.getSensorConfigData(),
			.hasCompletedRestCalibration = sensor.hasCompletedRestCalibration(),
			.sensorPosition = sensor.getSensorPosition(),
			.sensorDataType = sensor.getDataType(),

			.tpsCounterAveragedTps = sensor.m_tpsCounter.getAveragedTPS(),
			.dataCounterAveragedTps = sensor.m_dataCounter.getAveragedTPS(),
		}
	));
}

// PACKET_ROTATION_DATA 17
void Connection::sendRotationData(
	uint8_t sensorId,
	Quat* const quaternion,
	uint8_t dataType,
	uint8_t accuracyInfo
) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::RotationData,
		RotationDataPacket{
			.sensorId = sensorId,
			.dataType = dataType,
			.x = quaternion->x,
			.y = quaternion->y,
			.z = quaternion->z,
			.w = quaternion->w,
			.accuracyInfo = accuracyInfo,
		}
	));
}

// PACKET_MAGNETOMETER_ACCURACY 18
void Connection::sendMagnetometerAccuracy(uint8_t sensorId, float accuracyInfo) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::MagnetometerAccuracy,
		MagnetometerAccuracyPacket{
			.sensorId = sensorId,
			.accuracyInfo = accuracyInfo,
		}
	));
}

// PACKET_SIGNAL_STRENGTH 19
void Connection::sendSignalStrength(uint8_t signalStrength) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::SignalStrength,
		SignalStrengthPacket{
			.sensorId = 255,
			.signalStrength = signalStrength,
		}
	));
}

// PACKET_TEMPERATURE 20
void Connection::sendTemperature(uint8_t sensorId, float temperature) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::Temperature,
		TemperaturePacket{
			.sensorId = sensorId,
			.temperature = temperature,
		}
	));
}

// PACKET_FEATURE_FLAGS 22
void Connection::sendFeatureFlags() {
	MUST(m_Connected);
	sendPacketCallback(SendPacketType::FeatureFlags, [&]() {
		return write(FirmwareFeatures::flags.data(), FirmwareFeatures::flags.size());
	});
}

// PACKET_ACKNOWLEDGE_CONFIG_CHANGE 24

void Connection::sendAcknowledgeConfigChange(
	uint8_t sensorId,
	SensorToggles configType
) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::AcknowledgeConfigChange,
		AcknowledgeConfigChangePacket{
			.sensorId = sensorId,
			.configType = configType,
		}
	));
}

void Connection::sendTrackerDiscovery() {
	MUST(!m_Connected);
	MUST(sendPacketCallback(
		SendPacketType::Handshake,
		[&]() {
			uint8_t mac[6];
			WiFi.macAddress(mac);

			MUST_TRANSFER_BOOL(sendInt(BOARD));
			// This is kept for backwards compatibility,
			// but the latest SlimeVR server will not initialize trackers
			// with firmware build > 8 until it recieves a sensor info packet
			MUST_TRANSFER_BOOL(sendInt(static_cast<int>(sensorManager.getSensorType(0)))
			);
			MUST_TRANSFER_BOOL(sendInt(HARDWARE_MCU));
			// Backwards compatibility, unused IMU data
			MUST_TRANSFER_BOOL(sendInt(0));
			MUST_TRANSFER_BOOL(sendInt(0));
			MUST_TRANSFER_BOOL(sendInt(0));
			MUST_TRANSFER_BOOL(sendInt(PROTOCOL_VERSION));
			MUST_TRANSFER_BOOL(sendShortString(FIRMWARE_VERSION));
			// MAC address string
			MUST_TRANSFER_BOOL(sendBytes(mac, 6));
			// Tracker type to hint the server if it's a glove or normal tracker or
			// something else
			MUST_TRANSFER_BOOL(sendByte(static_cast<uint8_t>(TRACKER_TYPE)));
			return true;
		},
		0
	));
}

// PACKET_FLEX_DATA 24
void Connection::sendFlexData(uint8_t sensorId, float flexLevel) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::FlexData,
		FlexDataPacket{
			.sensorId = sensorId,
			.flexLevel = flexLevel,
		}
	));
}

#if ENABLE_INSPECTION
void Connection::sendInspectionRawIMUData(
	uint8_t sensorId,
	int16_t rX,
	int16_t rY,
	int16_t rZ,
	uint8_t rA,
	int16_t aX,
	int16_t aY,
	int16_t aZ,
	uint8_t aA,
	int16_t mX,
	int16_t mY,
	int16_t mZ,
	uint8_t mA
) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::Inspection,
		IntRawImuDataInspectionPacket{
			.inspectionPacketType = InspectionPacketType::RawImuData,
			.sensorId = sensorId,
			.inspectionDataType = InspectionDataType::Int,

			.rX = static_cast<uint32_t>(rX),
			.rY = static_cast<uint32_t>(rY),
			.rZ = static_cast<uint32_t>(rZ),
			.rA = rA,

			.aX = static_cast<uint32_t>(aX),
			.aY = static_cast<uint32_t>(aY),
			.aZ = static_cast<uint32_t>(aZ),
			.aA = aA,

			.mX = static_cast<uint32_t>(mX),
			.mY = static_cast<uint32_t>(mY),
			.mZ = static_cast<uint32_t>(mZ),
			.mA = mA,
		}
	))
}

void Connection::sendInspectionRawIMUData(
	uint8_t sensorId,
	float rX,
	float rY,
	float rZ,
	uint8_t rA,
	float aX,
	float aY,
	float aZ,
	uint8_t aA,
	float mX,
	float mY,
	float mZ,
	uint8_t mA
) {
	MUST(m_Connected);
	MUST(sendPacket(
		SendPacketType::Inspection,
		FloatRawImuDataInspectionPacket{
			.inspectionPacketType = InspectionPacketType::RawImuData,
			.sensorId = sensorId,
			.inspectionDataType = InspectionDataType::Float,

			.rX = rX,
			.rY = rY,
			.rZ = rZ,
			.rA = rA,

			.aX = aX,
			.aY = aY,
			.aZ = aZ,
			.aA = aA,

			.mX = mX,
			.mY = mY,
			.mZ = mZ,
			.mA = mA,
		}
	));
}
#endif

void Connection::returnLastPacket(int len) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendBytes(m_Packet, len));

	MUST(endPacket());
}

void Connection::updateSensorState(std::vector<std::unique_ptr<Sensor>>& sensors) {
	if (millis() - m_LastSensorInfoPacketTimestamp <= 1000) {
		return;
	}

	m_LastSensorInfoPacketTimestamp = millis();

	for (int i = 0; i < (int)sensors.size(); i++) {
		if (isSensorStateUpdated(i, sensors[i])) {
			sendSensorInfo(*sensors[i]);
		}
	}
}

void Connection::maybeRequestFeatureFlags() {
	if (m_ServerFeatures.isAvailable() || m_FeatureFlagsRequestAttempts >= 15) {
		return;
	}

	if (millis() - m_FeatureFlagsRequestTimestamp < 500) {
		return;
	}

	sendFeatureFlags();
	m_FeatureFlagsRequestTimestamp = millis();
	m_FeatureFlagsRequestAttempts++;
}

bool Connection::isSensorStateUpdated(int i, std::unique_ptr<Sensor>& sensor) {
	return (m_AckedSensorState[i] != sensor->getSensorState()
			|| m_AckedSensorCalibration[i] != sensor->hasCompletedRestCalibration()
			|| m_AckedSensorConfigData[i] != sensor->getSensorConfigData())
		&& sensor->getSensorType() != SensorTypeID::Unknown
		&& sensor->getSensorType() != SensorTypeID::Empty;
}

void Connection::searchForServer() {
	while (true) {
		int packetSize = m_UDP.parsePacket();
		if (!packetSize) {
			break;
		}

		// receive incoming UDP packets
		[[maybe_unused]] int len = m_UDP.read(m_Packet, sizeof(m_Packet));

#ifdef DEBUG_NETWORK
		m_Logger.trace(
			"Received %d bytes from %s, port %d",
			packetSize,
			m_UDP.remoteIP().toString().c_str(),
			m_UDP.remotePort()
		);
		m_Logger.traceArray("UDP packet contents: ", m_Packet, len);
#endif

		// Handshake is different, it has 3 in the first byte, not the 4th, and data
		// starts right after
		if (static_cast<ReceivePacketType>(m_Packet[0])
			== ReceivePacketType::Handshake) {
			if (strncmp((char*)m_Packet + 1, "Hey OVR =D 5", 12) != 0) {
				m_Logger.error("Received invalid handshake packet");
				continue;
			}

			m_ServerHost = m_UDP.remoteIP();
			m_ServerPort = m_UDP.remotePort();
			m_LastPacketTimestamp = millis();
			m_Connected = true;

			m_FeatureFlagsRequestAttempts = 0;
			m_ServerFeatures = ServerFeatures{};

			statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, false);
			ledManager.off();

			m_Logger.debug(
				"Handshake successful, server is %s:%d",
				m_UDP.remoteIP().toString().c_str(),
				m_UDP.remotePort()
			);

			break;
		}
	}

	auto now = millis();

	// This makes the LED blink for 20ms every second
	if (m_LastConnectionAttemptTimestamp + 1000 < now) {
		m_LastConnectionAttemptTimestamp = now;
		m_Logger.info("Searching for the server on the local network...");
		Connection::sendTrackerDiscovery();
		ledManager.on();
	} else if (m_LastConnectionAttemptTimestamp + 20 < now) {
		ledManager.off();
	}
}

void Connection::reset() {
	m_Connected = false;
	std::fill(
		m_AckedSensorState,
		m_AckedSensorState + MAX_SENSORS_COUNT,
		SensorStatus::SENSOR_OFFLINE
	);
	std::fill(
		m_AckedSensorCalibration,
		m_AckedSensorCalibration + MAX_SENSORS_COUNT,
		false
	);
	std::fill(
		m_AckedSensorConfigData,
		m_AckedSensorConfigData + MAX_SENSORS_COUNT,
		SlimeVR::Configuration::SensorConfigBits{}
	);

	m_UDP.begin(m_ServerPort);

	statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);
}

void Connection::update() {
	if (!m_Connected) {
		searchForServer();
		return;
	}

	auto& sensors = sensorManager.getSensors();

	updateSensorState(sensors);
	maybeRequestFeatureFlags();

	if (m_LastPacketTimestamp + TIMEOUT < millis()) {
		statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);

		m_Connected = false;
		std::fill(
			m_AckedSensorState,
			m_AckedSensorState + MAX_SENSORS_COUNT,
			SensorStatus::SENSOR_OFFLINE
		);
		std::fill(
			m_AckedSensorCalibration,
			m_AckedSensorCalibration + MAX_SENSORS_COUNT,
			false
		);
		m_Logger.warn("Connection to server timed out");

		return;
	}

	int packetSize = m_UDP.parsePacket();
	if (!packetSize) {
		return;
	}

	m_LastPacketTimestamp = millis();
	int len = m_UDP.read(m_Packet, sizeof(m_Packet));

#ifdef DEBUG_NETWORK
	m_Logger.trace(
		"Received %d bytes from %s, port %d",
		packetSize,
		m_UDP.remoteIP().toString().c_str(),
		m_UDP.remotePort()
	);
	m_Logger.traceArray("UDP packet contents: ", m_Packet, len);
#else
	(void)packetSize;
#endif

	switch (static_cast<ReceivePacketType>(m_Packet[3])) {
		case ReceivePacketType::HeartBeat:
			sendHeartbeat();
			break;

		case ReceivePacketType::Vibrate:
			break;

		case ReceivePacketType::Handshake:
			// Assume handshake successful
			m_Logger.warn("Handshake received again, ignoring");
			break;

		case ReceivePacketType::Command:
			break;

		case ReceivePacketType::Config:
			break;

		case ReceivePacketType::PingPong:
			returnLastPacket(len);
			break;

		case ReceivePacketType::SensorInfo: {
			if (len < 6) {
				m_Logger.warn("Wrong sensor info packet");
				break;
			}

			SensorInfoPacket sensorInfoPacket;
			memcpy(&sensorInfoPacket, m_Packet + 4, sizeof(sensorInfoPacket));

			for (int i = 0; i < (int)sensors.size(); i++) {
				if (sensorInfoPacket.sensorId == sensors[i]->getSensorId()) {
					m_AckedSensorState[i] = sensorInfoPacket.sensorState;
					if (len < 12) {
						m_AckedSensorCalibration[i]
							= sensors[i]->hasCompletedRestCalibration();
						m_AckedSensorConfigData[i] = sensors[i]->getSensorConfigData();
						break;
					}
					m_AckedSensorCalibration[i]
						= sensorInfoPacket.hasCompletedRestCalibration;
					break;
				}
			}

			break;
		}
		case ReceivePacketType::FeatureFlags: {
			// Packet type (4) + Packet number (8) + flags (len - 12)
			if (len < 13) {
				m_Logger.warn("Invalid feature flags packet: too short");
				break;
			}

			bool hadFlags = m_ServerFeatures.isAvailable();

			uint32_t flagsLength = len - 12;
			m_ServerFeatures = ServerFeatures::from(&m_Packet[12], flagsLength);

			if (!hadFlags) {
#if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
				if (m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT)) {
					m_Logger.debug("Server supports packet bundling");
				}
#endif
			}

			break;
		}

		case ReceivePacketType::SetConfigFlag: {
			// Packet type (4) + Packet number (8) + sensor_id(1) + flag_id (2) + state
			// (1)
			if (len < 16) {
				m_Logger.warn("Invalid sensor config flag packet: too short");
				break;
			}

			SetConfigFlagPacket setConfigFlagPacket;
			memcpy(&setConfigFlagPacket, m_Packet + 12, sizeof(SetConfigFlagPacket));

			uint8_t sensorId = setConfigFlagPacket.sensorId;
			SensorToggles flag = setConfigFlagPacket.flag;
			bool newState = setConfigFlagPacket.newState;
			if (sensorId == UINT8_MAX) {
				for (auto& sensor : sensors) {
					sensor->setFlag(flag, newState);
				}
			} else {
				auto& sensors = sensorManager.getSensors();

				if (sensorId >= sensors.size()) {
					m_Logger.warn("Invalid sensor config flag packet: invalid sensor id"
					);
					break;
				}

				auto& sensor = sensors[sensorId];
				sensor->setFlag(flag, newState);
			}
			sendAcknowledgeConfigChange(sensorId, flag);
			configuration.save();
			break;
		}
	}
}

}  // namespace SlimeVR::Network
