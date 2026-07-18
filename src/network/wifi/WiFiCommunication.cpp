/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 Gorbit99 & SlimeVR Contributors

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

#include "WiFiCommunication.h"

#include <memory>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#endif

#include "GlobalVars.h"
#include "network/featureflags.h"
#include "sensors/sensor.h"

namespace SlimeVR::Network::WiFiComms {

WiFiCommunication::WiFiCommunication(WiFiConnection& connection)
	: wifiConnection{connection}
	, wifiPackets{connection} {}

void WiFiCommunication::init() {}

void WiFiCommunication::tick() {
	wifiConnection.tick();

	bool wasConnectedToNetwork = connectedToNetwork;
	connectedToNetwork = wifiConnection.isConnected();

	if (!connectedToNetwork) {
		return;
	}

	if (!wasConnectedToNetwork) {
		reset();
		wasConnectedToNetwork = true;
	}

	if (!connectedToServer) {
		searchForServer();
		return;
	}

	auto& sensors = sensorManager.getSensors();

	updateSensorState(sensors);
	maybeRequestFeatureFlags();

	if (serverResponseTimeout.elapsed()) {
		statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);

		connectedToServer = false;
		std::fill(
			ackedSensorState,
			ackedSensorState + MAX_SENSORS_COUNT,
			SensorStatus::SENSOR_OFFLINE
		);
		std::fill(
			ackedSensorCalibration,
			ackedSensorCalibration + MAX_SENSORS_COUNT,
			false
		);
		logger.warn("Connection to server timed out");

		// Reset server address to broadcast if disconnected
		wifiConnection.setIPAddress(IPAddress(255, 255, 255, 255));

		return;
	}

	if (rssiTimeout.elapsed()) {
		uint8_t signalStrength = WiFi.RSSI();
		sendSignalStrength(signalStrength);
		rssiTimeout.restart();
	}

	size_t packetSize = wifiConnection.readUDPPacket(inPacket, sizeof(inPacket));
	if (packetSize == 0) {
		return;
	}

	if (static_cast<ReceivePacketType>(inPacket[3]) == ReceivePacketType::Handshake) {
		logger.warn("Handshake received again, ignoring");
		return;
	}

	serverResponseTimeout.restart();
	switch (static_cast<ReceivePacketType>(inPacket[3])) {
		case ReceivePacketType::HeartBeat:
			sendHeartbeat();
			break;

		case ReceivePacketType::Vibrate:
			break;

		case ReceivePacketType::Handshake:
			// handled above
			break;

		case ReceivePacketType::Command:
			break;

		case ReceivePacketType::Config:
			break;

		case ReceivePacketType::PingPong:
			returnLastPacket(packetSize);
			break;

		case ReceivePacketType::SensorInfo: {
			if (packetSize < 6) {
				logger.warn("Wrong sensor info packet");
				break;
			}

			SensorInfoPacket sensorInfoPacket;
			memcpy(&sensorInfoPacket, inPacket + 4, sizeof(sensorInfoPacket));

			for (int i = 0; i < (int)sensors.size(); i++) {
				if (sensorInfoPacket.sensorId == sensors[i]->getSensorId()) {
					ackedSensorState[i] = sensorInfoPacket.sensorState;
					if (packetSize < 12) {
						ackedSensorCalibration[i]
							= sensors[i]->hasCompletedRestCalibration();
						ackedSensorConfigData[i] = sensors[i]->getSensorConfigData();
						break;
					}
					ackedSensorCalibration[i]
						= sensorInfoPacket.hasCompletedRestCalibration;
					break;
				}
			}

			break;
		}
		case ReceivePacketType::FeatureFlags: {
			// Packet type (4) + Packet number (8) + flags (len - 12)
			if (packetSize < 13) {
				logger.warn("Invalid feature flags packet: too short");
				break;
			}

			bool hadFlags = serverFeatures.isAvailable();

			uint32_t flagsLength = packetSize - 12;
			serverFeatures = ServerFeatures::from(&inPacket[12], flagsLength);

			if (!hadFlags) {
#if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
				if (serverFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT)) {
					logger.debug("Server supports packet bundling");
				}
#endif
			}

			break;
		}

		case ReceivePacketType::SetConfigFlag: {
			// Packet type (4) + Packet number (8) + sensor_id(1) + flag_id (2) + state
			// (1)
			if (packetSize < 16) {
				logger.warn("Invalid sensor config flag packet: too short");
				break;
			}

			SetConfigFlagPacket setConfigFlagPacket;
			memcpy(&setConfigFlagPacket, inPacket + 12, sizeof(SetConfigFlagPacket));

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
					logger.warn("Invalid sensor config flag packet: invalid sensor id");
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

[[nodiscard]] bool WiFiCommunication::isConnected() const { return connectedToServer; }

bool WiFiCommunication::beginComms() {
#if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
	MUST_TRANSFER_BOOL(connectedToServer);

	if (!serverFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT)) {
		return true;
	}

	wifiPackets.beginBundle();
#endif
	return true;
}

bool WiFiCommunication::endComms() {
#if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
	wifiPackets.endBundle();
#endif
	return true;
}

void WiFiCommunication::sendFeatureFlags() {
	MUST(connectedToServer);
	wifiPackets.sendPacketCallback(SendPacketType::FeatureFlags, [&]() {
		return wifiPackets.write(
			FirmwareFeatures::flags.data(),
			FirmwareFeatures::flags.size()
		);
	});
}

void WiFiCommunication::sendSignalStrength(uint8_t signalStrength) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(SignalStrengthPacket{
		.sensorId = 255,
		.signalStrength = signalStrength,
	});
}

void WiFiCommunication::sendBatteryLevel(
	float batteryVoltage,
	float batteryPercentage
) {
	MUST(connectedToServer);
	MUST(wifiPackets.sendPacket(BatteryLevelPacket{
		.batteryVoltage = batteryVoltage,
		.batteryPercentage = batteryPercentage,
	}));
}

void WiFiCommunication::sendAcceleration(uint8_t sensorId, Vector3 accel) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(AccelPacket{
		.x = accel.x,
		.y = accel.y,
		.z = accel.z,
		.sensorId = sensorId,
	});
}

void WiFiCommunication::sendRotation(
	uint8_t sensorId,
	const Quat& quaternion,
	uint8_t accuracyInfo
) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(RotationDataPacket{
		.sensorId = sensorId,
		.dataType = DATA_TYPE_NORMAL,
		.x = quaternion.x,
		.y = quaternion.y,
		.z = quaternion.z,
		.w = quaternion.w,
		.accuracyInfo = accuracyInfo,
	});
}

void WiFiCommunication::sendSensorTap(uint8_t sensorId, uint8_t value) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(TapPacket{
		.sensorId = sensorId,
		.value = value,
	});
}

void WiFiCommunication::sendSensorError(uint8_t sensorId, SensorErrorCode error) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(ErrorPacket{
		.sensorId = sensorId,
		.error = error,
	});
}

void WiFiCommunication::sendTemperature(uint8_t sensorId, float temperature) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(TemperaturePacket{
		.sensorId = sensorId,
		.temperature = temperature,
	});
}

void WiFiCommunication::sendFlexData(uint8_t sensorId, float flexLevel) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(FlexDataPacket{
		.sensorId = sensorId,
		.flexLevel = flexLevel,
	});
}

void WiFiCommunication::sendRawIMUData(
	uint8_t sensorId,
	std::array<int16_t, 3>& gyro,
	std::array<int16_t, 3>& accel,
	std::array<int16_t, 3>& mag
) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(IntRawImuDataInspectionPacket{
		.inspectionPacketType = InspectionPacketType::RawImuData,
		.sensorId = sensorId,
		.inspectionDataType = InspectionDataType::Int,

		.rX = static_cast<uint32_t>(gyro[0]),
		.rY = static_cast<uint32_t>(gyro[1]),
		.rZ = static_cast<uint32_t>(gyro[2]),
		.rA = 0,

		.aX = static_cast<uint32_t>(accel[0]),
		.aY = static_cast<uint32_t>(accel[1]),
		.aZ = static_cast<uint32_t>(accel[2]),
		.aA = 0,

		.mX = static_cast<uint32_t>(mag[0]),
		.mY = static_cast<uint32_t>(mag[1]),
		.mZ = static_cast<uint32_t>(mag[2]),
		.mA = 0,
	});
}

void WiFiCommunication::sendRawIMUData(
	uint8_t sensorId,
	std::array<float, 3>& gyro,
	std::array<float, 3>& accel,
	std::array<float, 3>& mag
) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(FloatRawImuDataInspectionPacket{
		.inspectionPacketType = InspectionPacketType::RawImuData,
		.sensorId = sensorId,
		.inspectionDataType = InspectionDataType::Float,

		.rX = gyro[0],
		.rY = gyro[1],
		.rZ = gyro[2],
		.rA = 0,

		.aX = accel[0],
		.aY = accel[1],
		.aZ = accel[2],
		.aA = 0,

		.mX = mag[0],
		.mY = mag[1],
		.mZ = mag[2],
		.mA = 0,
	});
}

void WiFiCommunication::sendHeartbeat() {
	MUST(connectedToServer);
	wifiPackets.sendPacketCallback(SendPacketType::HeartBeat, []() { return true; });
}

void WiFiCommunication::sendTrackerDiscovery() {
	MUST(!connectedToServer);
	wifiPackets.sendPacketCallback(
		SendPacketType::Handshake,
		[&]() {
			uint8_t mac[6];
			WiFi.macAddress(mac);

			MUST_TRANSFER_BOOL(wifiPackets.sendInt(BOARD));
			// This is kept for backwards compatibility,
			// but the latest SlimeVR server will not initialize trackers
			// with firmware build > 8 until it recieves a sensor info packet
			MUST_TRANSFER_BOOL(
				wifiPackets.sendInt(static_cast<int>(sensorManager.getSensorType(0)))
			);
			MUST_TRANSFER_BOOL(wifiPackets.sendInt(HARDWARE_MCU));
			// Backwards compatibility, unused IMU data
			MUST_TRANSFER_BOOL(wifiPackets.sendInt(0));
			MUST_TRANSFER_BOOL(wifiPackets.sendInt(0));
			MUST_TRANSFER_BOOL(wifiPackets.sendInt(0));
			MUST_TRANSFER_BOOL(wifiPackets.sendInt(PROTOCOL_VERSION));
			MUST_TRANSFER_BOOL(wifiPackets.sendShortString(FIRMWARE_VERSION));
			// MAC address string
			MUST_TRANSFER_BOOL(wifiPackets.write(mac, 6));
			// Tracker type to hint the server if it's a glove or normal tracker or
			// something else
			MUST_TRANSFER_BOOL(wifiPackets.sendByte(static_cast<uint8_t>(TRACKER_TYPE))
			);
			static_assert(std::string_view{VENDOR_NAME}.size() <= 255);
			MUST_TRANSFER_BOOL(wifiPackets.sendShortString(VENDOR_NAME));
			static_assert(std::string_view{VENDOR_URL}.size() <= 255);
			MUST_TRANSFER_BOOL(wifiPackets.sendShortString(VENDOR_URL));
			static_assert(std::string_view{PRODUCT_NAME}.size() <= 255);
			MUST_TRANSFER_BOOL(wifiPackets.sendShortString(PRODUCT_NAME));
			static_assert(std::string_view{UPDATE_ADDRESS}.size() <= 255);
			MUST_TRANSFER_BOOL(wifiPackets.sendShortString(UPDATE_ADDRESS));
			static_assert(std::string_view{UPDATE_NAME}.size() <= 255);
			MUST_TRANSFER_BOOL(wifiPackets.sendShortString(UPDATE_NAME));
			return true;
		},
		0
	);
}

void WiFiCommunication::sendSensorInfo(::Sensor& sensor) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(SensorInfoPacket{
		.sensorId = sensor.getSensorId(),
		.sensorState = sensor.getSensorState(),
		.sensorType = sensor.getSensorType(),
		.sensorConfigData = sensor.getSensorConfigData(),
		.hasCompletedRestCalibration = sensor.hasCompletedRestCalibration(),
		.sensorPosition = sensor.getSensorPosition(),
		.sensorDataType = sensor.getDataType(),

		.tpsCounterAveragedTps = sensor.m_tpsCounter.getAveragedTPS(),
		.dataCounterAveragedTps = sensor.m_dataCounter.getAveragedTPS(),
	});
}

void WiFiCommunication::sendAcknowledgeConfigChange(
	uint8_t sensorId,
	SensorToggles configType
) {
	MUST(connectedToServer);
	wifiPackets.sendPacket(AcknowledgeConfigChangePacket{
		.sensorId = sensorId,
		.configType = configType,
	});
}

void WiFiCommunication::returnLastPacket(size_t length) {
	MUST(connectedToServer);
	MUST(wifiPackets.beginPacket());
	MUST(wifiPackets.write(inPacket, length));
	MUST(wifiPackets.endPacket());
}

void WiFiCommunication::updateSensorState(std::vector<std::unique_ptr<Sensor>>& sensors
) {
	if (!sensorStateSendTimeout.elapsed()) {
		return;
	}

	sensorStateSendTimeout.restart();

	for (size_t i = 0; i < sensors.size(); i++) {
		if (isSensorStateUpdated(i, sensors[i])) {
			sendSensorInfo(*sensors[i]);
		}
	}
}

bool WiFiCommunication::isSensorStateUpdated(
	size_t i,
	std::unique_ptr<Sensor>& sensor
) {
	return (ackedSensorState[i] != sensor->getSensorState()
			|| ackedSensorCalibration[i] != sensor->hasCompletedRestCalibration()
			|| ackedSensorConfigData[i] != sensor->getSensorConfigData())
		&& sensor->getSensorType() != SensorTypeID::Unknown
		&& sensor->getSensorType() != SensorTypeID::Empty;
}

void WiFiCommunication::maybeRequestFeatureFlags() {
	if (serverFeatures.isAvailable() || featureFlagsRequestAttempts >= 15) {
		return;
	}

	if (!featureFlagsRequestTimeout.elapsed()) {
		return;
	}

	sendFeatureFlags();
	featureFlagsRequestTimeout.restart();
	featureFlagsRequestAttempts++;
}

std::string WiFiCommunication::getAddressRepresentation() const {
	auto string = wifiConnection.getIPAddress().toString();
	return std::string(string.c_str(), string.length());
}

int WiFiCommunication::getState() const {
	return static_cast<int>(wifiConnection.getState());
}

void WiFiCommunication::searchForServer() {
	while (true) {
		size_t packetSize = wifiConnection.readUDPPacket(inPacket, sizeof(inPacket));
		if (packetSize == 0) {
			break;
		}

		// Handshake is different, it has 3 in the first byte, not the 4th, and data
		// starts right after
		if (static_cast<ReceivePacketType>(inPacket[0])
			== ReceivePacketType::Handshake) {
			if (strncmp((char*)inPacket + 1, "Hey OVR =D 5", 12) != 0) {
				logger.error("Received invalid handshake packet");
				continue;
			}

			wifiConnection.acceptHandshake();
			serverResponseTimeout.restart();
			connectedToServer = true;

			featureFlagsRequestAttempts = 0;
			serverFeatures = ServerFeatures{};

			statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, false);
			ledManager.off();

			break;
		}
	}

	auto now = millis();

	// This makes the LED blink for 20ms every second
	if (now - lastConnectionAttemptMillis > 1000) {
		lastConnectionAttemptMillis = now;
		logger.info("Searching for the server on the local network...");
		sendTrackerDiscovery();
		ledManager.on();
	} else if (now - lastConnectionAttemptMillis > 20) {
		ledManager.off();
	}
}

void WiFiCommunication::reset() {
	connectedToServer = false;
	std::fill(
		ackedSensorState,
		ackedSensorState + MAX_SENSORS_COUNT,
		SensorStatus::SENSOR_OFFLINE
	);
	std::fill(
		ackedSensorCalibration,
		ackedSensorCalibration + MAX_SENSORS_COUNT,
		false
	);
	std::fill(
		ackedSensorConfigData,
		ackedSensorConfigData + MAX_SENSORS_COUNT,
		SlimeVR::Configuration::SensorConfigBits{}
	);

	wifiConnection.reset();

	statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);
}

}  // namespace SlimeVR::Network::WiFiComms
