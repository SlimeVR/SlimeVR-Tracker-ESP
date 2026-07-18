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

#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>

#include <cstring>
#include <memory>

#include "../CommunicationStrategy.h"
#include "../featureflags.h"
#include "network/wifi/WiFiConnection.h"
#include "network/wifi/WiFiPackets.h"
#include "utils/Timeout.h"

namespace SlimeVR::Network::WiFiComms {

class WiFiCommunication final : public CommunicationStrategy {
public:
	explicit WiFiCommunication(WiFiConnection& connection);

	void init() final;
	void tick() final;
	[[nodiscard]] bool isConnected() const final;

	bool beginComms() final;
	bool endComms() final;

	void sendBatteryLevel(float batteryVoltage, float batteryPercentage) final;
	void sendFeatureFlags() final;
	void sendSignalStrength(uint8_t signalStrength) final;

	void sendAcceleration(uint8_t sensorId, Vector3 accel) final;
	void sendRotation(uint8_t sensorId, const Quat& quaternion, uint8_t accuracyInfo)
		final;
	void sendSensorTap(uint8_t sensorId, uint8_t value) final;
	void sendSensorError(uint8_t sensorId, SensorErrorCode error) final;
	void sendTemperature(uint8_t sensorId, float temperature) final;
	void sendFlexData(uint8_t sensorId, float flexLevel) final;

	void sendRawIMUData(
		uint8_t sensorId,
		std::array<int16_t, 3>& gyro,
		std::array<int16_t, 3>& accel,
		std::array<int16_t, 3>& mag
	) final;
	void sendRawIMUData(
		uint8_t sensorId,
		std::array<float, 3>& gyro,
		std::array<float, 3>& accel,
		std::array<float, 3>& mag
	) final;

	[[nodiscard]] std::string getAddressRepresentation() const final;
	[[nodiscard]] int getState() const final;

private:
	constexpr static uint64_t ServerResponseTimeoutMillis = 3000;
	constexpr static uint64_t RssiReportIntervalMillis = 2000;
	constexpr static uint64_t SensorStateSendIntervalMillis = 1000;
	constexpr static uint64_t FeatureFlagsRequestIntervalMillis = 500;

	void reset();

	void sendHeartbeat();
	void sendTrackerDiscovery();
	void sendSensorInfo(::Sensor& sensor);
	void sendAcknowledgeConfigChange(uint8_t sensorId, SensorToggles configType);

	void returnLastPacket(size_t length);

	void updateSensorState(std::vector<std::unique_ptr<Sensor>>& sensors);
	void maybeRequestFeatureFlags();
	bool isSensorStateUpdated(size_t i, std::unique_ptr<::Sensor>& sensor);

	void searchForServer();

	WiFiConnection& wifiConnection;
	WiFiPackets wifiPackets;

	SlimeVR::Logging::Logger logger = SlimeVR::Logging::Logger("WiFiComms");

	SensorStatus ackedSensorState[MAX_SENSORS_COUNT] = {SensorStatus::SENSOR_OFFLINE};
	SlimeVR::Configuration::SensorConfigBits ackedSensorConfigData[MAX_SENSORS_COUNT]
		= {};
	bool ackedSensorCalibration[MAX_SENSORS_COUNT] = {false};
	Timeout sensorStateSendTimeout{SensorStateSendIntervalMillis};

	uint8_t featureFlagsRequestAttempts = 0;
	Timeout featureFlagsRequestTimeout{FeatureFlagsRequestIntervalMillis};
	ServerFeatures serverFeatures{};

	bool connectedToNetwork = false;
	bool connectedToServer = false;

	Timeout serverResponseTimeout{ServerResponseTimeoutMillis};

	uint8_t inPacket[128];

	Timeout rssiTimeout{RssiReportIntervalMillis};

	uint64_t lastConnectionAttemptMillis = millis();
};

}  // namespace SlimeVR::Network::WiFiComms
