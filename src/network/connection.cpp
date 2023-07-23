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
unsigned char* convert_to_chars(T src, unsigned char* target) {
	union uwunion {
		unsigned char c[sizeof(T)];
		T v;
	} un;
	un.v = src;
	for (size_t i = 0; i < sizeof(T); i++) {
		target[i] = un.c[sizeof(T) - i - 1];
	}
	return target;
}

template <typename T>
T convert_chars(unsigned char* const src) {
	union uwunion {
		unsigned char c[sizeof(T)];
		T v;
	} un;
	for (size_t i = 0; i < sizeof(T); i++) {
		un.c[i] = src[sizeof(T) - i - 1];
	}
	return un.v;
}

namespace SlimeVR {
namespace Network {

#define MUST_TRANSFER_BOOL(b) \
	if (!b)                   \
		return false;

#define MUST(b) \
	if (!b)     \
		return;

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
		m_UDP.write(m_BundlePacketPosition >> 8);
		m_UDP.write(m_BundlePacketPosition & 0xFF);
		m_UDP.write(m_Packet, m_BundlePacketPosition);
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
	if (!m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT)) {
		return false;
	}

	if (!m_Connected || m_IsBundle) {
		return false;
	}

	if (beginPacket()) {
		sendPacketType(PACKET_BUNDLE);
		sendPacketNumber();
		m_IsBundle = true;
		return true;
	}
	
	return false;
}

bool Connection::endBundle() {
	m_IsBundle = false;

	if (!m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT)) {
		return false;
	}

	return endPacket();
}

size_t Connection::write(const uint8_t *buffer, size_t size) {
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

size_t Connection::write(uint8_t byte) {
	return write(&byte, 1);
}

bool Connection::sendFloat(float f) {
	convert_to_chars(f, m_Buf);

	return write(m_Buf, sizeof(f)) != 0;
}

bool Connection::sendByte(uint8_t c) { return write(&c, 1) != 0; }

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

bool Connection::sendPacketType(uint8_t type) {
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));

	return sendByte(type);
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

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_HEARTBEAT));
	MUST(sendPacketNumber());

	MUST(endPacket());
}

// PACKET_ACCEL 4
void Connection::sendSensorAcceleration(uint8_t sensorId, float* vector) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ACCEL));
	MUST(sendPacketNumber());
	MUST(sendFloat(vector[0]));
	MUST(sendFloat(vector[1]));
	MUST(sendFloat(vector[2]));
	MUST(sendByte(sensorId));

	MUST(endPacket());
}

// PACKET_BATTERY_LEVEL 12
void Connection::sendBatteryLevel(float batteryVoltage, float batteryPercentage) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_BATTERY_LEVEL));
	MUST(sendPacketNumber());
	MUST(sendFloat(batteryVoltage));
	MUST(sendFloat(batteryPercentage));

	MUST(endPacket());
}

// PACKET_TAP 13
void Connection::sendSensorTap(uint8_t sensorId, uint8_t value) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_TAP));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(value));

	MUST(endPacket());
}

// PACKET_ERROR 14
void Connection::sendSensorError(uint8_t sensorId, uint8_t error) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ERROR));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(error));

	MUST(endPacket());
}

// PACKET_SENSOR_INFO 15
void Connection::sendSensorInfo(Sensor* sensor) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_SENSOR_INFO));
	MUST(sendPacketNumber());
	MUST(sendByte(sensor->getSensorId()));
	MUST(sendByte((uint8_t)sensor->getSensorState()));
	MUST(sendByte(sensor->getSensorType()));

	MUST(endPacket());
}

// PACKET_ROTATION_DATA 17
void Connection::sendRotationData(
	uint8_t sensorId,
	Quat* const quaternion,
	uint8_t dataType,
	uint8_t accuracyInfo
) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ROTATION_DATA));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(dataType));
	MUST(sendFloat(quaternion->x));
	MUST(sendFloat(quaternion->y));
	MUST(sendFloat(quaternion->z));
	MUST(sendFloat(quaternion->w));
	MUST(sendByte(accuracyInfo));

	MUST(endPacket());
}

// PACKET_MAGNETOMETER_ACCURACY 18
void Connection::sendMagnetometerAccuracy(uint8_t sensorId, float accuracyInfo) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_MAGNETOMETER_ACCURACY));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendFloat(accuracyInfo));

	MUST(endPacket());
}

// PACKET_SIGNAL_STRENGTH 19
void Connection::sendSignalStrength(uint8_t signalStrength) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_SIGNAL_STRENGTH));
	MUST(sendPacketNumber());
	MUST(sendByte(255));
	MUST(sendByte(signalStrength));

	MUST(endPacket());
}

// PACKET_TEMPERATURE 20
void Connection::sendTemperature(uint8_t sensorId, float temperature) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_TEMPERATURE));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendFloat(temperature));

	MUST(endPacket());
}

// PACKET_FEATURE_FLAGS 22
void Connection::sendFeatureFlags() {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_FEATURE_FLAGS));
	MUST(sendPacketNumber());
	MUST(write(FirmwareFeatures::flags.data(), FirmwareFeatures::flags.size()));

	MUST(endPacket());
}

void Connection::sendTrackerDiscovery() {
	MUST(!m_Connected);

	uint8_t mac[6];
	WiFi.macAddress(mac);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_HANDSHAKE));
	// Packet number is always 0 for handshake
	MUST(sendLong(0));
	MUST(sendInt(BOARD));
	// This is kept for backwards compatibility,
	// but the latest SlimeVR server will not initialize trackers
	// with firmware build > 8 until it recieves a sensor info packet
	MUST(sendInt(IMU));
	MUST(sendInt(HARDWARE_MCU));
	MUST(sendInt(0));
	MUST(sendInt(0));
	MUST(sendInt(0));
	MUST(sendInt(FIRMWARE_BUILD_NUMBER));
	MUST(sendShortString(FIRMWARE_VERSION));
	// MAC address string
	MUST(sendBytes(mac, 6));

	MUST(endPacket());
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

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_INSPECTION));
	MUST(sendPacketNumber());

	MUST(sendByte(PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA));

	MUST(sendByte(sensorId));
	MUST(sendByte(PACKET_INSPECTION_DATATYPE_INT));

	MUST(sendInt(rX));
	MUST(sendInt(rY));
	MUST(sendInt(rZ));
	MUST(sendByte(rA));

	MUST(sendInt(aX));
	MUST(sendInt(aY));
	MUST(sendInt(aZ));
	MUST(sendByte(aA));

	MUST(sendInt(mX));
	MUST(sendInt(mY));
	MUST(sendInt(mZ));
	MUST(sendByte(mA));

	MUST(endPacket());
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

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_INSPECTION));
	MUST(sendPacketNumber());

	MUST(sendByte(PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA));

	MUST(sendByte(sensorId));
	MUST(sendByte(PACKET_INSPECTION_DATATYPE_FLOAT));

	MUST(sendFloat(rX));
	MUST(sendFloat(rY));
	MUST(sendFloat(rZ));
	MUST(sendByte(rA));

	MUST(sendFloat(aX));
	MUST(sendFloat(aY));
	MUST(sendFloat(aZ));
	MUST(sendByte(aA));

	MUST(sendFloat(mX));
	MUST(sendFloat(mY));
	MUST(sendFloat(mZ));
	MUST(sendByte(mA));

	MUST(endPacket());
}
#endif

void Connection::returnLastPacket(int len) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendBytes(m_Packet, len));

	MUST(endPacket());
}

void Connection::updateSensorState(Sensor* const sensor1, Sensor* const sensor2) {
	if (millis() - m_LastSensorInfoPacketTimestamp <= 1000) {
		return;
	}

	m_LastSensorInfoPacketTimestamp = millis();

	if (m_AckedSensorState1 != sensor1->getSensorState()) {
		sendSensorInfo(sensor1);
	}

	if (m_AckedSensorState2 != sensor2->getSensorState()) {
		sendSensorInfo(sensor2);
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

void Connection::searchForServer() {
	while (true) {
		int packetSize = m_UDP.parsePacket();
		if (!packetSize) {
			break;
		}

		// receive incoming UDP packets
		int len = m_UDP.read(m_Packet, sizeof(m_Packet));

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
		if (m_Packet[0] == PACKET_HANDSHAKE) {
			if (strncmp((char*)m_Packet + 1, "Hey OVR =D 5", 12) != 0) {
				m_Logger.error("Received invalid handshake packet");
				continue;
			}

			m_ServerHost = m_UDP.remoteIP();
			m_ServerPort = m_UDP.remotePort();
			m_LastPacketTimestamp = millis();
			m_Connected = true;
			
			m_FeatureFlagsRequestAttempts = 0;
			m_ServerFeatures = ServerFeatures { };

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
	m_AckedSensorState1 = SensorStatus::SENSOR_OFFLINE;
	m_AckedSensorState2 = SensorStatus::SENSOR_OFFLINE;

	m_UDP.begin(m_ServerPort);

	statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);
}

void Connection::update() {
	auto sensor1 = sensorManager.getFirst();
	auto sensor2 = sensorManager.getSecond();

	updateSensorState(sensor1, sensor2);
	maybeRequestFeatureFlags();

	if (!m_Connected) {
		searchForServer();
		return;
	}

	if (m_LastPacketTimestamp + TIMEOUT < millis()) {
		statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);

		m_Connected = false;
		m_AckedSensorState1 = SensorStatus::SENSOR_OFFLINE;
		m_AckedSensorState2 = SensorStatus::SENSOR_OFFLINE;
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

	switch (convert_chars<int>(m_Packet)) {
		case PACKET_RECEIVE_HEARTBEAT:
			sendHeartbeat();
			break;

		case PACKET_RECEIVE_VIBRATE:
			break;

		case PACKET_RECEIVE_HANDSHAKE:
			// Assume handshake successful
			m_Logger.warn("Handshake received again, ignoring");
			break;

		case PACKET_RECEIVE_COMMAND:
			break;

		case PACKET_CONFIG:
			break;

		case PACKET_PING_PONG:
			returnLastPacket(len);
			break;

		case PACKET_SENSOR_INFO:
			if (len < 6) {
				m_Logger.warn("Wrong sensor info packet");
				break;
			}

			if (m_Packet[4] == sensor1->getSensorId()) {
				m_AckedSensorState1 = (SensorStatus)m_Packet[5];
			} else if (m_Packet[4] == sensor2->getSensorId()) {
				m_AckedSensorState2 = (SensorStatus)m_Packet[5];
			}

			break;

		case PACKET_FEATURE_FLAGS:
			// Packet type (4) + Packet number (8) + flags (len - 12)
			if (len < 13) {
				m_Logger.warn("Invalid feature flags packet: too short");
				break;
			}

			uint32_t flagsLength = len - 12;

			if (m_ServerFeatures.isAvailable() || flagsLength <= 0) {
				break;
			}
			
			m_ServerFeatures = ServerFeatures::from(&m_Packet[12], flagsLength);

			#if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
				if (m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT)) {
					m_Logger.debug("Server supports packet bundling");
				}
			#endif

			break;
	}
}

}  // namespace Network
}  // namespace SlimeVR
