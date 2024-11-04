/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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
#include "espnowconnection.h"
#include <algorithm>

#include "../GlobalVars.h"
#include "espnowmessages.h"
#include "espnowpackets.h"

#ifndef ESP_OK
#define ESP_OK 0
#endif

#define MACSTR        "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2ARGS(mac) mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]

namespace SlimeVR::Network {

#if ESP8266
void onReceive(uint8_t *senderMacAddress,
               uint8_t *data,
               uint8_t dataLen) {
    espnowConnection.handleMessage(senderMacAddress, data, dataLen);
}
#elif ESP32
void onReceive(const esp_now_recv_info_t *espnowInfo,
               const uint8_t *data,
               int dataLen) {
    espnowConnection.handleMessage(espnowInfo->src_addr, data, static_cast<uint8_t>(dataLen));
}
#endif

void ESPNowConnection::setup() {
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        m_Logger.fatal("Couldn't initialize ESPNow!");
        return;
    }

	#ifdef ESP8266
	esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
	#endif

    if (!registerPeer(broadcastMacAddress)) {
        m_Logger.fatal("Couldn't add broadcast mac address as a peer!");
        return;
    }

    if (!configuration.loadDongleConnection(dongleMacAddress, trackerId)) {
        m_Logger.info("The tracker isn't paired to a dongle yet!");
    } else {
        m_Logger.info("Dongle mac address loaded as " MACSTR "!", MAC2ARGS(dongleMacAddress));

        if (!registerPeer(dongleMacAddress)) {
            m_Logger.fatal("Couldn't add mac address " MACSTR " as a peer!", MAC2ARGS(dongleMacAddress));
            return;
        }
        paired = true;
    }

    if (esp_now_register_recv_cb(onReceive) != ESP_OK) {
        m_Logger.fatal("Couldn't register message callback!");
        return;
    }

    if (paired) {
        sendConnectionRequest();
    }
}

void ESPNowConnection::broadcastPairingRequest() {
    ESPNow::ESPNowPairingMessage message;
    if (esp_now_send(
                broadcastMacAddress,
                reinterpret_cast<uint8_t *>(&message),
                sizeof(message)
            ) != ESP_OK) {
        m_Logger.fatal("Couldn't send pairing message!");
        return;
    }
}

void ESPNowConnection::sendFusionPacket(uint8_t sensorId, Quat fusedQuat, Vector3 accel) {
    if (!connected) {
        return;
    }

    int16_t endBuffer[7];
    endBuffer[0] = toFixed<15>(fusedQuat.x);
    endBuffer[1] = toFixed<15>(fusedQuat.y);
    endBuffer[2] = toFixed<15>(fusedQuat.z);
    endBuffer[3] = toFixed<15>(fusedQuat.w);
    endBuffer[4] = toFixed<7>(accel.x);
    endBuffer[5] = toFixed<7>(accel.y);
    endBuffer[6] = toFixed<7>(accel.z);

    ESPNow::ESPNowPacketMessage message = {};
	message.packet.fullSizeFusion.packetId = ESPNow::ESPNowPacketId::FullSizeFusion;
    message.packet.fullSizeFusion.sensorId = trackerId << 2 | (sensorId & 0x03);
    memcpy(&message.packet.fullSizeFusion.quat, endBuffer, sizeof(endBuffer));

    if (esp_now_send(
                broadcastMacAddress,
                reinterpret_cast<uint8_t *>(&message),
                sizeof(message)
            ) != ESP_OK) {
        // Logging this constantly would be insane
        // m_Logger.fatal("Couldn't send packet");
        return;
    }
}

bool ESPNowConnection::sendDeviceInfoPacket(uint8_t sensorId) {
    if (!connected) {
        return false;
    }

    ESPNow::ESPNowPacketMessage message = {};
	message.header = ESPNow::ESPNowMessageHeader::Packet;
	ESPNow::ESPNowPacketDeviceInfo packet;
    packet.sensorId = trackerId << 2 | (sensorId & 0x03);
	packet.battPercentage = 50;
	packet.batteryVoltage = 128;
	packet.temperature = 128;
	packet.boardId = 0;
	packet.firmwareId = 0;
	packet.imuId = 0;
	packet.magId = 0;
	packet.firmwareDate = 0;
	packet.firmwareMajor = 0;
	packet.firmwareMinor = 0;
	packet.firmwarePatch = 0;
	message.packet.deviceInfo = packet;

    if (esp_now_send(
                broadcastMacAddress,
                reinterpret_cast<uint8_t *>(&message),
                sizeof(message)
            ) != ESP_OK) {
        // Logging this constantly would be insane
        // m_Logger.fatal("Couldn't send packet");
        return false;
    }
	return true;
}

bool ESPNowConnection::registerPeer(uint8_t macAddress[6]) {
#if ESP8266
	return esp_now_add_peer(macAddress, ESP_NOW_ROLE_COMBO, espnowWifiChannel, NULL, 0) == ESP_OK;
#elif ESP32
    esp_now_peer_info_t peer;
    memcpy(peer.peer_addr, macAddress, sizeof(uint8_t[6]));
    peer.channel = 0;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;
    return esp_now_add_peer(&peer) == ESP_OK;
#endif
}

void ESPNowConnection::sendConnectionRequest() {
	if (!paired) {
		return;
	}
    ESPNow::ESPNowConnectionMessage message;
	message.trackerId = trackerId;
    if (esp_now_send(
                dongleMacAddress,
                reinterpret_cast<uint8_t *>(&message),
                sizeof(message)
            ) != ESP_OK) {
        m_Logger.fatal("Couldn't send connection message!");
        return;
    }
}

void ESPNowConnection::handleMessage(uint8_t *senderMacAddress, const uint8_t *data, uint8_t dataLen) {
    const auto *message = reinterpret_cast<const ESPNow::ESPNowMessage *>(data);
    auto header = message->base.header;

    switch (header) {
		case ESPNow::ESPNowMessageHeader::Pairing:
            return;
		case ESPNow::ESPNowMessageHeader::PairingAck: {
            configuration.saveDongleConnection(senderMacAddress, message->pairingAck.trackerId);
			trackerId = message->pairingAck.trackerId;
			memcpy(dongleMacAddress, senderMacAddress, sizeof(uint8_t) * 6);
            m_Logger.info("Paired to dongle at mac address " MACSTR "!", MAC2ARGS(senderMacAddress));
            registerPeer(senderMacAddress);
            paired = true;
            connected = true;
            return;
        }
		case ESPNow::ESPNowMessageHeader::Connection:
			if (message->connection.trackerId != trackerId) {
				return;
			}
            ledManager.pattern(100, 100, 2);
            connected = true;
			m_Logger.info("Connected to dongle at mac address " MACSTR "!", MAC2ARGS(dongleMacAddress));
            return;
		case ESPNow::ESPNowMessageHeader::Packet:
            return;
    }
}

uint8_t ESPNowConnection::broadcastMacAddress[6] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};


} // namespace SlimeVR::Network
