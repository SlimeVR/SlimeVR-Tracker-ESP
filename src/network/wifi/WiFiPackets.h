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

#include <cstdint>
#include <cstring>

#include "WiFiConnection.h"
#include "WiFiPacketDatatypes.h"

namespace SlimeVR::Network::WiFiComms {

#define MUST_TRANSFER_BOOL(b) \
	if (!(b))                 \
		return false;

#define MUST(b) \
	if (!(b))   \
		return;

class WiFiPackets {
public:
	explicit WiFiPackets(WiFiConnection& connection);

	bool beginPacket();
	bool endPacket();

	bool beginBundle();
	bool endBundle();

	bool sendPacketType(SendPacketType type);
	bool sendPacketNumber();

	bool write(const uint8_t* buffer, size_t size);
	bool write(uint8_t byte);

	template <typename Packet>
	bool sendPacket(
		Packet packet,
		std::optional<uint64_t> packetNumberOverride = std::nullopt
	) {
		MUST_TRANSFER_BOOL(beginPacket());
		MUST_TRANSFER_BOOL(sendPacketType(Packet::packetType));
		if (packetNumberOverride) {
			MUST_TRANSFER_BOOL(sendPrimitive(*packetNumberOverride));
		} else {
			MUST_TRANSFER_BOOL(sendPacketNumber());
		}

		MUST_TRANSFER_BOOL(write(reinterpret_cast<uint8_t*>(&packet), sizeof(Packet)));

		return endPacket();
	}

	template <typename Callback>
	bool sendPacketCallback(
		SendPacketType type,
		Callback bodyCallback,
		std::optional<uint64_t> packetNumberOverride = std::nullopt
	) {
		MUST_TRANSFER_BOOL(beginPacket());
		MUST_TRANSFER_BOOL(sendPacketType(type));
		if (packetNumberOverride) {
			MUST_TRANSFER_BOOL(sendPrimitive(*packetNumberOverride));
		} else {
			MUST_TRANSFER_BOOL(sendPacketNumber());
		}

		MUST_TRANSFER_BOOL(bodyCallback());

		return endPacket();
	}

	template <typename T>
	uint8_t* convertToChars(T src, uint8_t* target) {
		auto* rawBytes = reinterpret_cast<uint8_t*>(&src);
		std::memcpy(target, rawBytes, sizeof(T));
		std::reverse(target, target + sizeof(T));
		return target;
	}

	template <typename T>
	inline bool sendPrimitive(T value) {
		if constexpr (sizeof(T) == 1) {
			return write(static_cast<uint8_t>(value));
		}

		uint8_t buffer[8];
		convertToChars(value, buffer);
		return write(buffer, sizeof(T));
	}

	bool sendByte(uint8_t c);
	bool sendShort(uint16_t i);
	bool sendInt(uint32_t i);
	bool sendLong(uint64_t l);
	bool sendShortString(const char* str);
	bool sendLongString(const char* str);

private:
	WiFiConnection& connection;

	uint64_t packetNumber = 0;

	bool isBundle = false;
	uint16_t bundlePacketPosition = 0;
	uint16_t bundlePacketInnerCount = 0;
	uint8_t bundleBuffer[128] = {0};
};

}  // namespace SlimeVR::Network::WiFiComms
