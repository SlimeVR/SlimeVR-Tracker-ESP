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

#include "WiFiPackets.h"

#include <cstring>

namespace SlimeVR::Network::WiFiComms {

WiFiPackets::WiFiPackets(WiFiConnection& connection)
	: connection{connection} {}

bool WiFiPackets::beginPacket() {
	if (isBundle) {
		bundlePacketPosition = 0;
		return true;
	}

	return connection.beginUDPPacket();
}

bool WiFiPackets::endPacket() {
	if (isBundle) {
		uint32_t innerPacketSize = bundlePacketPosition;

		MUST_TRANSFER_BOOL((innerPacketSize > 0));

		isBundle = false;

		if (bundlePacketInnerCount == 0) {
			sendPacketType(SendPacketType::Bundle);
			sendPacketNumber();
		}
		sendShort(innerPacketSize);
		write(bundleBuffer, innerPacketSize);

		bundlePacketInnerCount++;
		isBundle = true;
		return true;
	}

	return connection.endUDPPacket();
}

bool WiFiPackets::beginBundle() {
	MUST_TRANSFER_BOOL(!isBundle);
	MUST_TRANSFER_BOOL(beginPacket());

	isBundle = true;
	bundlePacketInnerCount = 0;
	return true;
}

bool WiFiPackets::endBundle() {
	MUST_TRANSFER_BOOL(isBundle);

	isBundle = false;

	MUST_TRANSFER_BOOL((bundlePacketInnerCount > 0));

	return endPacket();
}

bool WiFiPackets::sendByte(uint8_t c) { return sendPrimitive(c); }

bool WiFiPackets::sendShort(uint16_t i) { return sendPrimitive(i); }

bool WiFiPackets::sendInt(uint32_t i) { return sendPrimitive(i); }

bool WiFiPackets::sendLong(uint64_t l) { return sendPrimitive(l); }

bool WiFiPackets::sendPacketNumber() {
	if (isBundle) {
		return true;
	}

	uint64_t pn = packetNumber++;

	return sendLong(pn);
}

bool WiFiPackets::sendPacketType(SendPacketType type) {
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));

	return sendByte(static_cast<uint8_t>(type));
}

bool WiFiPackets::sendShortString(const char* str) {
	size_t size = strlen(str);

	assert(size <= 255);

	MUST_TRANSFER_BOOL(sendByte(static_cast<uint8_t>(size)));
	if (size > 0) {
		MUST_TRANSFER_BOOL(write((const uint8_t*)str, size));
	}

	return true;
}

bool WiFiPackets::sendLongString(const char* str) {
	size_t size = strlen(str);

	MUST_TRANSFER_BOOL(sendInt(size));

	return write((const uint8_t*)str, size);
}

bool WiFiPackets::write(const uint8_t* buffer, size_t size) {
	if (isBundle) {
		if (bundlePacketPosition + size > sizeof(bundleBuffer)) {
			return false;
		}
		memcpy(bundleBuffer + bundlePacketPosition, buffer, size);
		bundlePacketPosition += size;
		return size;
	}
	return connection.write(buffer, size) > 0;
}

bool WiFiPackets::write(uint8_t byte) { return write(&byte, 1); }

}  // namespace SlimeVR::Network::WiFiComms
