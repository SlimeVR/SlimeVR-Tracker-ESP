#if ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include <cstdint>
#include <cstring>
#include <optional>

#include "IPAddress.h"
#include "WiFiUdp.h"
#include "logging/Logger.h"
#include "mdns.h"

namespace SlimeVR::Network {

MDNSResolver::MDNSResolver(WiFiUDP& udp, SlimeVR::Logging::Logger& logger)
	: udp{udp}
	, logger{logger} {}

void MDNSResolver::searchForMDNS() {
	if (millis() - lastMDNSQueryMillis
		>= static_cast<uint64_t>(MDNSSearchIntervalSeconds * 1000)) {
		lastMDNSQueryMillis = millis();
		sendMDNSQuery();
	}
}

bool MDNSResolver::isPacketMDNS(const uint8_t* buffer) {
	const uint8_t packetHeader[] = {0x00, 0x00, 0x84, 0x00, 0x00, 0x01, 0x00, 0x01};

	return memcmp(buffer, packetHeader, sizeof(packetHeader)) == 0;
}

void MDNSResolver::sendMDNSQuery() {
	logger.info("Searching for mDNS record");

	uint8_t packet[64] = {0};
	uint16_t id = 0;
	uint16_t flags = 0;
	uint16_t questionCount = htons(1);
	uint16_t answerCount = 0;
	uint16_t authorityRRs = 0;
	uint16_t additionalRRs = 0;

	memcpy(&packet[0], &id, sizeof(id));
	memcpy(&packet[2], &flags, sizeof(flags));
	memcpy(&packet[4], &questionCount, sizeof(questionCount));
	memcpy(&packet[6], &answerCount, sizeof(answerCount));
	memcpy(&packet[8], &authorityRRs, sizeof(authorityRRs));
	memcpy(&packet[10], &additionalRRs, sizeof(additionalRRs));

	uint8_t* packetWrite = &packet[12];
	size_t hostNameLength = strlen(MDNSHostName);
	*packetWrite = static_cast<uint8_t>(hostNameLength);
	packetWrite++;
	memcpy(packetWrite, MDNSHostName, hostNameLength);
	packetWrite += hostNameLength;
	const char* tld = "local";
	size_t tldLength = strlen(tld);
	*packetWrite = static_cast<uint8_t>(tldLength);
	packetWrite++;
	memcpy(packetWrite, tld, tldLength);
	packetWrite += tldLength;
	*packetWrite = '\0';
	packetWrite++;

	uint16_t questionType = ntohs(1);  // A record
	uint16_t questionClass = ntohs(1);  // IN class
	memcpy(packetWrite, &questionType, sizeof(questionType));
	packetWrite += sizeof(questionType);
	memcpy(packetWrite, &questionClass, sizeof(questionClass));
	packetWrite += sizeof(questionClass);

	IPAddress mdnsAddress{224, 0, 0, 251};
	const uint16_t mdnsPort = 5353;
#if ESP8266
	udp.beginPacketMulticast(mdnsAddress, mdnsPort, WiFi.localIP(), 255);
#else
	udp.beginPacket(mdnsAddress, mdnsPort);
#endif
	udp.write(packet, packetWrite - packet);
	udp.endPacket();
}

std::optional<IPAddress> MDNSResolver::parseMDNSPacket(const uint8_t* buffer) const {
	const uint8_t* packetRead = buffer;

	auto readUint16 = [&]() {
		uint16_t result = packetRead[0] << 8 | packetRead[1];
		packetRead += 2;
		return result;
	};

	uint16_t transactionId = readUint16();
	uint16_t flags = readUint16();
	uint16_t questionCount = readUint16();
	uint16_t answerCount = readUint16();
	uint16_t authorityRRs = readUint16();
	uint16_t additionalRRs = readUint16();

	if (transactionId != 0 || flags != 0x8400 || questionCount != 1 || answerCount != 1
		|| authorityRRs != 0 || additionalRRs != 0) {
		return {};
	}

	uint8_t hostNameSize = *packetRead;
	packetRead++;
	if (hostNameSize != strlen(MDNSHostName)
		|| memcmp(MDNSHostName, packetRead, hostNameSize) != 0) {
		return {};
	}
	packetRead += hostNameSize;

	uint8_t tldSize = *packetRead;
	packetRead++;
	if (tldSize != strlen("local") || memcmp("local", packetRead, tldSize) != 0) {
		return {};
	}
	packetRead += tldSize;

	if (*packetRead != '\0') {
		return {};
	}
	packetRead++;

	uint16_t recordType = readUint16();
	uint16_t recordClass = readUint16();
	if (recordType != 1 || recordClass != 1) {
		return {};
	}

	uint8_t sectionLength = *packetRead;
	while (sectionLength != 0) {
		if ((sectionLength & 0xc0) == 0xc0) {
			// Pointer to a previous section
			packetRead++;
			break;
		} else {
			packetRead += sectionLength + 1;
			sectionLength = *packetRead;
		}
	}
	packetRead++;

	// Skip record type, class and TTL
	packetRead += 8;

	uint16_t ipLength = readUint16();
	if (ipLength != 4) {
		return {};
	}

	return std::optional{
		IPAddress{packetRead[0], packetRead[1], packetRead[2], packetRead[3]}
	};
}

}  // namespace SlimeVR::Network
