
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

#ifndef SLIMEVR_FEATURE_FLAGS_H_
#define SLIMEVR_FEATURE_FLAGS_H_

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <unordered_map>

// I hate C++11 - they fixed this in C++14, but our compilers are old as the iceage
struct EnumClassHash {
	template <typename T>
	std::size_t operator()(T t) const {
		return static_cast<std::size_t>(t);
	}
};

enum EServerFeatureFlags : uint32_t {
	// Server can parse bundle packets: `PACKET_BUNDLE` = 100 (0x64).
	PROTOCOL_BUNDLE_SUPPORT,

	BITS_TOTAL,
};

enum class EFirmwareFeatureFlags : uint32_t {
	// EXAMPLE_FEATURE,
	B64_WIFI_SCANNING = 1,

	BITS_TOTAL,
};

static const std::unordered_map<EFirmwareFeatureFlags, bool, EnumClassHash>
	m_EnabledFirmwareFeatures = {{EFirmwareFeatureFlags::B64_WIFI_SCANNING, true}};

template <typename Flags>
class FeatureFlags {
public:
	static constexpr auto FLAG_BYTES
		= ((static_cast<size_t>(Flags::BITS_TOTAL)) + 7) / 8;

	FeatureFlags()
		: m_Available(false) {}
	FeatureFlags(uint8_t* packed, uint32_t length)
		: m_Available(true) {
		for (uint32_t bit = 0; bit < length * 8; bit++) {
			auto posInPacked = bit / 8;
			auto posInByte = bit % 8;

			m_Flags[static_cast<Flags>(bit)] = packed[posInPacked] & (1 << posInByte);
		}
	}
	FeatureFlags(std::unordered_map<Flags, bool, EnumClassHash> flags)
		: m_Available(true)
		, m_Flags(flags) {}

	std::array<uint8_t, FLAG_BYTES> pack() {
		std::array<uint8_t, FLAG_BYTES> packed{};

		for (auto& [flag, value] : m_Flags) {
			auto posInPacked = static_cast<size_t>(flag) / 8;
			auto posInByte = static_cast<size_t>(flag) % 8;

			if (value) {
				packed[posInPacked] |= 1 << posInByte;
			}
		}

		return packed;
	};

	bool has(Flags flag) { return m_Flags[flag]; }
	bool isAvailable() { return m_Available; }

private:
	bool m_Available = false;
	std::unordered_map<Flags, bool, EnumClassHash> m_Flags{};
};

#endif
