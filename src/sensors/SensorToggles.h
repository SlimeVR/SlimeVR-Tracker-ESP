/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Gorbit99 & SlimeVR Contributors

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
#include <functional>
#include <optional>

#include "../debug.h"

enum class SensorToggles : uint16_t {
	MagEnabled = 1,
	CalibrationEnabled = 2,
	TempGradientCalibrationEnabled = 3,
};

struct SensorToggleValues {
	bool magEnabled = !USE_6_AXIS;
	bool calibrationEnabled = true;
	bool tempGradientCalibrationEnabled
		= false;  // disable by default, it is not clear that it really helps
};

class SensorToggleState {
public:
	SensorToggleState() = default;
	explicit SensorToggleState(SensorToggleValues values);
	void setToggle(SensorToggles toggle, bool state);
	[[nodiscard]] bool getToggle(SensorToggles toggle) const;

	void onToggleChange(std::function<void(SensorToggles, bool)>&& callback);

	static const char* toggleToString(SensorToggles toggle);

	[[nodiscard]] SensorToggleValues getValues() const;

private:
	std::optional<std::function<void(SensorToggles, bool)>> callback;

	void emitToggleChange(SensorToggles toggle, bool state) const;

	SensorToggleValues values;
};
