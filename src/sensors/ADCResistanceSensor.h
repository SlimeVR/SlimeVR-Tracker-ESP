/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Eiren Rain & SlimeVR contributors

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

#include "../sensorinterface/RegisterInterface.h"
#include "sensor.h"

namespace SlimeVR::Sensors {
class ADCResistanceSensor : Sensor {
public:
	static constexpr auto TypeID = SensorTypeID::ADC_RESISTANCE;

	ADCResistanceSensor(
		uint8_t id,
		uint8_t pin,
		float VCC,
		float resistanceDivider,
		float smoothFactor = 0.1f
	)
		: Sensor(
			"ADCResistanceSensor",
			SensorTypeID::ADC_RESISTANCE,
			id,
			EmptyRegisterInterface::instance,
			0.0f,
			new SlimeVR::EmptySensorInterface
		)
		, m_Pin(pin)
		, m_VCC(VCC)
		, m_ResistanceDivider(resistanceDivider)
		, m_SmoothFactor(smoothFactor){};
	~ADCResistanceSensor();

	void motionLoop() override final;
	void sendData() override final;

	SensorStatus getSensorState() override final { return SensorStatus::SENSOR_OK; }

	SensorDataType getDataType() override final {
		return SensorDataType::SENSOR_DATATYPE_FLEX_RESISTANCE;
	};

private:
	uint8_t m_Pin;
	float m_VCC;
	float m_ResistanceDivider;
	float m_SmoothFactor;

	float m_Data = 0.0f;
};
}  // namespace SlimeVR::Sensors
