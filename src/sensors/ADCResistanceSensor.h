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

#include <PinInterface.h>

#include "../sensorinterface/SensorInterface.h"
#include "sensor.h"

class ADCResistanceSensor : public Sensor {
public:
	static constexpr auto TypeID = SensorTypeID::ADC_RESISTANCE;

	ADCResistanceSensor(
		uint8_t id,
		float resistanceDivider,
		PinInterface* pinInterface = nullptr,
		float smoothFactor = 0.1f
	);
	~ADCResistanceSensor() = default;

	void motionLoop() final;
	void sendData() final;
	bool hasNewDataToSend() final;

	SensorStatus getSensorState() override final { return SensorStatus::SENSOR_OK; }

	SensorDataType getDataType() override final {
		return SensorDataType::SENSOR_DATATYPE_FLEX_RESISTANCE;
	};

private:
	static constexpr uint32_t samplingRateHz = 60;
	static constexpr uint64_t samplingStepMicros = 1000'000 / samplingRateHz;

	PinInterface* m_PinInterface;
	float m_ResistanceDivider;
	float m_SmoothFactor;
	uint64_t lastSampleMicros = 0;
	bool hasNewSample = false;

	float m_Data = 0.0f;
};
