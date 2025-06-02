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
#include "ADCResistanceSensor.h"

#include "GlobalVars.h"

namespace SlimeVR::Sensors {
void ADCResistanceSensor::motionLoop() {
#if ESP8266
	float voltage = ((float)analogRead(m_Pin)) * ADCVoltageMax / ADCResolution;
	m_Data = m_ResistanceDivider
		   * (ADCVoltageMax / voltage - 1.0f);  // Convert voltage to resistance
#elif defined(ESP32)
	float voltage = ((float)analogReadMilliVolts(m_Pin)) / 1000;
	m_Data = m_ResistanceDivider
		   * (m_VCC / voltage - 1.0f);  // Convert voltage to resistance
#endif
}

void ADCResistanceSensor::sendData() {
	networkConnection.sendFlexData(sensorId, m_Data);
}

}  // namespace SlimeVR::Sensors
