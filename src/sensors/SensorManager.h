/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

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

#ifndef SLIMEVR_SENSORMANAGER
#define SLIMEVR_SENSORMANAGER

#include "globals.h"
#include "sensor.h"
#include "EmptySensor.h"
#include "logging/Logger.h"

namespace SlimeVR
{
    namespace Sensors
    {
        class SensorManager
        {
        public:
            SensorManager()
                : m_Logger(SlimeVR::Logging::Logger("SensorManager")) {
                    for (auto & u : m_Sensors) {
                        u = new EmptySensor(0);
                    }
                }
            ~SensorManager()
            {
                for (auto u : m_Sensors) {
                    if (u != NULL) {
                        delete u;
                    }
                }
            }

			void setup();
			void postSetup();

            void update();

            Sensor *getFirst() { return m_Sensors[0]; };
            Sensor *getSecond() { return m_Sensors[1]; };

        private:
            SlimeVR::Logging::Logger m_Logger;
            
            Sensor *m_Sensors[MAX_IMU_COUNT];
			Sensor* buildSensor(String &desc, uint8_t sensorID);
			Sensor* buildSensor(uint8_t sensorID, uint8_t address, uint8_t imuType,	float rotation,
                                uint8_t sclPin, uint8_t sdaPin, uint8_t intPin);
        };
    }
}

#endif // SLIMEVR_SENSORFACTORY_H_
