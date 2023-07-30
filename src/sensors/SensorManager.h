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
                : m_Logger(SlimeVR::Logging::Logger("SensorManager"))
                , m_Sensors(MAX_IMU_COUNT, nullptr) {
                    for (auto & u : m_Sensors) {
                        u = new EmptySensor(0);
                    }
                }
            ~SensorManager()
            {
                for (auto u : m_Sensors) {
                    if (u != nullptr) {
                        delete u;
                    }
                }
            }

            void setup();
            void postSetup();

            void update();
            
            std::vector<Sensor *> & getSensors() { return m_Sensors; };

        private:
            SlimeVR::Logging::Logger m_Logger;

            std::vector<Sensor *> m_Sensors;
            Sensor* buildSensor(uint8_t sensorID, uint8_t imuType, uint8_t address, float rotation, uint8_t sclPin, uint8_t sdaPin, int extraParam = 0);
            
            uint8_t activeSCL = 0;
            uint8_t activeSDA = 0;
            bool running = false;
            void swapI2C(uint8_t scl, uint8_t sda);
            
            uint32_t m_LastBundleSentAtMicros = micros();
        };
    }
}

#endif // SLIMEVR_SENSORFACTORY_H_
