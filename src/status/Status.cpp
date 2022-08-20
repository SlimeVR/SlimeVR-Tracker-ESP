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

#include "Status.h"

namespace SlimeVR
{
    namespace Status
    {
        const char *statusToString(Status status)
        {
            switch (status)
            {
            case LOADING:
                return "LOADING";
            case LOW_BATTERY:
                return "LOW_BATTERY";
            case IMU_ERROR:
                return "IMU_ERROR";
            case WIFI_CONNECTING:
                return "WIFI_CONNECTING";
            case SERVER_CONNECTING:
                return "SERVER_CONNECTING";
            case WIFI_CONNECTED:
                return "WIFI_CONNECTED";
            default:
                return "UNKNOWN";
            }
        }
    }
}
