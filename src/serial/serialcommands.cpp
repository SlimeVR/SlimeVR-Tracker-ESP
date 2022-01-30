/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

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

#include "serialcommands.h"
#include "network/network.h"
#include <CmdCallback.hpp>

namespace SerialCommands {
    CmdCallback<5> cmdCallbacks;
    CmdParser cmdParser;
    CmdBuffer<64> cmdBuffer;

    void cmdSet(CmdParser * parser) {
        if(parser->getParamCount() != 1 && parser->equalCmdParam(1, "WIFI")  ) {
            if(parser->getParamCount() < 3) {
                Serial.println("[ERR] CMD SET WIFI ERROR: Too few arguments");
                Serial.println("[NOTICE] Syntax: SET WIFI \"<SSID>\" \"<PASSWORD>\"");
            } else {
                WiFiNetwork::setWiFiCredentials(parser->getCmdParam(2), parser->getCmdParam(3));
                Serial.println("[OK] CMD SET WIFI OK: New wifi credentials set, reconnecting");
            }
        } else {
            Serial.println("[ERR] CMD SET ERROR: Unrecognized variable to set");
        }
    }

    void cmdGet(CmdParser * parser) {
        if(parser->getParamCount() != 1 && parser->equalCmdParam(1, "INFO")  ) {
            Serial.print("[OK] SlimeVR Tracker, ");
            Serial.print("board: ");
            Serial.print(BOARD);
            Serial.print(", hardware: ");
            Serial.print(HARDWARE_MCU);
            Serial.print(", build: ");
            Serial.print(FIRMWARE_BUILD_NUMBER);
            Serial.print(", firmware: ");
            Serial.print(FIRMWARE_VERSION);
            Serial.print(", address: ");
            Serial.println(WiFiNetwork::getAddress().toString());
            // TODO Print sensors number and types
        }
    }

    void cmdReport(CmdParser * parser) {
        // TODO Health and status report
    }

    void cmdReboot(CmdParser * parser) {
        ESP.restart();
    }

    void cmdFactoryReset(CmdParser * parser) {
        // TODO Factory reset
    }

    void setUp() {
        cmdCallbacks.addCmd("SET", &cmdSet);
        cmdCallbacks.addCmd("GET", &cmdGet);
        cmdCallbacks.addCmd("FRST", &cmdFactoryReset);
        cmdCallbacks.addCmd("REP", &cmdReport);
        cmdCallbacks.addCmd("REBOOT", &cmdReport);
    }

    void update() {
        cmdCallbacks.updateCmdProcessing(&cmdParser, &cmdBuffer, &Serial);
    }
}
