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

#include "udpclient.h"
#include "ledmgr.h"
#include "packets.h"

#define TIMEOUT 3000UL

WiFiUDP Udp;
unsigned char incomingPacket[128]; // buffer for incoming packets
uint64_t packetNumber = 0;
unsigned char handshake[12] = {0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0};

int port = 6969;
IPAddress host = IPAddress(255, 255, 255, 255);
unsigned long lastConnectionAttemptMs;
unsigned long lastPacketMs;

bool connected = false;

uint8_t sensorStateNotified1 = 0;
uint8_t sensorStateNotified2 = 0;
unsigned long lastSensorInfoPacket = 0;

uint8_t serialBuffer[128];
size_t serialLength = 0;

unsigned char buf[8];

template <typename T>
unsigned char * convert_to_chars(T src, unsigned char * target)
{
    union uwunion
    {
        unsigned char c[sizeof(T)];
        T v;
    } un;
    un.v = src;
    for (int i = 0; i < sizeof(T); i++)
    {
        target[i] = un.c[sizeof(T) - i - 1];
    }
    return target;
}

template <typename T>
T convert_chars(unsigned char * const src)
{
    union uwunion
    {
        unsigned char c[sizeof(T)];
        T v;
    } un;
    for (int i = 0; i < sizeof(T); i++)
    {
        un.c[i] = src[sizeof(T) - i - 1];
    }
    return un.v;
}

namespace DataTransfer {

    bool beginPacket() {
        int r = Udp.beginPacket(host, port);
        if(r == 0) {
            // Print error
        }
        return r > 0;
    }

    bool endPacket() {
        int r = Udp.endPacket();
        if(r == 0) {
            // Print error
        }
        return r > 0;
    }

    void sendPacketType(uint8_t type) {
        Udp.write(0);
        Udp.write(0);
        Udp.write(0);
        Udp.write(type);
    }

    void sendPacketNumber() {
        uint64_t pn = packetNumber++;
        sendLong(pn);
    }

    void sendFloat(float f) {
        Udp.write(convert_to_chars(f, buf), sizeof(f));
    }

    void sendByte(uint8_t c) {
        Udp.write(&c, 1);
    }

    void sendInt(int i) {
        Udp.write(convert_to_chars(i, buf), sizeof(i));
    }

    void sendLong(uint64_t l) {
        Udp.write(convert_to_chars(l, buf), sizeof(l));
    }

    void sendBytes(const uint8_t * c, size_t length) {
        Udp.write(c, length);
    }

    void sendShortString(const char * str) {
        uint8_t size = strlen(str);
        sendByte(size); // String size
        sendBytes((const uint8_t *) str, size); // Firmware version string
    }
    
    void sendLongString(const char * str) {
        int size = strlen(str);
        sendInt(size); // String size
        sendBytes((const uint8_t *) str, size); // Firmware version string
    }

    int getWriteError() {
        return Udp.getWriteError();
    }
}

// PACKET_HEARTBEAT 0
void Network::sendHeartbeat() {
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_HEARTBEAT);
        DataTransfer::sendPacketNumber();
        DataTransfer::endPacket();
    }
}

// PACKET_ACCEL 4
void Network::sendAccel(float* vector, uint8_t sensorId) {
    #ifndef SEND_UPDATES_UNCONNECTED
    if(!connected) return;   // bno080sensor.cpp function call not in sendData() but in motionLoop()
    #endif
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_ACCEL);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendFloat(vector[0]);
        DataTransfer::sendFloat(vector[1]);
        DataTransfer::sendFloat(vector[2]);
        DataTransfer::endPacket();
    }
}

// PACKET_RAW_CALIBRATION_DATA 6
void Network::sendRawCalibrationData(float* vector, uint8_t calibrationType, uint8_t sensorId) {
    #ifndef SEND_UPDATES_UNCONNECTED
    if(!connected) return;   // mpu9250sensor.cpp  startCalibration()
    #endif
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_RAW_CALIBRATION_DATA);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendInt(calibrationType);
        DataTransfer::sendFloat(vector[0]);
        DataTransfer::sendFloat(vector[1]);
        DataTransfer::sendFloat(vector[2]);
        DataTransfer::endPacket();
    }
}

void Network::sendRawCalibrationData(int* vector, uint8_t calibrationType, uint8_t sensorId) {
    #ifndef SEND_UPDATES_UNCONNECTED
    if(!connected) return;   // function not used?
    #endif
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_RAW_CALIBRATION_DATA);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendInt(calibrationType);
        DataTransfer::sendInt(vector[0]);
        DataTransfer::sendInt(vector[1]);
        DataTransfer::sendInt(vector[2]);
        DataTransfer::endPacket();
    }
}

// PACKET_CALIBRATION_FINISHED 7
void Network::sendCalibrationFinished(uint8_t calibrationType, uint8_t sensorId) {
    #ifndef SEND_UPDATES_UNCONNECTED
    if(!connected) return;   // mpu6050sensor.cpp mpu9250sensor.cpp  startCalibration()
    #endif
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_CALIBRATION_FINISHED);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendInt(calibrationType);
        DataTransfer::endPacket();
    }
}

// PACKET_BATTERY_LEVEL 12
void Network::sendBatteryLevel(float batteryVoltage, float batteryPercentage) {
    #ifndef SEND_UPDATES_UNCONNECTED
    if(!connected) return;
    #endif
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_BATTERY_LEVEL);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendFloat(batteryVoltage);
        DataTransfer::sendFloat(batteryPercentage);
        DataTransfer::endPacket();
    }
}

// PACKET_TAP 13
void Network::sendTap(uint8_t value, uint8_t sensorId) {
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_TAP);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendByte(value);
        DataTransfer::endPacket();
    }
}

// PACKET_ERROR 14
void Network::sendError(uint8_t reason, uint8_t sensorId) {
    #ifndef SEND_UPDATES_UNCONNECTED
    if(!connected) return;
    #endif
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_ERROR);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendByte(reason);
        DataTransfer::endPacket();
    }
}

// PACKET_SENSOR_INFO 15
void Network::sendSensorInfo(Sensor * sensor) {
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_SENSOR_INFO);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensor->getSensorId());
        DataTransfer::sendByte(sensor->getSensorState());
        DataTransfer::sendByte(sensor->getSensorType());
        DataTransfer::endPacket();
    }
}

// PACKET_ROTATION_DATA 17
void Network::sendRotationData(Quat * const quaternion, uint8_t dataType, uint8_t accuracyInfo, uint8_t sensorId) {
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_ROTATION_DATA);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendByte(dataType);
        DataTransfer::sendFloat(quaternion->x);
        DataTransfer::sendFloat(quaternion->y);
        DataTransfer::sendFloat(quaternion->z);
        DataTransfer::sendFloat(quaternion->w);
        DataTransfer::sendByte(accuracyInfo);
        DataTransfer::endPacket();
    }
}

// PACKET_MAGNETOMETER_ACCURACY 18
void Network::sendMagnetometerAccuracy(float accuracyInfo, uint8_t sensorId) {
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_MAGNETOMETER_ACCURACY);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendFloat(accuracyInfo);
        DataTransfer::endPacket();
    }
}

// PACKET_SIGNAL_STRENGTH 19
void Network::sendSignalStrength(uint8_t signalStrength) {
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_SIGNAL_STRENGTH);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(255);
        DataTransfer::sendByte(signalStrength);
        DataTransfer::endPacket();
    }
}

// PACKET_TEMPERATURE 20
void Network::sendTemperature(float temperature, uint8_t sensorId) {
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_TEMPERATURE);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendFloat(temperature);
        DataTransfer::endPacket();
    }
}

void Network::sendHandshake() {
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_HANDSHAKE);
        DataTransfer::sendLong(0); // Packet number is always 0 for handshake
        DataTransfer::sendInt(BOARD);
        // This is kept for backwards compatibility,
        // but the latest SlimeVR server will not initialize trackers
        // with firmware build > 8 until it recieves sensor info packet
        DataTransfer::sendInt(IMU);
        DataTransfer::sendInt(HARDWARE_MCU);
        DataTransfer::sendInt(0); 
        DataTransfer::sendInt(0);
        DataTransfer::sendInt(0);
        DataTransfer::sendInt(FIRMWARE_BUILD_NUMBER); // Firmware build number
        DataTransfer::sendShortString(FIRMWARE_VERSION);
        uint8_t mac[6];
        WiFi.macAddress(mac);
        DataTransfer::sendBytes(mac, 6); // MAC address string
        if(!DataTransfer::endPacket()) {
            Serial.print("Handshake write error: ");
            Serial.println(Udp.getWriteError());
        }
    } else {
        Serial.print("Handshake write error: ");
        Serial.println(Udp.getWriteError());
    }
}

void returnLastPacket(int len) {
    if(DataTransfer::beginPacket()) {
        DataTransfer::sendBytes(incomingPacket, len);
        DataTransfer::endPacket();
    }
}

void updateSensorState(Sensor * const sensor, Sensor * const sensor2) {
    if(millis() - lastSensorInfoPacket > 1000) {
        lastSensorInfoPacket = millis();
        if(sensorStateNotified1 != sensor->getSensorState())
            Network::sendSensorInfo(sensor);
        if(sensorStateNotified2 != sensor2->getSensorState())
            Network::sendSensorInfo(sensor2);
    }
}

bool ServerConnection::isConnected() {
    return connected;
}

void ServerConnection::connect()
{
    unsigned long now = millis();
    while(true) {
        int packetSize = Udp.parsePacket();
        if (packetSize)
        {
            // receive incoming UDP packets
            Serial.printf("[Handshake] Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
            int len = Udp.read(incomingPacket, sizeof(incomingPacket));
            Serial.print("[Handshake] UDP packet contents: ");
            for (int i = 0; i < len; ++i)
                Serial.print((byte)incomingPacket[i]);
            Serial.println();
            // Handshake is different, it has 3 in the first byte, not the 4th, and data starts right after
            switch (incomingPacket[0])
            {
            case PACKET_HANDSHAKE:
                // Assume handshake successful, don't check it
                // But proper handshake should contain "Hey OVR =D 5" ASCII string right after the packet number
                // Starting on 14th byte (packet number, 12 bytes greetings, null-terminator) we can transfer SlimeVR handshake data
                host = Udp.remoteIP();
                port = Udp.remotePort();
                lastPacketMs = now;
                connected = true;
                LEDManager::unsetLedStatus(LED_STATUS_SERVER_CONNECTING);
#ifndef SEND_UPDATES_UNCONNECTED
                LEDManager::off(LOADING_LED);
#endif
                Serial.printf("[Handshake] Handshake successful, server is %s:%d\n", Udp.remoteIP().toString().c_str(), + Udp.remotePort());
                return;
            default:
            continue;
            }
        }
        else
        {
            break;
        }   
    }
    if(lastConnectionAttemptMs + 1000 < now)
    {
        lastConnectionAttemptMs = now;
        Serial.println("Looking for the server...");
        Network::sendHandshake();
#ifndef SEND_UPDATES_UNCONNECTED
        LEDManager::on(LOADING_LED);
#endif
    }
#ifndef SEND_UPDATES_UNCONNECTED
    else if(lastConnectionAttemptMs + 20 < now)
    {
        LEDManager::off(LOADING_LED);
    }
#endif
}

void ServerConnection::resetConnection() {
    Udp.begin(port);
    connected = false;
    LEDManager::setLedStatus(LED_STATUS_SERVER_CONNECTING);
}

void ServerConnection::update(Sensor * const sensor, Sensor * const sensor2) {
    if(connected) {
        int packetSize = Udp.parsePacket();
        if (packetSize)
        {
            lastPacketMs = millis();
            int len = Udp.read(incomingPacket, sizeof(incomingPacket));
            // receive incoming UDP packets
            #if serialDebug == true
                Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
                Serial.print("UDP packet contents: ");
                for (int i = 0; i < len; ++i)
                    Serial.print((byte)incomingPacket[i]);
                Serial.println();
            #endif

            switch (convert_chars<int>(incomingPacket))
            {
            case PACKET_RECEIVE_HEARTBEAT:
                Network::sendHeartbeat();
                break;
            case PACKET_RECEIVE_VIBRATE:
                
                break;
            case PACKET_RECEIVE_HANDSHAKE:
                // Assume handshake successful
                Serial.println("Handshake received again, ignoring");
                break;
            case PACKET_RECEIVE_COMMAND:
                
                break;
            case PACKET_CONFIG:
                
                break;
            case PACKET_PING_PONG:
                returnLastPacket(len);
                break;
            case PACKET_SENSOR_INFO:
                if(len < 6) {
                    Serial.println("Wrong sensor info packet");
                    break;
                }
                if(incomingPacket[4] == 0) {
                    sensorStateNotified1 = incomingPacket[5];
                } else if(incomingPacket[4] == 1) {
                    sensorStateNotified2 = incomingPacket[5];
                }
                break;
            }
        }
        //while(Serial.available()) {
        //    size_t bytesRead = Serial.readBytes(serialBuffer, min(Serial.available(), sizeof(serialBuffer)));
        //    sendSerial(serialBuffer, bytesRead, PACKET_SERIAL);
        //}
        if(lastPacketMs + TIMEOUT < millis())
        {
            LEDManager::setLedStatus(LED_STATUS_SERVER_CONNECTING);
            connected = false;
            sensorStateNotified1 = false;
            sensorStateNotified2 = false;
            Serial.println("Connection to server timed out");
        }
    }
        
    if(!connected) {
        connect();
    } else if(sensorStateNotified1 != sensor->isWorking() || sensorStateNotified2 != sensor2->isWorking()) {
        updateSensorState(sensor, sensor2);
    }
}

