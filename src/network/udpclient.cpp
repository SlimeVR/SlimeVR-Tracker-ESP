/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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
#include "packets.h"
#include "logging/Logger.h"
#include "GlobalVars.h"

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

// TODO: Cleanup with proper classes
SlimeVR::Logging::Logger udpClientLogger("UDPClient");

template <typename T>
unsigned char * convert_to_chars(T src, unsigned char * target)
{
    union uwunion
    {
        unsigned char c[sizeof(T)];
        T v;
    } un;
    un.v = src;
    for (size_t i = 0; i < sizeof(T); i++)
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
    for (size_t i = 0; i < sizeof(T); i++)
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
    if(!connected)
    {
        return;
    }

    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_HEARTBEAT);
        DataTransfer::sendPacketNumber();
        DataTransfer::endPacket();
    }
}

// PACKET_ACCEL 4
void Network::sendAccel(float* vector, uint8_t sensorId) {
    if(!connected)
    {
        return;
    }

    if(DataTransfer::beginPacket()) {
        DataTransfer::sendPacketType(PACKET_ACCEL);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendFloat(vector[0]);
        DataTransfer::sendFloat(vector[1]);
        DataTransfer::sendFloat(vector[2]);
        DataTransfer::sendByte(sensorId);
        DataTransfer::endPacket();
    }
}

// PACKET_RAW_CALIBRATION_DATA 6
void Network::sendRawCalibrationData(float* vector, uint8_t calibrationType, uint8_t sensorId) {
    if(!connected)
    {
        return;
    }

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
    if(!connected)
    {
        return;
    }

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
    if(!connected)
    {
        return;
    }

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
    if(!connected)
    {
        return;
    }

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
    if(!connected)
    {
        return;
    }

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
    if(!connected)
    {
        return;
    }

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
    if(!connected)
    {
        return;
    }

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
    if(!connected)
    {
        return;
    }

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
    if(!connected)
    {
        return;
    }

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
    if(!connected)
    {
        return;
    }

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
    if(!connected)
    {
        return;
    }

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
            udpClientLogger.error("Handshake write error: %d", Udp.getWriteError());
        }
    } else {
        udpClientLogger.error("Handshake write error: %d", Udp.getWriteError());
    }
}

#if ENABLE_INSPECTION
void Network::sendInspectionRawIMUData(uint8_t sensorId, int16_t rX, int16_t rY, int16_t rZ, uint8_t rA, int16_t aX, int16_t aY, int16_t aZ, uint8_t aA, int16_t mX, int16_t mY, int16_t mZ, uint8_t mA)
{
    if (!connected)
    {
        return;
    }

    if(!DataTransfer::beginPacket()) 
    {
        udpClientLogger.error("RawIMUData write begin error: %d", Udp.getWriteError());
        return;
    }

    DataTransfer::sendPacketType(PACKET_INSPECTION);
    DataTransfer::sendPacketNumber();

    DataTransfer::sendByte(PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA);

    DataTransfer::sendByte(sensorId);
    DataTransfer::sendByte(PACKET_INSPECTION_DATATYPE_INT);

    DataTransfer::sendInt(rX);
    DataTransfer::sendInt(rY);
    DataTransfer::sendInt(rZ);
    DataTransfer::sendByte(rA);

    DataTransfer::sendInt(aX);
    DataTransfer::sendInt(aY);
    DataTransfer::sendInt(aZ);
    DataTransfer::sendByte(aA);

    DataTransfer::sendInt(mX);
    DataTransfer::sendInt(mY);
    DataTransfer::sendInt(mZ);
    DataTransfer::sendByte(mA);

    if(!DataTransfer::endPacket())
    {
        udpClientLogger.error("RawIMUData write end error: %d", Udp.getWriteError());
    }
}

void Network::sendInspectionRawIMUData(uint8_t sensorId, float rX, float rY, float rZ, uint8_t rA, float aX, float aY, float aZ, uint8_t aA, float mX, float mY, float mZ, uint8_t mA)
{
    if (!connected) 
    {
        return;
    }

    if (!DataTransfer::beginPacket())
    {
        udpClientLogger.error("RawIMUData write begin error: %d", Udp.getWriteError());
        return;
    }

    DataTransfer::sendPacketType(PACKET_INSPECTION);
    DataTransfer::sendPacketNumber();

    DataTransfer::sendByte(PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA);

    DataTransfer::sendByte(sensorId);
    DataTransfer::sendByte(PACKET_INSPECTION_DATATYPE_FLOAT);

    DataTransfer::sendFloat(rX);
    DataTransfer::sendFloat(rY);
    DataTransfer::sendFloat(rZ);
    DataTransfer::sendByte(rA);

    DataTransfer::sendFloat(aX);
    DataTransfer::sendFloat(aY);
    DataTransfer::sendFloat(aZ);
    DataTransfer::sendByte(aA);

    DataTransfer::sendFloat(mX);
    DataTransfer::sendFloat(mY);
    DataTransfer::sendFloat(mZ);
    DataTransfer::sendByte(mA);

    if(!DataTransfer::endPacket())
    {
        udpClientLogger.error("RawIMUData write end error: %d", Udp.getWriteError());
    }
}

void Network::sendInspectionFusedIMUData(uint8_t sensorId, Quat quaternion)
{
    if (!connected) 
    {
        return;
    }

    if (!DataTransfer::beginPacket())
    {
        udpClientLogger.error("FusedIMUData write begin error: %d", Udp.getWriteError());
        return;
    }

    DataTransfer::sendPacketType(PACKET_INSPECTION);
    DataTransfer::sendPacketNumber();

    DataTransfer::sendByte(PACKET_INSPECTION_PACKETTYPE_FUSED_IMU_DATA);

    DataTransfer::sendByte(sensorId);
    DataTransfer::sendByte(PACKET_INSPECTION_DATATYPE_FLOAT);

    DataTransfer::sendFloat(quaternion.x);
    DataTransfer::sendFloat(quaternion.y);
    DataTransfer::sendFloat(quaternion.z);
    DataTransfer::sendFloat(quaternion.w);

    if(!DataTransfer::endPacket())
    {
        udpClientLogger.error("FusedIMUData write end error: %d", Udp.getWriteError());
    }
}

void Network::sendInspectionCorrectionData(uint8_t sensorId, Quat quaternion)
{
    if (!connected) 
    {
        return;
    }

    if (!DataTransfer::beginPacket())
    {
        udpClientLogger.error("CorrectionData write begin error: %d", Udp.getWriteError());
        return;
    }

    DataTransfer::sendPacketType(PACKET_INSPECTION);
    DataTransfer::sendPacketNumber();

    DataTransfer::sendByte(PACKET_INSPECTION_PACKETTYPE_CORRECTION_DATA);

    DataTransfer::sendByte(sensorId);
    DataTransfer::sendByte(PACKET_INSPECTION_DATATYPE_FLOAT);

    DataTransfer::sendFloat(quaternion.x);
    DataTransfer::sendFloat(quaternion.y);
    DataTransfer::sendFloat(quaternion.z);
    DataTransfer::sendFloat(quaternion.w);

    if(!DataTransfer::endPacket())
    {
        udpClientLogger.error("CorrectionData write end error: %d", Udp.getWriteError());
    }
}
#endif

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
            int len = Udp.read(incomingPacket, sizeof(incomingPacket));
            
#ifdef DEBUG_NETWORK
            udpClientLogger.trace("Received %d bytes from %s, port %d", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
            udpClientLogger.traceArray("UDP packet contents: ", incomingPacket, len);
#endif

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
                statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, false);
                ledManager.off();
                udpClientLogger.debug("Handshake successful, server is %s:%d", Udp.remoteIP().toString().c_str(), + Udp.remotePort());
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
        udpClientLogger.info("Looking for the server...");
        Network::sendHandshake();
        ledManager.on();
    }
    else if(lastConnectionAttemptMs + 20 < now)
    {
        ledManager.off();
    }
}

void ServerConnection::resetConnection() {
    Udp.begin(port);
    connected = false;

    statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);
}

void ServerConnection::update(Sensor * const sensor, Sensor * const sensor2) {
    if(connected) {
        int packetSize = Udp.parsePacket();
        if (packetSize)
        {
            lastPacketMs = millis();
            int len = Udp.read(incomingPacket, sizeof(incomingPacket));
            // receive incoming UDP packets

#ifdef DEBUG_NETWORK
            udpClientLogger.trace("Received %d bytes from %s, port %d", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
            udpClientLogger.traceArray("UDP packet contents: ", incomingPacket, len);
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
                udpClientLogger.warn("Handshake received again, ignoring");
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
                    udpClientLogger.warn("Wrong sensor info packet");
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
            statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);

            connected = false;
            sensorStateNotified1 = false;
            sensorStateNotified2 = false;
            udpClientLogger.warn("Connection to server timed out");
        }
    }
        
    if(!connected) {
        connect();
    } else if(sensorStateNotified1 != sensor->isWorking() || sensorStateNotified2 != sensor2->isWorking()) {
        updateSensorState(sensor, sensor2);
    }
}
