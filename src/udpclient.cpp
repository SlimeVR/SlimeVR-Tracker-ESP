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
#include "ledstatus.h"

#define TIMEOUT 3000UL

AsyncUDP Udp;
AsyncUDPMessage Udpmsg;
unsigned char incomingPacket[128]; // buffer for incoming packets
uint64_t packetNumber = 1;
unsigned char handshake[12] = {0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char buf[128];
configReceivedCallback fp_configCallback;
commandReceivedCallback fp_commandCallback;

uint16_t port = 6969;
IPAddress host = IPAddress(255, 255, 255, 255);
bool connected = false;
unsigned long lastConnectionAttemptMs;
unsigned long lastPacketMs;

uint8_t serialBuffer[128];
size_t serialLength = 0;

bool sensorStateNotified1 = false;
bool sensorStateNotified2 = false;
unsigned long lastSensorInfoPacket = 0;

size_t packet_read(AsyncUDPPacket packet, uint8_t *data, size_t len)
{
    size_t i;
    if (len > packet.length())
    {
        len = packet.length();
    }
    for (i = 0; i < len; i++)
    {
        data[i] = packet.data()[i];
    }
    return len;
}

template <typename T>
unsigned char *convert_to_chars(T src, unsigned char *target)
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
T convert_chars(unsigned char *const src)
{
    union
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

void sendPacketNumber()
{
    //uint64_t pn = packetNumber++;
    // TODO Send packet number
    Udpmsg.write(0);
    Udpmsg.write(0);
    Udpmsg.write(0);
    Udpmsg.write(0);
    Udpmsg.write(0);
    Udpmsg.write(0);
    Udpmsg.write(0);
    Udpmsg.write(0);
}

void sendType(int type)
{
    Udpmsg.write(0);
    Udpmsg.write(0);
    Udpmsg.write(0);
    Udpmsg.write(type);
}

void sendVector(float *const result, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        float x = result[0];
        float y = result[1];
        float z = result[2];
        float w = 0;
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(convert_to_chars(x, buf), sizeof(x));
        Udpmsg.write(convert_to_chars(y, buf), sizeof(y));
        Udpmsg.write(convert_to_chars(z, buf), sizeof(z));
        Udpmsg.write(convert_to_chars(w, buf), sizeof(w));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendFloat(float const value, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(convert_to_chars(value, buf), sizeof(value));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void send2Floats(float const value1, float const value2, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(convert_to_chars(value1, buf), sizeof(value1));
        Udpmsg.write(convert_to_chars(value2, buf), sizeof(value2));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendByte(unsigned char const value, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(&value, 1);
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendByte(uint8_t const value, uint8_t sensorId, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(&sensorId, 1);
        Udpmsg.write(&value, 1);
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendQuat(Quat *const quaternion, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        float x = quaternion->x;
        float y = quaternion->y;
        float z = quaternion->z;
        float w = quaternion->w;
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(convert_to_chars(x, buf), sizeof(x));
        Udpmsg.write(convert_to_chars(y, buf), sizeof(y));
        Udpmsg.write(convert_to_chars(z, buf), sizeof(z));
        Udpmsg.write(convert_to_chars(w, buf), sizeof(w));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendRotationData(Quat *const quaternion, uint8_t dataType, uint8_t accuracyInfo, uint8_t sensorId, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        float x = quaternion->x;
        float y = quaternion->y;
        float z = quaternion->z;
        float w = quaternion->w;
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(&sensorId, 1);
        Udpmsg.write(&dataType, 1);
        Udpmsg.write(convert_to_chars(x, buf), sizeof(x));
        Udpmsg.write(convert_to_chars(y, buf), sizeof(y));
        Udpmsg.write(convert_to_chars(z, buf), sizeof(z));
        Udpmsg.write(convert_to_chars(w, buf), sizeof(w));
        Udpmsg.write(&accuracyInfo, 1);
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendMagnetometerAccuracy(float accuracyInfo, uint8_t sensorId, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(&sensorId, 1);
        Udpmsg.write(convert_to_chars(accuracyInfo, buf), sizeof(accuracyInfo));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendQuat(float *const quaternion, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        float x = quaternion[0];
        float y = quaternion[1];
        float z = quaternion[2];
        float w = quaternion[3];
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(convert_to_chars(x, buf), sizeof(x));
        Udpmsg.write(convert_to_chars(y, buf), sizeof(y));
        Udpmsg.write(convert_to_chars(z, buf), sizeof(z));
        Udpmsg.write(convert_to_chars(w, buf), sizeof(w));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendConfig(DeviceConfig *const config, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        DeviceConfig data = *config;
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(convert_to_chars(data, buf), sizeof(data));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendRawCalibrationData(int *const data, int calibrationType, unsigned char const sensorId, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        int x = data[0];
        int y = data[1];
        int z = data[2];
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(&sensorId, 1);
        Udpmsg.write(convert_to_chars(calibrationType, buf), sizeof(calibrationType));
        Udpmsg.write(convert_to_chars(x, buf), sizeof(x));
        Udpmsg.write(convert_to_chars(y, buf), sizeof(y));
        Udpmsg.write(convert_to_chars(z, buf), sizeof(z));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendRawCalibrationData(float *const data, int calibrationType, unsigned char const sensorId, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        float x = data[0];
        float y = data[1];
        float z = data[2];
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(&sensorId, 1);
        Udpmsg.write(convert_to_chars(calibrationType, buf), sizeof(calibrationType));
        Udpmsg.write(convert_to_chars(x, buf), sizeof(x));
        Udpmsg.write(convert_to_chars(y, buf), sizeof(y));
        Udpmsg.write(convert_to_chars(z, buf), sizeof(z));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendCalibrationFinished(int calibrationType, unsigned char const sensorId, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(&sensorId, 1);
        Udpmsg.write(convert_to_chars(calibrationType, buf), sizeof(calibrationType));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendSerial(uint8_t *const data, int length, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(convert_to_chars(length, buf), sizeof(length));
        Udpmsg.write(data, length);
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendSensorInfo(unsigned char const sensorId, unsigned char const sensorState, int type)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        sendType(type);
        sendPacketNumber();
        Udpmsg.write(&sensorId, 1);
        Udpmsg.write(&sensorState, 1);
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendHeartbeat()
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        sendType(PACKET_HEARTBEAT);
        Udpmsg.write(convert_to_chars((uint64_t)0, buf), sizeof(uint64_t));
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
    }
}

void sendHandshake()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        Udpmsg.flush();
        sendType(3);
        Udpmsg.write(convert_to_chars((uint64_t)0, buf), sizeof(uint64_t)); // Packet number is always 0 for handshake
        Udpmsg.write(convert_to_chars((uint32_t)BOARD, buf), sizeof(uint32_t));
        Udpmsg.write(convert_to_chars((uint32_t)IMU, buf), sizeof(uint32_t));
        Udpmsg.write(convert_to_chars((uint32_t)HARDWARE_MCU, buf), sizeof(uint32_t));
        Udpmsg.write(convert_to_chars((uint32_t)0, buf), sizeof(uint32_t)); // TODO Send actual IMU hw version read from the chip
        Udpmsg.write(convert_to_chars((uint32_t)0, buf), sizeof(uint32_t));
        Udpmsg.write(convert_to_chars((uint32_t)0, buf), sizeof(uint32_t));
        Udpmsg.write(convert_to_chars((uint32_t)FIRMWARE_BUILD_NUMBER, buf), sizeof(uint32_t)); // Firmware build number
        uint8_t size = (uint8_t)sizeof(FIRMWARE_VERSION);
        Udpmsg.write(&size, 1);                                                          // Firmware version string size
        Udpmsg.write((const unsigned char *)FIRMWARE_VERSION, sizeof(FIRMWARE_VERSION)); // Firmware version string
        uint8_t mac[6];
        WiFi.macAddress(mac);
        Udpmsg.write(mac, 6); // MAC address string
        if (Udp.broadcastTo(Udpmsg, port) == 0)
        {
            Serial.print("Write error: ");
            Serial.println(Udp.getWriteError());
        }
    }
    else
    {
        Serial.print("Write error: ");
        Serial.println(Udp.getWriteError());
    }
}

void returnLastPacket(int len)
{
    if (Udp.connect(host, port))
    {
        Udpmsg.flush();
        Udpmsg.write(incomingPacket, len);
        if (Udp.send(Udpmsg) == 0)
        {
            //Serial.print("Write error: ");
            //Serial.println(Udp.getWriteError());
        }
    }
}

void setConfigReceivedCallback(configReceivedCallback callback)
{
    fp_configCallback = callback;
}

void setCommandReceivedCallback(commandReceivedCallback callback)
{
    fp_commandCallback = callback;
}

void updateSensorState(Sensor *const sensor, Sensor *const sensor2)
{
    if (millis() - lastSensorInfoPacket > 1000)
    {
        lastSensorInfoPacket = millis();
        if (sensorStateNotified1 != sensor->isWorking())
            sendSensorInfo(0, sensor->isWorking(), PACKET_SENSOR_INFO);
        if (sensorStateNotified2 != sensor2->isWorking())
            sendSensorInfo(1, sensor2->isWorking(), PACKET_SENSOR_INFO);
    }
}

void onPacketCallBack(AsyncUDPPacket packet)
{
    {
        if (isWiFiConnected())
        {
            if (connected)
            {
                int packetSize = packet.length();
                if (packetSize)
                {
                    lastPacketMs = millis();
                    int len = packet_read(packet, incomingPacket, sizeof(incomingPacket));
                    // receive incoming UDP packets
                    if (serialDebug)
                    {
                        Serial.printf("Received %d bytes from %s, port %d\n", packetSize, packet.remoteIP().toString().c_str(), packet.remotePort());
                        Serial.print("UDP packet contents: ");
                        for (int i = 0; i < len; ++i)
                            Serial.print((byte)incomingPacket[i]);
                        Serial.println();
                    }

                    switch (convert_chars<int>(incomingPacket))
                    {
                    case PACKET_RECEIVE_HEARTBEAT:
                        sendHeartbeat();
                        break;
                    case PACKET_RECEIVE_VIBRATE:
                        if (fp_commandCallback)
                        {
                            fp_commandCallback(COMMAND_BLINK, nullptr, 0);
                        }
                        break;
                    case PACKET_RECEIVE_HANDSHAKE:
                        // Assume handshake sucessful
                        Serial.println("Handshale recived again, ignoring");
                        break;
                    case PACKET_RECEIVE_COMMAND:
                        if (len < 6)
                        {
                            Serial.println("Command packet too short");
                            break;
                        }
                        if (serialDebug)
                        {
                            Serial.printf("Recieved command %d\n", incomingPacket[4]);
                        }
                        if (fp_commandCallback)
                        {
                            fp_commandCallback(incomingPacket[4], &incomingPacket[5], len - 6);
                        }
                        break;
                    case PACKET_CONFIG:
                        if (len < sizeof(DeviceConfig) + 4)
                        {
                            Serial.println("config packet too short");
                            break;
                        }
                        if (fp_configCallback)
                        {
                            fp_configCallback(convert_chars<DeviceConfig>(&incomingPacket[4]));
                        }
                        break;
                    case PACKET_PING_PONG:
                        returnLastPacket(len);
                        break;
                    case PACKET_SENSOR_INFO:
                        if (len < 6)
                        {
                            Serial.println("Wrong sensor info packet");
                            break;
                        }
                        if (incomingPacket[4] == 0)
                        {
                            sensorStateNotified1 = incomingPacket[5];
                        }
                        else if (incomingPacket[4] == 1)
                        {
                            sensorStateNotified2 = incomingPacket[5];
                        }
                        break;
                    }
                }
                //while(Serial.available()) {
                //    size_t bytesRead = Serial.readBytes(serialBuffer, min(Serial.available(), sizeof(serialBuffer)));
                //    sendSerial(serialBuffer, bytesRead, PACKET_SERIAL);
                //}
                if (lastPacketMs + TIMEOUT < millis())
                {
                    setLedStatus(LED_STATUS_SERVER_CONNECTING);
                    connected = false;
                    sensorStateNotified1 = false;
                    sensorStateNotified2 = false;
                    Serial.println("Connection to server timed out");
                }
            }
            else
            {
                int packetSize = packet.length();
                if (packetSize)
                {
                    // receive incoming UDP packets
                    Serial.printf("[Handshake] Received %d bytes from %s, port %d\n", packetSize, packet.remoteIP().toString().c_str(), packet.remotePort());
                    int len = packet_read(packet, incomingPacket, sizeof(incomingPacket));
                    Serial.print("[Handshake] UDP packet contents: ");
                    for (int i = 0; i < len; ++i)
                        Serial.print((byte)incomingPacket[i]);
                    Serial.println();
                    // Handshake is different, it has 3 in the first byte, not the 4th, and data starts right after
                    switch (incomingPacket[0])
                    {
                    case PACKET_HANDSHAKE:
                        // Assume handshake sucessful, don't check it
                        // But proper handshake should contain "Hey OVR =D 5" ASCII string right after the packet number
                        // Starting on 14th byte (packet number, 12 bytes greetings, null-terminator) we can transfer SlimeVR handshake data
                        host = packet.remoteIP();
                        port = packet.remotePort();
                        lastPacketMs = millis();
                        connected = true;
                        unsetLedStatus(LED_STATUS_SERVER_CONNECTING);
#ifndef SEND_UPDATES_UNCONNECTED
                        digitalWrite(LOADING_LED, LOW);
#endif
                        Serial.printf("[Handshake] Handshale sucessful, server is %s:%d\n", packet.remoteIP().toString().c_str(), +packet.remotePort());
                    }
                }
            }
        }
    }
}
void onWiFiConnected()
{
    while (!Udp.listen(port))
    {
    }
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    Udp.onPacket(onPacketCallBack);
    connected = false;
    setLedStatus(LED_STATUS_SERVER_CONNECTING);
}

void clientUpdate(Sensor *const sensor, Sensor *const sensor2)
{
    if (isWiFiConnected())
    {
        if (connected)
        {
            if (sensorStateNotified1 != sensor->isWorking() || sensorStateNotified2 != sensor2->isWorking())
            {
                updateSensorState(sensor, sensor2);
            }
        }
        else
        {
            unsigned long now = millis();
            if (lastConnectionAttemptMs + 1000 < now)
            {
                lastConnectionAttemptMs = now;
                Serial.println("Looking for the server...");
                sendHandshake();
#ifndef SEND_UPDATES_UNCONNECTED
                digitalWrite(LOADING_LED, HIGH);
#endif
            }
#ifndef SEND_UPDATES_UNCONNECTED
            else if (lastConnectionAttemptMs + 20 < now)
            {
                digitalWrite(LOADING_LED, LOW);
            }
#endif
        }
    }
}

bool isConnected()
{
    return connected;
}
