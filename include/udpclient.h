#ifndef _UDP_CLIENT_H_
#define _UDP_CLIENT_H_

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include "quat.h"

#define PACKET_ROTATION 1
#define PACKET_GYRO 2
#define PACKET_HANDSHAKE 3
#define PACKET_ACCEL 4
#define PACKET_MAG 5

void connectClient();
void sendQuat(float * quaternion, int type);
void sendQuat(Quat * quaternion, int type);
void sendVector(float * result, int type);
template<typename T> T convert_chars(unsigned char* src);
template<typename T> unsigned char* convert_to_chars(T src, unsigned char* target);

#endif // _UDP_CLIENT_H_