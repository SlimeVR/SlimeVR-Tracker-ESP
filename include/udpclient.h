#ifndef _UDP_CLIENT_H_
#define _UDP_CLIENT_H_

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include "quat.h"
#include "configuration.h"

#define PACKET_ROTATION 1
#define PACKET_GYRO 2
#define PACKET_HANDSHAKE 3
#define PACKET_ACCEL 4
#define PACKET_MAG 5
#define PACKET_RAW_CALIBRATION_DATA 6
#define PACKET_GYRO_CALIBRATION_DATA 7
#define PACKET_CONFIG 8

#define PACKET_RECIEVE_HEARTBEAT 1
#define PACKET_RECIEVE_VIBRATE 2
#define PACKET_RECIEVE_HANDSHAKE 3
#define PACKET_RECIEVE_COMMAND 4

#define COMMAND_CALLIBRATE 1
#define COMMAND_SEND_CONFIG 2

typedef void (* configRecievedCallback)(DeviceConfig const);
typedef void (* commandRecievedCallback)(int, void * const, int);

void connectClient();
void clientUpdate();
void sendQuat(float * const quaternion, int type);
void sendQuat(Quat * const quaternion, int type);
void sendVector(float * const result, int type);
void sendConfig(DeviceConfig * const config, int type);
void sendRawCalibrationData(int * const data, int type);
void setConfigRecievedCallback(configRecievedCallback);
void setCommandRecievedCallback(commandRecievedCallback);

template<typename T> T convert_chars(unsigned char* src);
template<typename T> unsigned char* convert_to_chars(T src, unsigned char* target);

#endif // _UDP_CLIENT_H_