#include "udpclient.h"

///////////////////////////////////////////////////////////////////
//Wifi
///////////////////////////////////////////////////////////////////
#include "wificredentials.h"

WiFiUDP Udp;
unsigned char incomingPacket[128]; // buffer for incoming packets
uint64_t packetNumber = 1;
unsigned char handshake[12] = {0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char buf[128];
configRecievedCallback fp_configCallback;
commandRecievedCallback fp_commandCallback;

template <typename T>
unsigned char * convert_to_chars(T src, unsigned char * target)
{
    union uwunion
    {
        unsigned char c[sizeof(T)];
        T v;
    };
    for (int i = 0; i < sizeof(T); i++)
    {
        target[i] = ((uwunion *) &src)->c[sizeof(T) - i - 1];
    }
    return target;
}

template <typename T>
T convert_chars(unsigned char * const src)
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
    uint64_t pn = packetNumber++;
    // TODO Send packet number
    Udp.write(0);
    Udp.write(0);
    Udp.write(0);
    Udp.write(0);
    Udp.write(0);
    Udp.write(0);
    Udp.write(0);
    Udp.write(0);
}

void sendType(int type)
{
    Udp.write(0);
    Udp.write(0);
    Udp.write(0);
    Udp.write(type);
}

void sendVector(float *const result, int type)
{
    if (Udp.beginPacket(host, port) > 0)
    {
        float x = result[0];
        float y = result[1];
        float z = result[2];
        float w = 0;
        sendType(type);
        sendPacketNumber();
        Udp.write(convert_to_chars(x, buf), sizeof(x));
        Udp.write(convert_to_chars(y, buf), sizeof(y));
        Udp.write(convert_to_chars(z, buf), sizeof(z));
        Udp.write(convert_to_chars(w, buf), sizeof(w));
        if (Udp.endPacket() == 0)
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
    if (Udp.beginPacket(host, port) > 0)
    {
        float x = quaternion->x;
        float y = quaternion->y;
        float z = quaternion->z;
        float w = quaternion->w;
        sendType(type);
        sendPacketNumber();
        Udp.write(convert_to_chars(x, buf), sizeof(x));
        Udp.write(convert_to_chars(y, buf), sizeof(y));
        Udp.write(convert_to_chars(z, buf), sizeof(z));
        Udp.write(convert_to_chars(w, buf), sizeof(w));
        if (Udp.endPacket() == 0)
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
    if (Udp.beginPacket(host, port) > 0)
    {
        float x = quaternion[0];
        float y = quaternion[1];
        float z = quaternion[2];
        float w = quaternion[3];
        sendType(type);
        sendPacketNumber();
        Udp.write(convert_to_chars(x, buf), sizeof(x));
        Udp.write(convert_to_chars(y, buf), sizeof(y));
        Udp.write(convert_to_chars(z, buf), sizeof(z));
        Udp.write(convert_to_chars(w, buf), sizeof(w));
        if (Udp.endPacket() == 0)
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
    if (Udp.beginPacket(host, port) > 0)
    {
        DeviceConfig data = *config;
        sendType(type);
        sendPacketNumber();
        Udp.write(convert_to_chars(data, buf), sizeof(data));
        if (Udp.endPacket() == 0)
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

void sendRawCalibrationData(int *const data, int type)
{
    if (Udp.beginPacket(host, port) > 0)
    {
        int ax = data[0];
        int ay = data[1];
        int az = data[2];
        int mx = data[3];
        int my = data[4];
        int mz = data[5];
        sendType(type);
        sendPacketNumber();
        Udp.write(convert_to_chars(ax, buf), sizeof(ax));
        Udp.write(convert_to_chars(ay, buf), sizeof(ay));
        Udp.write(convert_to_chars(az, buf), sizeof(az));
        Udp.write(convert_to_chars(mx, buf), sizeof(mx));
        Udp.write(convert_to_chars(my, buf), sizeof(my));
        Udp.write(convert_to_chars(mz, buf), sizeof(mz));
        if (Udp.endPacket() == 0)
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

void setConfigRecievedCallback(configRecievedCallback callback)
{
    fp_configCallback = callback;
}

void setCommandRecievedCallback(commandRecievedCallback callback)
{
    fp_commandCallback = callback;
}

void clientUpdate()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        int packetSize = Udp.parsePacket();
        if (packetSize)
        {
            // receive incoming UDP packets
            Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
            int len = Udp.read(incomingPacket, sizeof(incomingPacket));
            Serial.print("UDP packet contents: ");
            for (int i = 0; i < len; ++i)
                Serial.print((byte)incomingPacket[i]);
            Serial.println();

            switch (convert_chars<int>(incomingPacket))
            {
            case PACKET_RECIEVE_HEARTBEAT:
                break;
            case PACKET_RECIEVE_VIBRATE:
                if(fp_commandCallback) {
                    fp_commandCallback(COMMAND_BLINK, nullptr, 0);
                }
                break;
            case PACKET_RECIEVE_HANDSHAKE:
                // Assume handshake sucessful
                Serial.println("Handshale recived again, ignoring");
                break;
            case PACKET_RECIEVE_COMMAND:
                if (len < 6)
                {
                    Serial.println("Command packet too short");
                    break;
                }
                Serial.printf("Recieved command %d\n", incomingPacket[4]);
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
            }
        }
    }
}

void connectClient()
{
    Serial.print("Connecting to wifi ");
    Serial.println(networkName);
    WiFi.mode(WIFI_STA);
    WiFi.begin(networkName, networkPassword);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Connecting to owoTrack server");
    Udp.begin(port);
    while (true)
    {
        int packetSize = Udp.parsePacket();
        if (packetSize)
        {
            // receive incoming UDP packets
            Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
            int len = Udp.read(incomingPacket, sizeof(incomingPacket));
            Serial.print("UDP packet contents: ");
            for (int i = 0; i < len; ++i)
                Serial.print((byte)incomingPacket[i]);
            Serial.println();

            switch (incomingPacket[0])
            {
            case PACKET_HANDSHAKE:
                // Assume handshake sucessful
                Serial.println("Handshale sucessful");
                break;
            }
        }
        Serial.println("Sending handshake...");
        if (Udp.beginPacket(host, port) > 0)
        {
            Udp.write(handshake, 12);
            if (Udp.endPacket() == 0)
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
        delay(1000);
    }
}