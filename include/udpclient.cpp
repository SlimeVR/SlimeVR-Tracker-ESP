#include "udpclient.h"
#include "configuration.h"

#define TIMEOUT 3000

WiFiUDP Udp;
unsigned char incomingPacket[128]; // buffer for incoming packets
uint64_t packetNumber = 1;
unsigned char handshake[12] = {0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char buf[128];
configRecievedCallback fp_configCallback;
commandRecievedCallback fp_commandCallback;

IPAddress broadcast = IPAddress(255, 255, 255, 255);

int port = 6969;
IPAddress host;
bool connected = false;
long lastConnectionAttemptMs;
long lastPacketMs;

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
            lastPacketMs = millis();
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
        if(connected && (lastPacketMs + TIMEOUT < millis()))
            connected = false;
        if(!connected)
            connectClient();
    }
}

bool startWPSPBC() {
    // from https://gist.github.com/copa2/fcc718c6549721c210d614a325271389
    // wpstest.ino
    Serial.println("WPS config start");
    bool wpsSuccess = WiFi.beginWPSConfig();
    if(wpsSuccess) {
        // Well this means not always success :-/ in case of a timeout we have an empty ssid
        String newSSID = WiFi.SSID();
        if(newSSID.length() > 0) {
            // WPSConfig has already connected in STA mode successfully to the new station. 
            Serial.printf("WPS finished. Connected successfully to SSID '%s'\n", newSSID.c_str());
        } else {
            wpsSuccess = false;
        }
    }
    return wpsSuccess; 
}

void setUpWiFi(DeviceConfig * const config) {
    Serial.print("Connecting to wifi ");
    WiFi.mode(WIFI_STA);

    WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str());
    while (WiFi.status() == WL_DISCONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    if(WiFi.status() != WL_CONNECTED) {
        Serial.printf("\nCould not connect to WiFi. state='%d'\n", WiFi.status());
        Serial.println("Please press WPS button on your router, until mode is indicated.");

        while(true) {
            if(!startWPSPBC()) {
                Serial.println("Failed to connect with WPS");
            } else {
                WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str()); // reading data from EPROM, 
                while (WiFi.status() == WL_DISCONNECTED) {          // last saved credentials
                    delay(500);
                    Serial.print("."); // show wait for connect to AP
                }
            }
            if(WiFi.status() == WL_CONNECTED)
                break;
            delay(1000);
        }
    }
    Serial.printf("\nConnected successfully to SSID '%s', ip address %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

    Udp.begin(port);
}

void connectClient()
{
    long now = millis();
    while(true) {
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
                host = Udp.remoteIP();
                port = Udp.remotePort();
                lastPacketMs = now;
                connected = true;
                digitalWrite(LOADING_LED, HIGH);
                Serial.printf("Handshale sucessful, server is %s:%d", Udp.remoteIP().toString().c_str(), + Udp.remotePort());
                return;
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
        if (Udp.beginPacket(broadcast, port) > 0)
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
        
        digitalWrite(LOADING_LED, LOW);
    }
    else if(lastConnectionAttemptMs + 20 < now)
    {
        digitalWrite(LOADING_LED, HIGH);
    }
}