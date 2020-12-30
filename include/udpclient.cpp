#include "udpclient.h"

///////////////////////////////////////////////////////////////////
//Wifi
///////////////////////////////////////////////////////////////////
#include "wificredentials.h"

WiFiUDP Udp;
char incomingPacket[64];  // buffer for incoming packets
uint64_t packetNumber = 1;
unsigned char handshake[12] = {0,0,0,3,0,0,0,0,0,0,0,0};
unsigned char buf[64];

template<typename T>
unsigned char* convert_to_chars(T src, unsigned char* target) {
	union {
		unsigned char c[sizeof(T)];
		T v;
	} un;
	un.v = src;
  for (int i = 0; i < sizeof(T); i++) {
		target[i] = un.c[sizeof(T) - i - 1];
	}
	return target;
}

template<typename T>
T convert_chars(unsigned char* src) {
	union {
		unsigned char c[sizeof(T)];
		T v;
	} un;
	for (int i = 0; i < sizeof(T); i++) {
		un.c[i] = src[sizeof(T) - i - 1];
	}
	return un.v;
}

void sendVector(float * result, int type) {
  if(Udp.beginPacket(host, port) > 0) {
      float x = result[0];
      float y = result[1];
      float z = result[2];
      float w = 0;
      uint64_t pn = packetNumber++;
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(type);
      // TODO Send packet number
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(convert_to_chars(x, buf), sizeof(x));
      Udp.write(convert_to_chars(y, buf), sizeof(y));
      Udp.write(convert_to_chars(z, buf), sizeof(z));
      Udp.write(convert_to_chars(w, buf), sizeof(w));
      if(Udp.endPacket() == 0) {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
      }
    } else {
      //Serial.print("Write error: ");
      //Serial.println(Udp.getWriteError());
    }
}

void sendQuat(Quat * quaternion, int type) {
  if(Udp.beginPacket(host, port) > 0) {
      float x = quaternion->x;
      float y = quaternion->y;
      float z = quaternion->z;
      float w = quaternion->w;
      uint64_t pn = packetNumber++;
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(type);
      // TODO Send packet number
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(convert_to_chars(x, buf), sizeof(x));
      Udp.write(convert_to_chars(y, buf), sizeof(y));
      Udp.write(convert_to_chars(z, buf), sizeof(z));
      Udp.write(convert_to_chars(w, buf), sizeof(w));
      if(Udp.endPacket() == 0) {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
      }
    } else {
      //Serial.print("Write error: ");
      //Serial.println(Udp.getWriteError());
    }
}

void sendQuat(float * quaternion, int type) {
  if(Udp.beginPacket(host, port) > 0) {
      float x = quaternion[0];
      float y = quaternion[1];
      float z = quaternion[2];
      float w = quaternion[3];
      uint64_t pn = packetNumber++;
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(type);
      // TODO Send packet number
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(0);
      Udp.write(convert_to_chars(x, buf), sizeof(x));
      Udp.write(convert_to_chars(y, buf), sizeof(y));
      Udp.write(convert_to_chars(z, buf), sizeof(z));
      Udp.write(convert_to_chars(w, buf), sizeof(w));
      if(Udp.endPacket() == 0) {
        //Serial.print("Write error: ");
        //Serial.println(Udp.getWriteError());
      }
    } else {
      //Serial.print("Write error: ");
      //Serial.println(Udp.getWriteError());
    }
}

void connectClient() {
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
  while(true) {
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      // receive incoming UDP packets
      Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(incomingPacket, 64);
      Serial.print("UDP packet contents: ");
      for(int i = 0; i < len; ++i)
        Serial.print((byte) incomingPacket[i]);
      Serial.println();

      if(incomingPacket[0] == 3) {
        // Assume handshake sucessful
        Serial.printf("Handshale sucessful");
        break;
      }
    }
    Serial.println("Sending handshake...");
    if(Udp.beginPacket(host, port) > 0) {
      Udp.write(handshake, 12);
      if(Udp.endPacket() == 0) {
        Serial.print("Write error: ");
        Serial.println(Udp.getWriteError());
      }
    } else {
      Serial.print("Write error: ");
      Serial.println(Udp.getWriteError());
    }
    delay(1000);
  }
}