#include "i2cscan.h"

uint8_t portArray[] = {16, 5, 4, 0, 2, 14, 12, 13};
String portMap[] = {"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7"};

namespace I2CSCAN
{
    void scani2cports()
    {
        bool found = false;
        for (uint8_t i = 0; i < sizeof(portArray); i++)
        {
            for (uint8_t j = 0; j < sizeof(portArray); j++)
            {
                if (i != j)
                {
                    if(checkI2C(i, j))
                        found = true;
                }
            }
        }
        if(!found) {
            Serial.println("[I2C SCAN] No I2C devices found");
        }
    }

    bool checkI2C(uint8_t i, uint8_t j)
    {
        bool found = false;
        Wire.begin(portArray[i], portArray[j]);
        byte error, address;
        int nDevices;
        nDevices = 0;
        for (address = 1; address < 127; address++)
        {
            // The i2c_scanner uses the return value of
            // the Write.endTransmisstion to see if
            // a device did acknowledge to the address.
            Wire.beginTransmission(address);
            error = Wire.endTransmission();

            if (error == 0)
            {
                Serial.print("[I2C SCAN] (@ " + portMap[i] + " : " + portMap[j] + ") ");
                Serial.print("I2C device found at address 0x");
                if (address < 16)
                    Serial.print("0");
                Serial.print(address, HEX);
                Serial.println("  !");

                nDevices++;
                found = true;
            }
            else if (error == 4)
            {
                Serial.print("[I2C SCAN] (@ " + portMap[i] + " : " + portMap[j] + ") ");
                Serial.print("Unknow error at address 0x");
                if (address < 16)
                    Serial.print("0");
                Serial.println(address, HEX);
            }
        }
        return found;
    }

    bool isI2CExist(uint8_t addr) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        if(error == 0)
            return true;
        return false;
    }
}
