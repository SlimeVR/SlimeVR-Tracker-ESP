// program to collect raw data from MPU9250 accel and magnetometer for later correction.
// output in form suitable for .csv file
// Use Magneto 1.2 to calculate corrections for BOTH accelerometer and magnetometer
// http://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html
// S. J. Remington 3/2020

//USAGE:
// 1. Allow the sensor to sit still upon program startup for gyro bias data collection.
// 2. TURN THE SENSOR VERY SLOWLY AND CAREFULLY DURING DATA COLLECTION TO AVOID EXCESS ACCELERATION!

#include "Wire.h"
// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.cpp"
#include "MPU9250.cpp"
#include "defines.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 imu;
I2Cdev I2C_M;

#define sample_num 300 //acc/mag scaling points to collect

uint8_t buffer_m[6]; //six points mag

//raw data and scaled as vector
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3] = {0};
float Gxyz[3] = {0};
float Mxyz[3] = {0};

//previously determined scale and offsets for accel and mag
float A_cal[6] = {0}; // 0..2 scale, 3..5 offsets
float M_cal[6] = {0};

void getAcc_Mag_raw(void)
{
  imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (float)ax;
  Axyz[1] = (float)ay;
  Axyz[2] = (float)az;
  Mxyz[0] = (float)mx;
  Mxyz[1] = (float)my;
  Mxyz[2] = (float)mz;
}

void getGyro_scaled(void)
{
  imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (float)gx * 250 / 32768; //250 LSB(d/s)
  Gxyz[1] = (float)gy * 250 / 32768;
  Gxyz[2] = (float)gz * 250 / 32768;
}

int N = sample_num;
void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(serialBaudRate);

  // initialize device
  Serial.println("Initializing I2C devices...");
  imu.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(imu.testConnection() ? "MPU9250 OK" : "MPU9250 ??");
  Serial.println("Gyro bias collection ... KEEP SENSOR STILL");
  delay(2000);
  for (int i = 0; i < N; i++)
  {
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] += float(gx);
    Gxyz[1] += float(gy);
    Gxyz[2] += float(gz);
  }
  Serial.print("Done. Gyro offsets (raw) ");
  Serial.print(Gxyz[0] / N, 1);
  Serial.print(", ");
  Serial.print(Gxyz[1] / N, 1);
  Serial.print(", ");
  Serial.print(Gxyz[2] / N, 1);
  Serial.println();

  Serial.print("Collecting ");
  Serial.print(N);
  Serial.println(" points for scaling, 3/second");
  Serial.println("TURN SENSOR VERY SLOWLY AND CAREFULLY IN 3D");
  delay(2000);

  float M_mag = 0, A_mag = 0;
  int i, j;
  j = N;
  while (j-- >= 0)
  {
    getAcc_Mag_raw();
    for (i = 0; i < 3; i++)
    {
      M_mag += Mxyz[i] * Mxyz[i];
      A_mag += Axyz[i] * Axyz[i];
    }
    Serial.print(ax);
    Serial.print(" ");
    Serial.print(ay);
    Serial.print(" ");
    Serial.print(az);
    Serial.print(" ");
    Serial.print(mx);
    Serial.print(" ");
    Serial.print(my);
    Serial.print(" ");
    Serial.println(mz);
    delay(300);
  }
  Serial.print("Done. ");
  Serial.print("rms Acc = ");
  Serial.print(sqrt(A_mag / N));
  Serial.print(", rms Mag = ");
  Serial.println(sqrt(M_mag / N));
}

void loop()
{
}
