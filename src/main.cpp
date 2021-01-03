//
// MPU-9250 Mahony AHRS  S.J. Remington 3/2020
// last update 12/17/2020
// added full matrix calibration for accel and mag

// ***Standard orientation defined by gyro/accel: X North Y West Z Up***

// VERY VERY IMPORTANT!
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work, and the gyro offset must be determned.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for calibration, use the companion programs MPU9250_cal and Magneto 1.2 from sailboatinstruments.blogspot.com
//
// For correcting the data, below I use the diagonal element of matrix A and ignore
// the off diagonal components. If those terms are large, (most likely only for the magnetometer)
// add them in to the corrections in function get_MPU_scaled()
//
// This version must be compiled with library routines in subfolder "libs"

#include "Wire.h"
// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.cpp"
#include "MPU9250.cpp"
#include "udpclient.cpp"
#include "defines.h"
#include "quat.cpp"
#include "util.cpp"
#include "configuration.cpp"

#define LOADING_LED LED_BUILTIN
#define CALIBRATING_LED LED_BUILTIN
#define sensorIdTime 1000
#define sensorIdInterval 100

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for Sparkfun module)
// AD0 high = 0x69
// MAHONY FILTER SELECTED BELOW

MPU9250 accelgyro;
I2Cdev I2C_M;
DeviceConfig config;
const CalibrationConfig &calibration = config.calibration;

//raw data and scaled as vector
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
bool isCalibrating = false;
#define gscale (250. / 32768.0) * (PI / 180.0) //gyro default 250 LSB per d/s -> rad/s

// NOW USING MAHONY FILTER

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
#define Kp 8.0
#define Ki 0.1

// Loop timing globals
unsigned long now = 0, last = 0;   //micros() timers
float deltat = 0;                  //loop time in seconds
unsigned long now_ms, last_ms = 0; //millis() timers
unsigned long update_ms = 20;      //send updates every "update_ms" milliseconds
bool blinking = false;
unsigned long blinkStart = 0;

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static const Quat rotationQuat = Quat(Vector3(0, 0, 1), PI / 2.0); // Adjust rotation to match Android rotations

void get_MPU_scaled(void);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

void setConfig(DeviceConfig newConfig)
{
    config = newConfig;
    saveConfig(&config);
}

void commandRecieved(int command, void *const data, int dataLength)
{
    switch (command)
    {
    case COMMAND_CALLIBRATE:
        isCalibrating = true;
        break;
    case COMMAND_SEND_CONFIG:
        sendConfig(&config, PACKET_CONFIG);
        break;
    case COMMAND_BLINK:
        blinking = true;
        blinkStart = now_ms;
        break;
    }
}

void performCalibration();
void processBlinking();

void setup()
{
    // Glow diode while loading
    pinMode(LOADING_LED, OUTPUT);
    pinMode(CALIBRATING_LED, OUTPUT);
    digitalWrite(CALIBRATING_LED, HIGH);
    digitalWrite(LOADING_LED, LOW);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Serial.begin(serialBaudRate);
    while (!Serial)
        ; // wait for connection

    // Load default calibration values and set up callbacks
    config.calibration = {
        {-807.71, 136.80, 290.84},
        {{0.60963, -0.00451, 0.00042},
         {-0.00451, 0.62762, 0.00492},
         {0.00042, 0.00492, 0.60485}},
        {32.06, 34.44, -97.59},
        {{1.68116, 0.08451, -0.01659},
         {0.08451, 1.55939, 0.06883},
         {-0.01659, 0.06883, 1.61732}},
        {421.3, -236.7, 4.8}};
    if (hasConfigStored())
    {
        loadConfig(&config);
    }
    setConfigRecievedCallback(setConfig);
    setCommandRecievedCallback(commandRecieved);

    // initialize device
    accelgyro.initialize();

    // Don't start if not connected to MPU
    /*while(!accelgyro.testConnection()) {
        Serial.print("Can't communicate with MPU9250, response ");
        Serial.println(accelgyro.getDeviceID(), HEX);
        delay(500);
    }
    Serial.println("Connected to MPU9250");
    //*/

    connectClient();
    digitalWrite(LOADING_LED, HIGH);
}

// AHRS loop

void loop()
{
    clientUpdate();
    if (isCalibrating)
    {
        performCalibration();
    }
    get_MPU_scaled();
    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;
    processBlinking();

    // correct for differing accelerometer and magnetometer alignment by circularly permuting mag axes
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                           Mxyz[1], Mxyz[0], -Mxyz[2], deltat);

    now_ms = millis();
    if (now_ms - last_ms >= update_ms)
    {
        last_ms = now_ms;
        Quat cq = {};
        //cq.set(q[0], q[1], q[2], q[3]);
        cq.set(-q[1], -q[2], -q[0], q[3]);
        cq *= rotationQuat;

        sendQuat(&cq, PACKET_ROTATION);
        sendQuat(Axyz, PACKET_ACCEL);
        //sendQuat(Mxyz, PACKET_MAG);
        sendQuat(Gxyz, PACKET_GYRO);
    }
}

void get_MPU_scaled(void)
{
    float temp[3];
    int i;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = ((float)gx - calibration.G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy - calibration.G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - calibration.G_off[2]) * gscale;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;
    //apply offsets (bias) and scale factors from Magneto
    for (i = 0; i < 3; i++)
        temp[i] = (Axyz[i] - calibration.A_B[i]);
    Axyz[0] = calibration.A_Ainv[0][0] * temp[0] + calibration.A_Ainv[0][1] * temp[1] + calibration.A_Ainv[0][2] * temp[2];
    Axyz[1] = calibration.A_Ainv[1][0] * temp[0] + calibration.A_Ainv[1][1] * temp[1] + calibration.A_Ainv[1][2] * temp[2];
    Axyz[2] = calibration.A_Ainv[2][0] * temp[0] + calibration.A_Ainv[2][1] * temp[1] + calibration.A_Ainv[2][2] * temp[2];
    vector_normalize(Axyz);

    Mxyz[0] = (float)mx;
    Mxyz[1] = (float)my;
    Mxyz[2] = (float)mz;
    //apply offsets and scale factors from Magneto
    for (i = 0; i < 3; i++)
        temp[i] = (Mxyz[i] - calibration.M_B[i]);
    Mxyz[0] = calibration.M_Ainv[0][0] * temp[0] + calibration.M_Ainv[0][1] * temp[1] + calibration.M_Ainv[0][2] * temp[2];
    Mxyz[1] = calibration.M_Ainv[1][0] * temp[0] + calibration.M_Ainv[1][1] * temp[1] + calibration.M_Ainv[1][2] * temp[2];
    Mxyz[2] = calibration.M_Ainv[2][0] * temp[0] + calibration.M_Ainv[2][1] * temp[1] + calibration.M_Ainv[2][2] * temp[2];
    vector_normalize(Mxyz);
}

// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    // Vector to hold integral error for Mahony method
    static float eInt[3] = {0.0, 0.0, 0.0};
    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;
    /*
    // already done in loop()

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;
  */
    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of the reference vectors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
        eInt[0] += ex; // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
        // Apply I feedback
        gx += Ki * eInt[0];
        gy += Ki * eInt[1];
        gz += Ki * eInt[2];
    }

    // Apply P feedback
    gx = gx + Kp * ex;
    gy = gy + Kp * ey;
    gz = gz + Kp * ez;

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void performCalibration()
{
    digitalWrite(CALIBRATING_LED, LOW);
    Serial.println("Gathering raw data for device calibration...");
    int calibrationSamples = 300;
    // Reset values
    Gxyz[0] = 0;
    Gxyz[1] = 0;
    Gxyz[2] = 0;

    // Wait for sensor to calm down before calibration
    Serial.println("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
    }
    Gxyz[0] /= calibrationSamples;
    Gxyz[1] /= calibrationSamples;
    Gxyz[2] /= calibrationSamples;
    Serial.printf("Gyro calibration results: %f %f %f\n", Gxyz[0], Gxyz[1], Gxyz[2]);
    sendVector(Gxyz, PACKET_GYRO_CALIBRATION_DATA);

    // Blink calibrating led before user should rotate the sensor
    Serial.println("Gently rotate the device while it's gathering accelerometer and magnetometer data");
    for (int i = 0; i < 2000 / 20; ++i)
    {
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(2000 / 10);
        digitalWrite(CALIBRATING_LED, LOW);
    }
    int calibrationData[6];
    for (int i = 0; i < calibrationSamples; i++)
    {
        digitalWrite(CALIBRATING_LED, LOW);
        accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        calibrationData[0] = ax;
        calibrationData[1] = ay;
        calibrationData[2] = az;
        calibrationData[3] = mx;
        calibrationData[4] = my;
        calibrationData[5] = mz;
        sendRawCalibrationData(calibrationData, PACKET_RAW_CALIBRATION_DATA);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(300);
    }
    Serial.println("Calibration data gathered and sent");
    digitalWrite(CALIBRATING_LED, HIGH);
}

void processBlinking() {
    if (blinking)
    {
        if (blinkStart + sensorIdTime < now)
        {
            blinking = false;
            digitalWrite(LOADING_LED, HIGH);
        }
        else
        {
            int t = (now - blinkStart) / sensorIdInterval;
            if(t % 2) {
                digitalWrite(LOADING_LED, LOW);
            } else {
                digitalWrite(LOADING_LED, HIGH);
            }
        }
        
    }
}