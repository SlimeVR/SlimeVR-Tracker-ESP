#include "MPU9250.h"
#include "sensor.h"
#include "udpclient.h"

void gatherCalibrationData(MPU9250 &imu);

void MPU6050Sensor::motionSetup(DeviceConfig * config) {
    // initialize device
    imu.initialize();
    devStatus = imu.dmpInitialize();

    imu.setXGyroOffset(config->calibration.G_off[0]);
    imu.setYGyroOffset(config->calibration.G_off[1]);
    imu.setZGyroOffset(config->calibration.G_off[2]);
    imu.setXAccelOffset(config->calibration.A_B[0]);
    imu.setYAccelOffset(config->calibration.A_B[1]);
    imu.setZAccelOffset(config->calibration.A_B[2]);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        imu.setDMPEnabled(true);
        mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void MPU6050Sensor::motionLoop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    mpuIntStatus = imu.getIntStatus();

    // get current FIFO count
    fifoCount = imu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        imu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = imu.getFIFOCount();

        // read a packet from FIFO
        imu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        imu.dmpGetQuaternion(&rawQuat, fifoBuffer);
        q[0] = rawQuat.x;
        q[1] = rawQuat.y;
        q[2] = rawQuat.z;
        q[3] = rawQuat.w;
        quaternion.set(-q[1], q[0], q[2], q[3]);
        quaternion *= sensorOffset;
    }
}

void MPU6050Sensor::sendData() {
    sendQuat(&quaternion, PACKET_ROTATION);
}

void MPU6050Sensor::startCalibration(int calibrationType) {
    digitalWrite(CALIBRATING_LED, LOW);
    Serial.println("Starting offset finder");
    gatherCalibrationData(imu);
    Serial.println("Process is over");
    digitalWrite(CALIBRATING_LED, HIGH);
}

void gatherCalibrationData(MPU9250 &imu) {
    Serial.println("Gathering raw data for device calibration...");
    int calibrationSamples = 500;
    // Reset values
    float Gxyz[3];
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;

    // Wait for sensor to calm down before calibration
    Serial.println("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
        delay(10);
    }
    Gxyz[0] /= calibrationSamples;
    Gxyz[1] /= calibrationSamples;
    Gxyz[2] /= calibrationSamples;
    Serial.printf("Gyro calibration results: %f %f %f\n", Gxyz[0], Gxyz[1], Gxyz[2]);
    sendVector(Gxyz, PACKET_GYRO_CALIBRATION_DATA);

    // Blink calibrating led before user should rotate the sensor
    Serial.println("Gently rotate the device while it's gathering accelerometer data");
    for (int i = 0; i < 3000 / 310; ++i)
    {
        digitalWrite(CALIBRATING_LED, LOW);
        delay(15);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(300);
    }
    int calibrationData[6];
    for (int i = 0; i < calibrationSamples; i++)
    {
        digitalWrite(CALIBRATING_LED, LOW);
        imu.getAcceleration(&ax, &ay, &az);
        calibrationData[0] = ax;
        calibrationData[1] = ay;
        calibrationData[2] = az;
        calibrationData[3] = 0;
        calibrationData[4] = 0;
        calibrationData[5] = 0;
        sendRawCalibrationData(calibrationData, PACKET_RAW_CALIBRATION_DATA);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(50);
    }
    Serial.println("Calibration data gathered and sent");
}