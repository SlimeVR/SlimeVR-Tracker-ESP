//This is useful if you want to use an address shifter
#define DEFAULT_IMU_ADDRESS true

//We use fixed address values instead of scanning to keep the sensorID consistent and to avoid issues with some breakout board with multiple active ic2 addresses
#if DEFAULT_IMU_ADDRESS

    #if IMU == IMU_BNO080 || IMU == IMU_BNO085 || IMU == IMU_BNO086
        #define PRIMARY_IMU_ADDRESS_ONE 0x4A
        #define PRIMARY_IMU_ADDRESS_TWO 0x4B
    #elif IMU == IMU_BNO055
        #define PRIMARY_IMU_ADDRESS_ONE 0x29
        #define PRIMARY_IMU_ADDRESS_TWO 0x28
    #elif IMU == IMU_MPU9250 || IMU == IMU_BMI160 || IMU == IMU_MPU6500 || IMU == IMU_MPU6050 || IMU == IMU_ICM20948
        #define PRIMARY_IMU_ADDRESS_ONE 0x68
        #define PRIMARY_IMU_ADDRESS_TWO 0x69
    #endif

    #if SECOND_IMU == IMU_BNO080 || SECOND_IMU == IMU_BNO085 || SECOND_IMU == IMU_BNO086
        #define SECONDARY_IMU_ADDRESS_ONE 0x4A
        #define SECONDARY_IMU_ADDRESS_TWO 0x4B
    #elif SECOND_IMU == IMU_BNO055
        #define SECONDARY_IMU_ADDRESS_ONE 0x29
        #define SECONDARY_IMU_ADDRESS_TWO 0x28
    #elif SECOND_IMU == IMU_MPU9250 || SECOND_IMU == IMU_BMI160 || SECOND_IMU == IMU_MPU6500 || SECOND_IMU == IMU_MPU6050 || SECOND_IMU == IMU_ICM20948
        #define SECONDARY_IMU_ADDRESS_ONE 0x68
        #define SECONDARY_IMU_ADDRESS_TWO 0x69
    #endif

#else
    //If not using the default address you can set custom addresses here
    #define PRIMARY_IMU_ADDRESS_ONE 0x69
    #define PRIMARY_IMU_ADDRESS_TWO 0x68
    #define SECONDARY_IMU_ADDRESS_ONE 0x69
    #define SECONDARY_IMU_ADDRESS_TWO 0x68
#endif