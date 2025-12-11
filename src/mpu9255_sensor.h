/**
 * @file mpu9255_sensor.h
 * @brief MPU9255 10-DOF IMU Sensor (Gyro + Accelerometer + Compass + Barometer)
 * @details MPU9255 (gyro+accel) + AK8963 (magnetometer) via I2C
 *          BMP180 barometer/temperature sensor (not included here, separate handling)
 * 
 * Hardware: 10 DOF IMU Module with MPU9255 and BMP180
 * Interface: I2C (SCL: PB6, SDA: PB7 on STM32)
 * 
 * @author Arduino-STM32 Port
 * @date 2025
 */

#ifndef MPU9255_SENSOR_H
#define MPU9255_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

// MPU9255 I2C Address
#define MPU9255_ADDR         0x68  // Default I2C address (can be 0x69 with AD0 high)
#define AK8963_ADDR          0x0C  // Magnetometer I2C address (inside MPU9255)

// MPU9255 Registers
#define MPU9255_WHO_AM_I     0x75
#define MPU9255_PWR_MGMT_1   0x6B
#define MPU9255_PWR_MGMT_2   0x6C
#define MPU9255_CONFIG       0x1A
#define MPU9255_GYRO_CONFIG  0x1B
#define MPU9255_ACCEL_CONFIG 0x1C
#define MPU9255_ACCEL_XOUT_H 0x3B
#define MPU9255_GYRO_XOUT_H  0x43
#define MPU9255_TEMP_OUT_H   0x41

// Magnetometer registers (AK8963)
#define AK8963_WHO_AM_I      0x00
#define AK8963_CNTL1         0x0A
#define AK8963_XOUT_L        0x03

// Scale factors
#define ACCEL_SCALE_2G       16384.0f  // LSB/g for ±2g range
#define GYRO_SCALE_250DPS    131.0f    // LSB/(°/s) for ±250°/s range
#define MAG_SCALE            0.15f     // µT/LSB for AK8963

/**
 * @brief MPU9255 IMU Sensor Class
 */
class MPU9255Sensor {
private:
    TwoWire* wire;
    uint8_t mpuAddr;
    
    // Calibration offsets
    float accelOffsetX, accelOffsetY, accelOffsetZ;
    float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    
    /**
     * @brief Write a byte to MPU9255 register
     */
    void writeRegister(uint8_t reg, uint8_t value) {
        wire->beginTransmission(mpuAddr);
        wire->write(reg);
        wire->write(value);
        wire->endTransmission();
    }
    
    /**
     * @brief Read a byte from MPU9255 register
     */
    uint8_t readRegister(uint8_t reg) {
        wire->beginTransmission(mpuAddr);
        wire->write(reg);
        wire->endTransmission(false);
        wire->requestFrom(mpuAddr, (uint8_t)1);
        return wire->read();
    }
    
    /**
     * @brief Read multiple bytes from MPU9255
     */
    void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
        wire->beginTransmission(mpuAddr);
        wire->write(reg);
        wire->endTransmission(false);
        wire->requestFrom(mpuAddr, length);
        for (uint8_t i = 0; i < length; i++) {
            buffer[i] = wire->read();
        }
    }

public:
    /**
     * @brief Constructor
     * @param i2c I2C interface pointer (&Wire)
     * @param address MPU9255 I2C address (default: 0x68)
     */
    MPU9255Sensor(TwoWire* i2c = &Wire, uint8_t address = MPU9255_ADDR)
        : wire(i2c), mpuAddr(address),
          accelOffsetX(0), accelOffsetY(0), accelOffsetZ(0),
          gyroOffsetX(0), gyroOffsetY(0), gyroOffsetZ(0) {
    }
    
    /**
     * @brief Initialize MPU9255 sensor
     * @return true if successful
     */
    bool begin() {
        wire->begin();
        wire->setClock(400000);  // 400kHz I2C
        
        delay(100);
        
        // Check WHO_AM_I
        uint8_t whoami = readRegister(MPU9255_WHO_AM_I);
        if (whoami != 0x73 && whoami != 0x71) {  // MPU9255/MPU9250
            return false;
        }
        
        // Reset device
        writeRegister(MPU9255_PWR_MGMT_1, 0x80);
        delay(100);
        
        // Wake up and set clock source
        writeRegister(MPU9255_PWR_MGMT_1, 0x01);
        delay(10);
        
        // Enable all sensors
        writeRegister(MPU9255_PWR_MGMT_2, 0x00);
        delay(10);
        
        // Configure gyro: ±250°/s
        writeRegister(MPU9255_GYRO_CONFIG, 0x00);
        
        // Configure accel: ±2g
        writeRegister(MPU9255_ACCEL_CONFIG, 0x00);
        
        // Set sample rate divider and DLPF
        writeRegister(MPU9255_CONFIG, 0x03);  // DLPF 41Hz
        
        return true;
    }
    
    /**
     * @brief Calibrate gyroscope (must be stationary)
     * @param samples Number of samples for calibration
     */
    void calibrateGyro(uint16_t samples = 1000) {
        float sumX = 0, sumY = 0, sumZ = 0;
        
        for (uint16_t i = 0; i < samples; i++) {
            int16_t gx, gy, gz;
            readRawGyro(gx, gy, gz);
            sumX += gx;
            sumY += gy;
            sumZ += gz;
            delay(3);
        }
        
        gyroOffsetX = sumX / samples;
        gyroOffsetY = sumY / samples;
        gyroOffsetZ = sumZ / samples;
    }
    
    /**
     * @brief Read raw accelerometer data
     */
    void readRawAccel(int16_t &ax, int16_t &ay, int16_t &az) {
        uint8_t buffer[6];
        readRegisters(MPU9255_ACCEL_XOUT_H, buffer, 6);
        ax = (int16_t)((buffer[0] << 8) | buffer[1]);
        ay = (int16_t)((buffer[2] << 8) | buffer[3]);
        az = (int16_t)((buffer[4] << 8) | buffer[5]);
    }
    
    /**
     * @brief Read raw gyroscope data
     */
    void readRawGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
        uint8_t buffer[6];
        readRegisters(MPU9255_GYRO_XOUT_H, buffer, 6);
        gx = (int16_t)((buffer[0] << 8) | buffer[1]);
        gy = (int16_t)((buffer[2] << 8) | buffer[3]);
        gz = (int16_t)((buffer[4] << 8) | buffer[5]);
    }
    
    /**
     * @brief Read accelerometer in g (gravity units)
     */
    void readAccel(float &ax, float &ay, float &az) {
        int16_t rawX, rawY, rawZ;
        readRawAccel(rawX, rawY, rawZ);
        ax = (rawX - accelOffsetX) / ACCEL_SCALE_2G;
        ay = (rawY - accelOffsetY) / ACCEL_SCALE_2G;
        az = (rawZ - accelOffsetZ) / ACCEL_SCALE_2G;
    }
    
    /**
     * @brief Read gyroscope in degrees/second
     */
    void readGyro(float &gx, float &gy, float &gz) {
        int16_t rawX, rawY, rawZ;
        readRawGyro(rawX, rawY, rawZ);
        gx = (rawX - gyroOffsetX) / GYRO_SCALE_250DPS;
        gy = (rawY - gyroOffsetY) / GYRO_SCALE_250DPS;
        gz = (rawZ - gyroOffsetZ) / GYRO_SCALE_250DPS;
    }
    
    /**
     * @brief Read temperature in Celsius
     */
    float readTemperature() {
        uint8_t buffer[2];
        readRegisters(MPU9255_TEMP_OUT_H, buffer, 2);
        int16_t rawTemp = (int16_t)((buffer[0] << 8) | buffer[1]);
        return (rawTemp / 333.87f) + 21.0f;
    }
    
    /**
     * @brief Read all IMU data at once
     */
    void readAll(float &ax, float &ay, float &az,
                 float &gx, float &gy, float &gz,
                 float &temp) {
        readAccel(ax, ay, az);
        readGyro(gx, gy, gz);
        temp = readTemperature();
    }
    
    /**
     * @brief Calculate pitch and roll angles from accelerometer
     */
    void calculateAngles(float ax, float ay, float az, float &pitch, float &roll) {
        pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;
        roll = atan2(-ax, az) * 180.0f / PI;
    }
    
    /**
     * @brief Print formatted output
     */
    void printFormatted(char* buffer, size_t bufferSize) {
        float ax, ay, az, gx, gy, gz, temp;
        readAll(ax, ay, az, gx, gy, gz, temp);
        
        float pitch, roll;
        calculateAngles(ax, ay, az, pitch, roll);
        
        snprintf(buffer, bufferSize,
                "IMU: Accel[%.2f,%.2f,%.2f]g Gyro[%.2f,%.2f,%.2f]°/s "
                "Angle[P:%.1f° R:%.1f°] Temp:%.1f°C",
                ax, ay, az, gx, gy, gz, pitch, roll, temp);
    }
};

#endif // MPU9255_SENSOR_H
