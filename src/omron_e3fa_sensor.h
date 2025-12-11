/**
 * @file omron_e3fa_sensor.h
 * @brief Omron E3FA-RP11 Reflective Photoelectric Sensor
 * @details NPN transistor output, detects objects up to 1 meter
 *          Output: HIGH when object detected, LOW when no object
 * 
 * Hardware: Omron E3FA-RP11 Photoelectric Sensor (x2)
 * Interface: Digital input with interrupt capability
 * 
 * @author Arduino-STM32 Port
 * @date 2025
 */

#ifndef OMRON_E3FA_SENSOR_H
#define OMRON_E3FA_SENSOR_H

#include <Arduino.h>

// Callback function type
typedef void (*E3FACallback)(void);

/**
 * @brief Omron E3FA-RP11 Photoelectric Sensor Class
 */
class OmronE3FASensor {
private:
    uint8_t pin;
    const char* name;
    E3FACallback callback;
    volatile bool detected;
    volatile bool stateChanged;
    
    // Debounce
    unsigned long lastChangeTime;
    unsigned long debounceDelay;
    
    // State tracking
    bool lastState;
    bool currentState;
    
    // Detection counter
    volatile uint32_t detectionCount;

public:
    /**
     * @brief Constructor
     * @param sensorPin Digital pin number
     * @param sensorName Sensor identifier
     * @param debounceMs Debounce delay in milliseconds
     */
    OmronE3FASensor(uint8_t sensorPin, const char* sensorName = "E3FA", unsigned long debounceMs = 50)
        : pin(sensorPin),
          name(sensorName),
          callback(nullptr),
          detected(false),
          stateChanged(false),
          lastChangeTime(0),
          debounceDelay(debounceMs),
          lastState(false),
          currentState(false),
          detectionCount(0) {
    }
    
    /**
     * @brief Initialize sensor
     * @param mode Pin mode (default: INPUT)
     */
    void begin(uint8_t mode = INPUT) {
        pinMode(pin, mode);
        lastState = digitalRead(pin);
        currentState = lastState;
        lastChangeTime = millis();
    }
    
    /**
     * @brief Attach interrupt for detection events
     * @param cb Callback function
     * @param mode Interrupt mode (RISING for object detected)
     */
    void attachInterrupt(E3FACallback cb, uint8_t mode = RISING) {
        callback = cb;
        ::attachInterrupt(digitalPinToInterrupt(pin), cb, mode);
    }
    
    /**
     * @brief Detach interrupt
     */
    void detachInterrupt() {
        ::detachInterrupt(digitalPinToInterrupt(pin));
        callback = nullptr;
    }
    
    /**
     * @brief Read current sensor state
     * @return true if object detected
     */
    bool isDetected() {
        return digitalRead(pin) == HIGH;
    }
    
    /**
     * @brief Update sensor state (call in loop for debouncing)
     */
    void update() {
        bool reading = digitalRead(pin);
        unsigned long currentTime = millis();
        
        if (reading != lastState) {
            lastChangeTime = currentTime;
        }
        
        if ((currentTime - lastChangeTime) >= debounceDelay) {
            if (reading != currentState) {
                currentState = reading;
                stateChanged = true;
                
                if (currentState == HIGH) {
                    detected = true;
                    detectionCount++;
                }
            }
        }
        
        lastState = reading;
    }
    
    /**
     * @brief Check if state changed since last check
     * @return true if state changed
     */
    bool hasStateChanged() {
        if (stateChanged) {
            stateChanged = false;
            return true;
        }
        return false;
    }
    
    /**
     * @brief Check if new detection occurred
     * @return true if new detection
     */
    bool hasNewDetection() {
        if (detected) {
            detected = false;
            return true;
        }
        return false;
    }
    
    /**
     * @brief Trigger from ISR
     */
    void trigger() {
        detected = true;
        stateChanged = true;
    }
    
    /**
     * @brief Get total detection count
     */
    uint32_t getDetectionCount() const {
        return detectionCount;
    }
    
    /**
     * @brief Reset detection counter
     */
    void resetCounter() {
        detectionCount = 0;
    }
    
    /**
     * @brief Get sensor name
     */
    const char* getName() const {
        return name;
    }
    
    /**
     * @brief Get pin number
     */
    uint8_t getPin() const {
        return pin;
    }
    
    /**
     * @brief Print formatted status
     */
    void printStatus(char* buffer, size_t bufferSize) {
        currentState = isDetected();
        snprintf(buffer, bufferSize, 
                "%s: %s (Count:%lu Pin:%d)",
                name,
                currentState ? "DETECTED" : "CLEAR",
                (unsigned long)detectionCount,
                pin);
    }
};


/**
 * @brief Dual Omron E3FA Sensor Manager
 */
class DualE3FASensors {
private:
    OmronE3FASensor* sensor1;
    OmronE3FASensor* sensor2;

public:
    DualE3FASensors(OmronE3FASensor* s1, OmronE3FASensor* s2)
        : sensor1(s1), sensor2(s2) {
    }
    
    void begin() {
        sensor1->begin();
        sensor2->begin();
    }
    
    void update() {
        sensor1->update();
        sensor2->update();
    }
    
    void checkAndReport(HardwareSerial* serial) {
        if (sensor1->hasNewDetection()) {
            serial->print(sensor1->getName());
            serial->println(" -> Object Detected!");
        }
        
        if (sensor2->hasNewDetection()) {
            serial->print(sensor2->getName());
            serial->println(" -> Object Detected!");
        }
    }
    
    bool anyDetected() {
        return sensor1->isDetected() || sensor2->isDetected();
    }
    
    bool bothDetected() {
        return sensor1->isDetected() && sensor2->isDetected();
    }
    
    void printStatus(char* buffer, size_t bufferSize) {
        char buf1[60], buf2[60];
        sensor1->printStatus(buf1, sizeof(buf1));
        sensor2->printStatus(buf2, sizeof(buf2));
        snprintf(buffer, bufferSize, "%s | %s", buf1, buf2);
    }
    
    uint32_t getTotalDetections() {
        return sensor1->getDetectionCount() + sensor2->getDetectionCount();
    }
};

#endif // OMRON_E3FA_SENSOR_H
