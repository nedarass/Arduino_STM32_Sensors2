/**
 * @file omron_e6b2_encoder.h
 * @brief Omron E6B2-CWD6C Incremental Rotary Encoder
 * @details 600 P/R (Pulse Per Revolution), NPN output, push-pull
 *          Two channel quadrature output (A/B phases) for direction detection
 * 
 * Hardware: Omron E6B2-CWD6C 600P/R Rotary Encoder
 * Interface: 
 *   - Channel A: TIM1_CH1 (PA8) or TIM2_CH1 (PA0) or TIM3_CH1 (PA6)
 *   - Channel B: TIM1_CH2 (PA9) or TIM2_CH2 (PA1) or TIM3_CH2 (PA7)
 * 
 * @author Arduino-STM32 Port
 * @date 2025
 */

#ifndef OMRON_E6B2_ENCODER_H
#define OMRON_E6B2_ENCODER_H

#include <Arduino.h>

/**
 * @brief Omron E6B2 Encoder Class
 * @note Uses hardware timer in encoder mode for reliable counting
 */
class OmronE6B2Encoder {
private:
    uint8_t pinA;           // Channel A pin
    uint8_t pinB;           // Channel B pin
    const char* name;
    
    // Encoder parameters
    uint16_t pulsesPerRev;  // Pulses per revolution (600 for E6B2-CWD6C)
    
    // Position tracking (software mode)
    volatile int32_t position;
    volatile int32_t lastPosition;
    
    // Speed calculation
    unsigned long lastUpdateTime;
    float rpm;
    
    // State for software quadrature decoding
    volatile uint8_t lastEncoded;
    
    /**
     * @brief Software quadrature decoder lookup table
     */
    static const int8_t QEM[16];

public:
    /**
     * @brief Constructor
     * @param channelA Pin for encoder channel A
     * @param channelB Pin for encoder channel B
     * @param encoderName Identifier
     * @param ppr Pulses per revolution (default: 600)
     */
    OmronE6B2Encoder(uint8_t channelA, uint8_t channelB, 
                     const char* encoderName = "E6B2",
                     uint16_t ppr = 600)
        : pinA(channelA),
          pinB(channelB),
          name(encoderName),
          pulsesPerRev(ppr),
          position(0),
          lastPosition(0),
          lastUpdateTime(0),
          rpm(0.0f),
          lastEncoded(0) {
    }
    
    /**
     * @brief Initialize encoder
     */
    void begin() {
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
        
        // Read initial state
        lastEncoded = (digitalRead(pinA) << 1) | digitalRead(pinB);
        lastUpdateTime = millis();
    }
    
    /**
     * @brief Update encoder state (call from ISR or fast loop)
     * @note For software decoding
     */
    void update() {
        uint8_t encoded = (digitalRead(pinA) << 1) | digitalRead(pinB);
        uint8_t sum = (lastEncoded << 2) | encoded;
        position += QEM[sum & 0x0F];
        lastEncoded = encoded;
    }
    
    /**
     * @brief Get current position (pulses)
     */
    int32_t getPosition() const {
        return position;
    }
    
    /**
     * @brief Get position in revolutions
     */
    float getRevolutions() const {
        return (float)position / (float)pulsesPerRev;
    }
    
    /**
     * @brief Get position in degrees
     */
    float getDegrees() const {
        return (getRevolutions() * 360.0f);
    }
    
    /**
     * @brief Reset position counter
     */
    void resetPosition() {
        position = 0;
        lastPosition = 0;
    }
    
    /**
     * @brief Set position to specific value
     */
    void setPosition(int32_t pos) {
        position = pos;
    }
    
    /**
     * @brief Calculate speed (RPM)
     * @return Speed in RPM
     */
    float calculateRPM() {
        unsigned long currentTime = millis();
        unsigned long deltaTime = currentTime - lastUpdateTime;
        
        if (deltaTime >= 100) {  // Update every 100ms minimum
            int32_t deltaPos = position - lastPosition;
            
            // RPM = (pulses * 60000) / (pulsesPerRev * deltaTime_ms)
            rpm = (deltaPos * 60000.0f) / (pulsesPerRev * deltaTime);
            
            lastPosition = position;
            lastUpdateTime = currentTime;
        }
        
        return rpm;
    }
    
    /**
     * @brief Get last calculated RPM
     */
    float getRPM() const {
        return rpm;
    }
    
    /**
     * @brief Get angular velocity in degrees/second
     */
    float getAngularVelocity() const {
        return (rpm * 360.0f) / 60.0f;  // Convert RPM to deg/s
    }
    
    /**
     * @brief Get direction
     * @return 1 for CW, -1 for CCW, 0 for stopped
     */
    int8_t getDirection() const {
        if (rpm > 0.5f) return 1;   // Clockwise
        if (rpm < -0.5f) return -1; // Counter-clockwise
        return 0;                    // Stopped
    }
    
    /**
     * @brief Get encoder name
     */
    const char* getName() const {
        return name;
    }
    
    /**
     * @brief Print formatted output
     */
    void printFormatted(char* buffer, size_t bufferSize) {
        float revs = getRevolutions();
        float degs = getDegrees();
        int8_t dir = getDirection();
        
        const char* dirStr = (dir > 0) ? "CW" : (dir < 0) ? "CCW" : "STOP";
        
        snprintf(buffer, bufferSize,
                "%s: Pos:%ld Rev:%.2f Deg:%.1f° RPM:%.1f Dir:%s",
                name,
                (long)position,
                revs,
                degs,
                rpm,
                dirStr);
    }
    
    /**
     * @brief Get pulses per revolution
     */
    uint16_t getPPR() const {
        return pulsesPerRev;
    }
};

// Quadrature encoder state machine lookup table
// Returns +1, -1, or 0 based on state transitions
const int8_t OmronE6B2Encoder::QEM[16] = {
    0, -1,  1,  0,   // 00 -> 00, 01, 10, 11
    1,  0,  0, -1,   // 01 -> 00, 01, 10, 11
   -1,  0,  0,  1,   // 10 -> 00, 01, 10, 11
    0,  1, -1,  0    // 11 -> 00, 01, 10, 11
};


/**
 * @brief Hardware Timer-based Encoder (more accurate, recommended)
 * @note Requires specific timer pins on STM32
 * @note Temporarily disabled for USB testing
 */
#if 0  // Disabled - use software encoder for now
#ifdef STM32_CORE_VERSION
#include <HardwareTimer.h>

class OmronE6B2EncoderHW {
private:
    HardwareTimer* timer;
    TIM_TypeDef* instance;
    uint16_t pulsesPerRev;
    const char* name;
    
    int32_t lastCount;
    unsigned long lastTime;
    float rpm;

public:
    /**
     * @brief Constructor for hardware timer encoder
     * @param tim Timer instance (TIM1, TIM2, TIM3, etc.)
     * @param encoderName Identifier
     * @param ppr Pulses per revolution
     */
    OmronE6B2EncoderHW(TIM_TypeDef* tim, const char* encoderName = "E6B2", uint16_t ppr = 600)
        : instance(tim),
          pulsesPerRev(ppr),
          name(encoderName),
          lastCount(0),
          lastTime(0),
          rpm(0.0f) {
        timer = new HardwareTimer(instance);
    }
    
    /**
     * @brief Initialize hardware encoder mode
     */
    void begin() {
        // Configure timer in encoder mode
        timer->setMode(1, ENCODER_MODE, NC);
        timer->setMode(2, ENCODER_MODE, NC);
        
        // Set counter limits (16-bit: 0-65535)
        timer->setOverflow(65535, TICK_FORMAT);
        timer->setCount(32768);  // Start at middle
        
        timer->resume();
        lastTime = millis();
    }
    
    /**
     * @brief Get current position
     */
    int32_t getPosition() {
        return (int32_t)timer->getCount() - 32768;
    }
    
    /**
     * @brief Get revolutions
     */
    float getRevolutions() {
        return (float)getPosition() / (float)pulsesPerRev;
    }
    
    /**
     * @brief Calculate RPM
     */
    float calculateRPM() {
        unsigned long currentTime = millis();
        unsigned long deltaTime = currentTime - lastTime;
        
        if (deltaTime >= 100) {
            int32_t currentCount = getPosition();
            int32_t deltaCount = currentCount - lastCount;
            
            rpm = (deltaCount * 60000.0f) / (pulsesPerRev * deltaTime);
            
            lastCount = currentCount;
            lastTime = currentTime;
        }
        
        return rpm;
    }
    
    /**
     * @brief Reset position
     */
    void resetPosition() {
        timer->setCount(32768);
        lastCount = 0;
    }
    
    float getRPM() const { return rpm; }
    const char* getName() const { return name; }
};
#endif // STM32_CORE_VERSION
#endif // Disabled hardware encoder

#endif // OMRON_E6B2_ENCODER_H
