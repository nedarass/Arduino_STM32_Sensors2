/**
 * @file ntc_sensor.h
 * @brief NTC Termistör sensör okuma ve sıcaklık hesaplama sınıfı
 * @details Beta formülü kullanarak NTC direnci üzerinden sıcaklık hesaplar
 * 
 * Devre Topolojisi:
 *   VCC (3.3V) ---[R_FIXED]--- Vout ---[NTC]--- GND
 *   
 * @author Arduino-STM32 Port
 * @date 2025
 */

#ifndef NTC_SENSOR_H
#define NTC_SENSOR_H

#include <Arduino.h>
#include <math.h>

class NTCSensor {
private:
    // Pin tanımları
    uint8_t adcPin;           // Analog pin (PA0, PA1, PA2...)
    
    // Sabit parametreler
    float vRef;               // Referans voltaj (3.3V)
    float rFixed;             // Sabit direnç değeri (Ω)
    float r0;                 // 25°C'deki NTC direnci (Ω)
    float t0Kelvin;           // 25°C = 298.15K
    float beta;               // Beta katsayısı
    
    // ADC çözünürlük
    const float ADC_MAX = 4095.0f;  // 12-bit ADC
    
    /**
     * @brief ADC değerini voltaja çevirir
     * @param adcValue Ham ADC değeri (0-4095)
     * @return Ölçülen voltaj (V)
     */
    float adcToVoltage(uint32_t adcValue) {
        return (vRef * adcValue) / ADC_MAX;
    }
    
    /**
     * @brief Voltaj değerinden NTC direncini hesaplar
     * @param vOut Ölçülen voltaj
     * @return NTC direnci (Ω)
     */
    float voltageToResistance(float vOut) {
        // Vout = Vref * Rntc / (Rfixed + Rntc)
        // Rntc = Rfixed * Vout / (Vref - Vout)
        
        if (vRef - vOut <= 1e-6f) {
            vOut = vRef - 1e-6f;  // Bölme sıfır hatası önleme
        }
        return (rFixed * vOut) / (vRef - vOut);
    }
    
    /**
     * @brief NTC direncinden sıcaklık hesaplar (Beta formülü)
     * @param rNtc NTC direnci (Ω)
     * @return Sıcaklık (°C)
     */
    float resistanceToTemperature(float rNtc) {
        // Beta formülü: 1/T = 1/T0 + (1/Beta) * ln(R/R0)
        float invT = (1.0f / t0Kelvin) + (1.0f / beta) * logf(rNtc / r0);
        float tempK = 1.0f / invT;
        return tempK - 273.15f;  // Kelvin'den Celsius'a
    }

public:
    /**
     * @brief NTC Sensör constructor
     * @param pin Analog pin numarası (örn: PA0)
     * @param vReference Referans voltaj (varsayılan: 3.3V)
     * @param fixedResistor Sabit direnç (varsayılan: 1kΩ)
     * @param r0Value 25°C'deki NTC direnci (varsayılan: 10kΩ)
     * @param betaValue Beta katsayısı (varsayılan: 3950)
     */
    NTCSensor(uint8_t pin, 
              float vReference = 3.3f, 
              float fixedResistor = 1000.0f,
              float r0Value = 10000.0f,
              float betaValue = 3950.0f) 
        : adcPin(pin), 
          vRef(vReference), 
          rFixed(fixedResistor),
          r0(r0Value),
          t0Kelvin(298.15f),
          beta(betaValue) {
    }
    
    /**
     * @brief Sensörü başlatır
     */
    void begin() {
        pinMode(adcPin, INPUT_ANALOG);
    }
    
    /**
     * @brief Ham ADC değerini okur
     * @return ADC değeri (0-4095)
     */
    uint32_t readADC() {
        return analogRead(adcPin);
    }
    
    /**
     * @brief Voltaj değerini okur
     * @return Voltaj (V)
     */
    float readVoltage() {
        uint32_t adc = readADC();
        return adcToVoltage(adc);
    }
    
    /**
     * @brief NTC direncini hesaplar
     * @return Direnç (Ω)
     */
    float readResistance() {
        float voltage = readVoltage();
        return voltageToResistance(voltage);
    }
    
    /**
     * @brief Sıcaklığı okur
     * @return Sıcaklık (°C)
     */
    float readTemperature() {
        float resistance = readResistance();
        return resistanceToTemperature(resistance);
    }
    
    /**
     * @brief Tüm verileri okur ve yapılandırılmış bir şekilde döndürür
     * @param adc ADC değeri referansı
     * @param voltage Voltaj referansı
     * @param resistance Direnç referansı
     * @param temperature Sıcaklık referansı
     */
    void readAll(uint32_t &adc, float &voltage, float &resistance, float &temperature) {
        adc = readADC();
        voltage = adcToVoltage(adc);
        resistance = voltageToResistance(voltage);
        temperature = resistanceToTemperature(resistance);
    }
    
    /**
     * @brief Formatlı string çıktısı üretir
     * @param buffer Çıktı buffer'ı
     * @param bufferSize Buffer boyutu
     * @param channelName Kanal adı (örn: "CH0")
     */
    void printFormatted(char* buffer, size_t bufferSize, const char* channelName = "NTC") {
        uint32_t adc;
        float voltage, resistance, temperature;
        readAll(adc, voltage, resistance, temperature);
        
        snprintf(buffer, bufferSize,
                "%s: ADC:%4lu V:%.3f R:%7.0f T:%.2f°C",
                channelName, 
                (unsigned long)adc, 
                voltage, 
                resistance, 
                temperature);
    }
};

#endif // NTC_SENSOR_H
