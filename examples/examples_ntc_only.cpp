/**
 * @file examples_ntc_only.cpp
 * @brief Sadece NTC sensör okuma örneği
 * @details Basitleştirilmiş NTC okuma ve serial çıktı
 */

#include <Arduino.h>
#include "ntc_sensor.h"

// 3 NTC Sensör tanımla
NTCSensor ntc1(PA0, 3.3f, 1000.0f, 10000.0f, 3950.0f);
NTCSensor ntc2(PA1, 3.3f, 1000.0f, 10000.0f, 3950.0f);
NTCSensor ntc3(PA2, 3.3f, 1000.0f, 10000.0f, 3950.0f);

void setup() {
    Serial.begin(115200);
    Serial.println("NTC Sensor Okuma Ornegi");
    
    // Sensörleri başlat
    ntc1.begin();
    ntc2.begin();
    ntc3.begin();
    
    Serial.println("Sensorler hazir!");
    delay(1000);
}

void loop() {
    char buffer[100];
    
    // NTC1 oku ve yazdır
    ntc1.printFormatted(buffer, sizeof(buffer), "NTC1");
    Serial.println(buffer);
    
    // NTC2 oku ve yazdır
    ntc2.printFormatted(buffer, sizeof(buffer), "NTC2");
    Serial.println(buffer);
    
    // NTC3 oku ve yazdır
    ntc3.printFormatted(buffer, sizeof(buffer), "NTC3");
    Serial.println(buffer);
    
    Serial.println("---");
    
    delay(500);  // 2Hz okuma
}
