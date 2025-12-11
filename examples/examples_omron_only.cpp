/**
 * @file examples_omron_only.cpp
 * @brief Sadece Omron sensör okuma örneği
 * @details Interrupt tabanlı Omron sensör tetikleme algılama
 */

#include <Arduino.h>
#include "omron_sensor.h"

// Omron sensörler
OmronSensor omron1(PA11, "Omron1", 50);
OmronSensor omron2(PA12, "Omron2", 50);

// ISR fonksiyonları
void omron1ISR() {
    omron1.trigger();
}

void omron2ISR() {
    omron2.trigger();
}

void setup() {
    Serial.begin(115200);
    Serial.println("Omron Sensor Interrupt Ornegi");
    
    // Sensörleri başlat
    omron1.begin(INPUT_PULLUP);
    omron2.begin(INPUT_PULLUP);
    
    // Interrupt'ları bağla
    omron1.attachInterrupt(omron1ISR, RISING);
    omron2.attachInterrupt(omron2ISR, RISING);
    
    Serial.println("Omron sensorler hazir!");
    Serial.println("Sensorleri tetikleyin...");
}

void loop() {
    // Tetikleme kontrolü
    if (omron1.isTriggered()) {
        Serial.println(">>> Omron1 TETIKLENDI! <<<");
    }
    
    if (omron2.isTriggered()) {
        Serial.println(">>> Omron2 TETIKLENDI! <<<");
    }
    
    // Periyodik durum kontrolü (her 2 saniyede)
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck >= 2000) {
        char buffer[100];
        omron1.printStatus(buffer, sizeof(buffer));
        Serial.println(buffer);
        
        omron2.printStatus(buffer, sizeof(buffer));
        Serial.println(buffer);
        
        Serial.println("---");
        lastCheck = millis();
    }
    
    delay(10);
}
