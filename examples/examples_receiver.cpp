/**
 * @file examples_receiver.cpp
 * @brief UART Protokol ile veri alan örnek kod
 * @details Bu örnek, başka bir STM32'den gelen verileri alır ve işler
 * 
 * Kullanım: main.cpp yerine bu dosyayı derleyin veya içeriğini kopyalayın
 */

#include <Arduino.h>
#include "uart_protocol.h"

// UART Protokol nesnesi (LED göstergeli)
UARTProtocol uartProto(&Serial1, PC13);

void setup() {
    // Debug serial
    Serial.begin(115200);
    Serial.println("========================================");
    Serial.println("  UART Protokol Alici Ornegi");
    Serial.println("========================================");
    
    // LED pin
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH);  // Başta söndür
    
    // UART Protokol başlat
    uartProto.begin(115200);
    
    Serial.println("Sistem hazir. Veri bekleniyor...");
    Serial.println("========================================\n");
}

void loop() {
    // Gelen verileri işle
    uartProto.update();
    
    // LED'i kısa süre sonra söndür
    static unsigned long ledOnTime = 0;
    static bool ledIsOn = false;
    
    if (ledIsOn && (millis() - ledOnTime > 100)) {
        digitalWrite(PC13, HIGH);  // LED söndür
        ledIsOn = false;
    }
    
    // LED yanıyorsa zamanı kaydet
    if (digitalRead(PC13) == LOW && !ledIsOn) {
        ledOnTime = millis();
        ledIsOn = true;
    }
    
    delay(10);
}
