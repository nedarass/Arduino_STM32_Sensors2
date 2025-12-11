/**
 * @file examples_sender.cpp
 * @brief UART Protokol ile veri gönderen örnek kod
 * @details Bu örnek, başka bir STM32'ye veri göndermek için kullanılabilir
 * 
 * Kullanım: main.cpp yerine bu dosyayı derleyin veya içeriğini kopyalayın
 */

#include <Arduino.h>
#include "uart_protocol.h"
#include "ntc_sensor.h"

// UART Protokol nesnesi
UARTProtocol uartProto(&Serial1, PC13);

// NTC Sensör (isteğe bağlı)
NTCSensor ntc1(PA0);

void setup() {
    // Debug serial
    Serial.begin(115200);
    Serial.println("UART Protokol Gonderici Ornegi");
    
    // UART Protokol başlat
    uartProto.begin(115200);
    
    // NTC başlat (isteğe bağlı)
    ntc1.begin();
    
    Serial.println("Sistem hazir. Veri gonderiliyor...");
}

void loop() {
    // Örnek 1: Float değer gönder (sıcaklık)
    float temperature = ntc1.readTemperature();
    uartProto.sendFloat(0x01, temperature);
    Serial.print("Gonderildi (float): ");
    Serial.println(temperature);
    
    delay(500);
    
    // Örnek 2: uint16_t değer gönder
    uint16_t counter = millis() / 1000;
    uartProto.sendUInt16(0x02, counter);
    Serial.print("Gonderildi (uint16): ");
    Serial.println(counter);
    
    delay(500);
    
    // Örnek 3: String gönder
    uartProto.sendString(0x03, "Merhaba STM32!");
    Serial.println("Gonderildi (string): Merhaba STM32!");
    
    delay(1000);
    
    // Örnek 4: uint32_t değer gönder
    uint32_t timestamp = millis();
    uartProto.sendUInt32(0x04, timestamp);
    Serial.print("Gonderildi (uint32): ");
    Serial.println(timestamp);
    
    delay(1000);
}
