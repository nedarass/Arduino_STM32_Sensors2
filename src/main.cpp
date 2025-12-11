/**
 * @file main.cpp
 * @brief STM32F103C8T6 Arduino Framework - Polaris Uyumlu JSON Sürümü
 * @details 
 * - Navigasyon: Encoder + Optik Sensör Füzyonu (updateNavigation)
 * - Sıcaklık: NTC1/NTC2 (Motor Max), NTC3 (Batarya)
 * - İletişim: Raspberry Pi'ye JSON formatında veri akışı
 */

#include <Arduino.h>
#include <Wire.h>
#include "ntc_sensor.h"
#include "uart_protocol.h"
#include "omron_e3fa_sensor.h"
#include "omron_e6b2_encoder.h"
#include "mpu9255_sensor.h"

// ============================================================================
// PIN TANIMLARI
// ============================================================================

// NTC Termistör pinleri
#define NTC1_PIN    PA0
#define NTC2_PIN    PA1
#define NTC3_PIN    PA2

// Omron E3FA-RP11 Optik sensör pinleri
#define E3FA1_PIN   PA3
#define E3FA2_PIN   PA4

// Omron E6B2-CWD6C Encoder pinleri (TIM3)
#define ENCODER_A   PA6  // TIM3_CH1
#define ENCODER_B   PA7  // TIM3_CH2

// MPU9255 IMU I2C pinleri
#define IMU_SCL     PB6
#define IMU_SDA     PB7

// LED pin
#define LED_PIN     PC13

// ============================================================================
// GLOBAL NESNELER
// ============================================================================

// NTC Sensörler
NTCSensor ntc1(NTC1_PIN, 3.3f, 1000.0f, 10000.0f, 3950.0f); // Araç 1
NTCSensor ntc2(NTC2_PIN, 3.3f, 1000.0f, 10000.0f, 3950.0f); // Araç 2
NTCSensor ntc3(NTC3_PIN, 3.3f, 1000.0f, 10000.0f, 3950.0f); // Batarya

// Optik Sensörler
OmronE3FASensor e3fa1(E3FA1_PIN, "E3FA-1");
OmronE3FASensor e3fa2(E3FA2_PIN, "E3FA-2");
DualE3FASensors opticalSensors(&e3fa1, &e3fa2);

// Encoder
OmronE6B2Encoder encoder(ENCODER_A, ENCODER_B, "E6B2-600", 600);

// IMU
MPU9255Sensor imu(&Wire);

// UART
UARTProtocol uartProto(&Serial, LED_PIN);

float podPosition = 0.0;          // Sadece Optik ile artan konum
unsigned long lastStripTime = 0;  // Son şeridin görülme zamanı (Burst tespiti için)

// ============================================================================
// INTERRUPT SERVISLERI (ISR)
// ============================================================================

void e3fa1ISR() { e3fa1.trigger(); }
void e3fa2ISR() { e3fa2.trigger(); }
void encoderISR() { encoder.update(); }

// ============================================================================
// NAVİGASYON ALGORİTMASI (SENİN YAZDIĞIN KOD - AYNI KALDI)
// ============================================================================
void updateNavigation() {
    opticalSensors.update();

    // Sağ veya Sol sensörden herhangi biri şerit görürse:
    if (e3fa1.hasNewDetection() || e3fa2.hasNewDetection()) {
        
        unsigned long currentTime = millis();
        unsigned long timeDiff = currentTime - lastStripTime;
        lastStripTime = currentTime;

        // --- KONUM HARİTALAMA MANTIĞI ---

        // A) BAŞLANGIÇ (0. Metre civarı)
        if (podPosition == 0.0) {
            // İlk gördüğümüz şey kesinlikle 11. metredeki ilk şerittir.
            podPosition = 11.0; 
        }

        // B) BURST TESPİTİ (Hızlı arka arkaya gelen şeritler)
        // Eğer bir önceki şeritten sonra çok kısa süre geçtiyse (Örn: < 200ms)
        // Ve zaten kritik bölgelere (86m veya 160m) yakınsak
        // (Not: Sabit hız olmadığı için timeDiff risklidir, ancak çok sık şeritleri (5cm) 
        //  4 metrelik boşluktan ayırmak için genelde bariz bir fark olur)
        else if (timeDiff < 300) { 
            // Bu bir "İşaretçi Şerit" (5cm aralıklı)
            // Konuma sadece 5cm ekle
            podPosition += 0.05; 
            
            // Eğer konum 86m civarındaysa tam 86'ya sabitle (Kalibrasyon)
            if (podPosition > 85.0 && podPosition < 87.0) podPosition = 86.0;
            
            // Eğer konum 160m civarındaysa tam 160'a sabitle
            if (podPosition > 159.0 && podPosition < 161.0) podPosition = 160.0;
        }
        // C) NORMAL SEYİR (4 Metre Aralıklar)
        else {
            // Aradan uzun zaman geçtiyse bu bir sonraki 4m şerididir.
            podPosition += 4.0;
        }
}

// ============================================================================
// JSON GÖNDERME FONKSİYONU
// ============================================================================
/**
 * @brief Tüm verileri toplar ve Raspberry Pi'nin beklediği JSON formatında basar.
 */
void sendTelemetryJSON() {
    char jsonBuffer[256]; 

    // 1. HIZ (Encoder'dan türetilir)
    // Hız (km/h) = (RPM / 60) * Çevre * 3.6
    float rpm = encoder.calculateRPM();
    float speed_kmh = (rpm / 60.0f) * WHEEL_CIRCUMFERENCE * 3.6f;
    if(speed_kmh < 0) speed_kmh = 0;

    // 2. KONUM (Navigasyon algoritmasından gelir)
    // Direkt olarak updateNavigation() fonksiyonunun hesapladığı global değişkeni alıyoruz.
    float current_pos_m = podPosition;

    // 3. İVME (IMU'dan okunur)
    float ax, ay, az, gx, gy, gz, temp_imu;
    imu.readAll(ax, ay, az, gx, gy, gz, temp_imu);
    float acceleration = ax * 9.81f; // g -> m/s²

    // 4. SICAKLIK MANTIĞI (Senin isteğin: 2 Araç, 1 Batarya)
    float t_vehicle1 = ntc1.readTemperature();
    float t_vehicle2 = ntc2.readTemperature();
    float t_battery  = ntc3.readTemperature(); // Batarya

    // Araç sıcaklığı: İki sensörden yüksek olanı al (Güvenlik)
    float vehicle_max_temp = (t_vehicle1 > t_vehicle2) ? t_vehicle1 : t_vehicle2;

    // 5. CHECKPOINT (Omron)
    // Anlık olarak şerit üzerinde miyiz?
    int checkpoint = (e3fa1.isDetected() || e3fa2.isDetected()) ? 1 : 0;

    // 6. FREN BASINCI (Analog sensör yoksa 0 gönder)
    int brake_pressure = 0;

    // --- JSON PAKETLEME ---
    // Python tarafı (final_client.py) bu anahtarları bekliyor:
    snprintf(jsonBuffer, sizeof(jsonBuffer), 
             "{\"speed\":%.2f,\"position\":%.2f,\"acceleration\":%.2f,\"temperature\":%.2f,\"battery_temp\":%.2f,\"brake_pressure\":%d,\"checkpoint\":%d}", 
             speed_kmh, current_pos_m, acceleration, vehicle_max_temp, t_battery, brake_pressure, checkpoint);
             
    Serial.println(jsonBuffer);
}

void heartbeatLED() {
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    if (millis() - lastBlink >= 1000) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? LOW : HIGH); 
        lastBlink = millis();
    }
}

// ============================================================================
// ANA FONKSIYONLAR
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000); 
    delay(500);

    // Başlatma mesajları (Bu kısımlar JSON olmadığı için Pi ilk başta 1-2 hata verir sonra düzelir)
    Serial.println("STM32 Polaris Sensor System Starting...");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Sensör Başlatmaları
    ntc1.begin(); ntc2.begin(); ntc3.begin();
    opticalSensors.begin();
    e3fa1.attachInterrupt(e3fa1ISR, RISING);
    e3fa2.attachInterrupt(e3fa2ISR, RISING);
    
    encoder.begin();
    encoder.resetPosition();
    
    Wire.setSCL(IMU_SCL);
    Wire.setSDA(IMU_SDA);
    if(imu.begin()) {
        imu.calibrateGyro(500);
    }

    uartProto.begin(115200);
    
    Serial.println("System Ready.");
}

void loop() {
    static unsigned long lastTelemetryTime = 0;
    
    // --- NAVİGASYON GÜNCELLEMESİ (Her döngüde) ---
    // Konumu ve şeritleri sürekli takip etmeliyiz
    updateNavigation();
    
    // --- ENCODER GÜNCELLEMESİ ---
    encoder.update();

    // --- JSON VERİ GÖNDERİMİ (20 Hz - 50ms) ---
    // Python tarafı her veriyi okuyup arayüze basacağı için bu hız idealdir.
    if (millis() - lastTelemetryTime >= 50) {
        sendTelemetryJSON();
        lastTelemetryTime = millis();
    }
    
    heartbeatLED();
}
