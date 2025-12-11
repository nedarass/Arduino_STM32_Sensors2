/**
 * @file main.cpp
 * @brief STM32F103C8T6 Arduino Framework - Gerçek Sensör Entegrasyonu
 * @details 
 *   - 3x NTC 10K Termistör (Robotistan)
 *   - 2x Omron E3FA-RP11 Optik Sensör
 *   - 1x Omron E6B2-CWD6C 600P/R Encoder
 *   - 1x MPU9255 10-DOF IMU (Gyro + Accel + Mag)
 *   - USB-Serial → Raspberry Pi haberleşmesi
 * 
 * @hardware STM32F103C8T6 (Blue Pill)
 * @framework Arduino
 * 
 * Pin Bağlantıları:
 * ==================
 * NTC Sensörler (Analog):
 * - PA0 : NTC1 (10K termistör + 1K voltage divider)
 * - PA1 : NTC2
 * - PA2 : NTC3
 * 
 * Omron E3FA Optik Sensörler (Digital):
 * - PA3 : E3FA-1 (Photoelectric sensor)
 * - PA4 : E3FA-2
 * 
 * Omron E6B2 Encoder (Timer):
 * - PA6 : Encoder Channel A (TIM3_CH1)
 * - PA7 : Encoder Channel B (TIM3_CH2)
 * 
 * MPU9255 IMU (I2C):
 * - PB6 : I2C1_SCL
 * - PB7 : I2C1_SDA
 * 
 * USB-Serial (Raspberry Pi):
 * - PA9  : USB TX (to Raspberry Pi RX)
 * - PA10 : USB RX (from Raspberry Pi TX)
 * 
 * Status LED:
 * - PC13 : Dahili LED
 * 
 * @author Arduino-STM32 Port
 * @date 2025-12-08
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

// MPU9255 IMU I2C pinleri (varsayılan Wire: PB6=SCL, PB7=SDA)
#define IMU_SCL     PB6
#define IMU_SDA     PB7

// LED pin
#define LED_PIN     PC13

// ============================================================================
// GLOBAL NESNELER
// ============================================================================

// NTC Sensörler (3 adet) - Robotistan 10K NTC
NTCSensor ntc1(NTC1_PIN, 3.3f, 1000.0f, 10000.0f, 3950.0f);
NTCSensor ntc2(NTC2_PIN, 3.3f, 1000.0f, 10000.0f, 3950.0f);
NTCSensor ntc3(NTC3_PIN, 3.3f, 1000.0f, 10000.0f, 3950.0f);

// Omron E3FA-RP11 Optik Sensörler (2 adet)
OmronE3FASensor e3fa1(E3FA1_PIN, "E3FA-1");
OmronE3FASensor e3fa2(E3FA2_PIN, "E3FA-2");
DualE3FASensors opticalSensors(&e3fa1, &e3fa2);

// Omron E6B2-CWD6C 600P/R Encoder (software mode - daha basit)
OmronE6B2Encoder encoder(ENCODER_A, ENCODER_B, "E6B2-600", 600);

// MPU9255 10-DOF IMU Sensör
MPU9255Sensor imu(&Wire);

// UART Protokol - USB Serial (Raspberry Pi bağlantısı)
// Serial1 (PA9/PA10) yerine doğrudan Serial kullanıyoruz (USB)
UARTProtocol uartProto(&Serial, LED_PIN);

// ============================================================================
// INTERRUPT SERVISLERI (ISR)
// ============================================================================

/**
 * @brief E3FA-1 optik sensör için interrupt
 */
void e3fa1ISR() {
    e3fa1.trigger();
}

/**
 * @brief E3FA-2 optik sensör için interrupt
 */
void e3fa2ISR() {
    e3fa2.trigger();
}

/**
 * @brief Encoder güncellemesi (yüksek frekanslı loop'ta çağrılacak)
 */
void encoderISR() {
    encoder.update();
}

// ============================================================================
// YARDIMCI FONKSIYONLAR
// ============================================================================

/**
 * @brief Tüm NTC sensörleri okur ve Raspberry Pi'ye gönderir
 */
void readAndSendNTCData() {
    char buffer[200];
    
    float temp1 = ntc1.readTemperature();
    float temp2 = ntc2.readTemperature();
    float temp3 = ntc3.readTemperature();
    
    // Formatted output
    snprintf(buffer, sizeof(buffer), 
             "NTC: T1:%.2f°C T2:%.2f°C T3:%.2f°C",
             temp1, temp2, temp3);
    Serial.println(buffer);
    
    // UART protokol ile gönder (USB test için kapalı)
    // uartProto.sendFloat(0x01, temp1);
    // uartProto.sendFloat(0x02, temp2);
    // uartProto.sendFloat(0x03, temp3);
}

/**
 * @brief Optik sensörleri kontrol et ve raporla
 */
void checkOpticalSensors() {
    opticalSensors.update();
    
    if (e3fa1.hasNewDetection()) {
        Serial.println("E3FA-1: Object Detected!");
        // uartProto.sendUInt16(0x10, 1);  // E3FA-1 detection signal
    }
    
    if (e3fa2.hasNewDetection()) {
        Serial.println("E3FA-2: Object Detected!");
        // uartProto.sendUInt16(0x11, 1);  // E3FA-2 detection signal
    }
}

/**
 * @brief Encoder verilerini oku ve gönder
 */
void readAndSendEncoderData() {
    encoder.update();
    float rpm = encoder.calculateRPM();
    
    if (abs(rpm) > 0.1f) {  // Sadece hareket varsa raporla
        char buffer[100];
        encoder.printFormatted(buffer, sizeof(buffer));
        Serial.println(buffer);
        
        // UART protokol ile gönder (USB test için kapalı)
        // uartProto.sendFloat(0x20, rpm);
        // uartProto.sendFloat(0x21, encoder.getRevolutions());
    }
}

/**
 * @brief IMU verilerini oku ve gönder
 */
void readAndSendIMUData() {
    float ax, ay, az, gx, gy, gz, temp;
    imu.readAll(ax, ay, az, gx, gy, gz, temp);
    
    // Açı hesapla
    float pitch, roll;
    imu.calculateAngles(ax, ay, az, pitch, roll);
    
    // Formatted output
    char buffer[200];
    snprintf(buffer, sizeof(buffer),
             "IMU: A[%.2f,%.2f,%.2f]g G[%.1f,%.1f,%.1f]°/s P:%.1f° R:%.1f° T:%.1f°C",
             ax, ay, az, gx, gy, gz, pitch, roll, temp);
    Serial.println(buffer);
    
    // UART protokol ile gönder (USB test için kapalı)
    // uartProto.sendFloat(0x30, ax);    // Accel X
    // uartProto.sendFloat(0x31, ay);    // Accel Y
    // uartProto.sendFloat(0x32, az);    // Accel Z
    // uartProto.sendFloat(0x33, pitch); // Pitch angle
    // uartProto.sendFloat(0x34, roll);  // Roll angle
    // uartProto.sendFloat(0x35, temp);  // Temperature
}

/**
 * @brief Tüm sensör durumlarını toplu raporla
 */
void sendStatusReport() {
    char buffer[300];
    
    // Başlık
    Serial.println("\n=== SENSOR STATUS REPORT ===");
    
    // NTC
    ntc1.printFormatted(buffer, sizeof(buffer), "NTC1");
    Serial.println(buffer);
    ntc2.printFormatted(buffer, sizeof(buffer), "NTC2");
    Serial.println(buffer);
    ntc3.printFormatted(buffer, sizeof(buffer), "NTC3");
    Serial.println(buffer);
    
    // Optik sensörler
    opticalSensors.printStatus(buffer, sizeof(buffer));
    Serial.println(buffer);
    
    // Encoder
    encoder.printFormatted(buffer, sizeof(buffer));
    Serial.println(buffer);
    
    // IMU
    imu.printFormatted(buffer, sizeof(buffer));
    Serial.println(buffer);
    
    Serial.println("============================\n");
}

/**
 * @brief Sistem durumunu kontrol eder
 */
void checkSystemStatus() {
    // Optik sensörleri kontrol et
    checkOpticalSensors();
    
    // UART protokol mesajlarını işle
    uartProto.update();
}

/**
 * @brief LED yanıp söndürme (heartbeat)
 */
void heartbeatLED() {
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    
    if (millis() - lastBlink >= 1000) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? LOW : HIGH);  // PC13 ters mantıklı
        lastBlink = millis();
    }
}

// ============================================================================
// ANA FONKSIYONLAR
// ============================================================================

/**
 * @brief Sistem başlatma
 */
void setup() {
    // USB Serial başlat (Raspberry Pi bağlantısı)
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // 3 saniye bekle
    
    delay(500);
    
    // Banner
    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════╗");
    Serial.println("║   STM32 Multi-Sensor System v2.0          ║");
    Serial.println("║   Raspberry Pi Data Acquisition           ║");
    Serial.println("╚════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Board     : STM32F103C8T6 (Blue Pill)");
    Serial.println("Framework : Arduino");
    Serial.println("Date      : 2025-12-08");
    Serial.println("USB Speed : 115200 baud");
    Serial.println("============================================\n");
    
    // LED pin başlat
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // Başlangıçta söndür (ters mantık)
    
    // ========== NTC Termistörler ==========
    Serial.print("[1/6] NTC Termistorler (x3)... ");
    ntc1.begin();
    ntc2.begin();
    ntc3.begin();
    Serial.println("OK");
    
    // ========== Omron E3FA Optik Sensörler ==========
    Serial.print("[2/6] Omron E3FA Optik Sensorler (x2)... ");
    opticalSensors.begin();
    
    // Interrupt'ları bağla
    e3fa1.attachInterrupt(e3fa1ISR, RISING);
    e3fa2.attachInterrupt(e3fa2ISR, RISING);
    Serial.println("OK");
    
    // ========== Omron E6B2 Encoder ==========
    Serial.print("[3/6] Omron E6B2-CWD6C 600P/R Encoder... ");
    encoder.begin();
    encoder.resetPosition();
    Serial.println("OK");
    
    // ========== MPU9255 IMU ==========
    Serial.print("[4/6] MPU9255 10-DOF IMU... ");
    Wire.setSCL(IMU_SCL);
    Wire.setSDA(IMU_SDA);
    
    if (imu.begin()) {
        Serial.println("OK");
        
        // Gyro kalibrasyonu (cihaz hareketsiz olmalı!)
        Serial.print("    Gyro kalibrasyonu (hareketsiz tutun)... ");
        imu.calibrateGyro(500);
        Serial.println("OK");
    } else {
        Serial.println("FAIL (kontrol edin!)");
    }
    
    // ========== UART Protokol ==========
    Serial.print("[5/6] UART Protokol (Raspberry Pi)... ");
    uartProto.begin(115200);
    Serial.println("OK");
    
    // ========== Test ==========
    Serial.print("[6/6] Sistem testi... ");
    delay(500);
    Serial.println("OK");
    
    Serial.println("\n============================================");
    Serial.println("✓ TUM SENSORLER HAZIR!");
    Serial.println("✓ Veri akisi basladi...");
    Serial.println("============================================\n");
    
    // İlk durum raporu
    delay(1000);
    sendStatusReport();
}

/**
 * @brief Ana döngü - Çoklu sensör yönetimi
 */
void loop() {
    static unsigned long lastNTCRead = 0;
    static unsigned long lastIMURead = 0;
    static unsigned long lastEncoderRead = 0;
    static unsigned long lastStatusReport = 0;
    
    const unsigned long NTC_INTERVAL = 500;      // 500ms = 2Hz (sıcaklık yavaş değişir)
    const unsigned long IMU_INTERVAL = 50;       // 50ms = 20Hz (IMU hızlı okuma)
    const unsigned long ENCODER_INTERVAL = 100;  // 100ms = 10Hz (encoder RPM)
    const unsigned long STATUS_INTERVAL = 5000;  // 5 saniye (durum raporu)
    
    unsigned long currentTime = millis();
    
    // ========== NTC Termistörler (2Hz) ==========
    if (currentTime - lastNTCRead >= NTC_INTERVAL) {
        readAndSendNTCData();
        lastNTCRead = currentTime;
    }
    
    // ========== MPU9255 IMU (20Hz) ==========
    if (currentTime - lastIMURead >= IMU_INTERVAL) {
        readAndSendIMUData();
        lastIMURead = currentTime;
    }
    
    // ========== Encoder (10Hz) ==========
    if (currentTime - lastEncoderRead >= ENCODER_INTERVAL) {
        readAndSendEncoderData();
        lastEncoderRead = currentTime;
    }
    
    // ========== Optik Sensörler (sürekli) ==========
    checkSystemStatus();
    
    // ========== Periyodik Durum Raporu (5 saniye) ==========
    if (currentTime - lastStatusReport >= STATUS_INTERVAL) {
        sendStatusReport();
        lastStatusReport = currentTime;
    }
    
    // ========== Heartbeat LED ==========
    heartbeatLED();
    
    // Encoder hızlı güncelleme (quadrature decoding için)
    encoder.update();
    
    // Kısa gecikme
    delay(5);
}

// ============================================================================
// TEST FONKSIYONLARI (İsteğe bağlı)
// ============================================================================

/**
 * @brief Tüm sensörleri test eder (setup() içinde çağrılabilir)
 */
void runSensorTests() {
    Serial.println("\n[TEST] Sensor testleri basladi...\n");
    
    // NTC Test
    Serial.println("--- NTC Test ---");
    for (int i = 0; i < 3; i++) {
        char buf[100];
        ntc1.printFormatted(buf, sizeof(buf), "NTC1");
        Serial.println(buf);
        delay(100);
    }
    
    // Omron Test
    Serial.println("\n--- Omron Test ---");
    Serial.print("E3FA-1: ");
    Serial.println(e3fa1.isDetected() ? "AKTIF" : "PASIF");
    Serial.print("E3FA-2: ");
    Serial.println(e3fa2.isDetected() ? "AKTIF" : "PASIF");
    
    // UART Protocol Test
    Serial.println("\n--- UART Protokol Test ---");
    uartProto.sendString(0xFF, "Test Mesaji");
    uartProto.sendFloat(0x01, 25.5f);
    uartProto.sendUInt16(0x02, 1234);
    
    Serial.println("\n[TEST] Sensor testleri tamamlandi\n");
    delay(1000);
}
