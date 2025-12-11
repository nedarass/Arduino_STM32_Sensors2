/**
 * @file usb_test_simple.cpp
 * @brief USB Haberleşme Basit Test Programı
 * @details STM32 Mini USB → Raspberry Pi bağlantı testi
 * 
 * Bu test programı:
 * - USB Serial bağlantısını kontrol eder
 * - Her saniye test mesajları gönderir
 * - LED ile görsel geri bildirim sağlar
 * - Gelen komutları dinler ve yanıt verir
 * 
 * Test Adımları:
 * 1. Bu dosyayı src/main.cpp olarak kaydedin
 * 2. PlatformIO ile STM32'ye yükleyin
 * 3. Mini USB kabloyu bilgisayara/Raspberry Pi'ye takın
 * 4. Serial monitör açın (115200 baud)
 * 5. Gelen mesajları izleyin
 * 
 * Windows Test:
 * - PlatformIO Serial Monitor
 * - Arduino Serial Monitor
 * - PuTTY (COMx port, 115200 baud)
 * 
 * Raspberry Pi Test:
 * - screen /dev/ttyACM0 115200
 * - minicom -D /dev/ttyACM0 -b 115200
 * - python script ile okuma
 * 
 * @hardware STM32F103C8T6 (Blue Pill)
 * @date 2025-12-08
 */

#include <Arduino.h>

// LED pin (Blue Pill dahili LED - ters mantıklı)
#define LED_PIN PC13

// Zamanlama değişkenleri
unsigned long lastSend = 0;
unsigned long counter = 0;
bool ledState = false;

/**
 * @brief Sistem başlatma
 */
void setup() {
    // LED pin ayarla
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // Başlangıçta söndür (ters mantık)
    
    // USB Serial başlat
    Serial.begin(115200);
    
    // USB bağlantısını bekle (maksimum 5 saniye)
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 5000)) {
        // LED yanıp söndür (bağlantı bekleniyor)
        digitalWrite(LED_PIN, (millis() / 250) % 2);
    }
    
    // LED'i söndür
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    
    // Başlangıç mesajı
    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════╗");
    Serial.println("║   STM32 USB Haberlesme Test v1.0          ║");
    Serial.println("║   Mini USB → Raspberry Pi Test            ║");
    Serial.println("╚════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Board      : STM32F103C8T6 (Blue Pill)");
    Serial.println("USB Port   : Mini USB (Native CDC)");
    Serial.println("Baud Rate  : 115200");
    Serial.println("Framework  : Arduino");
    Serial.println("Date       : 2025-12-08");
    Serial.println();
    Serial.println("TEST DURUM : USB Baglanti BASARILI! ✓");
    Serial.println();
    Serial.println("Komutlar:");
    Serial.println("  'i' veya 'I' - Sistem bilgisi");
    Serial.println("  'r' veya 'R' - Sayaci sifirla");
    Serial.println("  'h' veya 'H' - Yardim menusu");
    Serial.println();
    Serial.println("Veri gonderimi basladi...");
    Serial.println("============================================\n");
    
    // Başlangıç LED sinyali (3 kez yanıp sön)
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, LOW);   // Yak
        delay(200);
        digitalWrite(LED_PIN, HIGH);  // Söndür
        delay(200);
    }
}

/**
 * @brief Ana döngü
 */
void loop() {
    unsigned long currentTime = millis();
    
    // ========== Her 1 saniyede test mesajı gönder ==========
    if (currentTime - lastSend >= 1000) {
        counter++;
        
        // LED durumunu değiştir
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? LOW : HIGH);
        
        // Test mesajları
        Serial.println("----------------------------------------");
        Serial.print("Test #");
        Serial.println(counter);
        Serial.print("Uptime: ");
        Serial.print(currentTime / 1000);
        Serial.println(" saniye");
        Serial.print("Free RAM: ");
        Serial.print(freeMemory());
        Serial.println(" byte");
        
        // Örnek sensör benzeri veriler (simülasyon)
        float simTemp = 25.0 + (sin(counter * 0.1) * 5.0);
        float simHumidity = 50.0 + (cos(counter * 0.15) * 10.0);
        int simPressure = 1013 + (counter % 20);
        
        Serial.print("Sicaklik (sim): ");
        Serial.print(simTemp, 2);
        Serial.println(" °C");
        
        Serial.print("Nem (sim): ");
        Serial.print(simHumidity, 1);
        Serial.println(" %");
        
        Serial.print("Basinc (sim): ");
        Serial.print(simPressure);
        Serial.println(" hPa");
        
        Serial.println("USB Durum: OK ✓");
        Serial.println();
        
        lastSend = currentTime;
    }
    
    // ========== Gelen komutları kontrol et ==========
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        // LED'i yanıp söndür (veri alındı sinyali)
        digitalWrite(LED_PIN, LOW);
        delay(50);
        digitalWrite(LED_PIN, HIGH);
        
        // Komutu işle
        handleCommand(cmd);
    }
}

/**
 * @brief Gelen komutları işle
 */
void handleCommand(char cmd) {
    Serial.println("\n>>> Komut Alindi: ");
    Serial.println(cmd);
    Serial.println();
    
    switch (cmd) {
        case 'i':
        case 'I':
            printSystemInfo();
            break;
            
        case 'r':
        case 'R':
            counter = 0;
            Serial.println("✓ Sayac sifirlandi!");
            Serial.println();
            break;
            
        case 'h':
        case 'H':
            printHelp();
            break;
            
        case '\n':
        case '\r':
            // Satır sonu karakterlerini yoksay
            break;
            
        default:
            Serial.print("? Bilinmeyen komut: ");
            Serial.println(cmd);
            Serial.println("'h' tuslayarak yardim menusunu gorebilirsiniz.");
            Serial.println();
            break;
    }
}

/**
 * @brief Sistem bilgilerini yazdır
 */
void printSystemInfo() {
    Serial.println("=== SISTEM BILGILERI ===");
    Serial.println();
    
    Serial.println("Mikrodenetleyici : STM32F103C8T6");
    Serial.println("Clock Hizi       : 72 MHz");
    Serial.println("Flash Bellek     : 64 KB");
    Serial.println("SRAM             : 20 KB");
    Serial.println("USB              : Full-Speed (12 Mbps)");
    Serial.println();
    
    Serial.print("Calisma Suresi   : ");
    Serial.print(millis() / 1000);
    Serial.println(" saniye");
    
    Serial.print("Gonderilen Paket : ");
    Serial.println(counter);
    
    Serial.print("Serbest RAM      : ");
    Serial.print(freeMemory());
    Serial.println(" byte");
    
    Serial.println();
    Serial.println("USB Durum        : BAGLI ✓");
    Serial.println();
    Serial.println("========================\n");
}

/**
 * @brief Yardım menüsünü yazdır
 */
void printHelp() {
    Serial.println("=== YARDIM MENUSU ===");
    Serial.println();
    Serial.println("Kullanilabilir Komutlar:");
    Serial.println();
    Serial.println("  'i' veya 'I' - Sistem bilgilerini goster");
    Serial.println("  'r' veya 'R' - Test sayacini sifirla");
    Serial.println("  'h' veya 'H' - Bu yardim menusunu goster");
    Serial.println();
    Serial.println("Not:");
    Serial.println("- Her komut otomatik olarak islenir");
    Serial.println("- LED yanar (komut alindi)");
    Serial.println("- Test mesajlari her 1 saniyede gonderilir");
    Serial.println();
    Serial.println("=====================\n");
}

/**
 * @brief Serbest RAM miktarını hesapla
 * @return Serbest RAM (byte)
 */
int freeMemory() {
    extern int __heap_start;
    extern int *__brkval;
    int free_memory;
    int stack_here;
    
    if (__brkval == 0) {
        free_memory = (int)&stack_here - (int)&__heap_start;
    } else {
        free_memory = (int)&stack_here - (int)__brkval;
    }
    
    return free_memory;
}
