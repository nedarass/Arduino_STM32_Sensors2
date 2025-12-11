# STM32 Multi-Sensor System - Raspberry Pi USB Veri Toplama

**STM32F103C8T6 (Blue Pill) + Arduino Framework + USB Serial Haberleşme**

---

## 📋 Proje Özeti

STM32 mikrodenetleyici üzerinde birden fazla sensörden veri okuyup Mini USB üzerinden Raspberry Pi'ye aktaran sistem.

### Özellikler
- ✅ USB Serial haberleşme (115200 baud)
- ✅ Çoklu sensör desteği (NTC, Optik, Encoder, IMU)
- ✅ Float değer desteği (printf)
- ✅ Text tabanlı çıktı (okunabilir)
- ✅ Raspberry Pi uyumlu
- ✅ Gerçek zamanlı veri akışı

---

## 🔧 Donanım

### STM32F103C8T6 (Blue Pill)
- **İşlemci**: ARM Cortex-M3, 72MHz
- **Flash**: 64KB
- **RAM**: 20KB
- **USB**: Mini USB (Native CDC)

### Sensörler

| Sensör | Adet | Pinler | Açıklama |
|--------|------|--------|----------|
| NTC 10K Termistör | 3 | PA0, PA1, PA2 | Sıcaklık ölçümü (analog) |
| Omron E3FA-RP11 | 2 | PA3, PA4 | Optik sensör (digital) |
| Omron E6B2-CWD6C | 1 | PA6, PA7 | Encoder 600PPR (timer) |
| MPU9255 10-DOF | 1 | PB6, PB7 | IMU (I2C) |
| USB Serial | - | Mini USB | Raspberry Pi bağlantısı |
| LED | 1 | PC13 | Durum göstergesi |

---

## 🔌 Pin Bağlantıları

```
STM32F103C8T6
├─ PA0  → NTC1 (Analog)
├─ PA1  → NTC2 (Analog)
├─ PA2  → NTC3 (Analog)
├─ PA3  → E3FA-1 (Digital)
├─ PA4  → E3FA-2 (Digital)
├─ PA6  → Encoder A (TIM3_CH1)
├─ PA7  → Encoder B (TIM3_CH2)
├─ PB6  → I2C SCL (MPU9255)
├─ PB7  → I2C SDA (MPU9255)
├─ PC13 → LED (Dahili)
└─ Mini USB → Raspberry Pi (USB Serial)
```

### USB Bağlantısı

```
STM32 Blue Pill (Mini USB)
        ↓
    USB Kablo
        ↓
Raspberry Pi (USB Port)
```

Raspberry Pi'de `/dev/ttyACM0` veya `/dev/ttyUSB0` olarak görünür.

---

## 📦 Kurulum

### Gereksinimler

- **PlatformIO** (VS Code extension)
- **ST-Link V2** programmer
- **USB Mini Kablo**

### 1. Projeyi Klonla

```bash
git clone <repo-url>
cd Arduino_STM32_Sensors
```

### 2. Derle ve Yükle

```bash
# Derleme
pio run

# ST-Link ile yükleme
pio run --target upload
```

### 3. Serial Monitor

```bash
# Windows
pio device monitor -b 115200

# Linux/Raspberry Pi
screen /dev/ttyACM0 115200
```

---

## 📊 Çıktı Formatı

### Text Tabanlı Çıktı

```
╔════════════════════════════════════════════╗
║   STM32 Multi-Sensor System v2.0          ║
║   Raspberry Pi Data Acquisition           ║
╚════════════════════════════════════════════╝

[1/6] NTC Termistorler (x3)... OK
[2/6] Omron E3FA Optik Sensorler (x2)... OK
[3/6] Omron E6B2-CWD6C 600P/R Encoder... OK
[4/6] MPU9255 10-DOF IMU... OK
[5/6] UART Protokol (Raspberry Pi)... OK
[6/6] Sistem testi... OK

=== SENSOR STATUS REPORT ===
NTC1: ADC: 425 V:1.05V R:2340Ω T:25.34°C
NTC2: ADC: 607 V:1.50V R:3450Ω T:26.12°C
NTC3: ADC: 578 V:1.43V R:3210Ω T:24.89°C
E3FA-1: NOT DETECTED | E3FA-2: DETECTED
E6B2-600: Pos:1500 Rev:2.50 RPM:120.5 Dir:CW
IMU: Accel[0.02,0.98,0.01]g Gyro[1.2,-0.5,0.3]°/s
============================

NTC: T1:25.34°C T2:26.12°C T3:24.89°C
IMU: A[0.02,0.98,0.01]g G[1.2,-0.5,0.3]°/s P:5.2° R:-1.1° T:23.5°C
```

---

## 🐍 Raspberry Pi Python Kodu

```python
import serial
import time

# Serial port aç
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Bağlantı stabilize olsun

print("STM32'den veri okunuyor...")

while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if line:
            print(f"[{time.strftime('%H:%M:%S')}] {line}")
            
            # NTC verisi parse et
            if "NTC:" in line:
                # Veriyi parse edip veritabanına kaydet
                pass
```

---

## ⚙️ Konfigürasyon

### platformio.ini

```ini
[env:bluepill_f103c8]
platform = ststm32
board = bluepill_f103c8
framework = arduino

monitor_speed = 115200
upload_protocol = stlink

build_flags = 
    -D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC
    -D USBCON
    -D HAL_UART_MODULE_ENABLED
    -D HAL_ADC_MODULE_ENABLED
    -Wl,-u,_printf_float  ; Float printf desteği

# Klon STM32 chip ID düzeltme
upload_flags = 
    -c
    set CPUTAPID 0x2ba01477
```

---

## 📁 Proje Yapısı

```
Arduino_STM32_Sensors/
├── src/
│   ├── main.cpp                 # Ana program
│   ├── ntc_sensor.h            # NTC termistör sınıfı
│   ├── omron_e3fa_sensor.h     # Optik sensör sınıfı
│   ├── omron_e6b2_encoder.h    # Encoder sınıfı
│   ├── mpu9255_sensor.h        # IMU sensör sınıfı
│   └── uart_protocol.h         # UART protokol (kullanılmıyor)
├── platformio.ini              # PlatformIO konfigürasyonu
└── README.md                   # Bu dosya
```

---

## 🔬 Teknik Detaylar

### Sensör Okuma Frekansları

| Sensör | Frekans | Sebep |
|--------|---------|-------|
| NTC Termistör | 2 Hz | Sıcaklık yavaş değişir |
| Optik Sensör | Event-based | Interrupt ile tetiklenir |
| Encoder | 10 Hz | RPM hesabı için |
| IMU | 20 Hz | Hızlı hareket takibi |

### Bellek Kullanımı

- **RAM**: ~6KB / 20KB (29%)
- **Flash**: ~62KB / 64KB (95%)
  - Float printf kütüphanesi: ~12KB
  - Sensör sınıfları: ~8KB
  - Ana program: ~42KB

---

## 🐛 Sorun Giderme

### USB Port Bulunamıyor (Raspberry Pi)

```bash
# Port listesi
ls /dev/ttyACM* /dev/ttyUSB*

# İzin ver
sudo chmod 666 /dev/ttyACM0

# Kalıcı izin
sudo usermod -a -G dialout $USER
# (Logout/login gerekir)
```

### Upload Hatası (ST-Link)

```bash
# ST-Link bağlantısını kontrol et
# Kırmızı LED yanıyor olmalı

# OpenOCD çıktısını kontrol et
pio run --target upload -v
```

### Float Değerler Görünmüyor

`platformio.ini` dosyasında `-Wl,-u,_printf_float` bayrağının olduğundan emin olun.

### Sensörler Rastgele Değer Gösteriyor

Normal - sensörler bağlı değilse pinler havada kalır (floating) ve elektriksel gürültü okur.

---

## 📝 Lisans

Bu proje eğitim amaçlıdır.

---

## 👨‍💻 Geliştirici Notları

- Binary UART paketleri devre dışı (USB test için)
- Hardware encoder devre dışı (software encoder kullanılıyor)
- IMU okunamıyorsa "FAIL" gösterir (normal)
- LED PC13 pini ters mantıklı (LOW=yak, HIGH=söndür)

---

**Son Güncelleme**: 2025-12-08  
**Durum**: ✅ Çalışıyor - USB Serial test başarılı
