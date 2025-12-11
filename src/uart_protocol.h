/**
 * @file uart_protocol.h
 * @brief UART üzerinden paketli veri iletişim protokolü
 * @details Çerçeve formatı: [START][ID][TYPE][LEN_L][LEN_H][PAYLOAD...][END]
 * 
 * Paket Yapısı:
 * - START  : 0xAA (1 bayt)
 * - ID     : Mesaj ID'si (1 bayt)
 * - TYPE   : Veri tipi (1 bayt)
 * - LEN_L  : Payload uzunluğu düşük bayt (1 bayt)
 * - LEN_H  : Payload uzunluğu yüksek bayt (1 bayt)
 * - PAYLOAD: Veri (LEN bayt)
 * - END    : 0x55 (1 bayt)
 * 
 * @author Arduino-STM32 Port
 * @date 2025
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <Arduino.h>

// Protokol sabitleri
#define PKT_START 0xAA
#define PKT_END   0x55
#define MAX_FRAME 600
#define RX_BUFFER_SIZE MAX_FRAME

// Veri tipleri
enum PayloadType {
    TYPE_U8   = 0x01,  // uint8_t (1 bayt)
    TYPE_I8   = 0x02,  // int8_t  (1 bayt)
    TYPE_U16  = 0x03,  // uint16_t (2 bayt)
    TYPE_I16  = 0x04,  // int16_t (2 bayt)
    TYPE_U32  = 0x05,  // uint32_t (4 bayt)
    TYPE_I32  = 0x06,  // int32_t (4 bayt)
    TYPE_F32  = 0x07,  // float (4 bayt)
    TYPE_F64  = 0x08,  // double (8 bayt)
    TYPE_STR  = 0x10,  // String (metin)
    TYPE_BIN  = 0xFF   // Genel binary veri
};

// Alıcı durum makinesi durumları
enum RxState {
    RX_STATE_WAIT_START,
    RX_STATE_ID,
    RX_STATE_TYPE,
    RX_STATE_LEN_L,
    RX_STATE_LEN_H,
    RX_STATE_PAYLOAD,
    RX_STATE_END
};

/**
 * @brief UART Protokol sınıfı
 */
class UARTProtocol {
private:
    Stream* serial;     // UART/USB serial nesnesi
    
    // Alıcı durum makinesi değişkenleri
    RxState rxState;
    uint8_t rxId;
    uint8_t rxType;
    uint16_t rxLen;
    uint16_t rxIndex;
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    
    // LED pin (isteğe bağlı, veri alındığında yanacak)
    int ledPin;
    
    /**
     * @brief Gelen bir baytı işler
     * @param ch Alınan bayt
     */
    void processByte(uint8_t ch) {
        switch (rxState) {
            case RX_STATE_WAIT_START:
                if (ch == PKT_START) {
                    rxState = RX_STATE_ID;
                }
                break;
                
            case RX_STATE_ID:
                rxId = ch;
                rxState = RX_STATE_TYPE;
                break;
                
            case RX_STATE_TYPE:
                rxType = ch;
                rxState = RX_STATE_LEN_L;
                break;
                
            case RX_STATE_LEN_L:
                rxLen = ch;  // Düşük bayt
                rxState = RX_STATE_LEN_H;
                break;
                
            case RX_STATE_LEN_H:
                rxLen |= ((uint16_t)ch << 8);  // Yüksek bayt
                
                if (rxLen > RX_BUFFER_SIZE) {
                    // Hata: buffer overflow
                    rxState = RX_STATE_WAIT_START;
                } else if (rxLen == 0) {
                    rxState = RX_STATE_END;
                } else {
                    rxIndex = 0;
                    rxState = RX_STATE_PAYLOAD;
                }
                break;
                
            case RX_STATE_PAYLOAD:
                rxBuffer[rxIndex++] = ch;
                if (rxIndex >= rxLen) {
                    rxState = RX_STATE_END;
                }
                break;
                
            case RX_STATE_END:
                if (ch == PKT_END) {
                    // Paket başarıyla alındı
                    onPacketReceived(rxId, rxType, rxBuffer, rxLen);
                }
                rxState = RX_STATE_WAIT_START;
                break;
                
            default:
                rxState = RX_STATE_WAIT_START;
                break;
        }
    }
    
    /**
     * @brief Paket alındığında çağrılan callback
     * @param id Mesaj ID'si
     * @param type Veri tipi
     * @param payload Veri pointer'ı
     * @param len Veri uzunluğu
     */
    void onPacketReceived(uint8_t id, uint8_t type, uint8_t* payload, uint16_t len) {
        Serial.print("Paket Alindi: ID=0x");
        Serial.print(id, HEX);
        Serial.print(", Tip=0x");
        Serial.print(type, HEX);
        Serial.print(", Uzunluk=");
        Serial.println(len);
        
        // LED'i yak (eğer tanımlıysa)
        if (ledPin >= 0) {
            digitalWrite(ledPin, HIGH);
        }
        
        // Veri tipine göre işle
        switch (type) {
            case TYPE_F32: {
                float value;
                memcpy(&value, payload, sizeof(float));
                Serial.print("  -> Veri (float): ");
                Serial.println(value, 2);
                break;
            }
            
            case TYPE_U16: {
                uint16_t value;
                memcpy(&value, payload, sizeof(uint16_t));
                Serial.print("  -> Veri (uint16): ");
                Serial.println(value);
                break;
            }
            
            case TYPE_U32: {
                uint32_t value;
                memcpy(&value, payload, sizeof(uint32_t));
                Serial.print("  -> Veri (uint32): ");
                Serial.println(value);
                break;
            }
            
            case TYPE_STR: {
                Serial.print("  -> Veri (string): ");
                for (uint16_t i = 0; i < len; i++) {
                    Serial.print((char)payload[i]);
                }
                Serial.println();
                break;
            }
            
            default:
                Serial.println("  -> Bilinmeyen/Islenmeyen Veri Tipi");
                break;
        }
    }

public:
    /**
     * @brief Constructor
     * @param ser UART/USB Serial nesnesi (örn: &Serial, &Serial1)
     * @param led LED pin numarası (varsayılan: -1, devre dışı)
     */
    UARTProtocol(Stream* ser, int led = -1) 
        : serial(ser), 
          ledPin(led),
          rxState(RX_STATE_WAIT_START),
          rxId(0),
          rxType(0),
          rxLen(0),
          rxIndex(0) {
    }
    
    /**
     * @brief Protokolü başlatır (LED pin ayarı)
     * @param baud Kullanılmıyor (Serial zaten setup'ta başlatıldı)
     */
    void begin(uint32_t baud = 115200) {
        // Serial.begin() USB için gerekli değil, setup()'ta zaten yapıldı
        // Sadece LED pin ayarla
        rxState = RX_STATE_WAIT_START;
        
        if (ledPin >= 0) {
            pinMode(ledPin, OUTPUT);
            digitalWrite(ledPin, LOW);
        }
    }
    
    /**
     * @brief Gelen veriyi kontrol eder ve işler
     * @note Ana döngüden sürekli çağrılmalıdır
     */
    void update() {
        while (serial->available()) {
            uint8_t ch = serial->read();
            processByte(ch);
        }
    }
    
    /**
     * @brief Veri paketi gönderir
     * @param id Mesaj ID'si
     * @param type Veri tipi
     * @param payload Veri pointer'ı
     * @param payloadLen Veri uzunluğu
     * @return true: başarılı, false: hata
     */
    bool sendFrame(uint8_t id, uint8_t type, const void* payload, uint16_t payloadLen) {
        if (payloadLen > MAX_FRAME) {
            return false;  // Payload çok büyük
        }
        
        // Paket oluştur
        serial->write(PKT_START);
        serial->write(id);
        serial->write(type);
        serial->write((uint8_t)(payloadLen & 0xFF));        // LEN_L
        serial->write((uint8_t)((payloadLen >> 8) & 0xFF)); // LEN_H
        serial->write((const uint8_t*)payload, payloadLen);
        serial->write(PKT_END);
        
        return true;
    }
    
    /**
     * @brief Float değer gönderir
     */
    bool sendFloat(uint8_t id, float value) {
        return sendFrame(id, TYPE_F32, &value, sizeof(float));
    }
    
    /**
     * @brief uint16_t değer gönderir
     */
    bool sendUInt16(uint8_t id, uint16_t value) {
        return sendFrame(id, TYPE_U16, &value, sizeof(uint16_t));
    }
    
    /**
     * @brief uint32_t değer gönderir
     */
    bool sendUInt32(uint8_t id, uint32_t value) {
        return sendFrame(id, TYPE_U32, &value, sizeof(uint32_t));
    }
    
    /**
     * @brief String gönderir
     */
    bool sendString(uint8_t id, const char* str) {
        return sendFrame(id, TYPE_STR, str, strlen(str));
    }
};

#endif // UART_PROTOCOL_H
