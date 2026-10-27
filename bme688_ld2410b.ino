// BME688 + LD2410B di ESP32-S3-Zero
// - BME688 (I2C): suhu, kelembapan, tekanan, resistansi gas
// - LD2410B (UART): ada / tidak ada orang
//
// Library: "Adafruit BME680" (Library Manager), mendukung BME688.
// Arduino IDE: Board "ESP32S3 Dev Module", USB CDC On Boot: Enabled.
// Serial Monitor: 115200 baud.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// ---------- Pin ----------
#define I2C_SDA     8      // BME688 SDA
#define I2C_SCL     9      // BME688 SCL
#define BME_ADDR    0x77   // Ganti ke 0x76 sesuai hasil I2C scanner

#define LD_RX_PIN   44      // ESP32 RX  <- TX LD2410B
#define LD_TX_PIN   43      // ESP32 TX  -> RX LD2410B
#define LD_BAUD     256000

#define LED_PIN     21     // LED RGB onboard

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LED_WRITE(r, g, b) rgbLedWrite(LED_PIN, r, g, b)
#else
  #define LED_WRITE(r, g, b) neopixelWrite(LED_PIN, r, g, b)
#endif

const unsigned long BME_INTERVAL = 2000;  // baca BME688 tiap 2 detik

// ---------- Objek ----------
Adafruit_BME680 bme(&Wire);
HardwareSerial radar(1);

// ---------- Status ----------
bool bmeOk = false;
bool bmeBusy = false;
unsigned long bmeReadyAt = 0;
unsigned long lastBmeStart = 0;

int presence = -1;              // -1 = belum ada data, 0 = tidak ada, 1 = ada
unsigned long lastFrameMs = 0;
bool radarWarned = false;

// =====================================================
// LD2410B: parser frame
// Format: F4 F3 F2 F1 | len (2 byte LE) | data | F8 F7 F6 F5
// =====================================================
const uint8_t HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t FOOTER[4] = {0xF8, 0xF7, 0xF6, 0xF5};

enum ParseState { WAIT_HEADER, READ_LEN, READ_DATA, READ_FOOTER };
ParseState pState = WAIT_HEADER;
uint8_t  idx = 0;
uint8_t  lenBytes[2];
uint16_t dataLen = 0;
uint8_t  data[64];

bool readFrame() {
  while (radar.available()) {
    uint8_t b = radar.read();
    switch (pState) {
      case WAIT_HEADER:
        if (b == HEADER[idx]) { if (++idx == 4) { idx = 0; pState = READ_LEN; } }
        else idx = (b == HEADER[0]) ? 1 : 0;
        break;
      case READ_LEN:
        lenBytes[idx++] = b;
        if (idx == 2) {
          dataLen = lenBytes[0] | (lenBytes[1] << 8);
          idx = 0;
          pState = (dataLen > 0 && dataLen <= sizeof(data)) ? READ_DATA : WAIT_HEADER;
        }
        break;
      case READ_DATA:
        data[idx++] = b;
        if (idx == dataLen) { idx = 0; pState = READ_FOOTER; }
        break;
      case READ_FOOTER:
        if (b == FOOTER[idx]) { if (++idx == 4) { idx = 0; pState = WAIT_HEADER; return true; } }
        else { idx = 0; pState = WAIT_HEADER; }
        break;
    }
  }
  return false;
}

void updateRadar() {
  if (readFrame() && dataLen >= 13 && data[1] == 0xAA) {
    lastFrameMs = millis();
    radarWarned = false;

    // Status target: 0 = kosong, 1/2/3 = ada orang (bergerak, diam, atau keduanya)
    int now = (data[2] != 0) ? 1 : 0;
    if (now != presence) {
      presence = now;
      Serial.printf("[%7.1f s] >>> %s\n", millis() / 1000.0,
                    presence ? "ADA ORANG" : "TIDAK ADA ORANG");
      LED_WRITE(0, 0, presence ? 40 : 0);   // LED biru saat ada orang
    }
  }

  if (!radarWarned && millis() - lastFrameMs > 3000) {
    Serial.println("[RADAR] Tidak ada data. Cek TX/RX, baud 256000, dan daya 5V.");
    radarWarned = true;
    presence = -1;
    LED_WRITE(0, 0, 0);
  }
}

// =====================================================
// BME688: pembacaan non-blocking
// =====================================================
const char* presenceText() {
  if (presence == 1) return "ADA";
  if (presence == 0) return "TIDAK ADA";
  return "?";
}

void updateBME() {
  if (!bmeOk) return;

  // Mulai pembacaan baru tiap BME_INTERVAL
  if (!bmeBusy && millis() - lastBmeStart >= BME_INTERVAL) {
    lastBmeStart = millis();
    bmeReadyAt = bme.beginReading();
    if (bmeReadyAt == 0) {
      Serial.println("[BME688] Gagal memulai pembacaan");
      return;
    }
    bmeBusy = true;
  }

  // Ambil hasil setelah heater gas selesai, tanpa memblokir radar
  if (bmeBusy && (long)(millis() - bmeReadyAt) >= 0) {
    bmeBusy = false;
    if (!bme.endReading()) {
      Serial.println("[BME688] Gagal membaca sensor");
      return;
    }
    Serial.printf("[%7.1f s] Suhu: %.2f C | Lembap: %.2f %% | Tekanan: %.2f hPa | Gas: %.2f kOhm | Orang: %s\n",
                  millis() / 1000.0,
                  bme.temperature,
                  bme.humidity,
                  bme.pressure / 100.0,
                  bme.gas_resistance / 1000.0,
                  presenceText());
  }
}

// =====================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\n=== BME688 + LD2410B ===");

  LED_WRITE(0, 0, 0);

  // LD2410B
  radar.setRxBufferSize(512);
  radar.begin(LD_BAUD, SERIAL_8N1, LD_RX_PIN, LD_TX_PIN);

  // BME688
  Wire.begin(I2C_SDA, I2C_SCL);
  if (bme.begin(BME_ADDR)) {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);   // 320 C selama 150 ms
    bmeOk = true;
    Serial.println("[BME688] Siap");
  } else {
    // Tetap jalan dengan radar saja
    Serial.println("[BME688] Tidak ditemukan! Cek alamat I2C dan kabel SDA/SCL. Lanjut dengan radar saja.");
  }
}

void loop() {
  updateRadar();
  updateBME();
}
