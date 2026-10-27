// BME688 + LD2410B di ESP32-S3-Zero, terhubung ke Apple HomeKit lewat HomeSpan
// - BME688 (I2C): suhu, kelembapan, tekanan, resistansi gas
// - LD2410B (UART): ada / tidak ada orang
// - HomeSpan: bridge HomeKit berisi 4 accessory (Temperature, Humidity, Air Quality, Human Presence)
// - WiFi diatur lewat Bluetooth dari app iOS DapurSetup (SDK ESPProvision)
//
// Library (Library Manager):
//   - "Adafruit BME680" (mendukung BME688)
//   - "HomeSpan"
//
// Arduino IDE:
//   Board            : "ESP32S3 Dev Module"
//   USB CDC On Boot  : Enabled
//   Partition Scheme : "Huge APP (3MB No OTA)" --- HomeSpan + stack Bluetooth
//                      tidak muat di partisi default
//   Serial Monitor   : 115200 baud
//
// Keamanan pemasangan:
//   PoP (PIN provisioning) diacak sekali saat boot pertama dan disimpan di NVS,
//   jadi setiap board punya PIN sendiri -- tidak ada satu PIN untuk semua unit.
//   PoP tidak ada di dalam app; app membacanya dari QR yang ditempel di board.
//
// Cara pakai pertama kali:
//   1. Upload, buka Serial Monitor 115200.
//   2. Board mencetak PoP dan QR provisioning. Tempel QR itu di badan board.
//   3. Di app DapurSetup: pindai QR, pilih WiFi, isi kata sandi.
//   4. Board menyala ulang lalu muncul di Apple Home. Kode pairing 466-37-726.
//
// Perintah Serial Monitor (diawali @):
//   @Q  tampilkan PoP + QR provisioning
//   @R  buat PoP baru (QR lama jadi tidak berlaku)
//   Bawaan HomeSpan: ? bantuan | A provisioning ulang | X hapus WiFi | U hapus pairing

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "HomeSpan.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <Preferences.h>

// Generator QR bawaan ESP-IDF, dipakai untuk mencetak QR ke Serial Monitor.
#if __has_include("qrcode.h")
  #include "qrcode.h"
  #define ADA_QRCODE 1
#else
  #warning "qrcode.h tidak ada: QR hanya dicetak sebagai teks"
#endif

// ---------- Pin ----------
#define I2C_SDA     8      // BME688 SDA
#define I2C_SCL     9      // BME688 SCL
#define BME_ADDR    0x77   // Ganti ke 0x76 sesuai hasil I2C scanner

#define LD_RX_PIN   44      // ESP32 RX  <- TX LD2410B
#define LD_TX_PIN   43      // ESP32 TX  -> RX LD2410B
#define LD_BAUD     256000

#define LED_PIN     21     // LED RGB onboard

// ---------- HomeKit ----------
#define HK_NAME          "Masterpiece-A09"
#define HK_HOSTNAME      "a09-iot"
#define HK_MODEL         "ESP32-S3 BME688 LD2410B"
#define HK_PAIRING_CODE  "46637726"       // tampil di Home app sebagai 466-37-726
#define HK_SETUP_ID      "A09F"
// ---------- Provisioning WiFi lewat BLE ----------
// Protokol provisioning Espressif, berpasangan dengan app iOS DapurSetup.
// PoP tidak ditulis di sini: diacak per board dan disimpan di NVS, lalu
// disampaikan ke app lewat QR yang ditempel di badan perangkat.
#define PROV_PREFIX   "DAPUR_"      // nama BLE jadi DAPUR_xxxxxx (6 digit MAC)
#define PROV_NVS      "a09prov"     // ruang NVS tempat PoP disimpan
const unsigned long PROV_TIMEOUT = 300000;   // 5 menit menunggu app

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LED_WRITE(r, g, b) rgbLedWrite(LED_PIN, r, g, b)
#else
  #define LED_WRITE(r, g, b) neopixelWrite(LED_PIN, r, g, b)
#endif

const unsigned long BME_INTERVAL  = 2000;    // baca BME688 tiap 2 detik
const unsigned long GAS_WARMUP_MS = 180000;  // heater BME688 perlu pemanasan ~3 menit

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

// ---------- Characteristic HomeKit ----------
// Diisi di setupHomeKit(), dipakai untuk push nilai sensor ke Home app.
Characteristic::CurrentTemperature     *hkTemp      = nullptr;
Characteristic::StatusFault            *hkTempFault = nullptr;
Characteristic::CurrentRelativeHumidity *hkHum      = nullptr;
Characteristic::StatusFault            *hkHumFault  = nullptr;
Characteristic::AirQuality             *hkAir       = nullptr;
Characteristic::StatusFault            *hkAirFault  = nullptr;
Characteristic::OccupancyDetected      *hkOccupancy = nullptr;
Characteristic::StatusFault            *hkOccFault  = nullptr;

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
    if (radarWarned && hkOccFault) hkOccFault->setVal(0);
    radarWarned = false;

    // Status target: 0 = kosong, 1/2/3 = ada orang (bergerak, diam, atau keduanya)
    int now = (data[2] != 0) ? 1 : 0;
    if (now != presence) {
      presence = now;
      Serial.printf("[%7.1f s] >>> %s\n", millis() / 1000.0,
                    presence ? "ADA ORANG" : "TIDAK ADA ORANG");
      LED_WRITE(0, 0, presence ? 40 : 0);   // LED biru saat ada orang
      if (hkOccupancy) hkOccupancy->setVal(presence);
    }
  }

  if (!radarWarned && millis() - lastFrameMs > 3000) {
    Serial.println("[RADAR] Tidak ada data. Cek TX/RX, baud 256000, dan daya 5V.");
    radarWarned = true;
    presence = -1;
    LED_WRITE(0, 0, 0);
    if (hkOccupancy) hkOccupancy->setVal(0);
    if (hkOccFault)  hkOccFault->setVal(1);
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

// Perkiraan kasar kualitas udara dari resistansi gas (kOhm): makin tinggi makin bersih.
// Ambang di bawah ini heuristik, bukan IAQ terkalibrasi -- untuk itu perlu library BSEC.
// Nilai HomeKit: 0 = unknown, 1 = excellent ... 5 = poor.
uint8_t airQualityFromGas(float kOhm) {
  if (millis() < GAS_WARMUP_MS) return 0;   // heater belum stabil
  if (kOhm >= 200) return 1;
  if (kOhm >= 100) return 2;
  if (kOhm >=  50) return 3;
  if (kOhm >=  20) return 4;
  return 5;
}

void setBmeFault(uint8_t fault) {
  if (hkTempFault) hkTempFault->setVal(fault);
  if (hkHumFault)  hkHumFault->setVal(fault);
  if (hkAirFault)  hkAirFault->setVal(fault);
}

void updateBME() {
  if (!bmeOk) return;

  // Mulai pembacaan baru tiap BME_INTERVAL
  if (!bmeBusy && millis() - lastBmeStart >= BME_INTERVAL) {
    lastBmeStart = millis();
    bmeReadyAt = bme.beginReading();
    if (bmeReadyAt == 0) {
      Serial.println("[BME688] Gagal memulai pembacaan");
      setBmeFault(1);
      return;
    }
    bmeBusy = true;
  }

  // Ambil hasil setelah heater gas selesai, tanpa memblokir radar
  if (bmeBusy && (long)(millis() - bmeReadyAt) >= 0) {
    bmeBusy = false;
    if (!bme.endReading()) {
      Serial.println("[BME688] Gagal membaca sensor");
      setBmeFault(1);
      return;
    }
    setBmeFault(0);

    float gasKOhm = bme.gas_resistance / 1000.0;

    Serial.printf("[%7.1f s] Suhu: %.2f C | Lembap: %.2f %% | Tekanan: %.2f hPa | Gas: %.2f kOhm | Orang: %s\n",
                  millis() / 1000.0,
                  bme.temperature,
                  bme.humidity,
                  bme.pressure / 100.0,
                  gasKOhm,
                  presenceText());

    // Dibulatkan 0.1 supaya noise sensor tidak membanjiri notifikasi HomeKit
    if (hkTemp) hkTemp->setVal(roundf(bme.temperature * 10) / 10.0);
    if (hkHum)  hkHum->setVal(roundf(bme.humidity * 10) / 10.0);
    if (hkAir)  hkAir->setVal(airQualityFromGas(gasKOhm));
  }
}

// =====================================================
// Provisioning WiFi lewat BLE
// Dipasang ke HomeSpan lewat setApFunction(), jadi fungsi ini jalan saat
// kredensial WiFi belum ada, atau saat kamu ketik 'A' di Serial Monitor.
// =====================================================
static volatile bool provSukses = false;
static char provSsid[33];
static char provPwd[65];
static char provPop[9]  = "";      // 8 karakter, unik per board
static char namaBLE[16] = "";      // DAPUR_xxxxxx

void susunNamaBLE() {
  snprintf(namaBLE, sizeof(namaBLE), "%s%06X", PROV_PREFIX,
           (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF));
}

// PoP dibuat sekali saat boot pertama lalu disimpan permanen di NVS.
// Kamus sengaja tanpa 0/O/1/l/I supaya tidak salah baca kalau ditulis tangan.
void muatAtauBuatPop(bool paksaBaru) {
  static const char kamus[] = "abcdefghijkmnpqrstuvwxyz23456789";   // 32 karakter

  Preferences pref;
  pref.begin(PROV_NVS, false);
  String tersimpan = pref.getString("pop", "");

  if (paksaBaru || tersimpan.length() != 8) {
    char baru[9];
    for (int i = 0; i < 8; i++)
      baru[i] = kamus[esp_random() % (sizeof(kamus) - 1)];
    baru[8] = '\0';
    pref.putString("pop", baru);
    tersimpan = baru;
    Serial.println("[PROV] PoP baru dibuat untuk board ini.");
  }
  pref.end();

  strlcpy(provPop, tersimpan.c_str(), sizeof(provPop));
}

#ifdef ADA_QRCODE
// QR digambar dengan blok setengah supaya satu baris teks memuat dua baris modul.
static void gambarQR(esp_qrcode_handle_t qr) {
  static const char *BLOK[] = {
    "  ", "\u2580 ", " \u2580", "\u2580\u2580", "\u2584 ", "\u2588 ", "\u2584\u2580", "\u2588\u2580",
    " \u2584", "\u2580\u2584", " \u2588", "\u2580\u2588", "\u2584\u2584", "\u2588\u2584", "\u2584\u2588", "\u2588\u2588",
  };
  int ukuran = esp_qrcode_get_size(qr), tepi = 2;
  for (int y = -tepi; y < ukuran + tepi; y += 2) {
    for (int x = -tepi; x < ukuran + tepi; x += 2) {
      int n = 0;
      if (esp_qrcode_get_module(qr, x,     y))     n |= 1;
      if (esp_qrcode_get_module(qr, x + 1, y))     n |= 2;
      if (esp_qrcode_get_module(qr, x,     y + 1)) n |= 4;
      if (esp_qrcode_get_module(qr, x + 1, y + 1)) n |= 8;
      Serial.print(BLOK[n]);
    }
    Serial.print("\n");
  }
  Serial.print("\n");
}
#endif

// Format JSON ini yang dibaca scanQRCode() di SDK ESPProvision.
void cetakQrProvisioning() {
  char isi[128];
  // "security":1 wajib ada. Tanpa kunci itu SDK iOS menganggap Security 2,
  // sementara firmware ini memakai Security 1, dan pemasangan akan gagal.
  snprintf(isi, sizeof(isi),
           "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\","
           "\"transport\":\"ble\",\"security\":1}",
           namaBLE, provPop);

  Serial.printf("\n[PROV] Nama Bluetooth : %s\n", namaBLE);
  Serial.printf("[PROV] PoP            : %s\n", provPop);
  Serial.printf("[PROV] Isi QR         : %s\n\n", isi);

#ifdef ADA_QRCODE
  esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
  cfg.display_func = gambarQR;
  esp_qrcode_generate(&cfg, isi);
  Serial.println("[PROV] Pindai QR di atas dari app DapurSetup, atau cetak dan tempel di board.");
#else
  Serial.println("[PROV] Salin teks \"Isi QR\" di atas ke pembuat QR mana pun, lalu tempel di board.");
#endif
}

// Dipanggil dari task FreeRTOS lain, bukan dari loop().
void provEvent(arduino_event_t *e) {
  switch (e->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.println("[PROV] Bluetooth aktif, menunggu app...");
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
      memcpy(provSsid, e->event_info.prov_cred_recv.ssid, 32);      provSsid[32] = 0;
      memcpy(provPwd,  e->event_info.prov_cred_recv.password, 64);  provPwd[64]  = 0;
      Serial.printf("[PROV] WiFi diterima: %s, mencoba tersambung...\n", provSsid);
      break;
    case ARDUINO_EVENT_PROV_CRED_FAIL:
      Serial.println("[PROV] Gagal tersambung. Periksa kata sandi, kirim ulang dari app.");
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("[PROV] WiFi tersambung");
      provSukses = true;
      break;
    default:
      break;
  }
}

void bleProvisioning() {
  Serial.println("\n[PROV] Kredensial WiFi belum ada. Mulai provisioning lewat BLE.");
  LED_WRITE(30, 0, 30);              // ungu = menunggu provisioning
  cetakQrProvisioning();

  WiFi.onEvent(provEvent);

  // FREE_BTDM melepas memori Bluetooth begitu provisioning selesai.
  WiFiProv.beginProvision(NETWORK_PROV_SCHEME_BLE,
                          NETWORK_PROV_SCHEME_HANDLER_FREE_BTDM,
                          NETWORK_PROV_SECURITY_1,
                          provPop, namaBLE, NULL, NULL, true);

  unsigned long mulai = millis();
  while (!provSukses && millis() - mulai < PROV_TIMEOUT)
    delay(100);

  if (provSukses) {
    homeSpan.setWifiCredentials(provSsid, provPwd);
    LED_WRITE(0, 30, 0);
    Serial.printf("[PROV] Tersimpan untuk \"%s\". Restart...\n", provSsid);
  } else {
    LED_WRITE(0, 0, 0);
    Serial.println("[PROV] Waktu habis, tidak ada app yang menghubungi. Restart...");
  }

  delay(5000);                       // beri waktu app menerima status berhasil
  ESP.restart();
}

// ---------- Perintah Serial Monitor ----------
void cmdTampilQR(const char *buf) {
  cetakQrProvisioning();
}

void cmdPopBaru(const char *buf) {
  muatAtauBuatPop(true);
  Serial.println("[PROV] QR lama tidak berlaku lagi. Cetak ulang yang di bawah ini.");
  cetakQrProvisioning();
}

// =====================================================
// HomeKit: bridge + 4 accessory supaya tiap sensor punya tile sendiri di Home app
// =====================================================
void newAccessoryInfo(const char *name, const char *serial) {
  new SpanAccessory();
  new Service::AccessoryInformation();
    new Characteristic::Name(name);
    new Characteristic::Manufacturer("A09 Final");
    new Characteristic::SerialNumber(serial);
    new Characteristic::Model(HK_MODEL);
    new Characteristic::FirmwareRevision("1.0.0");
    new Characteristic::Identify();
}

void setupHomeKit() {
  homeSpan.setPairingCode(HK_PAIRING_CODE);
  homeSpan.setQRID(HK_SETUP_ID);
  homeSpan.setApFunction(bleProvisioning);  // WiFi lewat BLE, bukan Access Point
  homeSpan.enableAutoStartAP();            // jalankan saat kredensial WiFi belum ada
  homeSpan.setLogLevel(1);

  homeSpan.begin(Category::Bridges, HK_NAME, HK_HOSTNAME, HK_MODEL);

  new SpanUserCommand('Q', "- tampilkan PoP + QR provisioning", cmdTampilQR);
  new SpanUserCommand('R', "- buat PoP baru (QR lama tidak berlaku)", cmdPopBaru);

  // Accessory 0: bridge itu sendiri
  newAccessoryInfo(HK_NAME, "A09-BRIDGE");

  // Accessory 1: Temperature
  newAccessoryInfo("Temperature", "A09-TEMP");
  new Service::TemperatureSensor();
    hkTemp      = new Characteristic::CurrentTemperature(25);
    hkTempFault = new Characteristic::StatusFault(0);

  // Accessory 2: Humidity
  newAccessoryInfo("Humidity", "A09-HUM");
  new Service::HumiditySensor();
    hkHum      = new Characteristic::CurrentRelativeHumidity(50);
    hkHumFault = new Characteristic::StatusFault(0);

  // Accessory 3: Air Quality
  newAccessoryInfo("Air Quality", "A09-AIR");
  new Service::AirQualitySensor();
    hkAir      = new Characteristic::AirQuality(0);
    hkAirFault = new Characteristic::StatusFault(0);

  // Accessory 4: Human Presence
  newAccessoryInfo("Human Presence", "A09-OCC");
  new Service::OccupancySensor();
    hkOccupancy = new Characteristic::OccupancyDetected(0);
    hkOccFault  = new Characteristic::StatusFault(0);

  if (!bmeOk) setBmeFault(1);
}

// =====================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\n=== BME688 + LD2410B + HomeKit ===");

  LED_WRITE(0, 0, 0);

  // LD2410B
  // Buffer dibesarkan karena homeSpan.poll() bisa sejenak menahan loop saat
  // pairing / rekoneksi WiFi, sementara radar terus mengirim di 256000 baud.
  radar.setRxBufferSize(2048);
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

  // Identitas provisioning disiapkan sebelum HomeSpan, karena setApFunction()
  // bisa langsung dipanggil saat kredensial WiFi belum ada.
  susunNamaBLE();
  muatAtauBuatPop(false);

  setupHomeKit();
}

void loop() {
  homeSpan.poll();
  updateRadar();
  updateBME();
}
