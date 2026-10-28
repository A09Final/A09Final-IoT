// BME688 + LD2410B di ESP32-S3-Zero, terhubung ke Apple HomeKit lewat HomeSpan
// - BME688 (I2C) lewat BSEC: suhu & kelembapan terkompensasi, tekanan,
//   IAQ terkalibrasi beserta indikator akurasinya, dan perkiraan eCO2
// - LD2410B (UART): ada / tidak ada orang
// - HomeSpan: bridge HomeKit berisi 4 accessory (Temperature, Humidity, Air Quality, Human Presence)
// - WiFi diatur lewat Bluetooth dari app iOS DapurSetup (SDK ESPProvision)
//
// Library (Library Manager):
//   - "bsec2" (Bosch) + "BME68x Sensor library" -- IAQ terkalibrasi
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
//   PoP (PIN provisioning) dan kode pairing HomeKit sama-sama diacak saat boot
//   pertama lalu disimpan di NVS, jadi tiap board punya miliknya sendiri --
//   tidak ada satu nilai yang sama untuk semua unit. Keduanya tidak ada di
//   dalam app; app membacanya dari stiker yang ditempel di badan perangkat.
//
// Cara pakai pertama kali:
//   1. Upload, buka Serial Monitor 115200.
//   2. Board mencetak blok stiker: QR provisioning + kode pairing HomeKit.
//      Cetak dan tempel di badan board.
//   3. Di app DapurSetup: pindai QR, pilih WiFi, isi kata sandi.
//   4. Board menyala ulang, lalu tambahkan di Apple Home memakai kode pairing
//      yang tertera di stiker.
//
// Perintah Serial Monitor (diawali @):
//   @Q  cetak ulang stiker perangkat
//   @R  buat PoP baru (QR lama jadi tidak berlaku)
//   @C  buat kode HomeKit baru (perangkat perlu di-pair ulang)
//   Bawaan HomeSpan: ? bantuan | A provisioning ulang | X hapus WiFi | U hapus pairing

#include <Arduino.h>
#include <Wire.h>
#include <bsec2.h>
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
// Kode pairing HomeKit tidak ditulis di sini. Seperti PoP, kodenya diacak
// per board saat boot pertama dan disimpan di NVS, lalu dicetak ke Serial
// Monitor untuk ditempel di badan perangkat.
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

// BSEC memutuskan sendiri kapan membaca sensor (mode LP = tiap 3 detik).
#define BSEC_NVS "a09bsec"                        // ruang NVS untuk state kalibrasi
const unsigned long BSEC_SIMPAN_TIAP = 6UL * 60UL * 60UL * 1000UL;   // 6 jam

// ---------- Objek ----------
Bsec2 envSensor;
HardwareSerial radar(1);

// ---------- Status ----------
// Diisi task BSEC, dibaca loop(). Akses 32-bit selaras bersifat atomik di
// ESP32, jadi untuk sekumpulan angka seperti ini cukup tanpa mutex.
volatile bool    bsecOk      = false;   // sensor berhasil diinisialisasi
volatile bool    bsecAdaData = false;   // ada hasil baru yang belum dikirim
volatile float   bsecSuhu    = 0;       // sudah dikompensasi panas heater
volatile float   bsecLembap  = 0;
volatile float   bsecTekanan = 0;       // hPa
volatile float   bsecIaq     = 0;       // Static IAQ, 0..500
volatile float   bsecCo2     = 0;       // perkiraan eCO2, ppm
volatile uint8_t bsecAkurasi = 0;       // 0 = belum stabil ... 3 = terkalibrasi

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
// BME688 lewat BSEC
//
// BSEC menuntut pembacaan di interval tetap (mode LP = 3 detik). loop() tidak
// bisa menjaminnya karena homeSpan.poll() dapat menahan eksekusi beberapa detik
// saat pairing. Karena itu BSEC dijalankan di task FreeRTOS sendiri.
//
// Task dipasang di Core 1 dengan prioritas 2, satu tingkat di atas loopTask
// (Core-1, Priority=1), sehingga menyela loop() begitu waktunya tiba. Core 0
// sengaja dihindari karena di sana stack WiFi/BT berjalan pada prioritas jauh
// lebih tinggi dan justru akan menunda task ini.
//
// Pembagian tugas: task hanya MEMPRODUKSI angka. Seluruh pemanggilan HomeKit
// tetap di loop(), karena HomeSpan tidak dirancang thread-safe.
// =====================================================
const char* presenceText() {
  if (presence == 1) return "ADA";
  if (presence == 0) return "TIDAK ADA";
  return "?";
}

void setBmeFault(uint8_t fault) {
  if (hkTempFault) hkTempFault->setVal(fault);
  if (hkHumFault)  hkHumFault->setVal(fault);
  if (hkAirFault)  hkAirFault->setVal(fault);
}

// Ambang IAQ resmi Bosch, dipetakan ke enum AirQuality HomeKit (1..5).
// Akurasi 0 berarti algoritma belum punya dasar, jadi dilaporkan Unknown
// alih-alih menampilkan angka yang terlihat yakin tapi belum berarti.
uint8_t iaqKeHomeKit(float iaq, uint8_t akurasi) {
  if (akurasi == 0) return 0;      // Unknown
  if (iaq <=  50)   return 1;      // Excellent
  if (iaq <= 100)   return 2;      // Good
  if (iaq <= 150)   return 3;      // Fair
  if (iaq <= 200)   return 4;      // Inferior
  return 5;                        // Poor
}

// ---------- State kalibrasi di NVS ----------
// Tanpa ini, tiap reboot mengulang kalibrasi dari nol dan akurasi tidak pernah
// sempat mencapai 3 selama pengembangan.
static uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];

bool muatStateBsec() {
  Preferences pref;
  pref.begin(BSEC_NVS, true);
  size_t n = pref.getBytesLength("state");
  bool ada = (n == BSEC_MAX_STATE_BLOB_SIZE);
  if (ada) pref.getBytes("state", bsecState, BSEC_MAX_STATE_BLOB_SIZE);
  pref.end();

  if (!ada) {
    Serial.println("[BSEC] Belum ada state kalibrasi, mulai dari awal.");
    return true;
  }
  if (!envSensor.setState(bsecState)) return false;
  Serial.println("[BSEC] State kalibrasi dimuat dari NVS.");
  return true;
}

bool simpanStateBsec() {
  if (!envSensor.getState(bsecState)) return false;
  Preferences pref;
  pref.begin(BSEC_NVS, false);
  pref.putBytes("state", bsecState, BSEC_MAX_STATE_BLOB_SIZE);
  pref.end();
  Serial.printf("[BSEC] State kalibrasi disimpan (akurasi %u).\n", bsecAkurasi);
  return true;
}

void cekStatusBsec() {
  if (envSensor.status != BSEC_OK)
    Serial.printf("[BSEC] status perpustakaan: %d\n", envSensor.status);
  if (envSensor.sensor.status != BME68X_OK)
    Serial.printf("[BSEC] status sensor BME68x: %d\n", envSensor.sensor.status);
}

// Dipanggil BSEC dari dalam task-nya sendiri setiap ada keluaran baru.
void bsecDataBaru(const bme68xData data, const bsecOutputs outputs, const Bsec2 bsec) {
  if (!outputs.nOutputs) return;

  for (uint8_t i = 0; i < outputs.nOutputs; i++) {
    const bsecData o = outputs.output[i];
    switch (o.sensor_id) {
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
        bsecSuhu = o.signal;                 // sudah dikoreksi panas heater
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
        bsecLembap = o.signal;
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
        bsecTekanan = o.signal / 100.0f;     // Pa -> hPa
        break;
      case BSEC_OUTPUT_STATIC_IAQ:           // lebih cocok untuk alat yang diam
        bsecIaq     = o.signal;
        bsecAkurasi = o.accuracy;
        break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:
        bsecCo2 = o.signal;
        break;
      default:
        break;
    }
  }
  bsecAdaData = true;
}

void tugasBsec(void *) {
  bsecSensor daftar[] = {
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
  };

  if (!envSensor.begin(BME_ADDR, Wire)) {
    cekStatusBsec();
    Serial.println("[BSEC] Sensor tidak ditemukan. Lanjut dengan radar saja.");
    vTaskDelete(NULL);
    return;
  }

  if (!muatStateBsec()) cekStatusBsec();

  if (!envSensor.updateSubscription(daftar, ARRAY_LEN(daftar), BSEC_SAMPLE_RATE_LP)) {
    cekStatusBsec();
    Serial.println("[BSEC] Gagal berlangganan keluaran sensor.");
    vTaskDelete(NULL);
    return;
  }

  envSensor.attachCallback(bsecDataBaru);
  bsecOk = true;
  Serial.printf("[BSEC] Siap, versi %d.%d.%d.%d\n",
                envSensor.version.major, envSensor.version.minor,
                envSensor.version.major_bugfix, envSensor.version.minor_bugfix);

  unsigned long simpanTerakhir  = millis();
  uint8_t       akurasiTerakhir = 0;

  for (;;) {
    if (!envSensor.run()) cekStatusBsec();    // BSEC sendiri yang menjaga jadwal

    // Kalibrasi setengah jadi tetap layak disimpan: BSEC bisa melanjutkan dari
    // sana, jauh lebih baik daripada mengulang dari nol setiap reboot. Selain
    // berkala, state juga langsung disimpan tiap kali akurasi naik satu tingkat
    // supaya kemajuan penting tidak menunggu jadwal berikutnya.
    bool akurasiNaik = bsecAkurasi > akurasiTerakhir;
    if (bsecAkurasi >= 1 &&
        (akurasiNaik || millis() - simpanTerakhir >= BSEC_SIMPAN_TIAP)) {
      akurasiTerakhir = bsecAkurasi;
      simpanTerakhir  = millis();
      if (!simpanStateBsec()) cekStatusBsec();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Dipanggil dari loop(): satu-satunya tempat HomeKit disentuh.
void updateBME() {
  if (!bsecAdaData) return;
  bsecAdaData = false;

  float   suhu    = bsecSuhu;
  float   lembap  = bsecLembap;
  float   iaq     = bsecIaq;
  uint8_t akurasi = bsecAkurasi;

  Serial.printf("[%7.1f s] Suhu: %.2f C | Lembap: %.2f %% | Tekanan: %.2f hPa | "
                "IAQ: %.0f (akurasi %u) | eCO2: %.0f ppm | Orang: %s\n",
                millis() / 1000.0, suhu, lembap, bsecTekanan,
                iaq, akurasi, bsecCo2, presenceText());

  setBmeFault(0);
  // Dibulatkan 0.1 supaya noise sensor tidak membanjiri notifikasi HomeKit
  if (hkTemp) hkTemp->setVal(roundf(suhu * 10) / 10.0);
  if (hkHum)  hkHum->setVal(roundf(lembap * 10) / 10.0);
  if (hkAir)  hkAir->setVal(iaqKeHomeKit(iaq, akurasi));
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
static char hkCode[9]   = "";      // 8 angka kode pairing HomeKit, unik per board
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

// Apple menolak kode yang terlalu mudah ditebak. HomeSpan juga MENGHENTIKAN
// sketch kalau setPairingCode() diberi kode tidak sah, jadi divalidasi dulu.
bool kodeHkValid(const char *k) {
  if (strlen(k) != 8) return false;
  for (int i = 0; i < 8; i++)
    if (k[i] < '0' || k[i] > '9') return false;

  bool semuaSama = true;
  for (int i = 1; i < 8; i++)
    semuaSama &= (k[i] == k[0]);
  if (semuaSama) return false;

  return strcmp(k, "12345678") != 0 && strcmp(k, "87654321") != 0;
}

// Kode pairing HomeKit unik per board, disimpan di NVS kita sendiri karena
// HomeSpan hanya menyimpan versi ter-hash dan tidak bisa mengembalikannya.
void muatAtauBuatKodeHk(bool paksaBaru) {
  Preferences pref;
  pref.begin(PROV_NVS, false);
  String tersimpan = pref.getString("hkcode", "");

  if (paksaBaru || !kodeHkValid(tersimpan.c_str())) {
    char baru[9];
    do {
      for (int i = 0; i < 8; i++)
        baru[i] = '0' + (esp_random() % 10);
      baru[8] = '\0';
    } while (!kodeHkValid(baru));
    pref.putString("hkcode", baru);
    tersimpan = baru;
    Serial.println("[HK] Kode pairing baru dibuat untuk board ini.");
  }
  pref.end();

  strlcpy(hkCode, tersimpan.c_str(), sizeof(hkCode));
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

// Satu blok siap cetak untuk ditempel di badan perangkat.
// Hanya ADA SATU QR di sini, sengaja: QR provisioning wajib dipindai karena
// PoP-nya acak, sedangkan kode HomeKit cukup diketik 8 angka lewat alur
// standar Apple. Dua QR berdampingan hanya membuat pengguna salah pindai.
void cetakStiker() {
  // Format JSON ini yang dibaca scanQRCode() di SDK ESPProvision.
  // "security":1 wajib: tanpa itu SDK iOS menganggap Security 2 dan gagal.
  char isi[128];
  snprintf(isi, sizeof(isi),
           "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\","
           "\"transport\":\"ble\",\"security\":1}",
           namaBLE, provPop);

  HapQR qr;
  const char *payloadHk = qr.get(strtoul(hkCode, NULL, 10), HK_SETUP_ID,
                                 2 /* Category::Bridges */);

  Serial.println("\n======== STIKER PERANGKAT (cetak & tempel) ========");
  Serial.printf("  Nama         : %s\n", HK_NAME);
  Serial.printf("  Serial       : %s\n", namaBLE);
  Serial.printf("  Kode HomeKit : %.3s-%.2s-%.3s\n", hkCode, hkCode + 3, hkCode + 5);
  Serial.println();

#ifdef ADA_QRCODE
  esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
  cfg.display_func = gambarQR;
  esp_qrcode_generate(&cfg, isi);
#endif

  Serial.println("===================================================");
  Serial.printf("  PoP          : %s\n", provPop);
  Serial.printf("  Isi QR       : %s\n", isi);
  Serial.printf("  Payload HK   : %s   (opsional, untuk QR HomeKit)\n", payloadHk);
  Serial.println("===================================================\n");
#ifndef ADA_QRCODE
  Serial.println("[PROV] qrcode.h tidak tersedia: salin \"Isi QR\" ke pembuat QR mana pun.");
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
  cetakStiker();

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
void cmdTampilStiker(const char *buf) {
  cetakStiker();
}

void cmdPopBaru(const char *buf) {
  muatAtauBuatPop(true);
  Serial.println("[PROV] QR lama tidak berlaku lagi. Cetak ulang stiker di bawah.");
  cetakStiker();
}

void cmdKodeHkBaru(const char *buf) {
  muatAtauBuatKodeHk(true);
  homeSpan.setPairingCode(hkCode);
  Serial.println("[HK] Perangkat perlu di-pair ulang dengan kode baru di bawah.");
  cetakStiker();
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
  homeSpan.setPairingCode(hkCode);          // diacak per board, lihat muatAtauBuatKodeHk()
  homeSpan.setQRID(HK_SETUP_ID);
  homeSpan.setApFunction(bleProvisioning);  // WiFi lewat BLE, bukan Access Point
  homeSpan.enableAutoStartAP();            // jalankan saat kredensial WiFi belum ada
  homeSpan.setLogLevel(1);

  homeSpan.begin(Category::Bridges, HK_NAME, HK_HOSTNAME, HK_MODEL);

  new SpanUserCommand('Q', "- cetak ulang stiker perangkat", cmdTampilStiker);
  new SpanUserCommand('R', "- buat PoP baru (QR lama tidak berlaku)", cmdPopBaru);
  new SpanUserCommand('C', "- buat kode HomeKit baru (perlu pair ulang)", cmdKodeHkBaru);

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

  // Sensor masih dalam proses inisialisasi di task BSEC saat ini.
  setBmeFault(1);
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

  // BME688 lewat BSEC, di task sendiri -- lihat komentar di bagian BSEC.
  // Prioritas 2 agar menyela loopTask (Core-1, Priority=1) saat waktunya tiba.
  Wire.begin(I2C_SDA, I2C_SCL);
  xTaskCreatePinnedToCore(tugasBsec, "bsec", 4096, NULL, 2, NULL, 1);

  // Identitas provisioning disiapkan sebelum HomeSpan, karena setApFunction()
  // bisa langsung dipanggil saat kredensial WiFi belum ada.
  susunNamaBLE();
  muatAtauBuatPop(false);
  muatAtauBuatKodeHk(false);

  setupHomeKit();
  cetakStiker();
}

void loop() {
  homeSpan.poll();
  updateRadar();
  updateBME();
}
