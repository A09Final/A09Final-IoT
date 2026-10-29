// BME688 + LD2410B + MLX90640 di ESP32-S3-Zero, terhubung ke Apple HomeKit lewat HomeSpan
// - BME688 (I2C) lewat BSEC: suhu & kelembapan terkompensasi, tekanan,
//   IAQ terkalibrasi beserta indikator akurasinya, dan perkiraan eCO2
// - LD2410B (UART): ada / tidak ada orang
// - MLX90640 (I2C): kamera thermal 32x24, dipakai memantau panas kompor
// - HomeSpan: bridge HomeKit berisi 5 accessory (Temperature, Humidity, Air Quality,
//   Human Presence, Stove Heat)
// - WiFi diatur lewat Bluetooth dari app iOS DapurSetup (SDK ESPProvision)
//
// Library (Library Manager):
//   - "bsec2" (Bosch) + "BME68x Sensor library" -- IAQ terkalibrasi
//   - "Adafruit MLX90640" -- kamera thermal (ikut menarik "Adafruit BusIO")
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
//   @B  status BSEC: akurasi, IAQ, dan state kalibrasi di NVS
//   @T  status kamera thermal + peta panas untuk mengarahkan kamera ke kompor
//   Bawaan HomeSpan: ? bantuan | A provisioning ulang | X hapus WiFi | U hapus pairing

#include <Arduino.h>
#include <Wire.h>
#include <bsec2.h>
#include <Adafruit_MLX90640.h>
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
#define I2C_SDA     8      // SDA, dipakai bersama BME688 dan MLX90640
#define I2C_SCL     9      // SCL, dipakai bersama BME688 dan MLX90640
#define BME_ADDR    0x77   // Ganti ke 0x76 sesuai hasil I2C scanner
#define MLX_ADDR    0x33   // MLX90640, alamatnya tetap -- tidak bentrok dengan BME688

// MLX90640 memindahkan 1.664 byte tiap subpage; di 100 kHz satu frame penuh
// sudah lewat 0,3 detik. Chipnya sanggup sampai 1 MHz, tapi di atas 400 kHz
// hasilnya mulai bergantung panjang kabel dan nilai pull-up, dan BME688 ikut
// kena karena satu bus. 400 kHz aman dengan pull-up 10k bawaan modul.
#define I2C_FREQ    400000

#define LD_RX_PIN   44      // ESP32 RX  <- TX LD2410B
#define LD_TX_PIN   43      // ESP32 TX  -> RX LD2410B
#define LD_BAUD     256000

// ---------- Jangkauan deteksi radar ----------
// Radar menjangkau sampai ~6 m, jauh lebih luas dari yang kita perlukan.
// Target di luar jangkauan ini diabaikan.
//
// Ambang masuk dan keluar sengaja dibedakan (histeresis). Angka jarak dari
// radar bergetar beberapa sentimeter, jadi dengan satu ambang, orang yang
// berdiri tepat di batas akan membuat status berkedip terus-menerus.
// Di antara kedua angka ini, status dipertahankan apa adanya.
#define RADAR_MASUK_CM    180   // lebih dekat dari ini -> ada orang
#define RADAR_KELUAR_CM   220   // lebih jauh dari ini  -> tidak ada orang
#define RADAR_ENERGI_MIN   20   // di bawah ini dianggap pantulan lemah, bukan orang

// ---------- Pemantauan panas kompor ----------
// Yang dilaporkan bukan satu piksel terpanas, melainkan rata-rata beberapa
// piksel terpanas. Satu piksel bisa melonjak sendiri karena derau atau karena
// memang rusak, sedangkan tungku menyala selalu mengisi banyak piksel sekaligus.
#define KOMPOR_PIKSEL       16      // dari 768 piksel, sekitar 2% yang terpanas
#define KOMPOR_BACA_TIAP    1000    // ms, jeda antar frame
#define KOMPOR_LOG_TIAP     3000    // ms, seirama baris pembacaan BSEC
#define KOMPOR_HILANG_MS    10000   // tanpa frame selama ini -> StatusFault

// Rentang yang didaftarkan ke HomeKit. Bawaan CurrentTemperature berhenti di
// 100 C, jauh di bawah suhu wajan, jadi rentangnya diperlebar sendiri.
#define KOMPOR_SUHU_MIN     -40
#define KOMPOR_SUHU_MAX      300

// Nilai dikirim ke HomeKit kalau berubah sebanyak ini, supaya derau sepersepuluh
// derajat tidak membanjiri notifikasi. Tetap dikirim berkala walaupun diam,
// supaya Home app tahu angkanya masih hidup.
#define KOMPOR_KIRIM_DELTA  0.5f
#define KOMPOR_KIRIM_TIAP   30000

#define LED_PIN     21     // LED RGB onboard

// ---------- HomeKit ----------
#define HK_NAME          "Masterpiece-A09"
#define HK_HOSTNAME      "a09-iot"
#define HK_MODEL         "ESP32-S3 BME688 LD2410B MLX90640"
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

// Konfigurasi BSEC: algoritma IAQ, suplai 3,3V, baca tiap 3 detik, kalibrasi
// 4 hari. Wajib dimuat -- tanpa ini BSEC memakai interval bawaan yang berbeda
// dan menolak langganan kita. Dipakai konfigurasi milik BME680 karena folder
// bme688/ hanya berisi konfigurasi klasifikasi gas yang tidak menghasilkan IAQ.
const uint8_t bsecConfig[] = {
  #include "config/bme680/bme680_iaq_33v_3s_4d/bsec_iaq.txt"
};
HardwareSerial radar(1);
Adafruit_MLX90640 mlx;

// ---------- Status ----------
// Diisi task BSEC, dibaca loop(). Baca-tulis 32-bit di ESP32 tidak bisa terpotong
// di tengah, jadi untuk sekumpulan angka seperti ini tidak perlu mutex.
volatile bool    bsecOk      = false;   // sensor berhasil diinisialisasi
volatile bool    bsecAdaData = false;   // ada hasil baru yang belum dikirim
volatile float   bsecSuhu    = 0;       // sudah dikompensasi panas heater
volatile float   bsecLembap  = 0;
volatile float   bsecTekanan = 0;       // hPa
volatile bool    bsecStabil  = false;   // pemanasan awal sensor gas selesai
volatile float   bsecIaq     = 0;       // Static IAQ, 0..500
volatile float   bsecCo2     = 0;       // perkiraan eCO2, ppm
volatile uint8_t bsecAkurasi = 0;       // 0 = belum stabil ... 3 = terkalibrasi
volatile unsigned long bsecSimpanTerakhir = 0;   // 0 = belum pernah sejak boot

int presence = -1;              // -1 = belum ada data, 0 = tidak ada, 1 = ada

// Nilai mentah radar, sengaja tidak dihaluskan. Disediakan untuk lapisan
// pengambil keputusan yang berjalan di board ini juga.
uint16_t radarJarakCm = 0;      // jarak target terdekat yang sah
uint8_t  radarEnergi  = 0;      // energi target tersebut
unsigned long lastFrameMs = 0;
bool radarWarned = false;

// Ringkasan termal. Diisi task termal, dibaca loop() -- aman tanpa mutex dengan
// alasan yang sama seperti blok BSEC di atas.
volatile bool  mlxOk        = false;  // kamera berhasil diinisialisasi
volatile bool  mlxAdaData   = false;  // ada frame baru yang belum dikirim
volatile float mlxKompor    = 0;      // rata-rata KOMPOR_PIKSEL piksel terpanas
volatile float mlxMaks      = 0;      // piksel terpanas apa adanya
volatile float mlxMinim     = 0;      // piksel terdingin, gambaran suhu ruang
volatile float mlxAmbien    = 0;      // suhu badan sensornya sendiri
volatile bool  mlxMintaPeta = false;  // @T menyalakan ini, task yang mencetak
volatile unsigned long mlxFrameTerakhir = 0;
bool mlxWarned = false;               // hanya disentuh loop()

// Frame mentah 32x24, milik task termal. loop() cuma memakai ringkasan di atas,
// jadi tidak ada yang perlu dikunci.
static float mlxFrame[32 * 24];

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
Characteristic::CurrentTemperature     *hkStove      = nullptr;
Characteristic::StatusFault            *hkStoveFault = nullptr;

// Penanda waktu yang bentuknya mengikuti durasi, supaya tetap enak dibaca
// setelah board menyala berjam-jam:  [3.1 s]  ->  [2:05]  ->  [1:23:45]
// Memakai buffer statis, jadi hanya boleh dipanggil dari loop().
const char* penandaWaktu() {
  static char buf[16];
  unsigned long ms = millis();
  unsigned long detik = ms / 1000;

  if (detik < 60)
    snprintf(buf, sizeof(buf), "%.1f s", ms / 1000.0);
  else if (detik < 3600)
    snprintf(buf, sizeof(buf), "%lu:%02lu", detik / 60, detik % 60);
  else
    snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu",
             detik / 3600, (detik / 60) % 60, detik % 60);

  return buf;
}

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

    // Jarak kedua jenis target selalu terisi, tetapi hanya sah kalau energinya
    // di atas ambang -- saat tidak ada target bergerak, jarak geraknya tetap
    // berisi angka sisa. Jadi energi yang menentukan, bukan jaraknya.
    uint16_t jarakGerak  = data[3] | (data[4] << 8);
    uint8_t  energiGerak = data[5];
    uint16_t jarakDiam   = data[6] | (data[7] << 8);
    uint8_t  energiDiam  = data[8];

    bool     adaTarget = false;
    uint16_t jarak     = 0;
    uint8_t  energi    = 0;

    if (energiGerak >= RADAR_ENERGI_MIN) {
      adaTarget = true;  jarak = jarakGerak;  energi = energiGerak;
    }
    // Ambil yang terdekat kalau dua-duanya sah.
    if (energiDiam >= RADAR_ENERGI_MIN && (!adaTarget || jarakDiam < jarak)) {
      adaTarget = true;  jarak = jarakDiam;   energi = energiDiam;
    }

    radarJarakCm = adaTarget ? jarak : 0;
    radarEnergi  = adaTarget ? energi : 0;

    // Histeresis: di antara kedua ambang, keadaan sebelumnya dipertahankan.
    int now = (presence == 1) ? 1 : 0;
    if (!adaTarget)                    now = 0;
    else if (jarak <= RADAR_MASUK_CM)  now = 1;
    else if (jarak >= RADAR_KELUAR_CM) now = 0;
    if (now != presence) {
      presence = now;
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
// BSEC harus membaca sensor tepat tiap 3 detik. loop() tidak bisa menjamin itu,
// karena homeSpan.poll() kadang menahan eksekusi beberapa detik saat pairing.
// Jadi BSEC dijalankan di task FreeRTOS sendiri, di Core 1 dengan prioritas 2 --
// satu tingkat di atas loop() sehingga bisa menyelanya. Core 0 dihindari karena
// di sana WiFi dan Bluetooth berjalan pada prioritas jauh lebih tinggi.
//
// Task ini hanya mengisi angka. Semua pemanggilan HomeKit tetap di loop(),
// karena HomeSpan tidak aman dipanggil dari dua task sekaligus.
// =====================================================
// Dipakai di baris pembacaan berkala. Buffer statis, jadi hanya boleh
// dipanggil dari loop() -- sama seperti penandaWaktu().
const char* presenceText() {
  static char buf[32];
  if (presence == 1)
    snprintf(buf, sizeof(buf), "ADA (%u cm, energi %u)", radarJarakCm, radarEnergi);
  else if (presence == 0)
    snprintf(buf, sizeof(buf), "TIDAK ADA");
  else
    snprintf(buf, sizeof(buf), "?");
  return buf;
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
// Tanpa ini, tiap board dinyalakan ulang kalibrasi mulai dari nol lagi.
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
  bsecSimpanTerakhir = millis();
  Serial.printf("[BSEC] State kalibrasi disimpan (akurasi %u).\n", bsecAkurasi);
  return true;
}

// Mencetak status BSEC, dan mengembalikan true hanya kalau ada error sungguhan.
// BSEC memakai angka negatif untuk error dan positif untuk peringatan; peringatan
// tidak boleh menghentikan apa pun karena sensornya tetap bekerja.
bool adaErrorBsec() {
  bool error = false;

  if (envSensor.status < BSEC_OK) {
    Serial.printf("[BSEC] ERROR library: %d\n", envSensor.status);
    error = true;
  } else if (envSensor.status > BSEC_OK) {
    Serial.printf("[BSEC] peringatan library: %d\n", envSensor.status);
  }

  if (envSensor.sensor.status < BME68X_OK) {
    Serial.printf("[BSEC] ERROR sensor BME68x: %d\n", envSensor.sensor.status);
    error = true;
  } else if (envSensor.sensor.status > BME68X_OK) {
    Serial.printf("[BSEC] peringatan sensor BME68x: %d\n", envSensor.sensor.status);
  }

  return error;
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
        // Dokumentasi Bosch menyebut satuannya Pa, tapi yang benar-benar
        // keluar sudah hPa (terukur ~1005). Jangan dibagi lagi.
        bsecTekanan = o.signal;
        break;
      case BSEC_OUTPUT_RUN_IN_STATUS:
        bsecStabil = (o.signal != 0);
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
    BSEC_OUTPUT_RUN_IN_STATUS,
  };

  if (!envSensor.begin(BME_ADDR, Wire)) {
    adaErrorBsec();
    Serial.println("[BSEC] Sensor tidak ditemukan di I2C. Lanjut dengan radar saja.");
    vTaskDelete(NULL);
    return;
  }

  if (!envSensor.setConfig(bsecConfig) && adaErrorBsec()) {
    Serial.println("[BSEC] Konfigurasi ditolak. Lanjut dengan radar saja.");
    vTaskDelete(NULL);
    return;
  }

  if (!muatStateBsec()) adaErrorBsec();

  if (!envSensor.updateSubscription(daftar, ARRAY_LEN(daftar), BSEC_SAMPLE_RATE_LP)
      && adaErrorBsec()) {
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
    if (!envSensor.run()) adaErrorBsec();    // BSEC sendiri yang menjaga jadwal

    // Kalibrasi setengah jadi tetap disimpan -- BSEC bisa melanjutkan dari sana.
    // Selain berkala, disimpan juga tiap kali akurasi naik, supaya kemajuan
    // penting tidak menunggu jadwal berikutnya.
    bool akurasiNaik = bsecAkurasi > akurasiTerakhir;
    if (bsecAkurasi >= 1 &&
        (akurasiNaik || millis() - simpanTerakhir >= BSEC_SIMPAN_TIAP)) {
      akurasiTerakhir = bsecAkurasi;
      simpanTerakhir  = millis();
      if (!simpanStateBsec()) adaErrorBsec();
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

  Serial.printf("[%s] Suhu: %.2f C | Lembap: %.2f %% | Tekanan: %.2f hPa | "
                "IAQ: %.0f (akurasi %u) | eCO2: %.0f ppm | Orang: %s\n",
                penandaWaktu(), suhu, lembap, bsecTekanan,
                iaq, akurasi, bsecCo2, presenceText());

  setBmeFault(0);
  // Dibulatkan 0.1 supaya noise sensor tidak membanjiri notifikasi HomeKit
  if (hkTemp) hkTemp->setVal(roundf(suhu * 10) / 10.0);
  if (hkHum)  hkHum->setVal(roundf(lembap * 10) / 10.0);
  if (hkAir)  hkAir->setVal(iaqKeHomeKit(iaq, akurasi));
}

// =====================================================
// MLX90640: kamera thermal 32x24 untuk panas kompor
//
// Satu frame terdiri dari dua subpage, dan getFrame() menunggu masing-masing
// siap sambil terus menanyai sensor lewat I2C. Penantian itu akan menahan
// homeSpan.poll() kalau dijalankan di loop(), jadi kamera dapat task sendiri --
// alasan yang sama seperti BSEC, hanya prioritasnya lebih rendah.
//
// Catatan pemasangan: library memakai emisivitas tetap 0,95. Itu cocok untuk
// tungku keramik, besi tuang, dan panci gelap. Stainless mengkilap memantulkan
// panas dan akan terbaca jauh lebih rendah dari suhu sebenarnya, jadi arahkan
// kamera ke badan tungku, bukan ke panci yang berkilau.
// =====================================================

// Rata-rata KOMPOR_PIKSEL piksel terpanas, dicari dalam satu lintasan: tiap
// piksel disisipkan ke daftar pendek yang selalu terurut menurun, jadi 768
// angka tidak perlu disalin dan diurutkan dulu.
float suhuTitikPanas(const float *frame, int n) {
  float top[KOMPOR_PIKSEL];
  int   terisi = 0;

  for (int i = 0; i < n; i++) {
    float v = frame[i];
    if (isnan(v)) continue;                            // piksel rusak dilewati
    if (terisi == KOMPOR_PIKSEL && v <= top[terisi - 1]) continue;

    // Daftar penuh: yang terkecil terdorong keluar dari ujung.
    int pos = (terisi < KOMPOR_PIKSEL) ? terisi : KOMPOR_PIKSEL - 1;
    while (pos > 0 && top[pos - 1] < v) { top[pos] = top[pos - 1]; pos--; }
    top[pos] = v;
    if (terisi < KOMPOR_PIKSEL) terisi++;
  }

  if (terisi == 0) return NAN;
  float jumlah = 0;
  for (int i = 0; i < terisi; i++) jumlah += top[i];
  return jumlah / terisi;
}

// Peta panas kasar untuk mengarahkan kamera: 32x24 diringkas jadi 16x12 dengan
// mengambil piksel terpanas tiap blok 2x2, lalu dipetakan ke tangga karakter
// relatif terhadap frame itu sendiri -- jadi tetap terbaca baik saat kompor
// menyala maupun sudah dingin.
//
// Dicetak dari dalam task termal, memakai frame miliknya sendiri, supaya tidak
// ada frame setengah tertimpa yang terbaca.
void cetakPetaTermal() {
  static const char RAMP[]   = " .:-=+*#%@";
  const int         TERAKHIR = sizeof(RAMP) - 2;   // indeks '@'

  float bawah   = mlxMinim;
  float atas    = mlxMaks;
  float rentang = atas - bawah;
  if (rentang < 1.0f) rentang = 1.0f;              // frame rata, hindari bagi nol

  Serial.printf("\n[MLX] peta panas, %.1f C ('%c') .. %.1f C ('%c'), "
                "kiri-atas = piksel (0,0)\n",
                bawah, RAMP[0], atas, RAMP[TERAKHIR]);

  for (int y = 0; y < 24; y += 2) {
    Serial.print("      ");
    for (int x = 0; x < 32; x += 2) {
      float v = -1e9f;
      for (int dy = 0; dy < 2; dy++)
        for (int dx = 0; dx < 2; dx++) {
          float piksel = mlxFrame[(y + dy) * 32 + (x + dx)];
          if (!isnan(piksel) && piksel > v) v = piksel;
        }

      int level = (int)((v - bawah) / rentang * TERAKHIR);
      if (level < 0)        level = 0;
      if (level > TERAKHIR) level = TERAKHIR;

      // Dua kolom per blok supaya petanya tidak gepeng di Serial Monitor.
      Serial.write(RAMP[level]);
      Serial.write(RAMP[level]);
    }
    Serial.print("\n");
  }
  Serial.print("\n");
}

void tugasTermal(void *) {
  if (!mlx.begin(MLX_ADDR, &Wire)) {
    Serial.println("[MLX] Kamera tidak ditemukan di I2C (0x33). Lanjut tanpa panas kompor.");
    vTaskDelete(NULL);
    return;
  }

  // Chess: pola bawaan pabrik, paling sedikit artefaknya untuk benda diam.
  // 18 bit: kompromi antara derau dan waktu konversi.
  // 8 Hz per subpage berarti frame penuh siap tiap 0,25 detik, sementara kita
  // hanya mengambil satu frame per detik -- datanya selalu sudah menunggu, jadi
  // getFrame() tidak lama memutar polling I2C. Lebih cepat dari ini membuat
  // subpage berikutnya tiba di tengah pembacaan dan library menyerah (galat -8).
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_8_HZ);

  mlxOk = true;
  Serial.printf("[MLX] Siap, serial %04X-%04X-%04X\n",
                mlx.serialNumber[0], mlx.serialNumber[1], mlx.serialNumber[2]);

  for (;;) {
    int status = mlx.getFrame(mlxFrame);

    if (status != 0) {
      // Frame gagal tidak langsung dianggap kamera hilang; loop() yang
      // memutuskan itu setelah KOMPOR_HILANG_MS tanpa frame yang sah.
      Serial.printf("[MLX] Frame gagal dibaca: %d\n", status);
    } else {
      float terpanas  = -1e9f;
      float terdingin =  1e9f;
      for (int i = 0; i < 32 * 24; i++) {
        float v = mlxFrame[i];
        if (isnan(v)) continue;
        if (v > terpanas)  terpanas  = v;
        if (v < terdingin) terdingin = v;
      }

      float kompor = suhuTitikPanas(mlxFrame, 32 * 24);
      if (!isnan(kompor)) {                   // seluruh frame rusak -> lewati
        mlxKompor = kompor;
        mlxMaks   = terpanas;
        mlxMinim  = terdingin;
        mlxAmbien = mlx.getTa(false);         // dari frame yang baru dibaca
        mlxFrameTerakhir = millis();
        mlxAdaData = true;

        if (mlxMintaPeta) { cetakPetaTermal(); mlxMintaPeta = false; }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(KOMPOR_BACA_TIAP));
  }
}

// Dipanggil dari loop(): satu-satunya tempat HomeKit disentuh.
void updateTermal() {
  static float         kirimTerakhir = NAN;
  static unsigned long kirimMs = 0;
  static unsigned long logMs   = 0;

  if (mlxAdaData) {
    mlxAdaData = false;

    float kompor = mlxKompor;
    if (mlxWarned && hkStoveFault) hkStoveFault->setVal(0);
    mlxWarned = false;

    if (millis() - logMs >= KOMPOR_LOG_TIAP) {
      logMs = millis();
      Serial.printf("[%s] Kompor: %.1f C | terpanas: %.1f C | terdingin: %.1f C | "
                    "badan sensor: %.1f C\n",
                    penandaWaktu(), kompor, mlxMaks, mlxMinim, mlxAmbien);
    }

    // Rentangnya sudah diperlebar di setupHomeKit(), tapi wajan yang benar-benar
    // panas masih bisa melewatinya, dan HomeSpan menolak nilai di luar rentang.
    float kirim = constrain(roundf(kompor * 10) / 10.0f,
                            (float)KOMPOR_SUHU_MIN, (float)KOMPOR_SUHU_MAX);

    if (hkStove && (isnan(kirimTerakhir) ||
                    fabsf(kirim - kirimTerakhir) >= KOMPOR_KIRIM_DELTA ||
                    millis() - kirimMs >= KOMPOR_KIRIM_TIAP)) {
      hkStove->setVal(kirim);
      kirimTerakhir = kirim;
      kirimMs       = millis();
    }
  }

  // Kabel lepas atau kamera mati: jangan biarkan Home app memamerkan angka
  // lama seolah masih benar. Kamera yang tidak pernah terdeteksi sejak boot
  // (mlxOk false) sudah ditandai fault di setupHomeKit().
  if (mlxOk && !mlxWarned && millis() - mlxFrameTerakhir > KOMPOR_HILANG_MS) {
    Serial.println("[MLX] Tidak ada frame. Cek kabel SDA/SCL dan daya 3,3V kamera.");
    mlxWarned = true;
    if (hkStoveFault) hkStoveFault->setVal(1);
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
static char hkCode[9]   = "";      // 8 angka kode pairing HomeKit, unik per board
static char namaBLE[16] = "";      // DAPUR_xxxxxx

void susunNamaBLE() {
  snprintf(namaBLE, sizeof(namaBLE), "%s%06X", PROV_PREFIX,
           (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF));
}

// PoP dibuat sekali saat boot pertama lalu disimpan permanen di NVS.
// Karakter 0/O/1/l/I sengaja dibuang supaya tidak salah baca kalau ditulis tangan.
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

// Apple menolak kode yang terlalu mudah ditebak, dan HomeSpan menghentikan
// sketch sepenuhnya kalau diberi kode tidak sah. Jadi diperiksa dulu.
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

// Kode pairing HomeKit unik per board. Disimpan sendiri di NVS karena HomeSpan
// hanya menyimpan bentuk teracaknya dan tidak bisa memberikan kodenya kembali.
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

// Blok siap cetak untuk ditempel di badan perangkat. Sengaja hanya memuat satu
// QR: QR provisioning harus dipindai karena PoP-nya acak, sedangkan kode HomeKit
// cukup diketik. Dua QR berdampingan hanya membuat orang salah pindai.
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

// Ringkasan BSEC tanpa perlu menunggu boot atau mengorek log.
void cmdStatusBsec(const char *buf) {
  Preferences pref;
  pref.begin(BSEC_NVS, true);
  size_t n = pref.getBytesLength("state");
  pref.end();

  Serial.printf("\n[BSEC] sensor   : %s\n", bsecOk ? "siap" : "TIDAK AKTIF");
  Serial.printf("[BSEC] akurasi  : %u  (0 belum stabil .. 3 terkalibrasi)\n", bsecAkurasi);
  Serial.printf("[BSEC] pemanasan: %s\n", bsecStabil ? "selesai" : "masih berlangsung");
  Serial.printf("[BSEC] IAQ      : %.0f  -> HomeKit AirQuality %u\n",
                bsecIaq, iaqKeHomeKit(bsecIaq, bsecAkurasi));
  Serial.printf("[BSEC] eCO2     : %.0f ppm\n", bsecCo2);
  Serial.printf("[BSEC] suhu     : %.2f C | lembap %.2f %% | tekanan %.2f hPa\n",
                bsecSuhu, bsecLembap, bsecTekanan);

  if (n == 0)
    Serial.println("[BSEC] state NVS: BELUM ADA (akurasi harus >= 1 dulu)");
  else
    Serial.printf("[BSEC] state NVS: ADA, %u byte\n", (unsigned)n);

  if (bsecSimpanTerakhir == 0)
    Serial.println("[BSEC] simpan    : belum pernah sejak boot ini\n");
  else
    Serial.printf("[BSEC] simpan    : %.1f menit lalu\n\n",
                  (millis() - bsecSimpanTerakhir) / 60000.0);
}

// Ringkasan termal plus peta panas, untuk memastikan kamera benar-benar
// mengarah ke kompor dan bukan ke dinding atau ke langit-langit.
void cmdStatusTermal(const char *buf) {
  Serial.printf("\n[MLX] kamera  : %s\n", mlxOk ? "siap" : "TIDAK AKTIF");

  if (!mlxOk) {
    Serial.printf("[MLX] Cek SDA/SCL di pin %d/%d, daya 3,3V, dan alamat 0x%02X.\n\n",
                  I2C_SDA, I2C_SCL, MLX_ADDR);
    return;
  }

  Serial.printf("[MLX] kompor  : %.1f C  (rata-rata %d piksel terpanas)\n",
                mlxKompor, KOMPOR_PIKSEL);
  Serial.printf("[MLX] sebaran : %.1f C .. %.1f C | badan sensor %.1f C\n",
                mlxMinim, mlxMaks, mlxAmbien);

  if (mlxFrameTerakhir == 0)
    Serial.println("[MLX] frame   : belum ada yang berhasil dibaca");
  else
    Serial.printf("[MLX] frame   : %.1f detik lalu\n",
                  (millis() - mlxFrameTerakhir) / 1000.0);

  // Petanya dicetak task termal setelah frame berikutnya, jadi muncul sesaat
  // setelah baris-baris di atas.
  mlxMintaPeta = true;
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
  homeSpan.setLogLevel(0);

  homeSpan.begin(Category::Bridges, HK_NAME, HK_HOSTNAME, HK_MODEL);

  new SpanUserCommand('Q', "- cetak ulang stiker perangkat", cmdTampilStiker);
  new SpanUserCommand('R', "- buat PoP baru (QR lama tidak berlaku)", cmdPopBaru);
  new SpanUserCommand('C', "- buat kode HomeKit baru (perlu pair ulang)", cmdKodeHkBaru);
  new SpanUserCommand('B', "- status BSEC + state kalibrasi di NVS", cmdStatusBsec);
  new SpanUserCommand('T', "- status kamera thermal + peta panas", cmdStatusTermal);

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

  // Accessory 5: Stove Heat -- panas kompor dari kamera thermal
  newAccessoryInfo("Stove Heat", "A09-STOVE");
  new Service::TemperatureSensor();
    hkStove = new Characteristic::CurrentTemperature(25);
    // Bawaan HAP berhenti di 100 C. Rentang yang lebih lebar diterima Home app
    // dan ikut melebarkan pilihan ambang saat membuat automation.
    hkStove->setRange(KOMPOR_SUHU_MIN, KOMPOR_SUHU_MAX, 0.1);
    hkStoveFault = new Characteristic::StatusFault(0);

  // Kedua sensor masih dalam proses inisialisasi di task masing-masing.
  setBmeFault(1);
  hkStoveFault->setVal(1);
}

// =====================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\n=== BME688 + LD2410B + MLX90640 + HomeKit ===");

  LED_WRITE(0, 0, 0);

  // LD2410B
  // Buffer dibesarkan karena homeSpan.poll() bisa sejenak menahan loop saat
  // pairing / rekoneksi WiFi, sementara radar terus mengirim di 256000 baud.
  radar.setRxBufferSize(2048);
  radar.begin(LD_BAUD, SERIAL_8N1, LD_RX_PIN, LD_TX_PIN);

  // BME688 lewat BSEC, di task sendiri -- lihat komentar di bagian BSEC.
  // Prioritas 2 agar menyela loopTask (Core-1, Priority=1) saat waktunya tiba.
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  xTaskCreatePinnedToCore(tugasBsec, "bsec", 4096, NULL, 2, NULL, 1);

  // MLX90640 ikut di bus I2C yang sama, juga di task sendiri. Prioritas 1,
  // setara loopTask, supaya pembacaan frame yang panjang tidak menahan HomeKit
  // dan BSEC di prioritas 2 tetap bisa menyelanya saat jadwalnya tiba. Stack
  // 8 KB karena begin() dan getFrame() masing-masing menaruh buffer ~1,7 KB di
  // stack. Dua task memakai Wire bersamaan dengan aman: driver I2C
  // Arduino-ESP32 mengunci bus per transaksi, dan alamat keduanya berbeda.
  xTaskCreatePinnedToCore(tugasTermal, "termal", 8192, NULL, 1, NULL, 1);

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
  updateTermal();
}
