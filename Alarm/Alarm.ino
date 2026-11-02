// Tombol + Speaker + OLED di ESP32-C3 OLED (0.42"), terhubung ke Apple HomeKit lewat HomeSpan
// - Tekan tombol -> alarm berbunyi (dua nada bergantian), tekan lagi -> alarm mati.
// - OLED menampilkan status alarm dan status tombol.
// - HomeKit: satu accessory Switch "Alarm". Bisa dinyalakan dari Home app atau
//   dari automation (mis. saat Stove Heat di board Sensor terlalu panas), dan
//   ikut berubah saat tombol fisik ditekan.
// - WiFi diatur lewat Bluetooth dari app iOS DapurSetup (SDK ESPProvision),
//   sama seperti board Sensor.
//
// Sambungan:
//   Tombol (pasangan kontak NO) -> GPIO 3 dan GND (bebas mana yang mana)
//   Grove Speaker: SIG -> GPIO 4, VCC -> 5V, GND -> GND, NC tidak disambung
//   OLED onboard di GPIO 5 (SDA) dan GPIO 6 (SCL)
//
// Library (Library Manager): U8g2, HomeSpan
//
// Arduino IDE:
//   Board            : "ESP32C3 Dev Module"
//   USB CDC On Boot  : Enabled
//   Partition Scheme : "Huge APP (3MB No OTA/1MB SPIFFS)" --- HomeSpan + stack
//                      Bluetooth tidak muat di partisi default
//   Serial Monitor   : 115200 baud
//
// Keamanan pemasangan:
//   PoP (PIN provisioning) dan kode pairing HomeKit sama-sama diacak saat boot
//   pertama lalu disimpan di NVS, jadi tiap board punya miliknya sendiri.
//   Saat mode setup, PoP juga tampil di OLED -- hanya orang yang memegang
//   perangkat yang bisa membacanya, sama seperti stiker.
//
// Cara pakai pertama kali:
//   1. Upload, buka Serial Monitor 115200.
//   2. Board mencetak blok stiker: QR provisioning + kode pairing HomeKit.
//      OLED menampilkan "SETUP WIFI", nama BLE, dan PIN.
//   3. Di app DapurSetup: pilih DAPUR_ALM_xxxxxx (atau pindai QR), isi PIN,
//      pilih WiFi, isi kata sandi.
//   4. Board menyala ulang, lalu tambahkan di Apple Home memakai kode pairing
//      yang tertera di stiker.
//   Selama mode setup, tombol dan speaker tetap berfungsi sebagai alarm lokal.
//
// Perintah Serial Monitor (diawali @):
//   @Q  cetak ulang stiker perangkat
//   @R  buat PoP baru (QR lama jadi tidak berlaku)
//   @C  buat kode HomeKit baru (perangkat perlu di-pair ulang)
//   Bawaan HomeSpan: ? bantuan | A provisioning ulang | X hapus WiFi | U hapus pairing

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
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

// Pengaman: tanpa USB CDC On Boot, output Serial tidak sampai ke laptop
#if defined(CONFIG_IDF_TARGET_ESP32C3) && !ARDUINO_USB_CDC_ON_BOOT
#error "Aktifkan Tools > USB CDC On Boot: Enabled, lalu upload ulang"
#endif

#define BUTTON_PIN   3
#define SPEAKER_PIN  4
#define OLED_SDA     5
#define OLED_SCL     6

// Kontak tombol yang dipakai: false = NO, true = NC
#define USE_NC           true

// Jenis tombol:
//   false = tombol biasa: tekan sekali = alarm nyala, tekan lagi = alarm mati
//   true  = tombol terkunci (latching): alarm nyala selama tombol terkunci di posisi ditekan
#define BUTTON_LATCHING  false

// Nada alarm. Grove Speaker paling jelas di nada rendah, sesuaikan dengan hasil tes sweep.
const uint32_t TONE_HIGH = 440;            // Hz
const uint32_t TONE_LOW  = 330;            // Hz
const unsigned long TONE_STEP_MS = 250;    // lama tiap nada
const unsigned long DEBOUNCE_MS  = 50;

// ---------- HomeKit ----------
#define HK_NAME          "Alarm-A09"
#define HK_HOSTNAME      "a09-alarm"
#define HK_MODEL         "ESP32-C3 OLED Speaker"
// Kode pairing HomeKit tidak ditulis di sini: diacak per board saat boot
// pertama, disimpan di NVS, lalu dicetak ke Serial Monitor.
#define HK_SETUP_ID      "A09A"

// ---------- Provisioning WiFi lewat BLE ----------
// Protokol provisioning Espressif, berpasangan dengan app iOS DapurSetup.
// App hanya menampilkan perangkat berawalan "DAPUR_", jadi alarm memakai
// "DAPUR_ALM_" supaya tetap muncul tapi bisa dibedakan dari board Sensor.
#define PROV_PREFIX   "DAPUR_ALM_"  // nama BLE jadi DAPUR_ALM_xxxxxx (6 digit MAC)
#define PROV_NVS      "a09prov"     // ruang NVS tempat PoP dan kode HomeKit disimpan
const unsigned long PROV_TIMEOUT = 300000;   // 5 menit menunggu app

U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

// ---------- Speaker lewat LEDC ----------
// Frekuensi 0 = diam, dan pin tetap ditahan LOW supaya speaker tidak mendengung.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void speakerBegin()           { ledcAttach(SPEAKER_PIN, 1000, 10); ledcWriteTone(SPEAKER_PIN, 0); }
void speakerTone(uint32_t hz) { ledcWriteTone(SPEAKER_PIN, hz); }
#else
#define SPEAKER_CH 0
void speakerBegin()           { ledcSetup(SPEAKER_CH, 1000, 10); ledcAttachPin(SPEAKER_PIN, SPEAKER_CH); ledcWriteTone(SPEAKER_CH, 0); }
void speakerTone(uint32_t hz) { ledcWriteTone(SPEAKER_CH, hz); }
#endif

// ---------- Status ----------
bool buttonState = false;       // true = ditekan (sudah di-debounce)
bool lastReading = false;
unsigned long lastChangeMs = 0;

bool alarmOn = false;
bool tonePhase = false;         // false = nada tinggi, true = nada rendah
unsigned long lastToneMs = 0;

uint32_t pressCount = 0;
unsigned long lastReportMs = 0;

// Mode setup WiFi: OLED menampilkan data provisioning, bukan status tombol.
bool modeSetup = false;
const char *setupStatus = "Menunggu app";

SpanCharacteristic *hkAlarm = nullptr;    // Characteristic::On milik Switch "Alarm"

static volatile bool provSukses = false;
static char provSsid[33];
static char provPwd[65];
static char provPop[9]  = "";      // 8 karakter, unik per board
static char hkCode[9]   = "";      // 8 angka kode pairing HomeKit, unik per board
static char namaBLE[20] = "";      // DAPUR_ALM_xxxxxx

bool readPressed() {
  bool level = digitalRead(BUTTON_PIN);
  // NO + pull-up: ditekan = LOW. NC + pull-up: ditekan = HIGH.
  return USE_NC ? (level == HIGH) : (level == LOW);
}

// ---------- OLED ----------
void drawCentered(const char* s, int y) {
  int w = u8g2.getStrWidth(s);
  u8g2.drawStr((72 - w) / 2, y, s);
}

// Layar 72x40 hanya muat 14 karakter font 5x7, jadi awalan "DAPUR_" dibuang.
void drawSetupScreen() {
  char baris[20];

  u8g2.setFont(u8g2_font_6x10_tf);
  drawCentered("SETUP WIFI", 9);

  u8g2.setFont(u8g2_font_5x7_tf);
  drawCentered(namaBLE + strlen("DAPUR_"), 19);
  snprintf(baris, sizeof(baris), "PIN %s", provPop);
  drawCentered(baris, 29);
  drawCentered(setupStatus, 39);
}

void updateOLED() {
  u8g2.clearBuffer();

  // Saat alarm berbunyi, status alarm tetap didahulukan walau sedang setup.
  if (modeSetup && !alarmOn) {
    drawSetupScreen();
    u8g2.sendBuffer();
    return;
  }

  // Baris atas: status alarm (berkedip saat alarm nyala)
  u8g2.setFont(u8g2_font_7x14B_tf);
  if (alarmOn && !tonePhase) {
    u8g2.drawBox(0, 0, 72, 19);
    u8g2.setDrawColor(0);
    drawCentered("ALARM!", 15);
    u8g2.setDrawColor(1);
  } else {
    drawCentered(alarmOn ? "ALARM!" : "AMAN", 15);
  }

  // Baris bawah: status tombol
  u8g2.setFont(u8g2_font_6x10_tf);
  drawCentered("Tombol:", 29);
  drawCentered(buttonState ? "DITEKAN" : "DILEPAS", 40);

  u8g2.sendBuffer();
}

// ---------- Alarm ----------
// kabarkanHk = false saat perintah datang dari HomeKit sendiri, supaya nilainya
// tidak dikirim balik ke Home app.
void startAlarm(bool kabarkanHk = true) {
  if (alarmOn) return;
  alarmOn = true;
  tonePhase = false;
  lastToneMs = millis();
  speakerTone(TONE_HIGH);
  Serial.printf("[%7.1f s] >>> ALARM NYALA\n", millis() / 1000.0);
  if (kabarkanHk && hkAlarm && !modeSetup) hkAlarm->setVal(true);
  updateOLED();
}

void stopAlarm(bool kabarkanHk = true) {
  if (!alarmOn) return;
  alarmOn = false;
  speakerTone(0);
  Serial.printf("[%7.1f s] >>> ALARM MATI\n", millis() / 1000.0);
  if (kabarkanHk && hkAlarm && !modeSetup) hkAlarm->setVal(false);
  updateOLED();
}

// Ganti nada tiap TONE_STEP_MS tanpa delay(), supaya tombol tetap responsif
void updateAlarmTone() {
  if (!alarmOn) return;
  if (millis() - lastToneMs >= TONE_STEP_MS) {
    lastToneMs = millis();
    tonePhase = !tonePhase;
    speakerTone(tonePhase ? TONE_LOW : TONE_HIGH);
    updateOLED();   // kedipkan OLED seirama nada
  }
}

// ---------- Tombol ----------
void onButtonChange(bool pressed) {
  Serial.printf("[%7.1f s] Tombol %s\n", millis() / 1000.0, pressed ? "DITEKAN" : "DILEPAS");

  if (BUTTON_LATCHING) {
    if (pressed && !alarmOn) startAlarm();
    else if (!pressed && alarmOn) stopAlarm();
  } else if (pressed) {
    pressCount++;
    if (alarmOn) stopAlarm();
    else startAlarm();
  }
  updateOLED();
}

// Debounce tombol + nada alarm. Dipanggil dari loop() dan juga dari loop
// tunggu provisioning, supaya alarm lokal tetap jalan selama setup WiFi.
void layaniTombolDanAlarm() {
  bool reading = readPressed();
  if (reading != lastReading) {
    lastReading = reading;
    lastChangeMs = millis();
  }
  if (millis() - lastChangeMs > DEBOUNCE_MS && reading != buttonState) {
    buttonState = reading;
    onButtonChange(buttonState);
  }

  updateAlarmTone();
}

// =====================================================
// HomeKit: Switch "Alarm"
// =====================================================
struct AlarmSwitch : Service::Switch {
  AlarmSwitch() : Service::Switch() {
    hkAlarm = new Characteristic::On(false);
  }

  boolean update() override {
    if (hkAlarm->getNewVal()) {
      Serial.printf("[%7.1f s] HomeKit meminta alarm NYALA\n", millis() / 1000.0);
      startAlarm(false);
    } else {
      Serial.printf("[%7.1f s] HomeKit meminta alarm MATI\n", millis() / 1000.0);
      stopAlarm(false);
    }
    return true;
  }
};

// =====================================================
// Provisioning WiFi lewat BLE
// Dipasang ke HomeSpan lewat setApFunction(), jadi fungsi ini jalan saat
// kredensial WiFi belum ada, atau saat kamu ketik 'A' di Serial Monitor.
// =====================================================
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
    "  ", "▀ ", " ▀", "▀▀", "▄ ", "█ ", "▄▀", "█▀",
    " ▄", "▀▄", " █", "▀█", "▄▄", "█▄", "▄█", "██",
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
                                 (uint8_t)Category::Switches);

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

// Dipanggil dari task FreeRTOS lain, bukan dari loop(). Hanya mengisi variabel;
// OLED digambar ulang oleh loop tunggu di bleProvisioning().
void provEvent(arduino_event_t *e) {
  switch (e->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.println("[PROV] Bluetooth aktif, menunggu app...");
      setupStatus = "Menunggu app";
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
      memcpy(provSsid, e->event_info.prov_cred_recv.ssid, 32);      provSsid[32] = 0;
      memcpy(provPwd,  e->event_info.prov_cred_recv.password, 64);  provPwd[64]  = 0;
      Serial.printf("[PROV] WiFi diterima: %s, mencoba tersambung...\n", provSsid);
      setupStatus = "Menyambung..";
      break;
    case ARDUINO_EVENT_PROV_CRED_FAIL:
      Serial.println("[PROV] Gagal tersambung. Periksa kata sandi, kirim ulang dari app.");
      setupStatus = "Gagal, ulangi";
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("[PROV] WiFi tersambung");
      setupStatus = "Tersambung!";
      provSukses = true;
      break;
    default:
      break;
  }
}

void bleProvisioning() {
  Serial.println("\n[PROV] Kredensial WiFi belum ada. Mulai provisioning lewat BLE.");
  modeSetup = true;
  setupStatus = "Menunggu app";
  updateOLED();
  cetakStiker();

  WiFi.onEvent(provEvent);

  // FREE_BTDM melepas memori Bluetooth begitu provisioning selesai.
  WiFiProv.beginProvision(NETWORK_PROV_SCHEME_BLE,
                          NETWORK_PROV_SCHEME_HANDLER_FREE_BTDM,
                          NETWORK_PROV_SECURITY_1,
                          provPop, namaBLE, NULL, NULL, true);

  // Tombol dan speaker tetap dilayani selama menunggu: alarm lokal tidak
  // boleh mati hanya karena WiFi belum diatur.
  const char *statusTerakhir = nullptr;
  unsigned long mulai = millis();
  while (!provSukses && millis() - mulai < PROV_TIMEOUT) {
    layaniTombolDanAlarm();
    if (setupStatus != statusTerakhir) {
      statusTerakhir = setupStatus;
      updateOLED();
    }
    delay(5);
  }

  if (provSukses) {
    homeSpan.setWifiCredentials(provSsid, provPwd);
    Serial.printf("[PROV] Tersimpan untuk \"%s\". Restart...\n", provSsid);
  } else {
    setupStatus = "Waktu habis";
    Serial.println("[PROV] Waktu habis, tidak ada app yang menghubungi. Restart...");
  }
  updateOLED();

  // Beri waktu app menerima status berhasil, sambil tetap melayani tombol.
  unsigned long tunggu = millis();
  while (millis() - tunggu < 5000) {
    layaniTombolDanAlarm();
    delay(5);
  }
  speakerTone(0);
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

void setupHomeKit() {
  homeSpan.setPairingCode(hkCode);          // diacak per board, lihat muatAtauBuatKodeHk()
  homeSpan.setQRID(HK_SETUP_ID);
  homeSpan.setApFunction(bleProvisioning);  // WiFi lewat BLE, bukan Access Point
  homeSpan.enableAutoStartAP();             // jalankan saat kredensial WiFi belum ada
  homeSpan.setLogLevel(0);

  homeSpan.begin(Category::Switches, HK_NAME, HK_HOSTNAME, HK_MODEL);

  new SpanUserCommand('Q', "- cetak ulang stiker perangkat", cmdTampilStiker);
  new SpanUserCommand('R', "- buat PoP baru (QR lama tidak berlaku)", cmdPopBaru);
  new SpanUserCommand('C', "- buat kode HomeKit baru (perlu pair ulang)", cmdKodeHkBaru);

  new SpanAccessory();
  new Service::AccessoryInformation();
    new Characteristic::Name(HK_NAME);
    new Characteristic::Manufacturer("A09 Final");
    new Characteristic::SerialNumber(namaBLE);
    new Characteristic::Model(HK_MODEL);
    new Characteristic::FirmwareRevision("1.0.0");
    new Characteristic::Identify();
  new AlarmSwitch();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\n=== Alarm (Tombol + Speaker + OLED) + HomeKit ===");

  pinMode(BUTTON_PIN, INPUT_PULLUP);   // pull-up internal, tanpa resistor
  speakerBegin();
  u8g2.begin();

  // Bunyi pendek sebagai tanda speaker tersambung
  speakerTone(TONE_HIGH);
  delay(120);
  speakerTone(0);

  buttonState = lastReading = readPressed();

  // Identitas provisioning disiapkan sebelum HomeSpan, karena setApFunction()
  // bisa langsung dipanggil saat kredensial WiFi belum ada.
  susunNamaBLE();
  muatAtauBuatPop(false);
  muatAtauBuatKodeHk(false);

  setupHomeKit();
  cetakStiker();

  // Tombol terkunci yang sudah dalam posisi ditekan saat menyala langsung memicu alarm
  if (BUTTON_LATCHING && buttonState) startAlarm();
  updateOLED();
}

void loop() {
  homeSpan.poll();
  layaniTombolDanAlarm();

  // Laporan rutin tiap 1 detik, supaya tetap terlihat walau Serial Monitor dibuka terlambat
  if (millis() - lastReportMs >= 1000) {
    lastReportMs = millis();
    Serial.printf("[%7.1f s] Tombol: %s | Alarm: %s | Jumlah tekan: %lu\n",
                  millis() / 1000.0,
                  buttonState ? "DITEKAN" : "DILEPAS",
                  alarmOn ? "NYALA" : "MATI",
                  (unsigned long)pressCount);
  }
}
