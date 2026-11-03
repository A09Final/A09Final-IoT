// Tombol + Speaker + OLED di ESP32-C3 OLED (0.42")
// - Tekan tombol -> alarm berbunyi (dua nada bergantian), tekan lagi -> alarm mati.
// - OLED menampilkan status alarm dan status tombol.
// - WiFi diatur lewat Bluetooth dari app iOS DapurSetup (SDK ESPProvision),
//   sama seperti board Sensor. Kredensial disimpan di NVS lalu dipakai
//   tersambung otomatis setiap boot.
//
// Sambungan:
//   Tombol (pasangan kontak NO) -> GPIO 3 dan GND (bebas mana yang mana)
//   Grove Speaker: SIG -> GPIO 4, VCC -> 5V, GND -> GND, NC tidak disambung
//   OLED onboard di GPIO 5 (SDA) dan GPIO 6 (SCL)
//
// Library (Library Manager): U8g2
//
// Arduino IDE:
//   Board            : "ESP32C3 Dev Module"
//   USB CDC On Boot  : Enabled
//   Partition Scheme : "Huge APP (3MB No OTA/1MB SPIFFS)" --- stack WiFi +
//                      Bluetooth tidak muat di partisi default
//   Serial Monitor   : 115200 baud
//
// Keamanan pemasangan:
//   PoP (PIN provisioning) diacak saat boot pertama lalu disimpan di NVS,
//   jadi tiap board punya miliknya sendiri.
//   Saat mode setup, PoP juga tampil di OLED -- hanya orang yang memegang
//   perangkat yang bisa membacanya, sama seperti stiker.
//
// Cara pakai pertama kali:
//   1. Upload, buka Serial Monitor 115200.
//   2. Board mencetak blok stiker berisi QR provisioning.
//      OLED menampilkan "SETUP WIFI", nama BLE, dan PIN.
//   3. Di app DapurSetup: pilih DAPUR_ALM_xxxxxx (atau pindai QR), isi PIN,
//      pilih WiFi, isi kata sandi.
//   4. Board menyala ulang lalu tersambung ke WiFi tersebut.
//   Selama mode setup, tombol dan speaker tetap berfungsi sebagai alarm lokal.
//
// Perintah Serial Monitor (ketik lalu Enter):
//   @Q  cetak ulang stiker perangkat
//   @R  buat PoP baru (QR lama jadi tidak berlaku)
//   A   provisioning ulang lewat BLE
//   X   hapus kredensial WiFi lalu restart
//   ?   bantuan

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
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

#define DEVICE_NAME      "Alarm-A09"
#define WIFI_HOSTNAME    "a09-alarm"

// ---------- Provisioning WiFi lewat BLE ----------
// Protokol provisioning Espressif, berpasangan dengan app iOS DapurSetup.
// App hanya menampilkan perangkat berawalan "DAPUR_", jadi alarm memakai
// "DAPUR_ALM_" supaya tetap muncul tapi bisa dibedakan dari board Sensor.
#define PROV_PREFIX   "DAPUR_ALM_"  // nama BLE jadi DAPUR_ALM_xxxxxx (6 digit MAC)
#define PROV_NVS      "a09prov"     // ruang NVS tempat PoP disimpan
#define WIFI_NVS      "a09wifi"     // ruang NVS tempat SSID dan kata sandi WiFi disimpan
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

bool wifiTerakhir = false;       // untuk mencatat saat WiFi tersambung / putus

static volatile bool provSukses = false;
static char provSsid[33];
static char provPwd[65];
static char provPop[9]  = "";      // 8 karakter, unik per board
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
void startAlarm() {
  if (alarmOn) return;
  alarmOn = true;
  tonePhase = false;
  lastToneMs = millis();
  speakerTone(TONE_HIGH);
  Serial.printf("[%7.1f s] >>> ALARM NYALA\n", millis() / 1000.0);
  updateOLED();
}

void stopAlarm() {
  if (!alarmOn) return;
  alarmOn = false;
  speakerTone(0);
  Serial.printf("[%7.1f s] >>> ALARM MATI\n", millis() / 1000.0);
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
// Provisioning WiFi lewat BLE
// Jalan saat kredensial WiFi belum ada di NVS, atau saat kamu ketik 'A' di
// Serial Monitor. Setelah selesai (berhasil atau waktu habis) board restart.
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

// Blok siap cetak untuk ditempel di badan perangkat.
void cetakStiker() {
  // Format JSON ini yang dibaca scanQRCode() di SDK ESPProvision.
  // "security":1 wajib: tanpa itu SDK iOS menganggap Security 2 dan gagal.
  char isi[128];
  snprintf(isi, sizeof(isi),
           "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\","
           "\"transport\":\"ble\",\"security\":1}",
           namaBLE, provPop);

  Serial.println("\n======== STIKER PERANGKAT (cetak & tempel) ========");
  Serial.printf("  Nama         : %s\n", DEVICE_NAME);
  Serial.printf("  Serial       : %s\n", namaBLE);
  Serial.println();

#ifdef ADA_QRCODE
  esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
  cfg.display_func = gambarQR;
  esp_qrcode_generate(&cfg, isi);
#endif

  Serial.println("===================================================");
  Serial.printf("  PoP          : %s\n", provPop);
  Serial.printf("  Isi QR       : %s\n", isi);
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

// ---------- Kredensial WiFi di NVS ----------
void simpanWifi(const char *ssid, const char *pwd) {
  Preferences pref;
  pref.begin(WIFI_NVS, false);
  pref.putString("ssid", ssid);
  pref.putString("pwd", pwd);
  pref.end();
}

bool muatWifi(String &ssid, String &pwd) {
  Preferences pref;
  pref.begin(WIFI_NVS, true);
  ssid = pref.getString("ssid", "");
  pwd  = pref.getString("pwd", "");
  pref.end();
  return ssid.length() > 0;
}

void hapusWifi() {
  Preferences pref;
  pref.begin(WIFI_NVS, false);
  pref.clear();
  pref.end();
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
    simpanWifi(provSsid, provPwd);
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

// ---------- WiFi ----------
// Tidak menunggu tersambung: driver WiFi menyambung (dan menyambung ulang)
// sendiri di latar belakang, jadi tombol dan alarm tidak pernah tertahan.
void mulaiWifi(const String &ssid, const String &pwd) {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid.c_str(), pwd.c_str());
  Serial.printf("[WIFI] Menyambung ke \"%s\"...\n", ssid.c_str());
}

void pantauWifi() {
  bool sekarang = WiFi.isConnected();
  if (sekarang == wifiTerakhir) return;
  wifiTerakhir = sekarang;
  if (sekarang)
    Serial.printf("[WIFI] Tersambung, IP %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("[WIFI] Terputus, mencoba menyambung ulang...");
}

// ---------- Perintah Serial Monitor ----------
void cetakBantuan() {
  Serial.println("\nPerintah:");
  Serial.println("  @Q  cetak ulang stiker perangkat");
  Serial.println("  @R  buat PoP baru (QR lama tidak berlaku)");
  Serial.println("  A   provisioning ulang lewat BLE");
  Serial.println("  X   hapus kredensial WiFi lalu restart\n");
}

void jalankanPerintah(const String &cmd) {
  if (cmd.equalsIgnoreCase("@Q")) {
    cetakStiker();
  } else if (cmd.equalsIgnoreCase("@R")) {
    muatAtauBuatPop(true);
    Serial.println("[PROV] QR lama tidak berlaku lagi. Cetak ulang stiker di bawah.");
    cetakStiker();
  } else if (cmd.equalsIgnoreCase("A")) {
    WiFi.disconnect(true);
    bleProvisioning();               // selalu diakhiri restart
  } else if (cmd.equalsIgnoreCase("X")) {
    hapusWifi();
    Serial.println("[WIFI] Kredensial dihapus. Restart...");
    delay(500);
    ESP.restart();
  } else if (cmd == "?") {
    cetakBantuan();
  } else if (cmd.length()) {
    Serial.printf("Perintah \"%s\" tidak dikenal, ketik ? untuk bantuan.\n", cmd.c_str());
  }
}

// Dibaca per karakter supaya loop() tidak pernah menunggu Serial.
void bacaSerial() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf.trim();
      jalankanPerintah(buf);
      buf = "";
    } else if (buf.length() < 16) {
      buf += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\n=== Alarm (Tombol + Speaker + OLED) ===");

  pinMode(BUTTON_PIN, INPUT_PULLUP);   // pull-up internal, tanpa resistor
  speakerBegin();
  u8g2.begin();

  // Bunyi pendek sebagai tanda speaker tersambung
  speakerTone(TONE_HIGH);
  delay(120);
  speakerTone(0);

  buttonState = lastReading = readPressed();

  susunNamaBLE();
  muatAtauBuatPop(false);
  cetakStiker();

  // Belum ada WiFi tersimpan -> langsung masuk mode setup (diakhiri restart).
  String ssid, pwd;
  if (!muatWifi(ssid, pwd)) bleProvisioning();
  mulaiWifi(ssid, pwd);

  // Tombol terkunci yang sudah dalam posisi ditekan saat menyala langsung memicu alarm
  if (BUTTON_LATCHING && buttonState) startAlarm();
  updateOLED();
}

void loop() {
  layaniTombolDanAlarm();
  bacaSerial();
  pantauWifi();

  // Laporan rutin tiap 1 detik, supaya tetap terlihat walau Serial Monitor dibuka terlambat
  if (millis() - lastReportMs >= 1000) {
    lastReportMs = millis();
    Serial.printf("[%7.1f s] Tombol: %s | Alarm: %s | Jumlah tekan: %lu | WiFi: %s\n",
                  millis() / 1000.0,
                  buttonState ? "DITEKAN" : "DILEPAS",
                  alarmOn ? "NYALA" : "MATI",
                  (unsigned long)pressCount,
                  WiFi.isConnected() ? "TERSAMBUNG" : "PUTUS");
  }
}
