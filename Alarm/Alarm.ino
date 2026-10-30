// Tombol + Speaker + OLED di ESP32-C3 OLED (0.42")
// Tekan tombol -> alarm berbunyi (dua nada bergantian), tekan lagi -> alarm mati.
// OLED menampilkan status alarm dan status tombol.
// WiFi tidak diatur di board ini: diterima dari board Sensor lewat ESP-NOW.
//
// Mendapatkan WiFi:
//   - Board yang belum punya WiFi otomatis masuk mode pairing (OLED: "Pairing").
//   - Di Sensor, jendela pairing terbuka 2 menit setelah provisioning, atau
//     ketik @P di Serial Monitor Sensor.
//   - Untuk ganti WiFi: tahan tombol 5 detik (atau ketik P di Serial Monitor
//     Alarm). WiFi lama dihapus, board menyala ulang ke mode pairing.
//
// Sambungan:
//   Tombol (pasangan kontak NO) -> GPIO 3 dan GND (bebas mana yang mana)
//   Grove Speaker: SIG -> GPIO 4, VCC -> 5V, GND -> GND, NC tidak disambung
//   OLED onboard di GPIO 5 (SDA) dan GPIO 6 (SCL)
//
// Library: U8g2 (Library Manager)
// Arduino IDE: Board "ESP32C3 Dev Module", USB CDC On Boot: Enabled
// Serial Monitor: 115200 baud

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>
#include "BagiWifi.h"

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

// ---------- WiFi dari Sensor ----------
#define WIFI_NVS  "a09wifi"                    // ruang NVS untuk SSID dan kata sandi
const unsigned long LUPA_TAHAN_MS  = 5000;     // tahan tombol selama ini -> minta WiFi ulang
const unsigned long PAIR_DIAM_MS   = 1000;     // lama menunggu jawaban di tiap channel
const unsigned long PAIR_ULANG_MS  = 300;      // permintaan diulang dalam jeda ini
const uint8_t       PAIR_CH_MAKS   = 13;       // channel WiFi 2,4 GHz di Indonesia: 1..13

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
unsigned long pressStartMs = 0;   // kapan tombol mulai ditahan, 0 = tidak ditahan

// Pairing: diisi callback ESP-NOW (task WiFi), dibaca loop(). Callback tidak
// menimpa selama adaJawaban masih true, jadi loop() selalu membaca paket utuh.
bool pairing = false;
uint8_t pairChannel = 1;
unsigned long pairChannelMs = 0;
unsigned long pairKirimMs = 0;
volatile bool adaJawaban = false;
BwJawab jawaban;

mbedtls_ecp_group pairGrp;
mbedtls_mpi       pairD;
BwMinta           minta;

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

void updateOLED() {
  u8g2.clearBuffer();

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

  // Baris bawah: status tombol, atau channel yang sedang dicoba saat pairing
  u8g2.setFont(u8g2_font_6x10_tf);
  if (pairing) {
    char ch[8];
    snprintf(ch, sizeof(ch), "ch %u", pairChannel);
    drawCentered("Pairing", 29);
    drawCentered(ch, 40);
  } else {
    drawCentered("Tombol:", 29);
    drawCentered(buttonState ? "DITEKAN" : "DILEPAS", 40);
  }

  u8g2.sendBuffer();
}

// ---------- Alarm ----------
void startAlarm() {
  alarmOn = true;
  tonePhase = false;
  lastToneMs = millis();
  speakerTone(TONE_HIGH);
  Serial.printf("[%7.1f s] >>> ALARM NYALA\n", millis() / 1000.0);
  updateOLED();
}

void stopAlarm() {
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

// ---------- WiFi dari Sensor lewat ESP-NOW ----------
BW_CB_TERIMA(terimaJawaban) {
  if (!pairing || adaJawaban) return;
  if (!bwKepalaSah(isi, len, BW_JAWAB, sizeof(BwJawab))) return;
  memcpy(&jawaban, isi, sizeof(jawaban));
  adaJawaban = true;
}

void kirimMinta() {
  static const uint8_t SIARAN[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(SIARAN, (const uint8_t *)&minta, sizeof(minta));
  pairKirimMs = millis();
}

void pindahChannel(uint8_t ch) {
  pairChannel = ch;
  pairChannelMs = millis();
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  kirimMinta();
  updateOLED();
}

void mulaiPairing() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();          // STA tidak boleh mencoba tersambung, supaya channel bisa dipindah

  mbedtls_ecp_group_init(&pairGrp);
  mbedtls_mpi_init(&pairD);
  minta.magic = BW_MAGIC;
  minta.versi = BW_VERSI;
  minta.jenis = BW_MINTA;
  if (!bwBuatKunci(&pairGrp, &pairD, minta.pub)) {
    Serial.println("[WIFI] Gagal membuat kunci pairing.");
    return;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[WIFI] ESP-NOW gagal dimulai.");
    return;
  }
  esp_now_register_recv_cb(terimaJawaban);

  // Peer siaran, channel 0 = ikut channel radio saat ini.
  esp_now_peer_info_t peer = {};
  memset(peer.peer_addr, 0xFF, 6);
  peer.channel = 0;
  peer.ifidx   = WIFI_IF_STA;
  esp_now_add_peer(&peer);

  pairing = true;
  Serial.println("[WIFI] Belum ada WiFi. Mencari Sensor (buka pairing dengan @P di Sensor)...");
  pindahChannel(1);
}

// Jawaban yang tidak bisa dibuka diabaikan saja: bisa jadi paket rusak, atau
// board lain yang sedang pairing dengan kunci berbeda.
void prosesJawaban() {
  uint8_t kunci[16];
  BwKredensial kred = {};

  bool ok = bwTurunkanKunci(&pairGrp, &pairD, jawaban.pub, kunci) &&
            bwGcm(false, kunci, jawaban.nonce, (const uint8_t *)&jawaban, BW_KEPALA_LEN,
                  jawaban.sandi, (uint8_t *)&kred, sizeof(kred), jawaban.tag);
  memset(kunci, 0, sizeof(kunci));
  kred.ssid[sizeof(kred.ssid) - 1] = '\0';
  kred.pwd[sizeof(kred.pwd) - 1]   = '\0';

  if (!ok || kred.ssid[0] == '\0') {
    Serial.println("[WIFI] Jawaban tidak bisa dibuka, diabaikan.");
    memset(&kred, 0, sizeof(kred));
    adaJawaban = false;
    return;
  }

  Preferences pref;
  pref.begin(WIFI_NVS, false);
  pref.putString("ssid", kred.ssid);
  pref.putString("pwd", kred.pwd);
  pref.end();

  Serial.printf("[WIFI] Diterima dari Sensor: \"%s\". Restart...\n", kred.ssid);
  memset(&kred, 0, sizeof(kred));
  speakerTone(TONE_HIGH);     // bunyi pendek tanda berhasil
  delay(120);
  speakerTone(0);
  delay(500);
  ESP.restart();
}

void updatePairing() {
  if (!pairing) return;
  if (adaJawaban) prosesJawaban();

  if (millis() - pairChannelMs >= PAIR_DIAM_MS)
    pindahChannel(pairChannel % PAIR_CH_MAKS + 1);
  else if (millis() - pairKirimMs >= PAIR_ULANG_MS)
    kirimMinta();
}

// Tidak ada WiFi tersimpan -> pairing. Ada -> langsung tersambung.
void mulaiWifi() {
  WiFi.persistent(false);     // NVS kita sendiri satu-satunya tempat kredensial

  Preferences pref;
  pref.begin(WIFI_NVS, true);
  String ssid = pref.getString("ssid", "");
  String pwd  = pref.getString("pwd", "");
  pref.end();

  if (ssid.length() == 0) {
    mulaiPairing();
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pwd.c_str());
  Serial.printf("[WIFI] Menyambung ke \"%s\"...\n", ssid.c_str());
}

void lupakanWifi() {
  if (alarmOn) stopAlarm();
  Preferences pref;
  pref.begin(WIFI_NVS, false);
  pref.clear();
  pref.end();
  Serial.println("[WIFI] WiFi dihapus. Restart ke mode pairing...");
  delay(200);
  ESP.restart();
}

const char* statusWifi() {
  static char buf[24];
  if (pairing)
    snprintf(buf, sizeof(buf), "PAIRING ch %u", pairChannel);
  else if (WiFi.status() == WL_CONNECTED)
    snprintf(buf, sizeof(buf), "%s", WiFi.localIP().toString().c_str());
  else
    snprintf(buf, sizeof(buf), "terputus");
  return buf;
}

// ---------- Tombol ----------
void onButtonChange(bool pressed) {
  pressStartMs = pressed ? millis() : 0;   // 0 = tidak sedang ditahan
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

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);   // pull-up internal, tanpa resistor
  speakerBegin();
  u8g2.begin();

  // Bunyi pendek sebagai tanda speaker tersambung
  speakerTone(TONE_HIGH);
  delay(120);
  speakerTone(0);

  mulaiWifi();

  buttonState = lastReading = readPressed();
  // Tombol terkunci yang sudah dalam posisi ditekan saat menyala langsung memicu alarm
  if (BUTTON_LATCHING && buttonState) startAlarm();
  updateOLED();
}

void loop() {
  // Debounce tombol
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
  updatePairing();

  // Tahan tombol lama = minta WiFi ulang. Untuk tombol latching, tahan memang
  // posisi normalnya, jadi hanya lewat Serial Monitor.
  if (!BUTTON_LATCHING && pressStartMs && millis() - pressStartMs >= LUPA_TAHAN_MS)
    lupakanWifi();
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'P' || c == 'p') lupakanWifi();
  }

  // Laporan rutin tiap 1 detik, supaya tetap terlihat walau Serial Monitor dibuka terlambat
  if (millis() - lastReportMs >= 1000) {
    lastReportMs = millis();
    Serial.printf("[%7.1f s] Tombol: %s | Alarm: %s | Jumlah tekan: %lu | WiFi: %s\n",
                  millis() / 1000.0,
                  buttonState ? "DITEKAN" : "DILEPAS",
                  alarmOn ? "NYALA" : "MATI",
                  (unsigned long)pressCount,
                  statusWifi());
  }
}
