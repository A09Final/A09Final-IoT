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
// Perintah Serial Monitor (dari Bersama/SetupPerangkat.h, diawali @):
//   @Q  cetak ulang stiker perangkat
//   @R  buat PoP baru (QR lama jadi tidak berlaku)
//   @C  buat kode HomeKit baru (perangkat perlu di-pair ulang)
//   Bawaan HomeSpan: ? bantuan | A provisioning ulang | X hapus WiFi | U hapus pairing

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "../Bersama/SetupPerangkat.h"   // provisioning BLE + HomeKit, sama dengan Sensor

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
// Nama BLE memakai awalan "DAPUR_ALM_" (lihat PERANGKAT di bawah): app DapurSetup
// hanya menampilkan perangkat berawalan "DAPUR_", jadi alarm tetap muncul tapi
// bisa dibedakan dari board Sensor.

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
// Provisioning WiFi lewat BLE + identitas HomeKit
// Isinya ada di Bersama/SetupPerangkat.h, dipakai bersama board Sensor.
// Di sini hanya bagian yang khas Alarm: tampilan OLED dan alarm lokal yang
// tetap jalan selama setup WiFi.
// =====================================================
void saatTahapProv(TahapProv tahap, const char *teks) {
  modeSetup = true;
  setupStatus = teks;
  updateOLED();
  if (tahap == PROV_SELESAI) speakerTone(0);   // jangan restart sambil berbunyi
}

const InfoPerangkat PERANGKAT = {
  HK_NAME, HK_HOSTNAME, HK_MODEL, HK_SETUP_ID, Category::Switches,
  "DAPUR_ALM_",            // nama BLE jadi DAPUR_ALM_xxxxxx (6 digit MAC)
  saatTahapProv,
  layaniTombolDanAlarm,    // tombol dan speaker tetap dilayani selama setup WiFi
};

void setupHomeKit() {
  mulaiHomeSpan(PERANGKAT);

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

  setupHomeKit();   // juga mencetak stiker perangkat

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
