// Tombol + Speaker + OLED di ESP32-C3 OLED (0.42"), terhubung ke Apple HomeKit lewat HomeSpan
// - Alarm HANYA bisa nyala dari sinyal board Sensor (ESP32-S3). Tombol dan
//   Home app hanya bisa mematikan alarm, tidak bisa menyalakan.
// - OLED menampilkan status alarm dan status tombol.
// - HomeKit: satu accessory Switch "Alarm". Ikut menyala saat Sensor memicu
//   alarm; mematikannya dari Home app mematikan alarm. Permintaan nyala dari
//   Home app ditolak.
// - Board Sensor: saat fuzzy mencapai level Alarm, Sensor menyiarkan sinyal UDP
//   di WiFi yang sama dan alarm langsung berbunyi, tanpa home hub. Dimatikan
//   lewat tombol / Home app; tidak berbunyi lagi sampai ada kejadian bahaya baru.
//   Uji dengan @A di Serial Monitor Sensor.
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
//   Selama mode setup belum ada WiFi, jadi alarm belum bisa dipicu Sensor.
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
#define SPEAKER_CH 5          // kanal LEDC paling atas di ESP32-C3 (0–5), jauh dari kanal HomeSpan
bool speakerAktif = false;    // true = pin sedang dipegang LEDC

// Diam = pin dilepas dari LEDC dan tidak di-drive (high-Z),
// sama seperti saat ledcAttach dikomen
void speakerLepas() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (speakerAktif) ledcDetach(SPEAKER_PIN);
#else
  if (speakerAktif) ledcDetachPin(SPEAKER_PIN);
#endif
  speakerAktif = false;
  pinMode(SPEAKER_PIN, INPUT);
}

void speakerBegin() { speakerLepas(); }

// hz = 0 -> diam, selain itu LEDC dipasang dan nada hz dibunyikan
void speakerTone(uint32_t hz) {
  if (hz == 0) { speakerLepas(); return; }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!speakerAktif) speakerAktif = ledcAttachChannel(SPEAKER_PIN, hz, 10, SPEAKER_CH);
  if (speakerAktif) ledcWriteTone(SPEAKER_PIN, hz);
#else
  if (!speakerAktif) {
    ledcSetup(SPEAKER_CH, hz, 10);
    ledcAttachPin(SPEAKER_PIN, SPEAKER_CH);
    speakerAktif = true;
  }
  ledcWriteTone(SPEAKER_CH, hz);
#endif
}

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

// Sinyal bahaya dari board Sensor (lihat Bersama/SetupPerangkat.h).
WiFiUDP udpBahaya;
bool udpSiap = false;
bool bahayaSensor = false;      // Sensor sedang menyiarkan bahaya
bool dibungkam = false;         // alarm dimatikan orang selama bahaya ini berlangsung
unsigned long bahayaTerakhirMs = 0;

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
// Hanya dipanggil dari layaniSinyalSensor(): alarm cuma bisa nyala dari Sensor.
void startAlarm() {
  if (alarmOn) return;
  alarmOn = true;
  tonePhase = false;
  lastToneMs = millis();
  speakerTone(TONE_HIGH);
  Serial.printf("[%7.1f s] >>> ALARM NYALA\n", millis() / 1000.0);
  if (hkAlarm && !modeSetup) hkAlarm->setVal(true);
  updateOLED();
}

void stopAlarm(bool kabarkanHk = true) {
  if (!alarmOn) return;
  alarmOn = false;
  // Sensor terus menyiarkan selama bahaya; tanpa ini alarm langsung berbunyi
  // lagi sedetik setelah dimatikan.
  if (bahayaSensor) dibungkam = true;
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

  // Tombol hanya mematikan alarm; menyalakan hanya lewat sinyal Sensor.
  if (pressed) {
    pressCount++;
    stopAlarm();
  }
  updateOLED();
}

// Debounce tombol + nada alarm. Dipanggil dari loop() dan juga dari loop
// tunggu provisioning, supaya tombol tetap terbaca selama setup WiFi.
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

// ---------- Sinyal dari board Sensor ----------
// Pesan pertama dari suatu kejadian bahaya membunyikan alarm. Kalau orang
// mematikannya, pesan berikutnya diabaikan sampai Sensor berhenti menyiarkan
// (bahaya selesai), supaya kejadian bahaya berikutnya tetap membunyikan alarm.
void layaniSinyalSensor() {
  if (!udpSiap) {
    if (WiFi.status() == WL_CONNECTED) udpSiap = udpBahaya.begin(PORT_BAHAYA);
    return;
  }

  while (udpBahaya.parsePacket()) {
    char buf[16] = {0};
    udpBahaya.read(buf, sizeof(buf) - 1);
    if (strcmp(buf, PESAN_BAHAYA) != 0) continue;

    bahayaTerakhirMs = millis();
    if (!bahayaSensor) {
      bahayaSensor = true;
      dibungkam = false;
      Serial.printf("[%7.1f s] Sensor: BAHAYA dari %s\n", millis() / 1000.0,
                    udpBahaya.remoteIP().toString().c_str());
    }
    if (!dibungkam) startAlarm();
  }

  if (bahayaSensor && millis() - bahayaTerakhirMs > BAHAYA_HILANG_MS) {
    bahayaSensor = false;
    dibungkam = false;
    Serial.printf("[%7.1f s] Sensor: bahaya selesai\n", millis() / 1000.0);
  }
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
      // Alarm hanya boleh dinyalakan sinyal Sensor. false = ditolak, Home app
      // mengembalikan switch ke posisi mati.
      Serial.printf("[%7.1f s] HomeKit meminta alarm NYALA -> ditolak\n", millis() / 1000.0);
      return false;
    }
    Serial.printf("[%7.1f s] HomeKit meminta alarm MATI\n", millis() / 1000.0);
    stopAlarm(false);
    return true;
  }
};

// =====================================================
// Provisioning WiFi lewat BLE + identitas HomeKit
// Isinya ada di Bersama/SetupPerangkat.h, dipakai bersama board Sensor.
// Di sini hanya bagian yang khas Alarm: tampilan OLED dan tombol yang
// tetap dilayani selama setup WiFi.
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
  layaniTombolDanAlarm,    // tombol tetap dilayani selama setup WiFi
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
  updateOLED();
}

void loop() {
  homeSpan.poll();
  layaniTombolDanAlarm();
  layaniSinyalSensor();

  // Laporan rutin tiap 1 detik, supaya tetap terlihat walau Serial Monitor dibuka terlambat.
  // Ditahan selama WiFi belum tersambung, supaya pesan "Trying to connect" HomeSpan
  // tidak tenggelam.
  if (WiFi.status() == WL_CONNECTED && millis() - lastReportMs >= 1000) {
    lastReportMs = millis();
    Serial.printf("[%7.1f s] Tombol: %s | Alarm: %s | Jumlah tekan: %lu\n",
                  millis() / 1000.0,
                  buttonState ? "DITEKAN" : "DILEPAS",
                  alarmOn ? "NYALA" : "MATI",
                  (unsigned long)pressCount);
  }
}
