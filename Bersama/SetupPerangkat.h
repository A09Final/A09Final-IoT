// Setup perangkat yang sama untuk semua board A09: provisioning WiFi lewat BLE,
// HomeKit lewat HomeSpan, kode pairing per board, dan stiker perangkat.
//
// Dipakai oleh Sensor/Sensor.ino dan Alarm/Alarm.ino. Ubah di sini, maka kedua
// board ikut berubah -- jangan salin isi file ini ke dalam sketch.
//
// Cara pakai di sketch:
//   #include "../Bersama/SetupPerangkat.h"
//
//   const InfoPerangkat PERANGKAT = {
//     "Alarm-A09", "a09-alarm", "ESP32-C3 OLED Speaker", "A09A",
//     Category::Switches, "DAPUR_ALM_",
//     saatTahapProv,   // opsional, boleh nullptr
//     selamaProv,      // opsional, boleh nullptr
//   };
//
//   void setup() {
//     ...
//     mulaiHomeSpan(PERANGKAT);   // lalu tambahkan SpanUserCommand & accessory sendiri
//   }
//   void loop() { homeSpan.poll(); ... }
//
// Catatan:
//   - Include hanya dari file .ino, jangan dari .cpp lain: isinya berupa
//     definisi, bukan sekadar deklarasi.
//   - Path relatif "../Bersama/..." didukung Arduino IDE 2 dan arduino-cli.
//     File ini tidak muncul sebagai tab di IDE; buka dari folder Bersama.
//
// Perintah Serial Monitor yang dipasang di sini (diawali @):
//   @Q  cetak ulang stiker perangkat
//   @R  buat PoP baru (QR lama jadi tidak berlaku)
//   @C  buat kode HomeKit baru (perangkat perlu di-pair ulang)
//   Bawaan HomeSpan: ? bantuan | A provisioning ulang | X hapus WiFi | U hapus pairing

#pragma once

#include <Arduino.h>
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

#define PROV_NVS      "a09prov"     // ruang NVS tempat PoP dan kode HomeKit disimpan
const unsigned long PROV_TIMEOUT = 300000;   // 5 menit menunggu app

// Tahap provisioning, dikabarkan ke sketch supaya tiap board bisa menampilkan
// statusnya sendiri (LED di Sensor, OLED di Alarm).
enum TahapProv {
  PROV_MULAI,        // Bluetooth aktif, menunggu app
  PROV_MENYAMBUNG,   // kredensial diterima, mencoba tersambung ke WiFi
  PROV_GAGAL,        // gagal tersambung, app bisa mengirim ulang
  PROV_SUKSES,       // WiFi tersambung, kredensial disimpan
  PROV_WAKTU_HABIS,  // tidak ada app yang menghubungi
  PROV_SELESAI,      // sesaat sebelum restart
};

// Teks pendek untuk tiap tahap. Maksimal 14 karakter supaya muat di OLED 72x40.
const char *teksTahapProv(TahapProv t) {
  switch (t) {
    case PROV_MULAI:       return "Menunggu app";
    case PROV_MENYAMBUNG:  return "Menyambung..";
    case PROV_GAGAL:       return "Gagal, ulangi";
    case PROV_SUKSES:      return "Tersambung!";
    case PROV_WAKTU_HABIS: return "Waktu habis";
    case PROV_SELESAI:     return "Restart...";
  }
  return "";
}

struct InfoPerangkat {
  const char *nama;        // nama di Apple Home, juga tercetak di stiker
  const char *hostname;
  const char *model;
  const char *setupId;     // 4 karakter, Setup ID untuk payload QR HomeKit
  Category    kategori;
  // App DapurSetup hanya menampilkan perangkat berawalan "DAPUR_". Nama BLE
  // jadi <prefixBle> + 6 digit terakhir MAC, mis. DAPUR_ALM_A1B2C3.
  const char *prefixBle;

  // Dipanggil dari task utama setiap kali tahap provisioning berubah.
  void (*saatTahapProv)(TahapProv tahap, const char *teks);
  // Dipanggil berulang (tiap ~5 ms) selama menunggu app dan sebelum restart,
  // untuk board yang harus tetap bekerja selama setup WiFi.
  void (*selamaProv)();
};

static const InfoPerangkat *perangkat = nullptr;

static volatile bool      provSukses = false;
static volatile TahapProv tahapProv  = PROV_MULAI;
static char provSsid[33];
static char provPwd[65];

// Boleh dibaca sketch, mis. untuk ditampilkan di layar.
static char provPop[9]  = "";      // 8 karakter, unik per board
static char hkCode[9]   = "";      // 8 angka kode pairing HomeKit, unik per board
static char namaBLE[20] = "";      // <prefixBle>xxxxxx

void susunNamaBLE() {
  snprintf(namaBLE, sizeof(namaBLE), "%s%06X", perangkat->prefixBle,
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
  const char *payloadHk = qr.get(strtoul(hkCode, NULL, 10), perangkat->setupId,
                                 (uint8_t)perangkat->kategori);

  Serial.println("\n======== STIKER PERANGKAT (cetak & tempel) ========");
  Serial.printf("  Nama         : %s\n", perangkat->nama);
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

// =====================================================
// Provisioning WiFi lewat BLE
// Dipasang ke HomeSpan lewat setApFunction(), jadi fungsi ini jalan saat
// kredensial WiFi belum ada, atau saat kamu ketik 'A' di Serial Monitor.
// =====================================================

// Dipanggil dari task FreeRTOS lain, bukan dari loop(). Hanya mengisi variabel;
// sketch dikabari dari loop tunggu di bleProvisioning().
void provEvent(arduino_event_t *e) {
  switch (e->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.println("[PROV] Bluetooth aktif, menunggu app...");
      tahapProv = PROV_MULAI;
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
      memcpy(provSsid, e->event_info.prov_cred_recv.ssid, 32);      provSsid[32] = 0;
      memcpy(provPwd,  e->event_info.prov_cred_recv.password, 64);  provPwd[64]  = 0;
      Serial.printf("[PROV] WiFi diterima: %s, mencoba tersambung...\n", provSsid);
      tahapProv = PROV_MENYAMBUNG;
      break;
    case ARDUINO_EVENT_PROV_CRED_FAIL:
      Serial.println("[PROV] Gagal tersambung. Periksa kata sandi, kirim ulang dari app.");
      tahapProv = PROV_GAGAL;
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("[PROV] WiFi tersambung");
      tahapProv = PROV_SUKSES;
      provSukses = true;
      break;
    default:
      break;
  }
}

static void kabarkanTahapProv(TahapProv t) {
  if (perangkat->saatTahapProv) perangkat->saatTahapProv(t, teksTahapProv(t));
}

// Menunggu sambil tetap memberi giliran ke sketch lewat selamaProv().
static void tungguProv(unsigned long lama, bool berhentiSaatSukses) {
  unsigned long mulai = millis();
  while (millis() - mulai < lama) {
    if (berhentiSaatSukses && provSukses) return;
    if (perangkat->selamaProv) perangkat->selamaProv();
    delay(5);
  }
}

void bleProvisioning() {
  Serial.println("\n[PROV] Kredensial WiFi belum ada. Mulai provisioning lewat BLE.");
  tahapProv = PROV_MULAI;
  kabarkanTahapProv(PROV_MULAI);
  cetakStiker();

  WiFi.onEvent(provEvent);

  // FREE_BTDM melepas memori Bluetooth begitu provisioning selesai.
  WiFiProv.beginProvision(NETWORK_PROV_SCHEME_BLE,
                          NETWORK_PROV_SCHEME_HANDLER_FREE_BTDM,
                          NETWORK_PROV_SECURITY_1,
                          provPop, namaBLE, NULL, NULL, true);

  // Tahap diubah oleh provEvent() di task lain; sketch dikabari dari sini.
  TahapProv terakhir = PROV_MULAI;
  unsigned long mulai = millis();
  while (!provSukses && millis() - mulai < PROV_TIMEOUT) {
    tungguProv(5, true);
    TahapProv sekarang = tahapProv;
    if (sekarang != terakhir) {
      terakhir = sekarang;
      if (sekarang != PROV_SUKSES) kabarkanTahapProv(sekarang);
    }
  }

  if (provSukses) {
    homeSpan.setWifiCredentials(provSsid, provPwd);
    kabarkanTahapProv(PROV_SUKSES);
    Serial.printf("[PROV] Tersimpan untuk \"%s\". Restart...\n", provSsid);
  } else {
    kabarkanTahapProv(PROV_WAKTU_HABIS);
    Serial.println("[PROV] Waktu habis, tidak ada app yang menghubungi. Restart...");
  }

  tungguProv(5000, false);           // beri waktu app menerima status berhasil
  kabarkanTahapProv(PROV_SELESAI);
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
// Menyiapkan identitas board, HomeSpan, dan perintah @Q/@R/@C, lalu mencetak
// stiker. Setelah ini sketch menambahkan SpanUserCommand dan accessory sendiri.
// `info` harus tetap hidup selama program berjalan (variabel global).
// =====================================================
void mulaiHomeSpan(const InfoPerangkat &info) {
  perangkat = &info;

  // Identitas provisioning disiapkan sebelum HomeSpan, karena setApFunction()
  // bisa langsung dipanggil saat kredensial WiFi belum ada.
  susunNamaBLE();
  muatAtauBuatPop(false);
  muatAtauBuatKodeHk(false);

  homeSpan.setPairingCode(hkCode);          // diacak per board, lihat muatAtauBuatKodeHk()
  homeSpan.setQRID(info.setupId);
  homeSpan.setApFunction(bleProvisioning);  // WiFi lewat BLE, bukan Access Point
  homeSpan.enableAutoStartAP();             // jalankan saat kredensial WiFi belum ada
  homeSpan.setLogLevel(0);

  homeSpan.begin(info.kategori, info.nama, info.hostname, info.model);

  new SpanUserCommand('Q', "- cetak ulang stiker perangkat", cmdTampilStiker);
  new SpanUserCommand('R', "- buat PoP baru (QR lama tidak berlaku)", cmdPopBaru);
  new SpanUserCommand('C', "- buat kode HomeKit baru (perlu pair ulang)", cmdKodeHkBaru);

  cetakStiker();
}
