// Mengirim data sensor ke backend BeforeFire lewat MQTT, supaya app bisa
// menampilkannya di layar Monitor.
//
// Kontraknya ada di README BeforeFire-Backend, bagian "Kontrak MQTT" dan
// "Kontrak untuk firmware". Ringkasnya:
//   - Login ke broker: username = ID alat (DAPUR_xxxxxx), password = PoP.
//   - beforefire/<id>/status    "online" saat tersambung; "offline" dikirim
//                               broker sebagai MQTT Will kalau alat hilang.
//   - beforefire/<id>/telemetry data sensor terakhir (JSON, retained).
//
// Hasil fuzzy ikut dikirim sebagai "condition" dan "score", supaya app bisa
// menampilkan Safe / Unsafe / Dangerous.
//
// Yang belum ada di sini: alert (butuh push token) dan perintah dari app
// (topic cmd/ack).
//
// Cara menyambungkan board ke backend:
//   1. Di Mac:  scripts/mqtt-user.sh add <Serial> <PoP>   (keduanya ada di stiker)
//   2. Selesai. Board memakai alamat bawaan BACKEND_MQTT_URI (nama .local Mac
//      server) begitu WiFi tersambung.
//   Alamat lain bisa diatur di Serial Monitor:  @M mqtt://<IP Mac>:1883
//   Alamat itu disimpan di NVS dan menggantikan alamat bawaan. "@M" saja
//   menampilkan status, "@M off" mematikan pengiriman.
//   Kalau PoP diganti dengan @R, daftarkan ulang board di langkah 1 memakai
//   PoP baru; board otomatis login ulang dengan PoP itu.

#pragma once

#include <Arduino.h>
#include "FuzzyKompor.h"

// Satu paket data sensor. Nilai yang penandanya false tidak ikut dikirim,
// sehingga app menampilkannya sebagai "tidak tersedia".
struct DataTelemetri {
  bool    adaKompor  = false;
  float   komporC    = 0;     // MLX90640, rata-rata piksel terpanas

  bool    adaBme     = false;
  float   suhuC      = 0;     // BME688, sudah dikompensasi panas heater
  float   lembapRH   = 0;
  float   tekananHPa = 0;
  float   iaq        = 0;     // Static IAQ, 0..500
  uint8_t iaqAkurasi = 0;     // 0 belum stabil .. 3 terkalibrasi
  float   eco2Ppm    = 0;

  bool    adaOrang   = false;
  bool    orang      = false; // LD2410B

  // Hasil evaluateFuzzy(). Tidak dikirim kalau adaKondisi false, dan app
  // menampilkannya sebagai "Unknown".
  bool      adaKondisi = false;
  Condition kondisi    = Condition::Normal;
  float     skor       = NAN;

  // Sensor yang sedang bermasalah, dikirim di "unavailableSensors".
  bool    bmeRusak    = false;
  bool    radarRusak  = false;
  bool    termalRusak = false;
};

// Dipanggil sekali di setup(), setelah mulaiHomeSpan(). Kedua pointer harus
// tetap hidup selama program berjalan. `pop` boleh berubah isinya (perintah @R);
// perubahan itu terdeteksi sendiri.
void backendMulai(const char *deviceId, const char *pop);

// Dipanggil di setiap putaran loop(). Menyalakan NTP dan klien MQTT begitu WiFi
// tersambung. Setelah itu esp-mqtt menyambung ulang sendiri di task-nya.
void backendPoll();

bool backendTersambung();

// Sketch memanggilnya tiap TELEMETRI_TIAP, dan saat itu juga kalau level bahaya
// atau keberadaan orang berubah.
// Menaruh telemetri di antrean kirim tanpa menunggu jaringan, jadi aman
// dipanggil dari loop() tanpa menahan HomeSpan. false kalau belum tersambung.
bool backendKirimTelemetri(const DataTelemetri &d);

// Perintah Serial Monitor @M, dipasang lewat SpanUserCommand.
void cmdBackend(const char *buf);
