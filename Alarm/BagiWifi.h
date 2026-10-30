// Protokol berbagi WiFi dari Sensor ke Alarm lewat ESP-NOW
//
// File ini disalin identik ke folder Sensor/ dan Alarm/, karena Arduino IDE
// hanya mengompilasi file yang ada di folder sketch masing-masing.
// Kalau mengubah salah satunya, salin juga ke yang lain.
//
// Alur:
//   1. Alarm yang belum punya WiFi membuat pasangan kunci ECDH (P-256) lalu
//      menyiarkan MINTA berisi kunci publiknya, berpindah channel 1..13.
//   2. Sensor yang jendela pairing-nya sedang terbuka membuat pasangan kunci
//      sendiri, menurunkan rahasia bersama, lalu membalas JAWAB berisi kunci
//      publiknya dan SSID + kata sandi yang dienkripsi AES-128-GCM.
//   3. Alarm menurunkan rahasia yang sama dari kunci publik Sensor, membuka
//      kredensialnya, menyimpannya di NVS, lalu menyala ulang.
//
// Kata sandi WiFi tidak pernah terkirim terbuka: orang yang hanya menyadap
// udara tidak bisa membukanya. Yang tetap jadi batas pengamannya adalah
// jendela pairing -- di luar jendela itu Sensor tidak menjawab siapa pun.

#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_idf_version.h>
#include <mbedtls/ecdh.h>
#include <mbedtls/gcm.h>
#include <mbedtls/sha256.h>
#include <mbedtls/version.h>

// mbedtls 2.x (core Arduino 2.x) memakai nama _ret untuk versi yang mengembalikan kode.
#if MBEDTLS_VERSION_MAJOR < 3
  #define mbedtls_sha256 mbedtls_sha256_ret
#endif

#define BW_MAGIC      0x57393041UL   // "A09W" little-endian
#define BW_VERSI      1
#define BW_MINTA      1
#define BW_JAWAB      2

#define BW_PUB_LEN    65             // titik P-256 tak terkompresi
#define BW_NONCE_LEN  12
#define BW_TAG_LEN    16

struct __attribute__((packed)) BwKredensial {
  char ssid[33];
  char pwd[65];
};

struct __attribute__((packed)) BwMinta {
  uint32_t magic;
  uint8_t  versi;
  uint8_t  jenis;
  uint8_t  pub[BW_PUB_LEN];          // kunci publik Alarm
};

struct __attribute__((packed)) BwJawab {
  uint32_t magic;
  uint8_t  versi;
  uint8_t  jenis;
  uint8_t  pub[BW_PUB_LEN];          // kunci publik Sensor
  uint8_t  nonce[BW_NONCE_LEN];
  uint8_t  sandi[sizeof(BwKredensial)];
  uint8_t  tag[BW_TAG_LEN];
};

// Bagian depan JAWAB (sampai kunci publik) ikut diautentikasi GCM, jadi kunci
// publik Sensor tidak bisa ditukar diam-diam di tengah jalan.
#define BW_KEPALA_LEN  offsetof(BwJawab, nonce)

static_assert(sizeof(BwJawab) <= ESP_NOW_MAX_DATA_LEN, "JAWAB melebihi batas ESP-NOW");

// Callback terima ESP-NOW berganti bentuk di ESP-IDF 5 (core Arduino 3.x).
#if ESP_IDF_VERSION_MAJOR >= 5
  #define BW_CB_TERIMA(nama) void nama(const esp_now_recv_info_t *info, const uint8_t *isi, int len)
  #define BW_MAC_PENGIRIM    (info->src_addr)
#else
  #define BW_CB_TERIMA(nama) void nama(const uint8_t *mac, const uint8_t *isi, int len)
  #define BW_MAC_PENGIRIM    (mac)
#endif

static int bwAcak(void *, unsigned char *buf, size_t n) {
  esp_fill_random(buf, n);           // RNG hardware, aktif selama radio menyala
  return 0;
}

static bool bwKepalaSah(const uint8_t *isi, int len, uint8_t jenis, size_t ukuran) {
  if (len != (int)ukuran) return false;
  const BwMinta *k = (const BwMinta *)isi;   // tiga field pertama sama di kedua paket
  return k->magic == BW_MAGIC && k->versi == BW_VERSI && k->jenis == jenis;
}

// Membuat pasangan kunci baru. grp dan d harus sudah di-init pemanggil.
static bool bwBuatKunci(mbedtls_ecp_group *grp, mbedtls_mpi *d, uint8_t pub[BW_PUB_LEN]) {
  mbedtls_ecp_point q;
  mbedtls_ecp_point_init(&q);
  size_t n = 0;

  bool ok = mbedtls_ecp_group_load(grp, MBEDTLS_ECP_DP_SECP256R1) == 0 &&
            mbedtls_ecdh_gen_public(grp, d, &q, bwAcak, NULL) == 0 &&
            mbedtls_ecp_point_write_binary(grp, &q, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                           &n, pub, BW_PUB_LEN) == 0 &&
            n == BW_PUB_LEN;

  mbedtls_ecp_point_free(&q);
  return ok;
}

// Rahasia bersama dari kunci privat sendiri dan kunci publik lawan, diringkas
// SHA-256 menjadi kunci AES-128. Kunci publik lawan diperiksa dulu ada di kurva,
// supaya titik palsu tidak bisa membocorkan kunci privat kita.
static bool bwTurunkanKunci(mbedtls_ecp_group *grp, const mbedtls_mpi *d,
                            const uint8_t pubLawan[BW_PUB_LEN], uint8_t kunci[16]) {
  static const char LABEL[8] = {'A', '0', '9', 'W', 'I', 'F', 'I', '1'};

  mbedtls_ecp_point q;
  mbedtls_mpi z;
  mbedtls_ecp_point_init(&q);
  mbedtls_mpi_init(&z);
  uint8_t bahan[32 + sizeof(LABEL)];
  uint8_t hash[32];

  bool ok = mbedtls_ecp_point_read_binary(grp, &q, pubLawan, BW_PUB_LEN) == 0 &&
            mbedtls_ecp_check_pubkey(grp, &q) == 0 &&
            mbedtls_ecdh_compute_shared(grp, &z, &q, d, bwAcak, NULL) == 0 &&
            mbedtls_mpi_write_binary(&z, bahan, 32) == 0;

  if (ok) {
    memcpy(bahan + 32, LABEL, sizeof(LABEL));
    ok = mbedtls_sha256(bahan, sizeof(bahan), hash, 0) == 0;
    if (ok) memcpy(kunci, hash, 16);
  }

  memset(bahan, 0, sizeof(bahan));
  memset(hash, 0, sizeof(hash));
  mbedtls_mpi_free(&z);
  mbedtls_ecp_point_free(&q);
  return ok;
}

// enkripsi = true: masuk -> keluar + tag. false: masuk + tag -> keluar, gagal
// kalau tag tidak cocok (paket diubah, atau kuncinya memang bukan pasangan kita).
static bool bwGcm(bool enkripsi, const uint8_t kunci[16], const uint8_t *nonce,
                  const uint8_t *aad, size_t aadLen,
                  const uint8_t *masuk, uint8_t *keluar, size_t n, uint8_t *tag) {
  mbedtls_gcm_context g;
  mbedtls_gcm_init(&g);

  bool ok = mbedtls_gcm_setkey(&g, MBEDTLS_CIPHER_ID_AES, kunci, 128) == 0;
  if (ok && enkripsi)
    ok = mbedtls_gcm_crypt_and_tag(&g, MBEDTLS_GCM_ENCRYPT, n, nonce, BW_NONCE_LEN,
                                   aad, aadLen, masuk, keluar, BW_TAG_LEN, tag) == 0;
  else if (ok)
    ok = mbedtls_gcm_auth_decrypt(&g, n, nonce, BW_NONCE_LEN, aad, aadLen,
                                  tag, BW_TAG_LEN, masuk, keluar) == 0;

  mbedtls_gcm_free(&g);
  return ok;
}
