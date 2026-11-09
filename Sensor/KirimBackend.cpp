// Lihat KirimBackend.h untuk kontrak dan cara pakai.

#include "KirimBackend.h"

#include <WiFi.h>
#include <Preferences.h>
#include <math.h>
#include <stdarg.h>
#include <time.h>
#include "mqtt_client.h"     // esp-mqtt, sudah ikut di core ESP32 -- tanpa library tambahan
#include "mbedtls/md.h"
#include "esp_random.h"
#include <ctype.h>
#include <stdlib.h>

// Alamat broker bawaan kalau belum pernah diatur lewat @M. Memakai nama .local
// Mac yang menjalankan BeforeFire-Backend, karena nama itu tetap sama walaupun
// IP Mac berganti. ESP32 mencari nama .local lewat mDNS
// (CONFIG_LWIP_DNS_SUPPORT_MDNS_QUERIES di core ESP32).
// Host-nya harus sama dengan SensorBackendConfiguration.defaultBackendAddress di app.
// Ganti kalau server pindah ke Mac lain.
#ifndef BACKEND_MQTT_URI
  #define BACKEND_MQTT_URI "mqtt://Dericks-MacBook-Pro.local:1883"
#endif

#define BACKEND_NVS        "a09backend"
#define BACKEND_NVS_URI    "uri"

// Jam dianggap sudah sinkron kalau lewat dari 14-11-2023. Sebelum NTP berhasil,
// jam ESP32 mulai dari 1970, dan angka itu tidak boleh dikirim sebagai measuredAt.
#define JAM_SAH            1700000000

// Telemetri lama tidak perlu ditumpuk selama koneksi putus: yang penting hanya
// yang terbaru, dan antrean yang membengkak memakan heap.
#define BATAS_ANTREAN      (8 * 1024)

static const char *idAlat  = nullptr;
static const char *popAlat = nullptr;   // menunjuk provPop; @R mengganti isinya

static char uriBroker[96]     = "";
static char topikStatus[48]   = "";
static char topikTelemetri[48] = "";

// PoP yang sedang dipakai untuk login dan readKeyHash. Dibandingkan dengan
// popAlat di setiap backendPoll() untuk mendeteksi @R.
static char popTerpakai[9]  = "";
static char readKeyHash[65] = "";

// Push token & perintah dari app (topic cmd/ack/alert).
#define BACKEND_NVS_TOKEN "a09token"
#define MAKS_TOKEN     10            // README: 1..10 token per alert
#define PANJANG_TOKEN  200           // token APNs hex (bisa lebih dari 64)
#define MAKS_REQID     16            // berapa requestId terakhir diingat (anti-replay)
#define TOLERANSI_JAM  (5 * 60)      // ±5 menit untuk issuedAt perintah app

static char topikCmd[48]   = "";
static char topikAck[48]   = "";
static char topikAlert[48] = "";

static char token[MAKS_TOKEN][PANJANG_TOKEN];
static int  jumlahToken = 0;

static char reqIdTerakhir[MAKS_REQID][40];
static int  reqIdIdx = 0;

static uint32_t alertBoot = 0;   // diacak sekali di backendMulai: alertId unik antar boot
static uint32_t alertSeq  = 0;

static esp_mqtt_client_handle_t klien = nullptr;
static volatile bool tersambung  = false;
static volatile bool loginDitolak = false;   // supaya pesannya tidak berulang tiap kali menyambung ulang
static volatile bool brokerTakTerjangkau = false;   // sama, untuk gagal menjangkau alamat broker
static bool ntpDimulai = false;

// =====================================================
// Kunci baca data sensor
//   readKey     = hex(HMAC-SHA256(key = PoP, message = "beforefire-read-v1\n" + deviceId))
//   readKeyHash = hex(SHA-256(readKey))
// App mengirim readKey, backend hanya menyimpan hash-nya dari telemetri.
// =====================================================
static void keHex(const uint8_t *data, size_t n, char *keluar) {
  static const char ANGKA[] = "0123456789abcdef";
  for (size_t i = 0; i < n; i++) {
    keluar[i * 2]     = ANGKA[data[i] >> 4];
    keluar[i * 2 + 1] = ANGKA[data[i] & 0x0F];
  }
  keluar[n * 2] = '\0';
}

static bool hitungReadKeyHash(const char *pop, const char *id, char keluar[65]) {
  const mbedtls_md_info_t *sha256 = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!sha256) return false;

  char pesan[64];
  int panjang = snprintf(pesan, sizeof(pesan), "beforefire-read-v1\n%s", id);
  if (panjang < 0 || panjang >= (int)sizeof(pesan)) return false;

  uint8_t mac[32];
  if (mbedtls_md_hmac(sha256, (const uint8_t *)pop, strlen(pop),
                      (const uint8_t *)pesan, panjang, mac) != 0) return false;

  char readKey[65];
  keHex(mac, sizeof(mac), readKey);

  uint8_t hash[32];
  if (mbedtls_md(sha256, (const uint8_t *)readKey, 64, hash) != 0) return false;

  keHex(hash, sizeof(hash), keluar);
  return true;
}

// Contoh resmi dari README backend. Kalau hasilnya beda, app tidak akan pernah
// bisa membaca data alat ini, jadi lebih baik ketahuan saat boot.
static bool ujiReadKey() {
  char hasil[65];
  return hitungReadKeyHash("abcd2345", "DAPUR_A1B2C3", hasil) &&
         strcmp(hasil, "6b3f6671d12855f3048da9f9f26b67077a7e6ef568512789204146eb902e1b6b") == 0;
}

// =====================================================
// Push token di NVS
//   Disimpan sebagai daftar string hex. Backend menyertakannya di tiap alert,
//   lalu APNs memakainya sebagai alamat tujuan notifikasi.
// =====================================================

// Token APNs selalu hex. Memvalidasinya juga menutup celah: token masuk ke JSON
// alert/ack tanpa di-escape, jadi karakter seperti " tidak boleh lolos.
static bool tokenHex(const char *t) {
  size_t n = strlen(t);
  if (n < 1 || n >= PANJANG_TOKEN) return false;
  for (size_t i = 0; i < n; i++)
    if (!isxdigit((unsigned char)t[i])) return false;
  return true;
}

static int cariToken(const char *t) {
  for (int i = 0; i < jumlahToken; i++)
    if (strcmp(token[i], t) == 0) return i;
  return -1;
}

static void muatToken() {
  Preferences pref;
  pref.begin(BACKEND_NVS_TOKEN, true);
  jumlahToken = pref.getInt("n", 0);
  if (jumlahToken < 0 || jumlahToken > MAKS_TOKEN) jumlahToken = 0;
  char key[8];
  for (int i = 0; i < jumlahToken; i++) {
    snprintf(key, sizeof(key), "t%d", i);
    String s = pref.getString(key, "");
    strlcpy(token[i], s.c_str(), PANJANG_TOKEN);
  }
  pref.end();
}

static void simpanToken() {
  Preferences pref;
  pref.begin(BACKEND_NVS_TOKEN, false);
  pref.clear();
  pref.putInt("n", jumlahToken);
  char key[8];
  for (int i = 0; i < jumlahToken; i++) {
    snprintf(key, sizeof(key), "t%d", i);
    pref.putString(key, token[i]);
  }
  pref.end();
}

static bool tambahToken(const char *t) {
  if (cariToken(t) >= 0) return true;            // sudah ada: idempotent
  if (jumlahToken >= MAKS_TOKEN) return false;   // penuh
  strlcpy(token[jumlahToken++], t, PANJANG_TOKEN);
  simpanToken();
  return true;
}

static void hapusToken(const char *t) {
  int i = cariToken(t);
  if (i < 0) return;
  for (int j = i; j < jumlahToken - 1; j++) strlcpy(token[j], token[j + 1], PANJANG_TOKEN);
  jumlahToken--;
  simpanToken();
}

// =====================================================
// Perintah dari app (topic cmd)
// =====================================================

// Parser JSON seadanya untuk pesan datar & tepercaya (app menandatangani,
// ACL broker membatasi siapa yang boleh publish). Mencari "key":"value" (string)
// atau "key":angka. Menolak nilai yang mengandung escape, jadi field yang lolos
// aman ditaruh kembali ke JSON ack tanpa di-escape.
// ponytail: ganti dengan ArduinoJson kalau formatnya berkembang jadi bertingkat.
static bool ambilStr(const char *json, const char *key, char *out, size_t n) {
  char pola[32];
  int pj = snprintf(pola, sizeof(pola), "\"%s\"", key);
  if (pj < 0 || pj >= (int)sizeof(pola)) return false;
  const char *p = strstr(json, pola);
  if (!p) return false;
  p += pj;
  while (*p == ' ' || *p == ':') p++;
  if (*p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i < n - 1) {
    if (*p == '\\') return false;                // tolak escape
    out[i++] = *p++;
  }
  if (*p != '"') return false;
  out[i] = '\0';
  return true;
}

static bool ambilLong(const char *json, const char *key, long *out) {
  char pola[32];
  int pj = snprintf(pola, sizeof(pola), "\"%s\"", key);
  if (pj < 0 || pj >= (int)sizeof(pola)) return false;
  const char *p = strstr(json, pola);
  if (!p) return false;
  p += pj;
  while (*p == ' ' || *p == ':') p++;
  char *akhir;
  long v = strtol(p, &akhir, 10);
  if (akhir == p) return false;
  *out = v;
  return true;
}

// Tanda tangan perintah (lihat README backend):
//   teks = "beforefire-cmd-v1\n<id>\n<action>\n<requestId>\n<issuedAt>\n<pushToken>"
//   sig  = hex(HMAC-SHA256(key = PoP, teks))
static bool hitungTanda(const char *id, const char *pop, const char *action,
                        const char *reqId, long issuedAt, const char *pushTok,
                        char keluar[65]) {
  const mbedtls_md_info_t *sha256 = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!sha256) return false;

  char teks[512];
  int k = snprintf(teks, sizeof(teks), "beforefire-cmd-v1\n%s\n%s\n%s\n%ld\n%s",
                   id, action, reqId, issuedAt, pushTok);
  if (k < 0 || k >= (int)sizeof(teks)) return false;

  uint8_t mac[32];
  if (mbedtls_md_hmac(sha256, (const uint8_t *)pop, strlen(pop),
                      (const uint8_t *)teks, k, mac) != 0) return false;
  keHex(mac, sizeof(mac), keluar);
  return true;
}

// Contoh resmi dari README backend. Kalau hasilnya beda, perintah app tidak akan
// pernah diterima, jadi lebih baik ketahuan saat boot (seperti ujiReadKey).
static bool ujiTandaCmd() {
  char hasil[65];
  return hitungTanda("DAPUR_A1B2C3", "abcd2345", "silence",
                     "3f1c2b7e-0000-4000-8000-000000000001", 1730700000, "", hasil) &&
         strcmp(hasil, "c3543e3f62513b35540d7f523867b05129d7281707750f6f043e74728d9f499c") == 0;
}

// Perbandingan waktu-tetap supaya waktu kegagalan tidak membocorkan isi tanda tangan.
static bool samaAman(const char *a, const char *b) {
  size_t la = strlen(a), lb = strlen(b);
  if (la != lb) return false;
  uint8_t beda = 0;
  for (size_t i = 0; i < la; i++) beda |= (uint8_t)(a[i] ^ b[i]);
  return beda == 0;
}

// requestId dipantulkan ke JSON ack, jadi batasi charset-nya.
static bool reqIdSah(const char *id) {
  size_t n = strlen(id);
  if (n < 1 || n >= sizeof(reqIdTerakhir[0])) return false;
  for (size_t i = 0; i < n; i++) {
    char c = id[i];
    if (!(isalnum((unsigned char)c) || c == '-' || c == '_')) return false;
  }
  return true;
}

static bool reqIdPernah(const char *id) {
  for (int i = 0; i < MAKS_REQID; i++)
    if (reqIdTerakhir[i][0] && strcmp(reqIdTerakhir[i], id) == 0) return true;
  return false;
}

static void reqIdSimpan(const char *id) {
  strlcpy(reqIdTerakhir[reqIdIdx], id, sizeof(reqIdTerakhir[0]));
  reqIdIdx = (reqIdIdx + 1) % MAKS_REQID;
}

static void kirimAck(const char *reqId, const char *hasil, const char *alasan) {
  if (!klien) return;
  char json[160];
  int k;
  if (alasan)
    k = snprintf(json, sizeof(json),
                 "{\"requestId\":\"%s\",\"result\":\"%s\",\"reason\":\"%s\"}", reqId, hasil, alasan);
  else
    k = snprintf(json, sizeof(json), "{\"requestId\":\"%s\",\"result\":\"%s\"}", reqId, hasil);
  if (k > 0 && k < (int)sizeof(json))
    esp_mqtt_client_enqueue(klien, topikAck, json, k, 1, 0, false);
}

// Dijalankan di task esp-mqtt saat ada pesan di topic cmd. Pesan kecil & jarang.
static void tanganiCmd(const char *json) {
  char reqId[40], action[32], origin[16], sig[80], tok[PANJANG_TOKEN];

  if (!ambilStr(json, "requestId", reqId, sizeof(reqId)) || !reqIdSah(reqId)) {
    Serial.println("[BACKEND] cmd: requestId tidak sah, diabaikan");
    return;
  }
  if (!ambilStr(json, "action", action, sizeof(action))) {
    kirimAck(reqId, "rejected", "unknown_action");
    return;
  }
  if (!ambilStr(json, "origin", origin, sizeof(origin))) strlcpy(origin, "app", sizeof(origin));
  if (!ambilStr(json, "pushToken", tok, sizeof(tok))) tok[0] = '\0';

  // origin backend: tanpa tanda tangan, HANYA untuk unregister (APNs melaporkan
  // token 410). ACL broker menjamin hanya user backend yang boleh publish cmd.
  if (strcmp(origin, "backend") == 0) {
    if (strcmp(action, "unregister_push_token") != 0) {
      kirimAck(reqId, "rejected", "unknown_action");
      return;
    }
    if (tokenHex(tok)) hapusToken(tok);
    kirimAck(reqId, "done", nullptr);
    return;
  }

  // origin app: wajib periksa tanda tangan, kesegaran, dan replay.
  long issuedAt = 0;
  ambilLong(json, "issuedAt", &issuedAt);
  if (!ambilStr(json, "signature", sig, sizeof(sig))) {
    kirimAck(reqId, "rejected", "bad_signature");
    return;
  }
  if (reqIdPernah(reqId)) {
    kirimAck(reqId, "rejected", "replayed");
    return;
  }
  time_t sekarang = time(nullptr);
  if (sekarang > JAM_SAH && labs((long)sekarang - issuedAt) > TOLERANSI_JAM) {
    kirimAck(reqId, "rejected", "expired");
    return;
  }
  char hitung[65];
  if (!hitungTanda(idAlat, popTerpakai, action, reqId, issuedAt, tok, hitung) ||
      !samaAman(hitung, sig)) {
    kirimAck(reqId, "rejected", "bad_signature");
    return;
  }

  reqIdSimpan(reqId);   // hanya setelah tanda tangan sah: sampah tidak memenuhi daftar

  if (strcmp(action, "register_push_token") == 0) {
    if (!tokenHex(tok)) { kirimAck(reqId, "rejected", "bad_signature"); return; }
    if (tambahToken(tok)) kirimAck(reqId, "done", nullptr);
    else                  kirimAck(reqId, "rejected", "token_storage_full");
  } else if (strcmp(action, "unregister_push_token") == 0) {
    if (tokenHex(tok)) hapusToken(tok);
    kirimAck(reqId, "done", nullptr);
  } else {
    // `silence` (matikan board Alarm) belum diterapkan di firmware ini.
    // ponytail: balas unknown_action sampai kontrol board Alarm ditambah.
    kirimAck(reqId, "rejected", "unknown_action");
  }
}

// =====================================================
// Klien MQTT
// =====================================================

// Dijalankan di task esp-mqtt, bukan di loop(). Hanya mengubah penanda dan
// menaruh pesan di antrean; tidak menyentuh HomeSpan.
static void saatEventMqtt(void *, esp_event_base_t, int32_t idEvent, void *dataEvent) {
  esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)dataEvent;

  switch ((esp_mqtt_event_id_t)idEvent) {
    case MQTT_EVENT_CONNECTED:
      tersambung   = true;
      loginDitolak = false;
      brokerTakTerjangkau = false;
      // Retained, supaya backend yang baru menyala langsung tahu alat ini online.
      esp_mqtt_client_enqueue(e->client, topikStatus, "online", 0, 1, 1, true);
      esp_mqtt_client_subscribe(e->client, topikCmd, 1);   // perintah dari app/backend
      Serial.printf("[BACKEND] Tersambung ke %s sebagai %s\n", uriBroker, idAlat);
      break;

    case MQTT_EVENT_DISCONNECTED:
      if (tersambung) Serial.println("[BACKEND] Terputus dari broker, mencoba lagi...");
      tersambung = false;
      break;

    case MQTT_EVENT_ERROR:
      if (e->error_handle &&
          e->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED &&
          !loginDitolak) {
        loginDitolak = true;
        Serial.printf("[BACKEND] Broker menolak login (kode %d). Daftarkan board di Mac:\n"
                      "          scripts/mqtt-user.sh add %s <PoP dari stiker>\n",
                      e->error_handle->connect_return_code, idAlat);
      }
      // Gagal sebelum login: nama .local tidak ditemukan, IP salah, atau server mati.
      if (e->error_handle &&
          e->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT &&
          !brokerTakTerjangkau) {
        brokerTakTerjangkau = true;
        Serial.printf("[BACKEND] Tidak bisa menjangkau %s. Pastikan server menyala dan satu WiFi,\n"
                      "          atau atur alamat lain: @M mqtt://<IP Mac>:1883\n", uriBroker);
      }
      break;

    case MQTT_EVENT_DATA:
      // Pesan cmd kecil dan tak terpotong. ponytail: anggap satu fragmen;
      // pesan > ~300 B diabaikan (perintah app jauh lebih kecil dari itu).
      if (e->topic_len == (int)strlen(topikCmd) &&
          strncmp(e->topic, topikCmd, e->topic_len) == 0) {
        char buf[320];
        int n = e->data_len < (int)sizeof(buf) - 1 ? e->data_len : (int)sizeof(buf) - 1;
        memcpy(buf, e->data, n);
        buf[n] = '\0';
        tanganiCmd(buf);
      }
      break;

    default:
      break;
  }
}

static void mulaiKlien() {
  esp_mqtt_client_config_t cfg = {};
  cfg.broker.address.uri                  = uriBroker;
  cfg.credentials.client_id               = idAlat;
  cfg.credentials.username                = idAlat;     // ACL broker memakai %u = ID alat
  cfg.credentials.authentication.password = popTerpakai;
  // Broker mengumumkan "offline" sekitar 1,5x keepalive setelah alat hilang,
  // jadi app melihat alat mati dalam ~15 detik.
  cfg.session.keepalive                   = 10;
  // Broker mengumumkan "offline" sendiri kalau alat mati atau WiFi putus.
  cfg.session.last_will.topic             = topikStatus;
  cfg.session.last_will.msg               = "offline";
  cfg.session.last_will.qos               = 1;
  cfg.session.last_will.retain            = 1;
  cfg.network.reconnect_timeout_ms        = 3000;
  cfg.outbox.limit                        = BATAS_ANTREAN;

  klien = esp_mqtt_client_init(&cfg);
  if (!klien) {
    Serial.println("[BACKEND] Gagal membuat klien MQTT (alamat salah atau heap habis).");
    return;
  }
  esp_mqtt_client_register_event(klien, MQTT_EVENT_ANY, saatEventMqtt, nullptr);
  if (esp_mqtt_client_start(klien) != ESP_OK) {
    Serial.println("[BACKEND] Gagal menjalankan klien MQTT.");
    esp_mqtt_client_destroy(klien);
    klien = nullptr;
    return;
  }
  Serial.printf("[BACKEND] Menyambung ke %s ...\n", uriBroker);
}

// Dipakai saat alamat broker atau PoP berganti. Keluar dengan sopan tidak
// memicu Will, jadi "offline" dikirim sendiri lebih dulu.
static void hentikanKlien() {
  if (!klien) return;
  if (tersambung) esp_mqtt_client_publish(klien, topikStatus, "offline", 0, 1, 1);
  esp_mqtt_client_destroy(klien);
  klien        = nullptr;
  tersambung   = false;
  loginDitolak = false;
  brokerTakTerjangkau = false;
}

// =====================================================
// API untuk sketch
// =====================================================
void backendMulai(const char *deviceId, const char *pop) {
  idAlat  = deviceId;
  popAlat = pop;
  snprintf(topikStatus,    sizeof(topikStatus),    "beforefire/%s/status",    deviceId);
  snprintf(topikTelemetri, sizeof(topikTelemetri), "beforefire/%s/telemetry", deviceId);
  snprintf(topikCmd,       sizeof(topikCmd),       "beforefire/%s/cmd",       deviceId);
  snprintf(topikAck,       sizeof(topikAck),       "beforefire/%s/ack",       deviceId);
  snprintf(topikAlert,     sizeof(topikAlert),     "beforefire/%s/alert",     deviceId);
  muatToken();
  alertBoot = esp_random();

  // Alamat dari @M menang atas bawaan; "@M off" menyimpan string kosong.
  Preferences pref;
  pref.begin(BACKEND_NVS, true);
  bool adaDiNvs = pref.isKey(BACKEND_NVS_URI);
  String tersimpan = adaDiNvs ? pref.getString(BACKEND_NVS_URI, "") : String(BACKEND_MQTT_URI);
  pref.end();
  strlcpy(uriBroker, tersimpan.c_str(), sizeof(uriBroker));

  if (!ujiReadKey())
    Serial.println("[BACKEND] ERROR: perhitungan readKeyHash tidak sesuai contoh README backend.");
  if (!ujiTandaCmd())
    Serial.println("[BACKEND] ERROR: perhitungan tanda tangan perintah tidak sesuai contoh README backend.");
  Serial.printf("[BACKEND] %d push token tersimpan di NVS.\n", jumlahToken);

  if (uriBroker[0])
    Serial.printf("[BACKEND] Broker: %s (ubah dengan @M)\n", uriBroker);
  else
    Serial.println("[BACKEND] Alamat broker belum diatur. Ketik: @M mqtt://<IP Mac>:1883");
}

void backendPoll() {
  if (!idAlat) return;
  if (WiFi.status() != WL_CONNECTED) return;   // esp-mqtt menyambung ulang sendiri nanti

  if (!ntpDimulai) {
    // UTC; measuredAt dikirim dalam Unix detik, jadi zona waktu tidak berpengaruh.
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    ntpDimulai = true;
  }

  if (strcmp(popAlat, popTerpakai) != 0) {
    bool gantiPop = popTerpakai[0] != '\0';
    strlcpy(popTerpakai, popAlat, sizeof(popTerpakai));
    if (!hitungReadKeyHash(popTerpakai, idAlat, readKeyHash)) {
      readKeyHash[0] = '\0';
      Serial.println("[BACKEND] Gagal menghitung readKeyHash.");
    }
    if (gantiPop) {
      Serial.println("[BACKEND] PoP berganti: login ulang dengan PoP baru. Daftarkan ulang board di Mac:");
      Serial.printf("          scripts/mqtt-user.sh add %s %s\n", idAlat, popTerpakai);
      hentikanKlien();
    }
  }

  if (!klien && uriBroker[0]) mulaiKlien();
}

bool backendTersambung() {
  return klien && tersambung;
}

// ---------- Penyusun JSON ----------
// Cukup snprintf: isinya angka, boolean, dan teks tetap yang tidak perlu di-escape.
struct PenulisJson {
  char  *buf;
  size_t ukuran;
  size_t n     = 0;
  bool   penuh = false;
};

static void tulis(PenulisJson &p, const char *fmt, ...) {
  if (p.penuh) return;
  va_list args;
  va_start(args, fmt);
  int k = vsnprintf(p.buf + p.n, p.ukuran - p.n, fmt, args);
  va_end(args);
  if (k < 0 || (size_t)k >= p.ukuran - p.n) { p.penuh = true; return; }
  p.n += k;
}

// NaN atau tak hingga ditulis "nan"/"inf" oleh printf, yang bukan JSON sah dan
// membuat backend menolak seluruh pesan. Nilai seperti itu dilewati saja.
static void tulisAngka(PenulisJson &p, bool &pertama, const char *nama, float nilai, int desimal) {
  if (!isfinite(nilai)) return;
  tulis(p, "%s\"%s\":%.*f", pertama ? "" : ",", nama, desimal, nilai);
  pertama = false;
}

// Nama level harus sama dengan alert.Condition di backend.
static const char *namaKondisi(Condition c) {
  switch (c) {
    case Condition::Normal:   return "normal";
    case Condition::Warning1: return "warning1";
    case Condition::Warning2: return "warning2";
    case Condition::Alarm:    return "alarm";
  }
  return nullptr;
}

// Objek "readings" dipakai sama oleh telemetry dan alert. Tanpa koma di depan.
static void tulisReadings(PenulisJson &p, const DataTelemetri &d) {
  tulis(p, "\"readings\":{");
  bool pertama = true;
  if (d.adaKompor) tulisAngka(p, pertama, "stoveTemperatureC", d.komporC, 1);
  if (d.adaBme) {
    tulisAngka(p, pertama, "roomTemperatureC", d.suhuC, 1);
    tulisAngka(p, pertama, "roomHumidityRH", d.lembapRH, 1);
    tulisAngka(p, pertama, "pressureHPa", d.tekananHPa, 1);
    tulisAngka(p, pertama, "iaq", d.iaq, 0);
    tulis(p, "%s\"iaqAccuracy\":%u", pertama ? "" : ",", d.iaqAkurasi);
    pertama = false;
    tulisAngka(p, pertama, "eco2Ppm", d.eco2Ppm, 0);
  }
  if (d.adaOrang) {
    tulis(p, "%s\"presence\":%s", pertama ? "" : ",", d.orang ? "true" : "false");
    pertama = false;
  }
  tulis(p, "}");
}

bool backendKirimTelemetri(const DataTelemetri &d) {
  if (!backendTersambung() || !readKeyHash[0]) return false;

  char json[640];
  PenulisJson p{json, sizeof(json)};

  tulis(p, "{\"readKeyHash\":\"%s\"", readKeyHash);

  time_t sekarang = time(nullptr);
  if (sekarang > JAM_SAH) tulis(p, ",\"measuredAt\":%ld", (long)sekarang);

  const char *kondisi = d.adaKondisi ? namaKondisi(d.kondisi) : nullptr;
  if (kondisi) {
    tulis(p, ",\"condition\":\"%s\"", kondisi);
    if (isfinite(d.skor)) tulis(p, ",\"score\":%.1f", d.skor);
    if (d.ruleId != 0) {
      tulis(p, ",\"assessment\":{\"condition\":\"%s\",\"valid\":true,\"rule_id\":%u,\"absence_duration_seconds\":%lu}",
            kondisi, static_cast<unsigned int>(d.ruleId), d.absenceDurationS);
    }
  }

  tulis(p, ",");
  tulisReadings(p, d);

  if (d.bmeRusak || d.radarRusak || d.termalRusak) {
    tulis(p, ",\"unavailableSensors\":[");
    bool pertamaRusak = true;
    const char *nama[]  = {"bme", "radar", "thermal"};
    const bool  rusak[] = {d.bmeRusak, d.radarRusak, d.termalRusak};
    for (int i = 0; i < 3; i++) {
      if (!rusak[i]) continue;
      tulis(p, "%s\"%s\"", pertamaRusak ? "" : ",", nama[i]);
      pertamaRusak = false;
    }
    tulis(p, "]");
  }
  tulis(p, "}");

  if (p.penuh) {
    Serial.println("[BACKEND] Telemetri melebihi buffer, tidak dikirim.");
    return false;
  }

  // QoS 0 + retained: yang penting nilai terbaru, dan broker menyimpannya untuk
  // backend yang baru menyala. enqueue tidak menunggu jaringan.
  return esp_mqtt_client_enqueue(klien, topikTelemetri, json, p.n, 0, 1, true) >= 0;
}

bool backendKirimAlert(const DataTelemetri &d) {
  if (!backendTersambung()) return false;
  if (jumlahToken == 0) return false;             // tanpa token backend menolak alert
  const char *kondisi = d.adaKondisi ? namaKondisi(d.kondisi) : nullptr;
  if (!kondisi) return false;

  // 10 token x ~200 + field. Statis: hanya dipanggil dari loop(), tidak re-entran.
  static char json[2600];
  PenulisJson p{json, sizeof(json)};

  // alertId unik per kejadian: acak per boot + nomor urut. Charset aman (hex + '-').
  char alertId[40];
  snprintf(alertId, sizeof(alertId), "%08lx-%08lx",
           (unsigned long)alertBoot, (unsigned long)++alertSeq);

  tulis(p, "{\"alertId\":\"%s\",\"condition\":\"%s\"", alertId, kondisi);
  if (isfinite(d.skor)) tulis(p, ",\"score\":%.1f", d.skor);
  time_t sekarang = time(nullptr);
  if (sekarang > JAM_SAH) tulis(p, ",\"issuedAt\":%ld", (long)sekarang);
  tulis(p, ",");
  tulisReadings(p, d);
  tulis(p, ",\"pushTokens\":[");
  for (int i = 0; i < jumlahToken; i++)
    tulis(p, "%s\"%s\"", i ? "," : "", token[i]);
  tulis(p, "]}");

  if (p.penuh) {
    Serial.println("[BACKEND] Alert melebihi buffer, tidak dikirim.");
    return false;
  }
  Serial.printf("[BACKEND] Alert %s (%s) ke %d token\n", alertId, kondisi, jumlahToken);
  // QoS 1, tidak retained: alert harus sampai, tapi tidak boleh dikirim ulang
  // ke backend yang baru menyala.
  return esp_mqtt_client_enqueue(klien, topikAlert, json, p.n, 1, 0, false) >= 0;
}

// ---------- Perintah Serial Monitor ----------
//   @M                       status koneksi
//   @M mqtt://<host>:<port>  simpan alamat broker dan sambung ulang
//   @M off                   matikan pengiriman ke backend
void cmdBackend(const char *buf) {
  const char *arg = buf + 1;              // buf diawali huruf perintahnya sendiri ('M')
  while (*arg == ' ') arg++;

  if (*arg == '\0') {
    time_t sekarang = time(nullptr);
    Serial.printf("\n[BACKEND] ID alat    : %s\n", idAlat ? idAlat : "?");
    Serial.printf("[BACKEND] broker     : %s\n", uriBroker[0] ? uriBroker : "(belum diatur)");
    Serial.printf("[BACKEND] WiFi       : %s\n", WiFi.status() == WL_CONNECTED ? "tersambung" : "TIDAK tersambung");
    Serial.printf("[BACKEND] MQTT       : %s\n",
                  backendTersambung() ? "tersambung"
                  : loginDitolak        ? "DITOLAK (cek user/PoP di broker)"
                  : brokerTakTerjangkau ? "TIDAK TERJANGKAU (cek alamat/server)"
                  : klien               ? "menyambung..."
                                      : "mati");
    Serial.printf("[BACKEND] jam (NTP)  : %s\n", sekarang > JAM_SAH ? "sinkron" : "belum sinkron");
    Serial.printf("[BACKEND] readKeyHash: %s\n\n", readKeyHash[0] ? readKeyHash : "(belum dihitung)");
    return;
  }

  char baru[sizeof(uriBroker)];
  if (strcmp(arg, "off") == 0) {
    baru[0] = '\0';
  } else if (strncmp(arg, "mqtt://", 7) == 0 && strlen(arg) < sizeof(baru)) {
    strlcpy(baru, arg, sizeof(baru));
    // Buang spasi atau CR yang ikut terketik di belakang.
    for (int i = strlen(baru) - 1; i >= 0 && isspace((unsigned char)baru[i]); i--) baru[i] = '\0';
  } else {
    Serial.println("[BACKEND] Format: @M mqtt://<IP Mac>:1883   atau   @M off");
    return;
  }

  Preferences pref;
  pref.begin(BACKEND_NVS, false);
  pref.putString(BACKEND_NVS_URI, baru);
  pref.end();

  hentikanKlien();
  strlcpy(uriBroker, baru, sizeof(uriBroker));
  if (uriBroker[0])
    Serial.printf("[BACKEND] Alamat broker disimpan: %s\n", uriBroker);
  else
    Serial.println("[BACKEND] Pengiriman ke backend dimatikan.");
  // backendPoll() di putaran berikutnya yang menyambung lagi.
}
