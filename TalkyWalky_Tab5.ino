/*
 * ============================================================================
 *  TalkyWalky Tab5  —  Push-To-Talk voice messaging between M5Stack Tab5
 * ============================================================================
 *  - Peer discovery   : UDP broadcast beacon (port 45000) every 2 s
 *  - Voice transport  : TCP (port 45001), one short voice file per message
 *  - Voice codec      : IMA-ADPCM 4-bit, 16 kHz mono  (~8 kB per second)
 *                       (codec layer isolated -> can be swapped for AMR-NB)
 *  - UI               : WhatsApp-like (contacts list + chat bubbles + PTT)
 *  - QoS / user level : LOW / MEDIUM / HIGH
 *        * max message length + cooldown per level (sender side)
 *        * playback priority (HIGH interrupts, MEDIUM before LOW)
 *        * IP TOS / DSCP marking -> Wi-Fi WMM access category
 *        * receiver re-checks the policy (rejects oversized messages)
 *        * raising your own level requires an admin PIN
 *  - MASTER station   : Android app (App Inventor + TalkyMaster extension)
 *        * sends to one Tab5 or to all, level MASTER (always interrupts)
 *        * listen-only "Master" channel on every Tab5
 *        * master messages authenticated with a shared secret (FNV-1a tag)
 *
 *  Board   : M5Stack Tab5 (ESP32-P4 + ESP32-C6), Arduino-ESP32 core >= 3.2
 *            PSRAM enabled, partition with a large APP
 *  Libs    : M5Unified + M5GFX (latest)
 * ============================================================================
 */

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <esp_heap_caps.h>
#include <lwip/sockets.h>
#include <time.h>

// ============================ USER CONFIGURATION ============================
const char* WIFI_SSID  = "Kiiway";
const char* WIFI_PASS  = "endgame010824";
const char* ADMIN_PIN  = "2580";          // needed to RAISE the QoS level
const char* MASTER_SECRET = "invethic-master";   // must match the app's MasterSecret
const char* TZ_INFO    = "CET-1CEST,M3.5.0,M10.5.0/3";   // France
// #define DEVICE_NAME "Atelier-1"        // uncomment to force a name
                                          // (default: TAB5-XXXX from MAC)
#define SCREEN_ROTATION 3                 // landscape; try 1 if upside down

// ============================ TECHNICAL SETTINGS ============================
#define SAMPLE_RATE       16000
#define CHUNK             800             // 50 ms mic chunks
#define MAX_SECONDS       30              // longest possible message (HIGH)
#define MAX_SAMPLES       (MAX_SECONDS * SAMPLE_RATE)
#define UDP_PORT          45000
#define TCP_PORT          45001
#define BEACON_MS         2000
#define PEER_TIMEOUT_MS   7000
#define MAX_PEERS         16
#define MAX_MSGS          60
#define ENV_BARS          24              // waveform bars in a bubble
#define MASTER_KEY        1u              // reserved conversation key (master channel)
#define PROTO_VERSION     2

// Tab5 : ESP32-C6 Wi-Fi co-processor on SDIO2
#define SDIO2_CLK GPIO_NUM_12
#define SDIO2_CMD GPIO_NUM_13
#define SDIO2_D0  GPIO_NUM_11
#define SDIO2_D1  GPIO_NUM_10
#define SDIO2_D2  GPIO_NUM_9
#define SDIO2_D3  GPIO_NUM_8
#define SDIO2_RST GPIO_NUM_15

// ================================ COLOURS ===================================
// NOTE: must stay a macro. The Arduino IDE inserts auto-generated function
// prototypes above the FIRST function definition of the sketch; a function
// here would put them before the struct definitions -> "does not name a type".
#define C565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
static constexpr uint16_t COL_TEAL       = C565(0, 128, 105);
static constexpr uint16_t COL_TEAL_DARK  = C565(7, 94, 84);
static constexpr uint16_t COL_GREEN      = C565(37, 211, 102);
static constexpr uint16_t COL_BUBBLE_OUT = C565(217, 253, 211);
static constexpr uint16_t COL_BUBBLE_IN  = C565(255, 255, 255);
static constexpr uint16_t COL_CHAT_BG    = C565(239, 234, 226);
static constexpr uint16_t COL_PATTERN    = C565(226, 219, 207);
static constexpr uint16_t COL_PANEL      = C565(255, 255, 255);
static constexpr uint16_t COL_PANEL_SEL  = C565(240, 242, 245);
static constexpr uint16_t COL_TEXT       = C565(17, 27, 33);
static constexpr uint16_t COL_TEXT2      = C565(102, 119, 129);
static constexpr uint16_t COL_DIV        = C565(225, 230, 233);
static constexpr uint16_t COL_TICK       = C565(83, 189, 235);
static constexpr uint16_t COL_RED        = C565(234, 67, 53);
static constexpr uint16_t COL_RED_HALO   = C565(255, 205, 210);
static constexpr uint16_t COL_ORANGE     = C565(255, 152, 0);
static constexpr uint16_t COL_BLUEGREY   = C565(96, 125, 139);
static constexpr uint16_t COL_DARK       = C565(32, 44, 51);
static constexpr uint16_t COL_NOTICE     = C565(255, 243, 196);
static constexpr uint16_t COL_WHITE      = C565(255, 255, 255);
static constexpr uint16_t COL_MASTER     = C565(33, 33, 33);
static constexpr uint16_t COL_GOLD       = C565(255, 193, 7);

// ================================== QoS =====================================
enum : uint8_t { QOS_LOW = 0, QOS_MED = 1, QOS_HIGH = 2, QOS_MASTER = 3 };

struct QosProfile {
  const char* name;
  uint16_t    color;
  uint8_t     tos;          // IP TOS byte (DSCP << 2)
  uint16_t    maxSec;       // longest message allowed
  uint16_t    cooldownSec;  // min time between two messages
  const char* wmm;          // resulting Wi-Fi WMM access category
  const char* playback;     // receiver behaviour
};

// TOS precedence (3 MSB) -> 802.11e UP -> WMM AC :
//   CS1 0x20 -> UP1 -> AC_BK | AF41 0x88 -> UP4 -> AC_VI | CS6 0xC0 -> UP6 -> AC_VO
//   CS7 0xE0 -> UP7 -> AC_VO (master app only)
const QosProfile QOS[4] = {
  {"LOW",    COL_BLUEGREY, 0x20,  8, 10, "AC_BK (CS1)",  "after others"},
  {"MEDIUM", COL_ORANGE,   0x88, 15,  3, "AC_VI (AF41)", "before LOW"},
  {"HIGH",   COL_RED,      0xC0, 30,  0, "AC_VO (CS6)",  "interrupts"},
  {"MASTER", COL_MASTER,   0xE0, 30,  0, "AC_VO (CS7)",  "always interrupts"},
};

// =============================== DATA TYPES =================================
struct Rect {
  int16_t x = 0, y = 0, w = 0, h = 0;
  bool hit(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};

struct Peer {
  bool     used;
  char     name[24];
  uint32_t ip;
  uint8_t  qos;
  uint32_t lastSeen;
};

enum : uint8_t { ST_SENDING = 0, ST_DELIVERED, ST_PARTIAL, ST_FAILED };

struct VoiceMsg {
  uint32_t id;
  bool     outgoing;
  bool     broadcast;
  bool     seen;
  bool     played;
  uint8_t  qos;
  uint8_t  status;
  uint8_t  okCount, targets;
  uint32_t ip;               // peer IP (0 for outgoing broadcast)
  char     name[24];         // sender name (incoming)
  uint8_t* data;             // ADPCM payload (PSRAM)
  uint32_t dataLen;
  uint32_t numSamples;
  uint32_t sampleRate;
  time_t   ts;
  uint32_t ms;
  uint8_t  env[ENV_BARS];
};

#pragma pack(push, 1)
struct VoiceHeader {
  char     magic[4];         // "TWKV"
  uint8_t  version;          // 1
  uint8_t  qos;
  uint8_t  codec;            // 1 = IMA-ADPCM 4-bit mono
  uint8_t  flags;            // bit0 = broadcast
  uint32_t sampleRate;
  uint32_t numSamples;
  uint32_t payloadLen;
  char     name[24];
  uint8_t  env[ENV_BARS];
  uint32_t auth;             // MASTER only : FNV-1a(secret, numSamples, payloadLen)
};
#pragma pack(pop)

struct Incoming {
  uint32_t    ip;
  VoiceHeader h;
  uint8_t*    data;
};

// ================================ GLOBALS ===================================
M5Canvas     canvas(&M5.Display);
WiFiUDP      udp;
Preferences  prefs;
QueueHandle_t inQueue;

int W = 1280, H = 720;
const int LW  = 420;   // left panel width
const int HDR = 96;    // header height
const int BOT = 110;   // bottom bar height
const int ROW = 96;    // contact row height

char     myName[24];
uint8_t  myQos  = QOS_MED;
uint8_t  volume = 180;

Peer     peers[MAX_PEERS];
VoiceMsg msgs[MAX_MSGS];
int      msgCount  = 0;
uint32_t nextMsgId = 1;

uint32_t playQ[16];
int      playQLen    = 0;
uint32_t playingId   = 0;
uint32_t playStartMs = 0;
uint32_t playDurMs   = 1;

int16_t* pcmRec  = nullptr;
int16_t* pcmPlay = nullptr;

bool              recording    = false;
volatile bool     recActive    = false;
volatile bool     recFull      = false;
volatile bool     recTaskBusy  = false;
volatile uint32_t recPos       = 0;
uint32_t          recMaxSamples = 0;
uint32_t          recStartMs   = 0;
uint32_t          lastSendMs   = 0;

uint32_t selIp = 0;          // 0 = "All devices" broadcast channel
int      listScroll = 0;
int      chatScroll = 0;

enum UiMode { UI_MAIN, UI_SETTINGS, UI_PIN };
UiMode  uiMode = UI_MAIN;
char    pinBuf[8] = "";
uint8_t pendingQos = QOS_LOW;

char     toastText[80] = "";
uint32_t toastUntil = 0;
bool     dirty = true;
uint32_t lastDraw = 0, lastBeacon = 0;

// hit areas (filled while drawing)
struct RowHit    { Rect r; uint32_t ip; } rowHits[MAX_PEERS + 1];
struct BubbleHit { Rect r; uint32_t id; } bubbleHits[16];
int  rowHitCount = 0, bubbleHitCount = 0;
Rect gearRect, pttRect, closeRect, doneRect, volMinus, volPlus, cancelRect;
Rect qosRect[3], keyRect[12];
const char KEYS[12] = {'1','2','3','4','5','6','7','8','9','C','0','K'};

void drawAll();   // forward declaration

// ============================== SMALL HELPERS ===============================
void toast(const char* fmt, ...) {
  va_list ap; va_start(ap, fmt);
  vsnprintf(toastText, sizeof(toastText), fmt, ap);
  va_end(ap);
  toastUntil = millis() + 2500;
  dirty = true;
  Serial.printf("[toast] %s\n", toastText);
}

inline int16_t clamp16(int32_t v) { return v > 32767 ? 32767 : (v < -32768 ? -32768 : v); }

bool peerOnline(const Peer* p) { return p && p->used && (millis() - p->lastSeen) < PEER_TIMEOUT_MS; }

Peer* findPeer(uint32_t ip) {
  for (auto& p : peers) if (p.used && p.ip == ip) return &p;
  return nullptr;
}

int onlineCount() {
  int n = 0;
  for (auto& p : peers) if (peerOnline(&p)) n++;
  return n;
}

VoiceMsg* findMsg(uint32_t id) {
  if (!id) return nullptr;
  for (int i = 0; i < msgCount; i++) if (msgs[i].id == id) return &msgs[i];
  return nullptr;
}

uint32_t convKey(const VoiceMsg& m) {
  if (m.qos == QOS_MASTER) return MASTER_KEY;
  return m.broadcast ? 0 : m.ip;
}

// Authentication tag of master messages (same algorithm in the Android extension).
// Educational only: the tag can be replayed and the secret is shared -> discuss it!
uint32_t masterAuth(uint32_t numSamples, uint32_t payloadLen) {
  uint32_t h = 2166136261u;
  for (const char* p = MASTER_SECRET; *p; p++) { h ^= (uint8_t)*p; h *= 16777619u; }
  for (int i = 0; i < 4; i++) { h ^= (numSamples >> (8 * i)) & 0xFF; h *= 16777619u; }
  for (int i = 0; i < 4; i++) { h ^= (payloadLen >> (8 * i)) & 0xFF; h *= 16777619u; }
  return h;
}

time_t nowTs() { time_t t = time(nullptr); return t > 1700000000 ? t : 0; }

void formatTime(const VoiceMsg& m, char* out, size_t n) {
  if (m.ts) { struct tm tmv; localtime_r(&m.ts, &tmv); strftime(out, n, "%H:%M", &tmv); }
  else      { snprintf(out, n, "+%lum", (unsigned long)(m.ms / 60000)); }
}

uint16_t avatarColor(const char* name) {
  static const uint16_t pal[8] = {
    C565(0,150,136), C565(63,81,181), C565(233,30,99), C565(255,112,67),
    C565(124,77,255), C565(3,155,229), C565(67,160,71), C565(141,110,99) };
  uint32_t h = 5381;
  while (*name) h = h * 33 + (uint8_t)*name++;
  return pal[h % 8];
}

void initials(const char* name, char* out) {
  out[0] = toupper(name[0] ? name[0] : '?');
  const char* sep = strrchr(name, '-');
  if (!sep) sep = strrchr(name, ' ');
  out[1] = (sep && sep[1]) ? toupper(sep[1]) : (name[0] && name[1] ? toupper(name[1]) : 0);
  out[2] = 0;
}

// ========================= CODEC : IMA-ADPCM 4-bit ==========================
// Replace these two functions (+ codec id in the header) to switch to AMR-NB.
static const int16_t kStep[89] = {
  7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,
  107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,
  724,796,876,963,1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,3327,
  3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,
  15289,16818,18500,20350,22385,24623,27086,29794,32767 };
static const int8_t kIndex[16] = { -1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8 };

size_t adpcmEncode(const int16_t* in, uint32_t n, uint8_t* out) {
  int pred = 0, idx = 0;
  for (uint32_t i = 0; i < n; i++) {
    int step = kStep[idx];
    int diff = in[i] - pred;
    uint8_t code = 0;
    if (diff < 0) { code = 8; diff = -diff; }
    int vp = step >> 3;
    if (diff >= step) { code |= 4; diff -= step; vp += step; }
    step >>= 1;
    if (diff >= step) { code |= 2; diff -= step; vp += step; }
    step >>= 1;
    if (diff >= step) { code |= 1; vp += step; }
    pred += (code & 8) ? -vp : vp;
    pred = clamp16(pred);
    idx += kIndex[code];
    idx = idx < 0 ? 0 : (idx > 88 ? 88 : idx);
    if (i & 1) out[i >> 1] |= (code << 4);
    else       out[i >> 1]  = code;
  }
  return (n + 1) / 2;
}

void adpcmDecode(const uint8_t* in, uint32_t n, int16_t* out) {
  int pred = 0, idx = 0;
  for (uint32_t i = 0; i < n; i++) {
    uint8_t code = (i & 1) ? (in[i >> 1] >> 4) : (in[i >> 1] & 0x0F);
    int step = kStep[idx];
    int vp = step >> 3;
    if (code & 4) vp += step;
    if (code & 2) vp += step >> 1;
    if (code & 1) vp += step >> 2;
    pred += (code & 8) ? -vp : vp;
    pred = clamp16(pred);
    idx += kIndex[code];
    idx = idx < 0 ? 0 : (idx > 88 ? 88 : idx);
    out[i] = (int16_t)pred;
  }
}

// DC removal + short fade-in + automatic gain (normalisation, capped x8)
void conditionAudio(int16_t* s, uint32_t n) {
  float px = 0, py = 0;
  for (uint32_t i = 0; i < n; i++) {
    float x = s[i];
    float y = x - px + 0.995f * py;
    px = x; py = y;
    s[i] = clamp16((int32_t)y);
  }
  uint32_t fade = min<uint32_t>(n, SAMPLE_RATE / 50);   // 20 ms fade-in
  for (uint32_t i = 0; i < fade; i++) s[i] = (int32_t)s[i] * (int32_t)i / (int32_t)fade;
  int peak = 1;
  for (uint32_t i = 0; i < n; i++) peak = max(peak, abs((int)s[i]));
  float g = min(8.0f, 26000.0f / peak);
  for (uint32_t i = 0; i < n; i++) s[i] = clamp16((int32_t)(s[i] * g));
}

void computeEnvelope(const int16_t* s, uint32_t n, uint8_t* env) {
  uint32_t seg = n / ENV_BARS;
  if (!seg) { memset(env, 0, ENV_BARS); return; }
  for (int b = 0; b < ENV_BARS; b++) {
    int peak = 0;
    for (uint32_t i = b * seg; i < (b + 1) * seg; i++) peak = max(peak, abs((int)s[i]));
    env[b] = (uint8_t)(sqrtf(peak / 32768.0f) * 255.0f);
  }
}

// ============================== MESSAGE STORE ===============================
VoiceMsg* addMsg(const VoiceMsg& m) {
  if (msgCount >= MAX_MSGS) {               // drop the oldest
    free(msgs[0].data);
    memmove(&msgs[0], &msgs[1], sizeof(VoiceMsg) * (MAX_MSGS - 1));
    msgCount--;
  }
  msgs[msgCount] = m;
  return &msgs[msgCount++];
}

void selectConv(uint32_t key) {
  selIp = key;
  chatScroll = 0;
  for (int i = 0; i < msgCount; i++)
    if (convKey(msgs[i]) == key) msgs[i].seen = true;
  dirty = true;
}

// ================================= PEERS ====================================
Peer* upsertPeer(uint32_t ip, const char* name, uint8_t qos) {
  Peer* p = findPeer(ip);
  if (!p) {
    int slot = -1;
    for (int i = 0; i < MAX_PEERS; i++) if (!peers[i].used) { slot = i; break; }
    if (slot < 0) {                           // recycle the oldest offline peer
      uint32_t oldest = UINT32_MAX;
      for (int i = 0; i < MAX_PEERS; i++)
        if (!peerOnline(&peers[i]) && peers[i].lastSeen < oldest) { oldest = peers[i].lastSeen; slot = i; }
      if (slot < 0) return nullptr;
    }
    p = &peers[slot];
    memset(p, 0, sizeof(Peer));
    p->used = true;
    p->ip = ip;
    toast("%s joined the network", name);
  }
  if (strcmp(p->name, name) || p->qos != qos || !peerOnline(p)) dirty = true;
  strlcpy(p->name, name, sizeof(p->name));
  p->qos = qos > QOS_HIGH ? QOS_HIGH : qos;
  p->lastSeen = millis();
  return p;
}

void sendBeacon() {
  if (WiFi.status() != WL_CONNECTED) return;
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "TWK1;%s;%u", myName, myQos);
  udp.beginPacket(WiFi.broadcastIP(), UDP_PORT);
  udp.write((const uint8_t*)buf, n);
  udp.endPacket();
}

void pollBeacons() {
  int sz;
  while ((sz = udp.parsePacket()) > 0) {
    char buf[80];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = 0;
    IPAddress rip = udp.remoteIP();
    if (rip == WiFi.localIP()) continue;
    if (strncmp(buf, "TWK1;", 5)) continue;
    char* name = buf + 5;
    char* sep = strchr(name, ';');
    if (!sep) continue;
    *sep = 0;
    int q = constrain(atoi(sep + 1), 0, 2);
    upsertPeer((uint32_t)rip, name, (uint8_t)q);
  }
}

// =============================== NETWORK I/O ================================
bool readExact(WiFiClient& c, uint8_t* buf, size_t n, uint32_t timeoutMs) {
  size_t got = 0;
  uint32_t t0 = millis();
  while (got < n) {
    int a = c.available();
    if (a > 0) {
      int r = c.read(buf + got, min((size_t)a, n - got));
      if (r > 0) { got += r; t0 = millis(); }
    } else {
      if (!c.connected()) return false;
      if (millis() - t0 > timeoutMs) return false;
      vTaskDelay(1);
    }
  }
  return true;
}

// Receive task : accepts voice files and hands them to the UI loop
void netTask(void*) {
  WiFiServer server(TCP_PORT);
  server.begin();
  server.setNoDelay(true);
  for (;;) {
    WiFiClient c = server.accept();
    if (!c) { vTaskDelay(10); continue; }

    VoiceHeader h;
    bool ok = readExact(c, (uint8_t*)&h, sizeof(h), 3000);
    ok = ok && !memcmp(h.magic, "TWKV", 4) && h.version == PROTO_VERSION && h.codec == 1;
    ok = ok && h.qos <= QOS_MASTER;
    // a MASTER message must carry a valid authentication tag
    if (ok && h.qos == QOS_MASTER && h.auth != masterAuth(h.numSamples, h.payloadLen)) {
      Serial.printf("[sec] rejected fake MASTER message from %s\n", c.remoteIP().toString().c_str());
      ok = false;
    }
    ok = ok && h.sampleRate >= 8000 && h.sampleRate <= 48000;
    ok = ok && h.numSamples > 0 && h.numSamples <= MAX_SAMPLES;
    ok = ok && h.payloadLen == (h.numSamples + 1) / 2;
    // receiver-side policy check : a LOW user cannot push a 30 s message
    ok = ok && h.numSamples <= (uint32_t)QOS[h.qos].maxSec * h.sampleRate + CHUNK;

    uint8_t* data = nullptr;
    if (ok) {
      data = (uint8_t*)heap_caps_malloc(h.payloadLen, MALLOC_CAP_SPIRAM);
      ok = data && readExact(c, data, h.payloadLen, 5000);
    }
    if (ok) {
      h.name[sizeof(h.name) - 1] = 0;
      Incoming* in = new Incoming;
      in->ip = (uint32_t)c.remoteIP();
      in->h = h;
      in->data = data;
      if (xQueueSend(inQueue, &in, 0) == pdTRUE) {
        c.write((uint8_t)'A');                // application-level ACK
      } else {
        free(data); delete in;
        c.write((uint8_t)'B');                // busy
      }
    } else {
      if (data) free(data);
      c.write((uint8_t)'R');                  // rejected
    }
    c.flush();
    vTaskDelay(5);
    c.stop();
  }
}

bool sendVoice(uint32_t ip, const VoiceHeader& h, const uint8_t* data, size_t len) {
  WiFiClient c;
  if (!c.connect(IPAddress(ip), TCP_PORT, 2000)) return false;
  int fd = c.fd();
  if (fd >= 0) {
    int tos = QOS[myQos].tos;                 // DSCP marking -> WMM category
    setsockopt(fd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));
  }
  c.setNoDelay(true);

  auto writeAll = [&](const uint8_t* p, size_t n) -> bool {
    size_t sent = 0;
    uint32_t t0 = millis();
    while (sent < n) {
      size_t w = c.write(p + sent, min((size_t)4096, n - sent));
      if (w > 0) { sent += w; t0 = millis(); }
      else { if (!c.connected() || millis() - t0 > 3000) return false; delay(2); }
    }
    return true;
  };

  bool ok = writeAll((const uint8_t*)&h, sizeof(h)) && writeAll(data, len);
  if (ok) {
    uint8_t ack = 0;
    ok = readExact(c, &ack, 1, 3000) && ack == 'A';
  }
  c.stop();
  return ok;
}

// ================================ PLAYBACK ==================================
uint8_t qosOfId(uint32_t id) { VoiceMsg* m = findMsg(id); return m ? m->qos : 0; }

void stopPlayback() {
  M5.Speaker.stop();
  playingId = 0;
  dirty = true;
}

void startPlay(VoiceMsg* m) {
  if (!m || recording) return;
  M5.Speaker.stop();
  uint32_t n = min<uint32_t>(m->numSamples, MAX_SAMPLES);
  adpcmDecode(m->data, n, pcmPlay);
  M5.Speaker.playRaw(pcmPlay, n, m->sampleRate, false, 1, 0);
  playingId   = m->id;
  playStartMs = millis();
  playDurMs   = max<uint32_t>(1, n * 1000UL / m->sampleRate);
  m->played   = true;
  for (int i = 0; i < playQLen; i++)          // remove from queue if present
    if (playQ[i] == m->id) { memmove(&playQ[i], &playQ[i + 1], (playQLen - i - 1) * 4); playQLen--; break; }
  dirty = true;
}

void enqueuePlay(uint32_t id, uint8_t qos) {
  if (qos >= QOS_HIGH && !recording) {        // HIGH / MASTER preempt lower playback
    VoiceMsg* cur = findMsg(playingId);
    if (!cur || cur->qos < qos) { startPlay(findMsg(id)); return; }
  }
  if (playQLen >= 16) {
    if (qosOfId(playQ[playQLen - 1]) > qos) return;   // queue full of higher prio
    playQLen--;
  }
  int pos = playQLen;
  while (pos > 0 && qosOfId(playQ[pos - 1]) < qos) pos--;
  memmove(&playQ[pos + 1], &playQ[pos], (playQLen - pos) * 4);
  playQ[pos] = id;
  playQLen++;
}

void servicePlayback() {
  if (recording) return;
  if (playingId && !M5.Speaker.isPlaying()) { playingId = 0; dirty = true; }
  while (!playingId && playQLen > 0) {
    uint32_t id = playQ[0];
    memmove(&playQ[0], &playQ[1], (playQLen - 1) * 4);
    playQLen--;
    VoiceMsg* m = findMsg(id);
    if (m) startPlay(m);
  }
}

// ================================ RECORDING =================================
void recTask(void*) {
  for (;;) {
    if (recActive) {
      recTaskBusy = true;
      if (recPos + CHUNK <= recMaxSamples) {
        if (M5.Mic.isRecording() < 2) {
          if (M5.Mic.record(pcmRec + recPos, CHUNK, SAMPLE_RATE)) recPos += CHUNK;
        } else {
          recTaskBusy = false;
          vTaskDelay(2);
          continue;
        }
      } else {
        recActive = false;
        recFull = true;
      }
      recTaskBusy = false;
    } else {
      vTaskDelay(5);
    }
  }
}

bool canTalk(char* reason, size_t n) {
  if (WiFi.status() != WL_CONNECTED) { snprintf(reason, n, "Wi-Fi disconnected"); return false; }
  if (selIp == MASTER_KEY) { snprintf(reason, n, "Master channel is listen-only"); return false; }
  if (selIp == 0 && onlineCount() == 0) { snprintf(reason, n, "No other Tab5 online yet"); return false; }
  if (selIp != 0) {
    Peer* p = findPeer(selIp);
    if (!peerOnline(p)) { snprintf(reason, n, "%s is offline", p ? p->name : "Device"); return false; }
  }
  uint32_t cd = QOS[myQos].cooldownSec * 1000UL;
  if (lastSendMs && millis() - lastSendMs < cd) {
    snprintf(reason, n, "Cooldown %lus (%s level)",
             (unsigned long)((cd - (millis() - lastSendMs)) / 1000 + 1), QOS[myQos].name);
    return false;
  }
  if (selIp == 0) snprintf(reason, n, "Hold to talk to all (%d online)", onlineCount());
  else            snprintf(reason, n, "Hold to talk to %s", findPeer(selIp)->name);
  return true;
}

void tryStartRecording() {
  char why[64];
  if (!canTalk(why, sizeof(why))) { toast("%s", why); return; }
  stopPlayback();
  M5.Speaker.end();
  M5.Mic.begin();
  recPos        = 0;
  recFull       = false;
  recMaxSamples = (uint32_t)QOS[myQos].maxSec * SAMPLE_RATE;
  recStartMs    = millis();
  recording     = true;
  recActive     = true;
  dirty         = true;
}

void stopRecordingAndSend() {
  recActive = false;
  recording = false;
  delay(20);
  uint32_t t0 = millis();
  while ((recTaskBusy || M5.Mic.isRecording()) && millis() - t0 < 600) delay(1);
  M5.Mic.end();
  M5.Speaker.begin();
  M5.Speaker.setVolume(volume);

  uint32_t n = recPos;
  uint32_t held = millis() - recStartMs;
  if (held < 500 || n < SAMPLE_RATE / 3) { toast("Hold the button longer to record"); return; }

  conditionAudio(pcmRec, n);
  uint8_t* data = (uint8_t*)heap_caps_malloc((n + 1) / 2, MALLOC_CAP_SPIRAM);
  if (!data) { toast("Out of memory"); return; }
  size_t len = adpcmEncode(pcmRec, n, data);

  VoiceMsg m = {};
  m.id = nextMsgId++;
  m.outgoing = true;
  m.broadcast = (selIp == 0);
  m.seen = true;
  m.qos = myQos;
  m.status = ST_SENDING;
  m.ip = selIp;
  strlcpy(m.name, myName, sizeof(m.name));
  m.data = data;
  m.dataLen = len;
  m.numSamples = n;
  m.sampleRate = SAMPLE_RATE;
  m.ts = nowTs();
  m.ms = millis();
  computeEnvelope(pcmRec, n, m.env);
  uint32_t id = m.id;
  addMsg(m);
  chatScroll = 0;
  drawAll();                                  // show the bubble with a clock

  VoiceHeader h = {};
  memcpy(h.magic, "TWKV", 4);
  h.version = PROTO_VERSION;
  h.qos = myQos;
  h.codec = 1;
  h.flags = (selIp == 0) ? 1 : 0;
  h.sampleRate = SAMPLE_RATE;
  h.numSamples = n;
  h.payloadLen = len;
  strlcpy(h.name, myName, sizeof(h.name));
  memcpy(h.env, m.env, ENV_BARS);
  h.auth = 0;

  int targets = 0, ok = 0;
  if (selIp == 0) {
    for (auto& p : peers) if (peerOnline(&p)) { targets++; if (sendVoice(p.ip, h, data, len)) ok++; }
  } else {
    targets = 1;
    if (sendVoice(selIp, h, data, len)) ok = 1;
  }

  VoiceMsg* pm = findMsg(id);
  if (pm) {
    pm->targets = targets;
    pm->okCount = ok;
    pm->status = (targets > 0 && ok == targets) ? ST_DELIVERED : (ok == 0 ? ST_FAILED : ST_PARTIAL);
  }
  if (ok == 0) toast("Message not delivered");
  else if (ok < targets) toast("Delivered to %d of %d devices", ok, targets);
  lastSendMs = millis();
  dirty = true;
}

// =============================== INCOMING ===================================
void handleIncoming(Incoming* in) {
  if (in->h.qos != QOS_MASTER) upsertPeer(in->ip, in->h.name, in->h.qos);  // master is not a contact

  VoiceMsg m = {};
  m.id = nextMsgId++;
  m.outgoing = false;
  m.broadcast = in->h.flags & 1;
  m.qos = in->h.qos;
  m.ip = in->ip;
  strlcpy(m.name, in->h.name, sizeof(m.name));
  m.data = in->data;
  m.dataLen = in->h.payloadLen;
  m.numSamples = in->h.numSamples;
  m.sampleRate = in->h.sampleRate;
  m.ts = nowTs();
  m.ms = millis();
  memcpy(m.env, in->h.env, ENV_BARS);
  delete in;

  uint32_t key = convKey(m);
  if (m.qos >= QOS_HIGH && uiMode == UI_MAIN && !recording && key != selIp) selectConv(key);
  m.seen = (key == selIp);
  if (key == selIp) chatScroll = 0;
  addMsg(m);
  if (m.qos == QOS_MASTER) toast("MASTER message (%s)", m.broadcast ? "to all" : "direct");
  else                     toast("%s voice from %s", QOS[m.qos].name, m.name);
  enqueuePlay(m.id, m.qos);
  dirty = true;
}

// ================================ DRAWING ===================================
int drawQosBadge(int x, int cy, uint8_t q, bool small = true) {
  canvas.setFont(small ? &fonts::DejaVu12 : &fonts::DejaVu18);
  int tw = canvas.textWidth(QOS[q].name) + 16;
  int bh = small ? 22 : 30;
  canvas.fillSmoothRoundRect(x, cy - bh / 2, tw, bh, bh / 2, QOS[q].color);
  canvas.setTextColor(COL_WHITE);
  canvas.setTextDatum(middle_center);
  canvas.drawString(QOS[q].name, x + tw / 2, cy + 1);
  return tw;
}

void drawMasterAvatar(int cx, int cy, int r) {
  canvas.fillSmoothCircle(cx, cy, r, COL_MASTER);
  // small crown
  int w = r, top = cy - r / 3, base = cy + r / 4;
  canvas.fillTriangle(cx - w / 2, base, cx - w / 2, top, cx - w / 6, base, COL_GOLD);
  canvas.fillTriangle(cx - w / 4, base, cx, top - 4, cx + w / 4, base, COL_GOLD);
  canvas.fillTriangle(cx + w / 2, base, cx + w / 2, top, cx + w / 6, base, COL_GOLD);
  canvas.fillRect(cx - w / 2, base, w, 6, COL_GOLD);
}

void drawAvatar(int cx, int cy, int r, const char* name, bool all) {
  canvas.fillSmoothCircle(cx, cy, r, all ? COL_TEAL_DARK : avatarColor(name));
  canvas.setTextColor(COL_WHITE);
  canvas.setTextDatum(middle_center);
  if (all) { canvas.setFont(&fonts::DejaVu18); canvas.drawString("ALL", cx, cy + 1); }
  else {
    char ini[3]; initials(name, ini);
    canvas.setFont(r >= 28 ? &fonts::DejaVu24 : &fonts::DejaVu18);
    canvas.drawString(ini, cx, cy + 1);
  }
}

void drawGear(int cx, int cy, uint16_t c, uint16_t bg) {
  for (int k = 0; k < 8; k++) {
    float a = k * PI / 4;
    canvas.fillSmoothCircle(cx + cosf(a) * 15, cy + sinf(a) * 15, 5, c);
  }
  canvas.fillSmoothCircle(cx, cy, 14, c);
  canvas.fillSmoothCircle(cx, cy, 6, bg);
}

void drawMic(int cx, int cy, uint16_t c) {
  canvas.fillSmoothRoundRect(cx - 9, cy - 24, 18, 30, 9, c);
  canvas.fillArc(cx, cy - 6, 18, 15, 0, 180, c);
  canvas.fillRect(cx - 2, cy + 12, 4, 9, c);
  canvas.fillRect(cx - 10, cy + 20, 20, 4, c);
}

void drawTick(int x, int y, uint16_t c) {
  canvas.drawWideLine(x, y, x + 5, y + 5, 1.6f, c);
  canvas.drawWideLine(x + 5, y + 5, x + 14, y - 6, 1.6f, c);
}

const VoiceMsg* lastMsgOf(uint32_t key) {
  for (int i = msgCount - 1; i >= 0; i--) if (convKey(msgs[i]) == key) return &msgs[i];
  return nullptr;
}

int unreadOf(uint32_t key) {
  int n = 0;
  for (int i = 0; i < msgCount; i++) if (convKey(msgs[i]) == key && !msgs[i].outgoing && !msgs[i].seen) n++;
  return n;
}

void drawRow(int y, uint32_t key, const char* name, bool all, const Peer* p) {
  bool sel = (selIp == key);
  canvas.fillRect(0, y, LW, ROW, sel ? COL_PANEL_SEL : COL_PANEL);
  if (key == MASTER_KEY) drawMasterAvatar(50, y + ROW / 2, 30);
  else                   drawAvatar(50, y + ROW / 2, 30, name, all);
  if (!all && peerOnline(p)) {
    canvas.fillSmoothCircle(72, y + ROW / 2 + 22, 9, COL_WHITE);
    canvas.fillSmoothCircle(72, y + ROW / 2 + 22, 6, COL_GREEN);
  }

  canvas.setFont(&fonts::DejaVu18);
  canvas.setTextColor(COL_TEXT);
  canvas.setTextDatum(middle_left);
  canvas.drawString(name, 96, y + 32);
  if (!all && p) drawQosBadge(96 + canvas.textWidth(name) + 10, y + 32, p->qos);

  // subtitle
  char sub[64];
  const VoiceMsg* lm = lastMsgOf(key);
  if (lm) {
    uint32_t s = lm->numSamples / lm->sampleRate;
    snprintf(sub, sizeof(sub), "%sVoice message %lu:%02lu", lm->outgoing ? "You: " : "",
             (unsigned long)(s / 60), (unsigned long)(s % 60));
  } else if (key == MASTER_KEY) {
    snprintf(sub, sizeof(sub), "Listen-only channel (master app)");
  } else if (all) {
    snprintf(sub, sizeof(sub), "Broadcast - %d online", onlineCount());
  } else if (peerOnline(p)) {
    snprintf(sub, sizeof(sub), "online - %s", IPAddress(p->ip).toString().c_str());
  } else {
    snprintf(sub, sizeof(sub), "offline");
  }
  canvas.setFont(&fonts::DejaVu12);
  canvas.setTextColor(COL_TEXT2);
  canvas.drawString(sub, 96, y + 66);

  int unread = unreadOf(key);
  if (lm) {
    char t[12]; formatTime(*lm, t, sizeof(t));
    canvas.setTextDatum(middle_right);
    canvas.setTextColor(unread ? COL_GREEN : COL_TEXT2);
    canvas.drawString(t, LW - 18, y + 30);
  }
  if (unread) {
    char u[6]; snprintf(u, sizeof(u), "%d", unread);
    canvas.fillSmoothCircle(LW - 32, y + 66, 14, COL_GREEN);
    canvas.setTextColor(COL_WHITE);
    canvas.setTextDatum(middle_center);
    canvas.drawString(u, LW - 32, y + 67);
  }
  canvas.drawFastHLine(96, y + ROW - 1, LW - 96, COL_DIV);

  if (rowHitCount < MAX_PEERS + 1) {
    rowHits[rowHitCount].r = {0, (int16_t)y, (int16_t)LW, (int16_t)ROW};
    rowHits[rowHitCount].ip = key;
    rowHitCount++;
  }
}

void drawLeftPanel() {
  canvas.fillRect(0, 0, LW, H, COL_PANEL);

  // header : my identity
  canvas.fillRect(0, 0, LW, HDR, COL_TEAL);
  drawAvatar(50, HDR / 2, 30, myName, false);
  canvas.setFont(&fonts::DejaVu24);
  canvas.setTextColor(COL_WHITE);
  canvas.setTextDatum(top_left);
  canvas.drawString(myName, 92, 14);
  drawQosBadge(92, 66, myQos);
  gearRect = {(int16_t)(LW - 84), 4, 80, (int16_t)(HDR - 8)};
  drawGear(LW - 44, HDR / 2, COL_WHITE, COL_TEAL);

  // info strip
  canvas.fillRect(0, HDR, LW, 40, COL_PANEL_SEL);
  char info[64];
  snprintf(info, sizeof(info), "IP %s   |   %d online",
           WiFi.localIP().toString().c_str(), onlineCount());
  canvas.setFont(&fonts::DejaVu12);
  canvas.setTextColor(COL_TEXT2);
  canvas.setTextDatum(middle_left);
  canvas.drawString(info, 16, HDR + 20);

  // contact list (scrollable)
  int listTop = HDR + 40;
  int nRows = 2;
  for (auto& p : peers) if (p.used) nRows++;
  int maxScroll = max(0, nRows * ROW - (H - listTop));
  listScroll = constrain(listScroll, 0, maxScroll);

  rowHitCount = 0;
  canvas.setClipRect(0, listTop, LW, H - listTop);
  int y = listTop - listScroll;
  drawRow(y, 0, "All devices", true, nullptr);
  y += ROW;
  drawRow(y, MASTER_KEY, "Master", true, nullptr);
  y += ROW;
  for (auto& p : peers) {                     // online first, then offline
    if (p.used && peerOnline(&p)) { if (y + ROW > listTop && y < H) drawRow(y, p.ip, p.name, false, &p); y += ROW; }
  }
  for (auto& p : peers) {
    if (p.used && !peerOnline(&p)) { if (y + ROW > listTop && y < H) drawRow(y, p.ip, p.name, false, &p); y += ROW; }
  }
  canvas.clearClipRect();
  canvas.drawFastVLine(LW - 1, 0, H, COL_DIV);
}

bool showSenderIn(const VoiceMsg& m) { return !m.outgoing && (selIp == 0 || selIp == MASTER_KEY); }
int bubbleH(const VoiceMsg& m) { return showSenderIn(m) ? 108 : 86; }

void drawBubble(const VoiceMsg& m, int x, int y, int bw, int bh, bool showSender) {
  uint16_t bg = m.outgoing ? COL_BUBBLE_OUT : COL_BUBBLE_IN;
  canvas.fillSmoothRoundRect(x, y, bw, bh, 14, bg);
  if (m.outgoing) canvas.fillTriangle(x + bw - 16, y, x + bw + 10, y, x + bw - 16, y + 18, bg);
  else            canvas.fillTriangle(x + 16, y, x - 10, y, x + 16, y + 18, bg);

  int top = y;
  if (showSender) {
    canvas.setFont(&fonts::DejaVu12);
    canvas.setTextDatum(top_left);
    if (m.qos == QOS_MASTER) {
      char lbl[48];
      snprintf(lbl, sizeof(lbl), "%s  -  %s", m.name, m.broadcast ? "to all" : "direct to you");
      canvas.setTextColor(COL_MASTER);
      canvas.drawString(lbl, x + 18, y + 8);
    } else {
      canvas.setTextColor(avatarColor(m.name));
      canvas.drawString(m.name, x + 18, y + 8);
    }
    top += 22;
  }

  // play / pause button
  bool isPlaying = (playingId == m.id);
  int pcx = x + 44, pcy = top + 36;
  canvas.fillSmoothCircle(pcx, pcy, 24, (m.outgoing || m.played) ? COL_TEAL : COL_GREEN);
  if (isPlaying) {
    canvas.fillRect(pcx - 9, pcy - 10, 6, 20, COL_WHITE);
    canvas.fillRect(pcx + 3, pcy - 10, 6, 20, COL_WHITE);
  } else {
    canvas.fillTriangle(pcx - 7, pcy - 11, pcx - 7, pcy + 11, pcx + 12, pcy, COL_WHITE);
  }

  // waveform with playback progress
  float prog = isPlaying ? min(1.0f, (millis() - playStartMs) / (float)playDurMs) : 0.0f;
  for (int i = 0; i < ENV_BARS; i++) {
    int hgt = 4 + m.env[i] * 32 / 255;
    int bx = x + 82 + i * 10;
    uint16_t c = (isPlaying && i < prog * ENV_BARS) ? COL_TICK
               : ((!m.outgoing && !m.played) ? COL_GREEN : COL_TEXT2);
    canvas.fillSmoothRoundRect(bx, pcy - hgt / 2, 5, hgt, 2, c);
  }

  // duration + QoS badge
  uint32_t s = m.numSamples / m.sampleRate;
  char dur[10]; snprintf(dur, sizeof(dur), "%lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
  canvas.setFont(&fonts::DejaVu12);
  canvas.setTextColor(COL_TEXT2);
  canvas.setTextDatum(bottom_left);
  canvas.drawString(dur, x + 82, y + bh - 8);
  drawQosBadge(x + 132, y + bh - 17, m.qos);

  // time + delivery status
  char t[12]; formatTime(m, t, sizeof(t));
  int tx = x + bw - 14;
  if (m.outgoing) {
    int ix = x + bw - 36, iy = y + bh - 16;
    switch (m.status) {
      case ST_SENDING:
        canvas.drawCircle(ix + 8, iy, 7, COL_TEXT2);
        canvas.drawLine(ix + 8, iy, ix + 8, iy - 5, COL_TEXT2);
        canvas.drawLine(ix + 8, iy, ix + 12, iy, COL_TEXT2);
        break;
      case ST_DELIVERED: drawTick(ix, iy, COL_TICK); drawTick(ix + 7, iy, COL_TICK); break;
      case ST_PARTIAL:   drawTick(ix, iy, COL_TEXT2); drawTick(ix + 7, iy, COL_TEXT2); break;
      default:
        canvas.fillSmoothCircle(ix + 8, iy, 9, COL_RED);
        canvas.setTextColor(COL_WHITE);
        canvas.setTextDatum(middle_center);
        canvas.drawString("!", ix + 8, iy + 1);
        break;
    }
    tx = ix - 6;
  }
  canvas.setFont(&fonts::DejaVu12);
  canvas.setTextColor(COL_TEXT2);
  canvas.setTextDatum(bottom_right);
  canvas.drawString(t, tx, y + bh - 8);
}

void drawMessages() {
  int top = HDR, bottom = H - BOT;
  bubbleHitCount = 0;

  int idx[MAX_MSGS], cnt = 0;
  for (int i = 0; i < msgCount; i++) if (convKey(msgs[i]) == selIp) idx[cnt++] = i;

  if (!cnt) {
    const char* txt = "No messages yet - hold the green button to talk";
    canvas.setFont(&fonts::DejaVu12);
    int tw = canvas.textWidth(txt) + 40;
    int cx = LW + (W - LW) / 2;
    canvas.fillSmoothRoundRect(cx - tw / 2, top + 30, tw, 40, 10, COL_NOTICE);
    canvas.setTextColor(COL_TEXT2);
    canvas.setTextDatum(middle_center);
    canvas.drawString(txt, cx, top + 51);
    return;
  }

  int total = 0;
  for (int k = 0; k < cnt; k++) total += bubbleH(msgs[idx[k]]) + 14;
  int maxScroll = max(0, total - (bottom - top - 24));
  chatScroll = constrain(chatScroll, 0, maxScroll);

  const int bw = 420;
  canvas.setClipRect(LW, top, W - LW, bottom - top);
  int y = bottom - 12 + chatScroll;
  for (int k = cnt - 1; k >= 0; k--) {
    const VoiceMsg& m = msgs[idx[k]];
    int bh = bubbleH(m);
    y -= bh;
    if (y + bh > top && y < bottom) {
      int x = m.outgoing ? W - 30 - bw : LW + 30;
      drawBubble(m, x, y, bw, bh, showSenderIn(m));
      if (bubbleHitCount < 16) {
        int ry = max(y, top), rb = min(y + bh, bottom);
        bubbleHits[bubbleHitCount].r = {(int16_t)x, (int16_t)ry, (int16_t)bw, (int16_t)(rb - ry)};
        bubbleHits[bubbleHitCount].id = m.id;
        bubbleHitCount++;
      }
    }
    y -= 14;
    if (y + 200 < top) break;
  }
  canvas.clearClipRect();
}

void drawBottomBar() {
  int by = H - BOT;
  canvas.fillRect(LW, by, W - LW, BOT, COL_PANEL_SEL);

  int px = LW + 20, py = by + 20, ph = BOT - 40, pw = W - LW - 20 - 150;
  canvas.fillSmoothRoundRect(px, py, pw, ph, ph / 2, COL_WHITE);

  char why[64];
  bool ok = canTalk(why, sizeof(why));

  if (recording) {
    if ((millis() / 400) % 2) canvas.fillSmoothCircle(px + 36, py + ph / 2, 10, COL_RED);
    uint32_t el = (millis() - recStartMs) / 1000;
    char t[24];
    snprintf(t, sizeof(t), "%lu:%02lu / 0:%02u", (unsigned long)(el / 60), (unsigned long)(el % 60),
             QOS[myQos].maxSec);
    canvas.setFont(&fonts::DejaVu24);
    canvas.setTextColor(COL_TEXT);
    canvas.setTextDatum(middle_left);
    canvas.drawString(t, px + 60, py + ph / 2);

    // live level meter (reads a chunk that is already complete)
    int level = 0;
    uint32_t rp = recPos;
    if (rp >= 3 * CHUNK) {
      int peak = 0;
      for (uint32_t i = rp - 3 * CHUNK; i < rp - 2 * CHUNK; i++) peak = max(peak, abs((int)pcmRec[i]));
      level = min(20, (int)(sqrtf(peak / 32768.0f) * 28));
    }
    int mx = px + 280;
    for (int i = 0; i < 20; i++) {
      uint16_t c = i < level ? (i < 12 ? COL_GREEN : (i < 17 ? COL_ORANGE : COL_RED)) : COL_DIV;
      canvas.fillSmoothRoundRect(mx + i * 12, py + ph / 2 - 12, 8, 24, 3, c);
    }
    canvas.setFont(&fonts::DejaVu12);
    canvas.setTextColor(COL_TEXT2);
    canvas.setTextDatum(middle_right);
    canvas.drawString("Release to send", px + pw - 26, py + ph / 2);
  } else {
    canvas.setFont(&fonts::DejaVu18);
    canvas.setTextColor(ok ? COL_TEXT2 : COL_RED);
    canvas.setTextDatum(middle_left);
    canvas.drawString(why, px + 28, py + ph / 2);
    char lim[24];
    snprintf(lim, sizeof(lim), "max %u s", QOS[myQos].maxSec);
    canvas.setFont(&fonts::DejaVu12);
    canvas.setTextColor(COL_TEXT2);
    canvas.setTextDatum(middle_right);
    canvas.drawString(lim, px + pw - 26, py + ph / 2);
  }

  // Push-To-Talk button
  int cx = W - 75, cy = by + BOT / 2;
  if (recording) canvas.fillSmoothCircle(cx, cy, 58 + (millis() / 120) % 6, COL_RED_HALO);
  int r = recording ? 50 : 44;
  uint16_t c = recording ? COL_RED : (ok ? COL_GREEN : COL_TEXT2);
  canvas.fillSmoothCircle(cx, cy, r, c);
  drawMic(cx, cy + 2, COL_WHITE);
  pttRect = {(int16_t)(cx - 65), (int16_t)by, 130, (int16_t)BOT};
}

void drawRightPanel() {
  // header
  canvas.fillRect(LW, 0, W - LW, HDR, COL_TEAL);
  char title[32], sub[80];
  if (selIp == MASTER_KEY) {
    drawMasterAvatar(LW + 52, HDR / 2, 30);
    snprintf(title, sizeof(title), "Master");
    snprintf(sub, sizeof(sub), "Messages from the master app - listen-only");
  } else if (selIp == 0) {
    drawAvatar(LW + 52, HDR / 2, 30, "ALL", true);
    snprintf(title, sizeof(title), "All devices");
    snprintf(sub, sizeof(sub), "Broadcast to %d online Tab5", onlineCount());
  } else {
    Peer* p = findPeer(selIp);
    const char* nm = p ? p->name : "?";
    drawAvatar(LW + 52, HDR / 2, 30, nm, false);
    strlcpy(title, nm, sizeof(title));
    if (peerOnline(p)) snprintf(sub, sizeof(sub), "online - level %s - %s", QOS[p->qos].name,
                                IPAddress(p->ip).toString().c_str());
    else if (p)        snprintf(sub, sizeof(sub), "offline - last seen %lus ago",
                                (unsigned long)((millis() - p->lastSeen) / 1000));
    else               snprintf(sub, sizeof(sub), "unknown");
  }
  canvas.setFont(&fonts::DejaVu24);
  canvas.setTextColor(COL_WHITE);
  canvas.setTextDatum(top_left);
  canvas.drawString(title, LW + 96, 16);
  canvas.setFont(&fonts::DejaVu12);
  canvas.drawString(sub, LW + 96, 58);

  // chat wallpaper
  canvas.fillRect(LW, HDR, W - LW, H - HDR - BOT, COL_CHAT_BG);
  for (int y = HDR + 18, row = 0; y < H - BOT; y += 36, row++)
    for (int x = LW + 18 + (row % 2) * 18; x < W; x += 36)
      canvas.fillCircle(x, y, 2, COL_PATTERN);

  drawMessages();
  drawBottomBar();
}

void dimBackground() {
  for (int y = 0; y < H; y += 2) canvas.drawFastHLine(0, y, W, COL_DARK);
}

void drawSettings() {
  dimBackground();
  int pw = 820, ph = 600, px = (W - pw) / 2, py = (H - ph) / 2;
  canvas.fillSmoothRoundRect(px, py, pw, ph, 20, COL_WHITE);
  canvas.fillSmoothRoundRect(px, py, pw, 80, 20, COL_TEAL);
  canvas.fillRect(px, py + 60, pw, 20, COL_TEAL);
  canvas.setFont(&fonts::DejaVu24);
  canvas.setTextColor(COL_WHITE);
  canvas.setTextDatum(middle_left);
  canvas.drawString("Settings", px + 30, py + 40);
  closeRect = {(int16_t)(px + pw - 80), (int16_t)py, 80, 80};
  canvas.drawWideLine(px + pw - 52, py + 28, px + pw - 28, py + 52, 2.5f, COL_WHITE);
  canvas.drawWideLine(px + pw - 28, py + 28, px + pw - 52, py + 52, 2.5f, COL_WHITE);

  char info[96];
  snprintf(info, sizeof(info), "Device %s   |   IP %s   |   IMA-ADPCM %d kHz (%d kB/s)",
           myName, WiFi.localIP().toString().c_str(), SAMPLE_RATE / 1000, SAMPLE_RATE / 2000);
  canvas.setFont(&fonts::DejaVu12);
  canvas.setTextColor(COL_TEXT2);
  canvas.setTextDatum(top_left);
  canvas.drawString(info, px + 30, py + 96);

  canvas.setFont(&fonts::DejaVu18);
  canvas.setTextColor(COL_TEXT);
  canvas.drawString("User level (QoS)", px + 30, py + 128);

  const int cw = 240, ch = 205, gap = 20;
  for (int i = 0; i < 3; i++) {
    int x = px + 30 + i * (cw + gap), y = py + 162;
    qosRect[i] = {(int16_t)x, (int16_t)y, (int16_t)cw, (int16_t)ch};
    bool sel = (i == myQos);
    canvas.fillSmoothRoundRect(x, y, cw, ch, 14, sel ? COL_PANEL_SEL : COL_WHITE);
    if (sel) for (int k = 0; k < 3; k++) canvas.drawRoundRect(x + k, y + k, cw - 2 * k, ch - 2 * k, 14, QOS[i].color);
    else     canvas.drawRoundRect(x, y, cw, ch, 14, COL_DIV);
    canvas.fillSmoothRoundRect(x + 12, y + 12, cw - 24, 44, 10, QOS[i].color);
    canvas.setFont(&fonts::DejaVu18);
    canvas.setTextColor(COL_WHITE);
    canvas.setTextDatum(middle_center);
    canvas.drawString(QOS[i].name, x + cw / 2, y + 35);

    char l[4][40];
    snprintf(l[0], 40, "Max message : %u s", QOS[i].maxSec);
    if (QOS[i].cooldownSec) snprintf(l[1], 40, "Cooldown    : %u s", QOS[i].cooldownSec);
    else                    snprintf(l[1], 40, "Cooldown    : none");
    snprintf(l[2], 40, "Playback    : %s", QOS[i].playback);
    snprintf(l[3], 40, "Wi-Fi       : %s", QOS[i].wmm);
    canvas.setFont(&fonts::DejaVu12);
    canvas.setTextColor(COL_TEXT);
    canvas.setTextDatum(top_left);
    for (int k = 0; k < 4; k++) canvas.drawString(l[k], x + 16, y + 70 + k * 24);

    canvas.setTextDatum(middle_center);
    if (sel)           { canvas.setTextColor(QOS[i].color); canvas.drawString("CURRENT", x + cw / 2, y + ch - 18); }
    else if (i > myQos){ canvas.setTextColor(COL_TEXT2);    canvas.drawString("PIN required", x + cw / 2, y + ch - 18); }
  }

  // volume
  int vy = py + 410;
  canvas.setFont(&fonts::DejaVu18);
  canvas.setTextColor(COL_TEXT);
  canvas.setTextDatum(middle_left);
  canvas.drawString("Speaker volume", px + 30, vy);
  volMinus = {(int16_t)(px + 260), (int16_t)(vy - 30), 60, 60};
  volPlus  = {(int16_t)(px + 660), (int16_t)(vy - 30), 60, 60};
  canvas.fillSmoothCircle(px + 290, vy, 26, COL_PANEL_SEL);
  canvas.fillSmoothCircle(px + 690, vy, 26, COL_PANEL_SEL);
  canvas.fillRect(px + 278, vy - 2, 24, 4, COL_TEXT);
  canvas.fillRect(px + 678, vy - 2, 24, 4, COL_TEXT);
  canvas.fillRect(px + 688, vy - 12, 4, 24, COL_TEXT);
  canvas.fillSmoothRoundRect(px + 340, vy - 7, 300, 14, 7, COL_DIV);
  canvas.fillSmoothRoundRect(px + 340, vy - 7, 300 * volume / 255, 14, 7, COL_GREEN);

  canvas.setFont(&fonts::DejaVu12);
  canvas.setTextColor(COL_TEXT2);
  canvas.setTextDatum(top_left);
  canvas.drawString("HIGH messages interrupt lower playback and open the conversation automatically.",
                    px + 30, py + 462);
  canvas.drawString("Lowering your level is free; raising it requires the admin PIN.", px + 30, py + 486);

  doneRect = {(int16_t)(px + pw - 220), (int16_t)(py + ph - 82), 190, 60};
  canvas.fillSmoothRoundRect(doneRect.x, doneRect.y, doneRect.w, doneRect.h, 30, COL_GREEN);
  canvas.setFont(&fonts::DejaVu18);
  canvas.setTextColor(COL_WHITE);
  canvas.setTextDatum(middle_center);
  canvas.drawString("Done", doneRect.x + doneRect.w / 2, doneRect.y + doneRect.h / 2);
}

void drawPin() {
  dimBackground();
  int pw = 460, ph = 660, px = (W - pw) / 2, py = (H - ph) / 2;
  canvas.fillSmoothRoundRect(px, py, pw, ph, 20, COL_WHITE);
  canvas.setFont(&fonts::DejaVu24);
  canvas.setTextColor(COL_TEXT);
  canvas.setTextDatum(middle_center);
  canvas.drawString("Admin PIN", W / 2, py + 40);
  char sub[48];
  snprintf(sub, sizeof(sub), "Required to switch to %s", QOS[pendingQos].name);
  canvas.setFont(&fonts::DejaVu12);
  canvas.setTextColor(COL_TEXT2);
  canvas.drawString(sub, W / 2, py + 76);

  int len = strlen(pinBuf);
  for (int i = 0; i < 4; i++) {
    int cx = W / 2 + (int)((i - 1.5f) * 44);
    if (i < len) canvas.fillSmoothCircle(cx, py + 120, 12, COL_TEAL);
    else         canvas.drawCircle(cx, py + 120, 12, COL_TEXT2);
  }

  const int kw = 120, kh = 86, gap = 16;
  int kx0 = W / 2 - (3 * kw + 2 * gap) / 2, ky0 = py + 160;
  for (int k = 0; k < 12; k++) {
    int x = kx0 + (k % 3) * (kw + gap), y = ky0 + (k / 3) * (kh + gap);
    keyRect[k] = {(int16_t)x, (int16_t)y, (int16_t)kw, (int16_t)kh};
    char c = KEYS[k];
    uint16_t bg = (c == 'K') ? COL_GREEN : COL_PANEL_SEL;
    canvas.fillSmoothRoundRect(x, y, kw, kh, 16, bg);
    canvas.setFont(&fonts::DejaVu24);
    canvas.setTextColor(c == 'K' ? COL_WHITE : (c == 'C' ? COL_RED : COL_TEXT));
    char lbl[3] = {c, 0, 0};
    if (c == 'K') { lbl[0] = 'O'; lbl[1] = 'K'; }
    canvas.drawString(lbl, x + kw / 2, y + kh / 2);
  }
  cancelRect = {(int16_t)(W / 2 - 100), (int16_t)(py + ph - 70), 200, 56};
  canvas.setFont(&fonts::DejaVu18);
  canvas.setTextColor(COL_TEXT2);
  canvas.drawString("Cancel", W / 2, cancelRect.y + 28);
}

void drawToast() {
  canvas.setFont(&fonts::DejaVu18);
  int tw = canvas.textWidth(toastText) + 48;
  int cx = (uiMode == UI_MAIN) ? LW + (W - LW) / 2 : W / 2;
  int ty = (uiMode == UI_MAIN) ? HDR + 16 : 8;
  canvas.fillSmoothRoundRect(cx - tw / 2, ty, tw, 48, 24, COL_DARK);
  canvas.setTextColor(COL_WHITE);
  canvas.setTextDatum(middle_center);
  canvas.drawString(toastText, cx, ty + 25);
}

void drawAll() {
  lastDraw = millis();
  dirty = false;
  drawLeftPanel();
  drawRightPanel();
  if (uiMode == UI_SETTINGS) drawSettings();
  else if (uiMode == UI_PIN) drawPin();
  if (toastUntil) drawToast();
  canvas.pushSprite(0, 0);
}

// ================================= TOUCH ====================================
void applyQos(uint8_t q) {
  myQos = q;
  prefs.putUChar("qos", q);
  sendBeacon();
  toast("Your level is now %s", QOS[q].name);
}

void requestQos(uint8_t q) {
  if (q == myQos) return;
  if (q > myQos) { pendingQos = q; pinBuf[0] = 0; uiMode = UI_PIN; dirty = true; }
  else applyQos(q);
}

void handleTouch() {
  auto t = M5.Touch.getDetail();

  if (recording && !t.isPressed()) { stopRecordingAndSend(); return; }

  if (uiMode == UI_MAIN) {
    if (t.wasPressed() && pttRect.hit(t.x, t.y)) { tryStartRecording(); return; }
    if (recording) return;

    if (t.isDragging()) {
      if (t.x < LW && t.y > HDR + 40)          { listScroll -= t.deltaY(); dirty = true; }
      else if (t.x >= LW && t.y > HDR && t.y < H - BOT) { chatScroll += t.deltaY(); dirty = true; }
    }

    if (t.wasClicked()) {
      int x = t.x, y = t.y;
      if (gearRect.hit(x, y)) { uiMode = UI_SETTINGS; dirty = true; return; }
      if (x < LW && y > HDR + 40) {
        for (int i = 0; i < rowHitCount; i++)
          if (rowHits[i].r.hit(x, y)) { selectConv(rowHits[i].ip); return; }
      }
      for (int i = 0; i < bubbleHitCount; i++) {
        if (bubbleHits[i].r.hit(x, y)) {
          if (playingId == bubbleHits[i].id) stopPlayback();
          else startPlay(findMsg(bubbleHits[i].id));
          return;
        }
      }
    }
  }
  else if (uiMode == UI_SETTINGS) {
    if (!t.wasClicked()) return;
    int x = t.x, y = t.y;
    if (closeRect.hit(x, y) || doneRect.hit(x, y)) { uiMode = UI_MAIN; dirty = true; return; }
    for (int i = 0; i < 3; i++) if (qosRect[i].hit(x, y)) { requestQos(i); return; }
    if (volMinus.hit(x, y) || volPlus.hit(x, y)) {
      int v = volume + (volPlus.hit(x, y) ? 25 : -25);
      volume = constrain(v, 0, 255);
      M5.Speaker.setVolume(volume);
      M5.Speaker.tone(1000, 80);
      prefs.putUChar("vol", volume);
      dirty = true;
    }
  }
  else if (uiMode == UI_PIN) {
    if (!t.wasClicked()) return;
    int x = t.x, y = t.y;
    if (cancelRect.hit(x, y)) { uiMode = UI_SETTINGS; dirty = true; return; }
    for (int k = 0; k < 12; k++) {
      if (!keyRect[k].hit(x, y)) continue;
      char c = KEYS[k];
      size_t len = strlen(pinBuf);
      if (c == 'C') pinBuf[0] = 0;
      else if (c == 'K') {
        if (!strcmp(pinBuf, ADMIN_PIN)) { applyQos(pendingQos); uiMode = UI_SETTINGS; }
        else toast("Wrong PIN");
        pinBuf[0] = 0;
      } else if (len < 6) { pinBuf[len] = c; pinBuf[len + 1] = 0; }
      dirty = true;
      return;
    }
  }
}

// ============================== SETUP / LOOP ================================
void splash(const char* line1, const char* line2) {
  canvas.fillScreen(COL_TEAL);
  drawMic(W / 2, H / 2 - 80, COL_WHITE);
  canvas.setTextColor(COL_WHITE);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::DejaVu40);
  canvas.drawString("TalkyWalky", W / 2, H / 2 + 10);
  canvas.setFont(&fonts::DejaVu18);
  canvas.drawString(line1, W / 2, H / 2 + 70);
  canvas.setFont(&fonts::DejaVu12);
  canvas.drawString(line2, W / 2, H / 2 + 105);
  canvas.pushSprite(0, 0);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  M5.Display.setRotation(SCREEN_ROTATION);
  W = M5.Display.width();
  H = M5.Display.height();

  canvas.setColorDepth(16);
  canvas.setPsram(true);
  canvas.createSprite(W, H);

  prefs.begin("twalky", false);
  myQos  = min<uint8_t>(prefs.getUChar("qos", QOS_MED), QOS_HIGH);
  volume = prefs.getUChar("vol", 180);

  pcmRec  = (int16_t*)heap_caps_malloc(MAX_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  pcmPlay = (int16_t*)heap_caps_malloc(MAX_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  if (!pcmRec || !pcmPlay) {
    splash("PSRAM allocation failed", "Enable PSRAM in the board settings");
    while (true) delay(1000);
  }

  auto mc = M5.Mic.config();
  mc.sample_rate = SAMPLE_RATE;
  M5.Mic.config(mc);
  M5.Mic.end();
  M5.Speaker.begin();
  M5.Speaker.setVolume(volume);

  splash("Connecting to Wi-Fi...", WIFI_SSID);
#if CONFIG_ESP_WIFI_REMOTE_ENABLED
  WiFi.setPins(SDIO2_CLK, SDIO2_CMD, SDIO2_D0, SDIO2_D1, SDIO2_D2, SDIO2_D3, SDIO2_RST);
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    if (millis() - t0 > 20000) {
      splash("Wi-Fi connection failed - retrying", WIFI_SSID);
      WiFi.disconnect();
      delay(500);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      t0 = millis();
    }
  }
  WiFi.setSleep(false);                       // low latency, reliable broadcasts

#ifdef DEVICE_NAME
  strlcpy(myName, DEVICE_NAME, sizeof(myName));
#else
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(myName, sizeof(myName), "TAB5-%02X%02X", mac[4], mac[5]);
#endif
  Serial.printf("TalkyWalky %s  IP %s  level %s\n", myName,
                WiFi.localIP().toString().c_str(), QOS[myQos].name);

  configTzTime(TZ_INFO, "pool.ntp.org", "time.google.com");
  udp.begin(UDP_PORT);

  inQueue = xQueueCreate(8, sizeof(Incoming*));
  xTaskCreatePinnedToCore(netTask, "twk_net", 8192, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(recTask, "twk_rec", 4096, nullptr, 6, nullptr, 1);

  sendBeacon();
  lastBeacon = millis();
  dirty = true;
}

void loop() {
  M5.update();
  handleTouch();

  if (recording && recFull) {                 // QoS max length reached
    toast("%s limit: %u s", QOS[myQos].name, QOS[myQos].maxSec);
    stopRecordingAndSend();
  }

  pollBeacons();

  Incoming* in;
  while (xQueueReceive(inQueue, &in, 0) == pdTRUE) handleIncoming(in);

  servicePlayback();

  uint32_t now = millis();
  if (now - lastBeacon > BEACON_MS) { lastBeacon = now; sendBeacon(); }
  if (toastUntil && now > toastUntil) { toastUntil = 0; dirty = true; }

  uint32_t interval = (recording || playingId) ? 120 : 1000;
  if (dirty || now - lastDraw > interval) drawAll();

  delay(2);
}
