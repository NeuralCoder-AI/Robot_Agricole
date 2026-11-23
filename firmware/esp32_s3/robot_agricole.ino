// =============================================================================
//  ROBOT AGRICOLE – Firmware ESP32-S3 – robot_agricole.ino
//  Référence : "Robot Agricole – Référence UART et architecture firmware v2"
//
//  Matériel : ESP32-S3, 8 moteurs via ponts BTS7960 (R_EN/L_EN communs sur
//             GPIO 38), commande par UART1 (GPIO 1/2) et USB CDC natif,
//             point d'accès WiFi + serveur HTTP (page web embarquée).
//
//  Architecture FreeRTOS double cœur :
//    - taskControl (cœur 1, prio 5) : boucle 200 Hz – watchdogs, TMAX,
//      minuteries, rampes, temps mort, PWM. Mesures PERF.
//    - taskUart    (cœur 1, prio 4) : lecture UART1 + USB, exécution des
//      commandes, émission de la télémétrie "#T" et des notifications "#D".
//    - taskWeb     (cœur 0, prio 2) : WiFi AP + serveur web + API.
//    - taskSave    (cœur 0, prio 1) : écriture des réglages en NVS.
//  Un seul mutex protège l'état partagé (moteurs + réglages).
//
//  Compilation : core Arduino-ESP32 3.x (API LEDC 3.x), carte
//  "ESP32S3 Dev Module", "USB CDC On Boot: Enabled".
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "webpage.h"

// =============================================================================
//  TYPES ET STRUCTURES
//  (déclarés avant toute fonction : le préprocesseur Arduino insère les
//   prototypes générés juste avant la première définition de fonction)
// =============================================================================

// Origine de la dernière commande reçue par un moteur (pour les watchdogs)
enum Source : uint8_t {
  SRC_NONE = 0,   // aucune commande reçue depuis le démarrage
  SRC_UART = 1,   // UART1 ou USB (même groupe : watchdog UWDT)
  SRC_WEB  = 2    // interface web (watchdog WDT)
};

// Résultat du parsing strict d'un nombre
enum ParseRes : uint8_t {
  P_OK    = 0,    // nombre valide
  P_ARG   = 1,    // vide ou non numérique      -> ERR:ARG
  P_RANGE = 2     // décimale là où un entier est exigé -> ERR:RANGE
};

// Réglages persistants d'un moteur (5 paramètres)
struct MotorCfg {
  uint8_t  inv;     // inversion de sens (0/1)
  uint8_t  minP;    // PWM min en % (0..90)
  uint8_t  maxP;    // PWM max en % (0..100)
  uint16_t ramp;    // rampe en %/s (1..1000)
  uint32_t tmaxMs;  // durée max de marche continue en ms (0 = illimité)
};

// État dynamique d'un moteur
struct MotorState {
  int16_t  target;          // commande cible (-100..100)
  float    current;         // commande courante après rampe (-100..100)
  Source   source;          // origine de la dernière commande
  uint32_t deadline;        // échéance (ms) de l'action temporisée, 0 = aucune
  uint32_t tmax_start;      // début (ms) de la marche continue en cours
  int8_t   prev_sign;       // signe de la cible au passage précédent (détection TMAX)
  uint32_t dead_time_until; // instant (ms) à partir duquel le sens opposé est autorisé
  int8_t   active_dir;      // sens physiquement attaché au canal LEDC : -1, 0, +1
  int8_t   last_dir;        // dernier sens physique ayant tourné (pour le temps mort)
};

// Réglages globaux persistants
struct GlobalCfg {
  uint32_t wdtMs;     // watchdog web (0 = inactif)
  uint32_t uwdtMs;    // watchdog UART/USB (0 = inactif)
  uint8_t  autosave;  // sauvegarde automatique (0/1)
};

// Instantané complet des réglages (copié sous mutex, écrit hors mutex)
struct CfgSnapshot {
  MotorCfg  m[NUM_MOTORS];
  GlobalCfg g;
  uint32_t  version;  // compteur de modification au moment de la copie
};

// Compteurs de performance de taskControl
struct PerfStats {
  uint32_t loops;        // passages de boucle depuis la dernière lecture
  uint64_t execSumUs;    // somme des durées d'exécution
  uint32_t execMaxUs;    // durée max d'un passage
  uint32_t periodMaxUs;  // période max observée entre deux passages (gigue)
};

// Tampon de réception d'une ligne de commande (un par port série)
struct LineBuf {
  char   buf[LINE_MAX + 2];
  size_t len;
  bool   overflow;       // ligne trop longue : rejetée par ERR:LONG au prochain \n
};

// =============================================================================
//  ÉTAT GLOBAL (protégé par g_mutex sauf mention contraire)
// =============================================================================

static const uint8_t PIN_RPWM[NUM_MOTORS] = PINS_RPWM;
static const uint8_t PIN_LPWM[NUM_MOTORS] = PINS_LPWM;

static MotorCfg   g_cfg[NUM_MOTORS];
static MotorState g_mot[NUM_MOTORS];
static GlobalCfg  g_glob;

static volatile bool g_armed = false;       // ponts alimentés (GPIO 38 haut)
static uint32_t  g_lastWebMs  = 0;          // dernière requête HTTP
static uint32_t  g_lastUartMs = 0;          // dernière ligne UART1/USB
static uint32_t  g_telemMs    = 0;          // période télémétrie (0 = arrêt, non sauvegardé)

// Gestion de la sauvegarde NVS
static uint32_t  g_cfgVersion   = 0;        // incrémenté à chaque modification de réglage
static bool      g_cfgDirty     = false;    // réglages modifiés non enregistrés
static uint32_t  g_dirtyAtMs    = 0;        // instant de la dernière modification
static bool      g_saveRequest  = false;    // SAVE / DEFAULTS / REBOOT demandés
static bool      g_saved        = false;    // NVS == réglages courants
static volatile uint32_t g_rebootAtMs = 0;  // 0 = pas de redémarrage programmé

// Synchronisation
static SemaphoreHandle_t g_mutex     = nullptr;
static QueueHandle_t     g_doneQueue = nullptr;   // notifications "#D n" (uint8_t)
static portMUX_TYPE      g_perfMux   = portMUX_INITIALIZER_UNLOCKED;
static PerfStats         g_perf      = {0, 0, 0, 0};

static Preferences g_prefs;
static WebServer   g_server(HTTP_PORT);

// Prototypes explicites des fonctions utilisant des types personnalisés
String handleCommand(const String& line, Source src);
static void doEstop();

// =============================================================================
//  UTILITAIRES
// =============================================================================

static inline int8_t sgn(float v) { return (v > 0.0f) ? 1 : ((v < 0.0f) ? -1 : 0); }
static inline int8_t sgnI(int v)  { return (v > 0) ? 1 : ((v < 0) ? -1 : 0); }

static inline int clampVel(long v) {
  if (v >  VEL_MAX) return  VEL_MAX;
  if (v < -VEL_MAX) return -VEL_MAX;
  return (int)v;
}

// Verrouillage du mutex (helpers lisibles)
static inline void lock()   { xSemaphoreTake(g_mutex, portMAX_DELAY); }
static inline void unlock() { xSemaphoreGive(g_mutex); }

// Formatage d'une durée en secondes sans décimales inutiles ("0", "2.5", "0.125")
static String fmtSeconds(uint32_t ms) {
  char buf[24];
  uint32_t s = ms / 1000UL;
  uint32_t r = ms % 1000UL;
  if (r == 0) {
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)s);
  } else {
    snprintf(buf, sizeof(buf), "%lu.%03lu", (unsigned long)s, (unsigned long)r);
    // Suppression des zéros de fin
    size_t n = strlen(buf);
    while (n > 0 && buf[n - 1] == '0') buf[--n] = '\0';
  }
  return String(buf);
}

// -----------------------------------------------------------------------------
//  Parsing strict
// -----------------------------------------------------------------------------

// Entier strict : [+-]digits. Une décimale ("2.5") -> P_RANGE, autre -> P_ARG.
static ParseRes parseIntStrict(const String& in, long& out) {
  String s = in;
  s.trim();
  if (s.length() == 0) return P_ARG;
  int i = 0;
  bool neg = false;
  if (s[0] == '+' || s[0] == '-') { neg = (s[0] == '-'); i = 1; }
  bool digits = false, dec = false;
  long v = 0;
  for (; i < (int)s.length(); i++) {
    char c = s[i];
    if (c >= '0' && c <= '9') {
      digits = true;
      if (!dec) { v = v * 10 + (c - '0'); if (v > 1000000000L) v = 1000000000L; }
    } else if (c == '.' && !dec) {
      dec = true;
    } else {
      return P_ARG;
    }
  }
  if (!digits) return P_ARG;
  if (dec) return P_RANGE;
  out = neg ? -v : v;
  return P_OK;
}

// Nombre réel strict : [+-]digits[.digits]
static ParseRes parseFloatStrict(const String& in, double& out) {
  String s = in;
  s.trim();
  if (s.length() == 0) return P_ARG;
  int i = 0;
  bool neg = false;
  if (s[0] == '+' || s[0] == '-') { neg = (s[0] == '-'); i = 1; }
  bool digits = false, dec = false;
  double v = 0.0, scale = 0.1;
  for (; i < (int)s.length(); i++) {
    char c = s[i];
    if (c >= '0' && c <= '9') {
      digits = true;
      if (!dec) v = v * 10.0 + (c - '0');
      else { v += (c - '0') * scale; scale *= 0.1; }
    } else if (c == '.' && !dec) {
      dec = true;
    } else {
      return P_ARG;
    }
  }
  if (!digits) return P_ARG;
  out = neg ? -v : v;
  return P_OK;
}

// Numéro de moteur 1..8 -> indice 0..7. false si absent / non numérique / hors plage.
static bool parseMotor(const String& tok, int& idx) {
  long n;
  if (parseIntStrict(tok, n) != P_OK) return false;
  if (n < 1 || n > NUM_MOTORS) return false;
  idx = (int)(n - 1);
  return true;
}

// Durée d'action temporisée : P_OK si 1..3 600 000 ms
static ParseRes parseDuration(const String& tok, uint32_t& ms) {
  long v;
  ParseRes r = parseIntStrict(tok, v);
  if (r != P_OK) return r;
  if (v < (long)DUR_MIN_MS || v > (long)DUR_MAX_MS) return P_RANGE;
  ms = (uint32_t)v;
  return P_OK;
}

static inline bool isHexChar(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}
static inline uint8_t hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return c - 'A' + 10;
}

// Découpe "A:B:C" en jetons ; renvoie le nombre de jetons (max maxTok).
static int splitTokens(const String& line, String* tok, int maxTok) {
  int n = 0, start = 0;
  while (n < maxTok) {
    int p = line.indexOf(':', start);
    if (p < 0) { tok[n++] = line.substring(start); break; }
    tok[n++] = line.substring(start, p);
    start = p + 1;
  }
  for (int i = 0; i < n; i++) tok[i].trim();
  return n;
}

// =============================================================================
//  PWM – gestion des canaux LEDC (appelé uniquement depuis taskControl,
//  sauf à l'initialisation et pour la coupure immédiate sous mutex)
// =============================================================================

// Attache le canal LEDC du moteur i à la broche du sens dir (+1 RPWM, -1 LPWM)
static void pwmAttach(uint8_t i, int8_t dir) {
  uint8_t pin = (dir > 0) ? PIN_RPWM[i] : PIN_LPWM[i];
  ledcAttachChannel(pin, PWM_FREQ, PWM_RES, i);   // rapport cyclique initial = 0
}

// Détache la broche du sens dir et la force en sortie LOW
static void pwmDetach(uint8_t i, int8_t dir) {
  uint8_t pin = (dir > 0) ? PIN_RPWM[i] : PIN_LPWM[i];
  ledcDetach(pin);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

// Coupure immédiate du PWM d'un moteur (sans toucher à l'attache, qui sera
// nettoyée par taskControl au passage suivant)
static void pwmForceZero(uint8_t i) {
  if (g_mot[i].active_dir != 0) {
    uint8_t pin = (g_mot[i].active_dir > 0) ? PIN_RPWM[i] : PIN_LPWM[i];
    ledcWrite(pin, 0);
  }
}

// Rapport cyclique (0..PWM_MAX) pour une commande |v| (0..100) selon MIN/MAX
static uint16_t dutyFor(float absV, const MotorCfg& c) {
  if (absV <= 0.0f) return 0;
  float pct = (float)c.minP + ((float)c.maxP - (float)c.minP) * absV / 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (uint16_t)lroundf(pct * (float)PWM_MAX / 100.0f);
}

// =============================================================================
//  OPÉRATIONS SUR L'ÉTAT (à appeler sous mutex)
// =============================================================================

static bool motorRunning(int i) {
  return g_mot[i].target != 0 || g_mot[i].current != 0.0f;
}

static bool allStopped() {
  for (int i = 0; i < NUM_MOTORS; i++) if (motorRunning(i)) return false;
  return true;
}

// Arrêt immédiat d'un moteur (sans rampe), minuterie annulée
static void stopMotorNow(int i) {
  g_mot[i].target   = 0;
  g_mot[i].current  = 0.0f;
  g_mot[i].deadline = 0;
  pwmForceZero(i);
}

static void stopAllNow() {
  for (int i = 0; i < NUM_MOTORS; i++) stopMotorNow(i);
}

// Nouvelle commande sur un moteur (annule toute minuterie précédente)
static void setMotor(int i, int v, Source src, uint32_t durMs, uint32_t now) {
  g_mot[i].target = (int16_t)v;
  g_mot[i].source = src;
  if (durMs > 0) {
    uint32_t dl = now + durMs;
    if (dl == 0) dl = 1;                 // 0 est réservé à "pas de minuterie"
    g_mot[i].deadline = dl;
  } else {
    g_mot[i].deadline = 0;
  }
}

// Marque les réglages comme modifiés (déclenche l'autosave)
static void markDirty(uint32_t now) {
  g_cfgVersion++;
  g_cfgDirty  = true;
  g_dirtyAtMs = now;
  g_saved     = false;
}

static void setMotorDefaults(MotorCfg& c) {
  c.inv    = DEF_INV;
  c.minP   = DEF_MIN;
  c.maxP   = DEF_MAX;
  c.ramp   = DEF_RAMP;
  c.tmaxMs = DEF_TMAX_MS;
}

static void setGlobalDefaults(GlobalCfg& g) {
  g.wdtMs    = DEF_WDT_MS;
  g.uwdtMs   = DEF_UWDT_MS;
  g.autosave = DEF_AUTOSAVE;
}

// Arrêt d'urgence : GPIO 38 bas AVANT la prise du mutex, puis 8 PWM à 0.
static void doEstop() {
  digitalWrite(PIN_EN, LOW);     // coupure matérielle immédiate
  lock();
  g_armed = false;
  stopAllNow();
  unlock();
}

// =============================================================================
//  NVS – Preferences
// =============================================================================

static void nvsKey(char* out, const char* prefix, int i) {
  snprintf(out, 8, "%s%d", prefix, i);
}

// Relecture au démarrage avec clamp des valeurs
static void loadConfig() {
  for (int i = 0; i < NUM_MOTORS; i++) setMotorDefaults(g_cfg[i]);
  setGlobalDefaults(g_glob);

  uint8_t ver = g_prefs.getUChar("ver", 0);
  if (ver != NVS_SCHEMA_VER) {
    // Première exécution ou schéma différent : on enregistre les défauts
    g_saveRequest = true;
    g_saved = false;
    return;
  }

  char k[8];
  for (int i = 0; i < NUM_MOTORS; i++) {
    MotorCfg& c = g_cfg[i];
    nvsKey(k, "iv", i); c.inv  = g_prefs.getUChar(k, DEF_INV) ? 1 : 0;
    nvsKey(k, "mx", i); { uint32_t v = g_prefs.getUChar(k, DEF_MAX); c.maxP = (v > MAX_LIMIT) ? MAX_LIMIT : (uint8_t)v; }
    nvsKey(k, "mn", i); { uint32_t v = g_prefs.getUChar(k, DEF_MIN); if (v > MIN_LIMIT) v = MIN_LIMIT; if (v > c.maxP) v = c.maxP; c.minP = (uint8_t)v; }
    nvsKey(k, "rp", i); { uint32_t v = g_prefs.getUShort(k, DEF_RAMP); if (v < RAMP_LO) v = RAMP_LO; if (v > RAMP_HI) v = RAMP_HI; c.ramp = (uint16_t)v; }
    nvsKey(k, "tm", i); { uint32_t v = g_prefs.getUInt(k, DEF_TMAX_MS); if (v > (uint32_t)(TMAX_LIMIT_S * 1000.0)) v = (uint32_t)(TMAX_LIMIT_S * 1000.0); c.tmaxMs = v; }
  }
  {
    uint32_t v = g_prefs.getUInt("wdt", DEF_WDT_MS);
    g_glob.wdtMs = (v == 0 || (v >= WDT_LO && v <= WDT_HI)) ? v : DEF_WDT_MS;
    v = g_prefs.getUInt("uwdt", DEF_UWDT_MS);
    g_glob.uwdtMs = (v == 0 || (v >= WDT_LO && v <= WDT_HI)) ? v : DEF_UWDT_MS;
    g_glob.autosave = g_prefs.getUChar("as", DEF_AUTOSAVE) ? 1 : 0;
  }
  g_saved = true;
}

// Écriture d'un instantané (hors mutex). NVS n'écrit physiquement que les
// valeurs qui changent : pas d'usure inutile de la flash.
static void writeConfig(const CfgSnapshot& s) {
  char k[8];
  for (int i = 0; i < NUM_MOTORS; i++) {
    nvsKey(k, "iv", i); g_prefs.putUChar(k, s.m[i].inv);
    nvsKey(k, "mx", i); g_prefs.putUChar(k, s.m[i].maxP);
    nvsKey(k, "mn", i); g_prefs.putUChar(k, s.m[i].minP);
    nvsKey(k, "rp", i); g_prefs.putUShort(k, s.m[i].ramp);
    nvsKey(k, "tm", i); g_prefs.putUInt(k, s.m[i].tmaxMs);
  }
  g_prefs.putUInt("wdt", s.g.wdtMs);
  g_prefs.putUInt("uwdt", s.g.uwdtMs);
  g_prefs.putUChar("as", s.g.autosave);
  g_prefs.putUChar("ver", NVS_SCHEMA_VER);
}

// Copie les réglages courants (sous mutex) et les écrit (hors mutex)
static void saveNow() {
  CfgSnapshot s;
  lock();
  memcpy(s.m, g_cfg, sizeof(s.m));
  s.g = g_glob;
  s.version = g_cfgVersion;
  g_saveRequest = false;
  g_cfgDirty = false;
  unlock();

  writeConfig(s);

  lock();
  if (g_cfgVersion == s.version) g_saved = true;   // rien n'a bougé pendant l'écriture
  unlock();
}

// =============================================================================
//  PARSEUR DE COMMANDES
//  String handleCommand(line, src) : renvoie la ligne de réponse (sans \r\n),
//  ou une chaîne vide pour une ligne vide (à ignorer). Réutilisé par UART1,
//  USB et le serveur web.
// =============================================================================

static String okMotor(int idx, int v, uint32_t durMs) {
  char b[40];
  if (durMs) snprintf(b, sizeof(b), "OK M%d %d %lu", idx + 1, v, (unsigned long)durMs);
  else       snprintf(b, sizeof(b), "OK M%d %d", idx + 1, v);
  return String(b);
}

// Lecture "a,b" -> deux entiers stricts. Renvoie P_OK / P_ARG / P_RANGE.
static ParseRes parsePair(const String& tok, long& a, long& b) {
  int c = tok.indexOf(',');
  if (c < 0) return P_ARG;
  ParseRes r1 = parseIntStrict(tok.substring(0, c), a);
  if (r1 != P_OK) return r1;
  ParseRes r2 = parseIntStrict(tok.substring(c + 1), b);
  return r2;
}

static const char* HELP_LINE =
  "HELP Mn:v[:ms] STOP[:n] DRIVE:a,d[:ms] TANK:l,r[:ms] PULSE:n:v:ms ARM ESTOP PING "
  "INV:n:b MAX:n:p MIN:n:p RAMP:n:r TMAX:n:s CFG:n WDT:ms UWDT:ms AUTOSAVE:b "
  "STATUS GCFG PERF VER HELP TELEM:ms SAVE DEFAULTS REBOOT [*HH checksum XOR]";

String handleCommand(const String& rawLine, Source src) {
  String line = rawLine;
  line.trim();
  if (line.length() == 0) return String();            // ligne vide : ignorée
  if (line.length() > LINE_MAX) return String("ERR:LONG");

  // --- Checksum optionnel "*HH" (XOR de tous les octets avant '*') ---------
  int star = line.indexOf('*');
  if (star >= 0) {
    String hex = line.substring(star + 1);
    hex.trim();
    if (hex.length() != 2 || !isHexChar(hex[0]) || !isHexChar(hex[1])) return String("ERR:CRC");
    uint8_t x = 0;
    for (int i = 0; i < star; i++) x ^= (uint8_t)line[i];
    uint8_t given = (uint8_t)(hexVal(hex[0]) * 16 + hexVal(hex[1]));
    if (x != given) return String("ERR:CRC");
    line = line.substring(0, star);
    line.trim();
  }

  line.toUpperCase();
  String tok[4];
  int nTok = splitTokens(line, tok, 4);
  const String& cmd = tok[0];
  if (cmd.length() == 0) return String("ERR:UNKNOWN");

  uint32_t now = millis();
  char b[96];

  // ===================== SÉCURITÉ ==========================================
  if (cmd == "PING")  return String("PONG");

  if (cmd == "ARM") {
    lock();
    stopAllNow();
    g_armed = true;
    unlock();
    digitalWrite(PIN_EN, HIGH);
    return String("OK ARMED");
  }

  if (cmd == "ESTOP") {
    doEstop();                        // GPIO 38 bas avant le mutex
    return String("OK ESTOP");
  }

  // ===================== ARRÊT =============================================
  if (cmd == "STOP") {
    if (nTok == 1) {
      lock(); stopAllNow(); unlock();
      return String("OK STOP");
    }
    int idx;
    if (!parseMotor(tok[1], idx)) return String("ERR:MOTOR");
    lock(); stopMotorNow(idx); unlock();
    snprintf(b, sizeof(b), "OK STOP M%d", idx + 1);
    return String(b);
  }

  // ===================== MOUVEMENT : DRIVE / TANK ==========================
  if (cmd == "DRIVE" || cmd == "TANK") {
    if (nTok < 2) return String("ERR:ARG");
    long a, d;
    ParseRes r = parsePair(tok[1], a, d);
    if (r == P_ARG)   return String("ERR:ARG");
    if (r == P_RANGE) return String("ERR:RANGE");
    uint32_t dur = 0;
    if (nTok >= 3) {
      r = parseDuration(tok[2], dur);
      if (r == P_ARG)   return String("ERR:ARG");
      if (r == P_RANGE) return String("ERR:RANGE");
    }
    int l, rr;
    if (cmd == "DRIVE") { l = clampVel(a + d); rr = clampVel(a - d); }
    else                { l = clampVel(a);     rr = clampVel(d);     }
    if (!g_armed) return String("ERR:ESTOP");
    lock();
    setMotor(IDX_WHEEL_FL, l,  src, dur, now);
    setMotor(IDX_WHEEL_RL, l,  src, dur, now);
    setMotor(IDX_WHEEL_FR, rr, src, dur, now);
    setMotor(IDX_WHEEL_RR, rr, src, dur, now);
    unlock();
    if (dur) snprintf(b, sizeof(b), "OK %s %d,%d %lu", cmd.c_str(), l, rr, (unsigned long)dur);
    else     snprintf(b, sizeof(b), "OK %s %d,%d", cmd.c_str(), l, rr);
    return String(b);
  }

  // ===================== MOUVEMENT : PULSE =================================
  if (cmd == "PULSE") {
    if (nTok < 2) return String("ERR:MOTOR");
    int idx;
    if (!parseMotor(tok[1], idx)) return String("ERR:MOTOR");
    if (nTok < 4) return String("ERR:ARG");            // vitesse ET durée obligatoires
    long v;
    ParseRes r = parseIntStrict(tok[2], v);
    if (r == P_ARG)   return String("ERR:ARG");
    if (r == P_RANGE) return String("ERR:RANGE");
    uint32_t dur;
    r = parseDuration(tok[3], dur);
    if (r == P_ARG)   return String("ERR:ARG");
    if (r == P_RANGE) return String("ERR:RANGE");
    int vv = clampVel(v);
    if (!g_armed) return String("ERR:ESTOP");
    lock(); setMotor(idx, vv, src, dur, now); unlock();
    return okMotor(idx, vv, dur);
  }

  // ===================== RÉGLAGES PAR MOTEUR ===============================
  if (cmd == "INV" || cmd == "MAX" || cmd == "MIN" || cmd == "RAMP" || cmd == "TMAX" || cmd == "CFG") {
    if (nTok < 2) return String("ERR:MOTOR");
    int idx;
    if (!parseMotor(tok[1], idx)) return String("ERR:MOTOR");

    if (cmd == "CFG") {
      lock();
      MotorCfg c = g_cfg[idx];
      unlock();
      String tm = fmtSeconds(c.tmaxMs);
      snprintf(b, sizeof(b), "CFG %d inv=%u min=%u max=%u ramp=%u tmax=%s",
               idx + 1, c.inv, c.minP, c.maxP, c.ramp, tm.c_str());
      return String(b);
    }

    if (nTok < 3) return String("ERR:ARG");

    if (cmd == "TMAX") {
      double s;
      ParseRes r = parseFloatStrict(tok[2], s);
      if (r != P_OK) return String("ERR:ARG");
      if (s < 0.0 || s > TMAX_LIMIT_S) return String("ERR:RANGE");
      uint32_t ms = (uint32_t)llround(s * 1000.0);
      lock();
      if (g_cfg[idx].tmaxMs != ms) { g_cfg[idx].tmaxMs = ms; markDirty(now); }
      unlock();
      String tm = fmtSeconds(ms);
      snprintf(b, sizeof(b), "OK TMAX %d %s", idx + 1, tm.c_str());
      return String(b);
    }

    long v;
    ParseRes r = parseIntStrict(tok[2], v);
    if (r == P_ARG)   return String("ERR:ARG");
    if (r == P_RANGE) return String("ERR:RANGE");

    if (cmd == "INV") {
      if (v != 0 && v != 1) return String("ERR:RANGE");
      lock();
      if (motorRunning(idx)) { unlock(); return String("ERR:BUSY"); }
      if (g_cfg[idx].inv != (uint8_t)v) { g_cfg[idx].inv = (uint8_t)v; markDirty(now); }
      unlock();
      snprintf(b, sizeof(b), "OK INV %d %ld", idx + 1, v);
      return String(b);
    }
    if (cmd == "MAX") {
      if (v < 0 || v > MAX_LIMIT) return String("ERR:RANGE");
      lock();
      if (v < g_cfg[idx].minP) { unlock(); return String("ERR:RANGE"); }
      if (g_cfg[idx].maxP != (uint8_t)v) { g_cfg[idx].maxP = (uint8_t)v; markDirty(now); }
      unlock();
      snprintf(b, sizeof(b), "OK MAX %d %ld", idx + 1, v);
      return String(b);
    }
    if (cmd == "MIN") {
      if (v < 0 || v > MIN_LIMIT) return String("ERR:RANGE");
      lock();
      if (v > g_cfg[idx].maxP) { unlock(); return String("ERR:RANGE"); }
      if (g_cfg[idx].minP != (uint8_t)v) { g_cfg[idx].minP = (uint8_t)v; markDirty(now); }
      unlock();
      snprintf(b, sizeof(b), "OK MIN %d %ld", idx + 1, v);
      return String(b);
    }
    // RAMP
    if (v < RAMP_LO || v > RAMP_HI) return String("ERR:RANGE");
    lock();
    if (g_cfg[idx].ramp != (uint16_t)v) { g_cfg[idx].ramp = (uint16_t)v; markDirty(now); }
    unlock();
    snprintf(b, sizeof(b), "OK RAMP %d %ld", idx + 1, v);
    return String(b);
  }

  // ===================== RÉGLAGES GLOBAUX ==================================
  if (cmd == "WDT" || cmd == "UWDT" || cmd == "AUTOSAVE" || cmd == "TELEM") {
    if (nTok < 2) return String("ERR:ARG");
    long v;
    ParseRes r = parseIntStrict(tok[1], v);
    if (r == P_ARG)   return String("ERR:ARG");
    if (r == P_RANGE) return String("ERR:RANGE");

    if (cmd == "AUTOSAVE") {
      if (v != 0 && v != 1) return String("ERR:RANGE");
      lock();
      if (g_glob.autosave != (uint8_t)v) { g_glob.autosave = (uint8_t)v; markDirty(now); }
      unlock();
      snprintf(b, sizeof(b), "OK AUTOSAVE %ld", v);
      return String(b);
    }
    if (cmd == "TELEM") {
      if (v != 0 && (v < TELEM_LO || v > TELEM_HI)) return String("ERR:RANGE");
      lock(); g_telemMs = (uint32_t)v; unlock();      // non sauvegardé
      snprintf(b, sizeof(b), "OK TELEM %ld", v);
      return String(b);
    }
    // WDT / UWDT
    if (v != 0 && (v < WDT_LO || v > WDT_HI)) return String("ERR:RANGE");
    lock();
    if (cmd == "WDT") {
      if (g_glob.wdtMs != (uint32_t)v) { g_glob.wdtMs = (uint32_t)v; markDirty(now); }
    } else {
      if (g_glob.uwdtMs != (uint32_t)v) { g_glob.uwdtMs = (uint32_t)v; markDirty(now); }
    }
    unlock();
    snprintf(b, sizeof(b), "OK %s %ld", cmd.c_str(), v);
    return String(b);
  }

  // ===================== ÉTAT / SYSTÈME ====================================
  if (cmd == "STATUS") {
    int cur[NUM_MOTORS];
    bool armed;
    lock();
    armed = g_armed;
    for (int i = 0; i < NUM_MOTORS; i++) cur[i] = (int)lroundf(g_mot[i].current);
    unlock();
    String s = "STATUS armed=";
    s += armed ? "1" : "0";
    for (int i = 0; i < NUM_MOTORS; i++) {
      snprintf(b, sizeof(b), " M%d=%d", i + 1, cur[i]);
      s += b;
    }
    return s;
  }

  if (cmd == "GCFG") {
    lock();
    GlobalCfg g = g_glob;
    uint32_t telem = g_telemMs;
    bool saved = g_saved;
    unlock();
    snprintf(b, sizeof(b), "GCFG wdt=%lu uwdt=%lu autosave=%u telem=%lu saved=%d baud=%lu fw=%s",
             (unsigned long)g.wdtMs, (unsigned long)g.uwdtMs, g.autosave,
             (unsigned long)telem, saved ? 1 : 0, (unsigned long)CMD_BAUD, FW_VERSION);
    return String(b);
  }

  if (cmd == "PERF") {
    PerfStats p;
    portENTER_CRITICAL(&g_perfMux);
    p = g_perf;
    g_perf.loops = 0; g_perf.execSumUs = 0; g_perf.execMaxUs = 0; g_perf.periodMaxUs = 0;
    portEXIT_CRITICAL(&g_perfMux);
    uint32_t avg = p.loops ? (uint32_t)(p.execSumUs / p.loops) : 0;
    char pb[200];
    snprintf(pb, sizeof(pb),
             "PERF ctl_period_us=%lu loops=%lu exec_avg_us=%lu exec_max_us=%lu period_max_us=%lu "
             "heap=%lu heap_min=%lu core_ctl=%d core_net=%d",
             (unsigned long)CTRL_PERIOD_US, (unsigned long)p.loops, (unsigned long)avg,
             (unsigned long)p.execMaxUs, (unsigned long)p.periodMaxUs,
             (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
             CORE_CTL, CORE_NET);
    return String(pb);
  }

  if (cmd == "VER") {
    snprintf(b, sizeof(b), "VER %s %s %s motors=%d", FW_VERSION, FW_NAME, FW_TARGET, NUM_MOTORS);
    return String(b);
  }

  if (cmd == "HELP") return String(HELP_LINE);

  if (cmd == "SAVE") {
    lock();
    bool busy = !allStopped();
    g_saveRequest = true;            // taskSave écrira dès que tout est à l'arrêt
    unlock();
    return String(busy ? "OK SAVE_PENDING" : "OK SAVE");
  }

  if (cmd == "DEFAULTS") {
    lock();
    if (!allStopped()) { unlock(); return String("ERR:BUSY"); }
    for (int i = 0; i < NUM_MOTORS; i++) setMotorDefaults(g_cfg[i]);
    setGlobalDefaults(g_glob);
    markDirty(now);
    g_saveRequest = true;            // usine + enregistrement
    unlock();
    return String("OK DEFAULTS");
  }

  if (cmd == "REBOOT") {
    lock();
    if (g_cfgDirty) g_saveRequest = true;   // un SAVE en attente sera écrit avant
    unlock();
    uint32_t t = now + REBOOT_DELAY_MS;
    g_rebootAtMs = t ? t : 1;
    return String("OK REBOOT");
  }

  // ===================== MOUVEMENT : Mn:v[:ms] =============================
  if (cmd[0] == 'M') {
    String num = cmd.substring(1);
    if (num.length() == 0) return String("ERR:MOTOR");
    long n;
    ParseRes r = parseIntStrict(num, n);
    if (r == P_ARG) return String("ERR:UNKNOWN");     // "MXYZ" : pas une commande moteur
    if (r == P_RANGE || n < 1 || n > NUM_MOTORS) return String("ERR:MOTOR");
    int idx = (int)(n - 1);
    if (nTok < 2) return String("ERR:ARG");
    long v;
    r = parseIntStrict(tok[1], v);
    if (r == P_ARG)   return String("ERR:ARG");
    if (r == P_RANGE) return String("ERR:RANGE");
    uint32_t dur = 0;
    if (nTok >= 3) {
      r = parseDuration(tok[2], dur);
      if (r == P_ARG)   return String("ERR:ARG");
      if (r == P_RANGE) return String("ERR:RANGE");
    }
    int vv = clampVel(v);
    if (!g_armed) return String("ERR:ESTOP");
    lock(); setMotor(idx, vv, src, dur, now); unlock();
    return okMotor(idx, vv, dur);
  }

  return String("ERR:UNKNOWN");
}

// =============================================================================
//  TÂCHE DE CONTRÔLE – 200 Hz (cœur 1, prio 5)
//  Aucune allocation dynamique ni String : buffers statiques uniquement.
// =============================================================================

// Un pas de contrôle. Appelé sous mutex. Remplit done[] avec les numéros
// de moteurs (1..8) dont l'action temporisée vient de se terminer.
static void controlStep(uint32_t now, float dt, uint8_t* done, uint8_t& nDone) {
  const bool webTimeout  = (g_glob.wdtMs  > 0) && ((uint32_t)(now - g_lastWebMs)  > g_glob.wdtMs);
  const bool uartTimeout = (g_glob.uwdtMs > 0) && ((uint32_t)(now - g_lastUartMs) > g_glob.uwdtMs);
  const bool armed = g_armed;

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    MotorState& m = g_mot[i];
    const MotorCfg& c = g_cfg[i];

    // --- 1. Watchdogs : arrêt immédiat des moteurs de la source silencieuse,
    //        sauf si une action temporisée est en cours.
    if (m.deadline == 0) {
      if ((m.source == SRC_WEB && webTimeout) || (m.source == SRC_UART && uartTimeout)) {
        if (m.target != 0 || m.current != 0.0f) { m.target = 0; m.current = 0.0f; }
      }
    }

    // --- 2. TMAX : décompte relancé à chaque départ ou changement de sens
    int8_t s = sgnI(m.target);
    if (s != 0 && s != m.prev_sign) m.tmax_start = now;
    m.prev_sign = s;
    if (c.tmaxMs > 0 && s != 0 && (uint32_t)(now - m.tmax_start) >= c.tmaxMs) {
      m.target = 0;                              // retour à 0 AVEC la rampe
      if (m.deadline != 0) {                     // action temporisée écourtée par TMAX :
        m.deadline = 0;                          // on signale tout de même la fin (#D)
        if (nDone < NUM_MOTORS) done[nDone++] = i + 1;
      }
    }

    // --- 3. Minuterie d'action temporisée
    if (m.deadline != 0 && (int32_t)(now - m.deadline) >= 0) {
      m.deadline = 0;
      m.target   = 0;                            // retour à 0 AVEC la rampe
      if (nDone < NUM_MOTORS) done[nDone++] = i + 1;
    }

    // --- 4. Arrêt d'urgence : tout à zéro, sans rampe
    if (!armed) { m.target = 0; m.current = 0.0f; m.deadline = 0; }

    // --- 5. Rampe calculée sur le temps réellement écoulé
    float step = (float)c.ramp * dt;             // %/s × s
    float diff = (float)m.target - m.current;
    if (fabsf(diff) <= step) m.current = (float)m.target;
    else                     m.current += (diff > 0.0f) ? step : -step;

    // --- 6. Sens physique souhaité (INV appliqué ici) et temps mort
    float absV = fabsf(m.current);
    if (absV < 0.05f) absV = 0.0f;               // bruit numérique autour de 0
    int8_t wantDir = (absV > 0.0f) ? sgn(m.current) : 0;
    if (c.inv) wantDir = -wantDir;

    if (wantDir != m.active_dir) {
      if (m.active_dir != 0) {
        // Coupure du sens actif : les deux broches sont désormais à LOW
        pwmDetach(i, m.active_dir);
        m.last_dir = m.active_dir;
        m.dead_time_until = now + DEAD_TIME_MS;
        m.active_dir = 0;
      }
      if (wantDir != 0) {
        // Activation du nouveau sens : temps mort respecté si inversion
        bool deadOk = (m.last_dir == 0) || (m.last_dir == wantDir) ||
                      ((int32_t)(now - m.dead_time_until) >= 0);
        if (deadOk) {
          pwmAttach(i, wantDir);
          m.active_dir = wantDir;
        }
        // sinon : on attend la fin du temps mort, sortie coupée
      }
    }

    // --- 7. Écriture du rapport cyclique
    if (m.active_dir != 0) {
      uint8_t pin = (m.active_dir > 0) ? PIN_RPWM[i] : PIN_LPWM[i];
      ledcWrite(pin, armed ? dutyFor(absV, c) : 0);
    }
  }
}

static void taskControl(void* arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t prevUs = micros();
  uint32_t prevStartUs = 0;
  uint8_t done[NUM_MOTORS];

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CTRL_PERIOD_MS));

    const uint32_t t0 = micros();
    uint32_t period = prevStartUs ? (t0 - prevStartUs) : 0;
    prevStartUs = t0;

    float dt = (float)(t0 - prevUs) * 1e-6f;
    prevUs = t0;
    if (dt > 0.1f) dt = 0.1f;                    // garde-fou (reprise après blocage)

    const uint32_t now = millis();
    uint8_t nDone = 0;

    lock();
    controlStep(now, dt, done, nDone);
    unlock();

    for (uint8_t k = 0; k < nDone; k++) xQueueSend(g_doneQueue, &done[k], 0);

    const uint32_t exec = micros() - t0;
    portENTER_CRITICAL(&g_perfMux);
    g_perf.loops++;
    g_perf.execSumUs += exec;
    if (exec   > g_perf.execMaxUs)   g_perf.execMaxUs   = exec;
    if (period > g_perf.periodMaxUs) g_perf.periodMaxUs = period;
    portEXIT_CRITICAL(&g_perfMux);
  }
}

// =============================================================================
//  TÂCHE UART / USB (cœur 1, prio 4)
//  Lit UART1 et USB, exécute les commandes, émet "#T" et "#D" sur les deux ports.
// =============================================================================

static LineBuf g_lineUart = {{0}, 0, false};
static LineBuf g_lineUsb  = {{0}, 0, false};

// Émission d'une ligne sur UART1 et USB (télémétrie et notifications)
static void broadcastLine(const char* s) {
  Serial1.print(s); Serial1.print(LINE_TERM);
  Serial.print(s);  Serial.print(LINE_TERM);
}

// Alimente le tampon de ligne d'un port avec les octets disponibles
static void pumpPort(Stream& port, LineBuf& lb) {
  while (port.available() > 0) {
    int ch = port.read();
    if (ch < 0) break;
    if (ch == '\r') continue;                    // toléré, ignoré
    if (ch == '\n') {
      // Toute ligne reçue (même vide, même invalide) entretient le watchdog UART
      lock(); g_lastUartMs = millis(); unlock();
      String resp;
      if (lb.overflow) {
        resp = "ERR:LONG";
      } else {
        lb.buf[lb.len] = '\0';
        resp = handleCommand(String(lb.buf), SRC_UART);
      }
      lb.len = 0; lb.overflow = false;
      if (resp.length() > 0) { port.print(resp); port.print(LINE_TERM); }
      continue;
    }
    if (lb.overflow) continue;                   // on jette jusqu'au \n
    if (lb.len >= LINE_MAX) { lb.overflow = true; continue; }
    lb.buf[lb.len++] = (char)ch;
  }
}

static void taskUart(void* arg) {
  (void)arg;
  uint32_t lastTelem = millis();
  char tb[96];

  for (;;) {
    pumpPort(Serial1, g_lineUart);
    pumpPort(Serial,  g_lineUsb);

    // Notifications de fin d'action temporisée
    uint8_t n;
    while (xQueueReceive(g_doneQueue, &n, 0) == pdTRUE) {
      snprintf(tb, sizeof(tb), "#D %u", n);
      broadcastLine(tb);
    }

    // Télémétrie périodique
    uint32_t telem;
    lock(); telem = g_telemMs; unlock();
    if (telem > 0) {
      uint32_t now = millis();
      if ((uint32_t)(now - lastTelem) >= telem) {
        lastTelem = now;
        int cur[NUM_MOTORS]; bool armed;
        lock();
        armed = g_armed;
        for (int i = 0; i < NUM_MOTORS; i++) cur[i] = (int)lroundf(g_mot[i].current);
        unlock();
        snprintf(tb, sizeof(tb), "#T %d %d,%d,%d,%d,%d,%d,%d,%d %lu",
                 armed ? 1 : 0, cur[0], cur[1], cur[2], cur[3], cur[4], cur[5], cur[6], cur[7],
                 (unsigned long)now);
        broadcastLine(tb);
      }
    } else {
      lastTelem = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(UART_POLL_MS));
  }
}

// =============================================================================
//  TÂCHE WEB (cœur 0, prio 2) – WiFi AP + serveur HTTP
// =============================================================================

static void touchWeb() {
  lock(); g_lastWebMs = millis(); unlock();
}

static void handleRoot() {
  touchWeb();
  g_server.sendHeader("Cache-Control", "no-cache");
  g_server.send_P(200, "text/html; charset=utf-8", WEBPAGE);
}

// GET /api/state -> JSON de l'état courant (entretient le watchdog web)
static void handleState() {
  touchWeb();
  int cur[NUM_MOTORS], tgt[NUM_MOTORS]; uint8_t srcs[NUM_MOTORS]; uint32_t dl[NUM_MOTORS];
  bool armed, saved; GlobalCfg g; uint32_t telem;
  uint32_t now = millis();
  lock();
  armed = g_armed; saved = g_saved; g = g_glob; telem = g_telemMs;
  for (int i = 0; i < NUM_MOTORS; i++) {
    cur[i]  = (int)lroundf(g_mot[i].current);
    tgt[i]  = g_mot[i].target;
    srcs[i] = g_mot[i].source;
    dl[i]   = g_mot[i].deadline ? (uint32_t)(g_mot[i].deadline - now) : 0;
  }
  unlock();

  // Construction JSON dans un tampon borné (taille max réelle ~450 octets)
  char js[768];
  size_t p = 0;
  #define JS_ADD(...) do { if (p < sizeof(js)) { int w = snprintf(js + p, sizeof(js) - p, __VA_ARGS__); if (w > 0) p += (size_t)w; } } while (0)
  JS_ADD("{\"armed\":%d,\"ms\":%lu,\"cur\":[", armed ? 1 : 0, (unsigned long)now);
  for (int i = 0; i < NUM_MOTORS; i++) JS_ADD("%d%s", cur[i], i < NUM_MOTORS - 1 ? "," : "");
  JS_ADD("],\"tgt\":[");
  for (int i = 0; i < NUM_MOTORS; i++) JS_ADD("%d%s", tgt[i], i < NUM_MOTORS - 1 ? "," : "");
  JS_ADD("],\"src\":[");
  for (int i = 0; i < NUM_MOTORS; i++) JS_ADD("%u%s", srcs[i], i < NUM_MOTORS - 1 ? "," : "");
  JS_ADD("],\"left\":[");
  for (int i = 0; i < NUM_MOTORS; i++) JS_ADD("%lu%s", (unsigned long)dl[i], i < NUM_MOTORS - 1 ? "," : "");
  JS_ADD("],\"wdt\":%lu,\"uwdt\":%lu,\"autosave\":%u,\"telem\":%lu,\"saved\":%d,"
         "\"heap\":%lu,\"fw\":\"%s\",\"clients\":%d}",
         (unsigned long)g.wdtMs, (unsigned long)g.uwdtMs, g.autosave, (unsigned long)telem,
         saved ? 1 : 0, (unsigned long)ESP.getFreeHeap(), FW_VERSION, (int)WiFi.softAPgetStationNum());
  #undef JS_ADD
  if (p >= sizeof(js)) js[sizeof(js) - 1] = '\0';
  g_server.sendHeader("Cache-Control", "no-cache");
  g_server.send(200, "application/json", js);
}

// GET/POST /api/cmd?c=<commande> -> réponse texte (même parseur que l'UART)
static void handleCmd() {
  touchWeb();
  String c;
  if (g_server.hasArg("c"))          c = g_server.arg("c");
  else if (g_server.hasArg("plain")) c = g_server.arg("plain");   // corps brut POST
  String resp = handleCommand(c, SRC_WEB);
  if (resp.length() == 0) resp = "ERR:ARG";
  g_server.sendHeader("Cache-Control", "no-cache");
  g_server.send(200, "text/plain; charset=utf-8", resp);
}

static void handleNotFound() {
  touchWeb();
  g_server.send(404, "text/plain", "404");
}

static void taskWeb(void* arg) {
  (void)arg;
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(AP_IP), IPAddress(AP_GW), IPAddress(AP_MASK));
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL, 0, WIFI_MAX_CLIENTS);

  g_server.on("/", HTTP_GET, handleRoot);
  g_server.on("/api/state", HTTP_GET, handleState);
  g_server.on("/api/cmd", HTTP_ANY, handleCmd);
  g_server.onNotFound(handleNotFound);
  g_server.begin();

  for (;;) {
    g_server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// =============================================================================
//  TÂCHE SAUVEGARDE (cœur 0, prio 1) – écriture Preferences + REBOOT
// =============================================================================

static void taskSave(void* arg) {
  (void)arg;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(SAVE_POLL_MS));
    uint32_t now = millis();

    // --- Redémarrage programmé : ESTOP, écriture éventuelle, puis restart
    uint32_t rb = g_rebootAtMs;
    if (rb != 0 && (int32_t)(now - rb) >= 0) {
      doEstop();
      bool pending;
      lock(); pending = g_saveRequest || g_cfgDirty; unlock();
      if (pending) saveNow();
      delay(50);
      ESP.restart();
    }

    // --- Sauvegarde : seulement quand tous les moteurs sont à 0
    bool doSave = false;
    lock();
    if (allStopped()) {
      if (g_saveRequest) doSave = true;
      else if (g_glob.autosave && g_cfgDirty && (uint32_t)(now - g_dirtyAtMs) >= AUTOSAVE_DELAY_MS) doSave = true;
    }
    unlock();
    if (doSave) saveNow();
  }
}

// =============================================================================
//  INITIALISATION
// =============================================================================

void setup() {
  // 1. Sécurité d'abord : ponts coupés, toutes les broches PWM à LOW
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(PIN_RPWM[i], OUTPUT); digitalWrite(PIN_RPWM[i], LOW);
    pinMode(PIN_LPWM[i], OUTPUT); digitalWrite(PIN_LPWM[i], LOW);
  }

  // 2. Ports de commande
  Serial.begin(USB_BAUD);                       // USB CDC natif
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);                     // jamais bloquant si aucun hôte USB
#endif
  Serial1.setRxBufferSize(512);                 // à appeler AVANT begin()
  Serial1.begin(CMD_BAUD, SERIAL_8N1, PIN_CMD_RX, PIN_CMD_TX);

  // 3. État initial des moteurs
  memset(g_mot, 0, sizeof(g_mot));
  for (int i = 0; i < NUM_MOTORS; i++) g_mot[i].source = SRC_NONE;
  g_armed = false;
  g_lastUartMs = millis();
  g_lastWebMs  = millis();

  // 4. Synchronisation
  g_mutex     = xSemaphoreCreateMutex();
  g_doneQueue = xQueueCreate(DONE_QUEUE_LEN, sizeof(uint8_t));

  // 5. Réglages persistants
  g_prefs.begin(NVS_NAMESPACE, false);
  loadConfig();

  // 6. Bannière de démarrage (le robot démarre toujours en arrêt d'urgence)
  char ban[80];
  snprintf(ban, sizeof(ban), "#BOOT %s %s fw=%s motors=%d estop=1", FW_NAME, FW_TARGET, FW_VERSION, NUM_MOTORS);
  broadcastLine(ban);

  // 7. Tâches
  xTaskCreatePinnedToCore(taskControl, "ctl",  TASK_CTL_STACK,  nullptr, TASK_CTL_PRIO,  nullptr, CORE_CTL);
  xTaskCreatePinnedToCore(taskUart,    "uart", TASK_UART_STACK, nullptr, TASK_UART_PRIO, nullptr, CORE_CTL);
  xTaskCreatePinnedToCore(taskWeb,     "web",  TASK_WEB_STACK,  nullptr, TASK_WEB_PRIO,  nullptr, CORE_NET);
  xTaskCreatePinnedToCore(taskSave,    "save", TASK_SAVE_STACK, nullptr, TASK_SAVE_PRIO, nullptr, CORE_NET);
}

// loop() inutilisée : tout est géré par les tâches FreeRTOS
void loop() {
  vTaskDelete(NULL);
}
