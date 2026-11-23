// =============================================================================
//  ROBOT AGRICOLE – Firmware ESP32-S3 – config.h
//  Référence : "Robot Agricole – Référence UART et architecture firmware v2"
//
//  Toutes les constantes matérielles et logicielles du firmware sont
//  regroupées ici. Aucune constante "magique" ne doit apparaître dans
//  robot_agricole.ino : on modifie ce fichier, on recompile.
//
//  Cible  : carte "ESP32S3 Dev Module", core Arduino-ESP32 3.x,
//           option "USB CDC On Boot: Enabled".
// =============================================================================
#pragma once
#include <stdint.h>

// -----------------------------------------------------------------------------
//  Identification
// -----------------------------------------------------------------------------
#define FW_VERSION        "2.1"            // Version firmware (VER / GCFG)
#define FW_NAME           "ROBOT-AGRICOLE" // Nom affiché par VER
#define FW_TARGET         "ESP32-S3"       // Cible affichée par VER

// -----------------------------------------------------------------------------
//  Moteurs : 8 ponts BTS7960 (demi-pont double), 1 moteur par module
// -----------------------------------------------------------------------------
#define NUM_MOTORS        8

// Broche d'activation commune : les 8 R_EN et les 8 L_EN sont reliés ensemble.
// Niveau HAUT = ponts actifs, BAS = tous les ponts coupés (arrêt d'urgence).
#define PIN_EN            38

// Broches RPWM (sens positif) et LPWM (sens négatif) de chaque moteur.
// Indice 0 = M1 ... indice 7 = M8.
//            M1  M2  M3  M4  M5  M6  M7  M8
#define PINS_RPWM         { 4,  6, 15, 17,  8, 10, 12, 14 }
#define PINS_LPWM         { 5,  7, 16, 18,  9, 11, 13, 21 }

// Rôle de chaque moteur (libellés utilisés par la page web et la doc).
//  M1 : roue avant gauche      (positif = avance)
//  M2 : roue avant droite      (positif = avance)
//  M3 : roue arrière gauche    (positif = avance)
//  M4 : roue arrière droite    (positif = avance)
//  M5 : descente des dents     (positif = descendre)
//  M6 : rotation désherbage
//  M7 : panneaux solaires      (positif = déployer)
//  M8 : semoir                 (positif = semer)

// Indices (0-based) des roues utilisées par DRIVE / TANK
#define IDX_WHEEL_FL      0   // M1 avant gauche
#define IDX_WHEEL_FR      1   // M2 avant droite
#define IDX_WHEEL_RL      2   // M3 arrière gauche
#define IDX_WHEEL_RR      3   // M4 arrière droite

// -----------------------------------------------------------------------------
//  PWM (LEDC)
//  L'ESP32-S3 ne possède que 8 canaux LEDC pour nos 16 broches : chaque moteur
//  dispose d'UN canal (canal i = moteur i), basculé sur RPWM ou LPWM selon le
//  sens (ledcDetach / ledcAttachChannel). La broche inactive est forcée en
//  sortie LOW. RPWM et LPWM ne sont jamais actifs simultanément.
// -----------------------------------------------------------------------------
#define PWM_FREQ          20000           // Hz   – fréquence PWM (20 kHz : inaudible)
#define PWM_RES           10              // bits – résolution (0..1023)
#define PWM_MAX           ((1u << PWM_RES) - 1u)
#define DEAD_TIME_MS      80              // ms   – temps mort entre deux sens opposés

// -----------------------------------------------------------------------------
//  Liaison de commande
// -----------------------------------------------------------------------------
#define CMD_BAUD          115200          // UART1 : 115200 par défaut, jusqu'à 921600
#define PIN_CMD_RX        1               // UART1 RX
#define PIN_CMD_TX        2               // UART1 TX
#define USB_BAUD          115200          // Valeur nominale (ignorée par le CDC natif)
#define LINE_MAX          96              // Longueur max d'une ligne de commande (sinon ERR:LONG)
#define LINE_TERM         "\r\n"          // Terminaison des réponses

// -----------------------------------------------------------------------------
//  Boucle de contrôle temps réel (taskControl)
// -----------------------------------------------------------------------------
#define CTRL_PERIOD_MS    5               // 200 Hz
#define CTRL_PERIOD_US    (CTRL_PERIOD_MS * 1000UL)

// -----------------------------------------------------------------------------
//  Plages et valeurs par défaut des commandes
// -----------------------------------------------------------------------------
#define VEL_MAX           100             // Commande moteur : -100..+100 (au-delà : saturé)
#define DUR_MIN_MS        1UL             // Durée minimale d'une action temporisée
#define DUR_MAX_MS        3600000UL       // Durée maximale (1 h)

// Réglages par moteur (défauts usine)
#define DEF_INV           0               // Inversion de sens (0/1)
#define DEF_MAX           100             // PWM max en % (0..100, >= MIN)
#define DEF_MIN           0               // PWM min en % (0..90, <= MAX)
#define DEF_RAMP          100             // Rampe en %/s (1..1000)
#define DEF_TMAX_MS       0               // Durée max de marche continue (0 = illimité)

#define MAX_LIMIT         100             // Borne sup. de MAX
#define MIN_LIMIT         90              // Borne sup. de MIN
#define RAMP_LO           1               // Borne inf. de RAMP (%/s)
#define RAMP_HI           1000            // Borne sup. de RAMP (%/s)
#define TMAX_LIMIT_S      86400.0         // Borne sup. de TMAX (24 h)

// Watchdogs
#define DEF_WDT_MS        2000            // Watchdog web  : 0 ou 100..60000
#define DEF_UWDT_MS       1000            // Watchdog UART : 0 ou 100..60000
#define WDT_LO            100
#define WDT_HI            60000

// Télémétrie
#define TELEM_LO          20              // ms – période mini (0 = arrêt)
#define TELEM_HI          60000

// Sauvegarde automatique (AUTOSAVE)
#define DEF_AUTOSAVE      1
#define AUTOSAVE_DELAY_MS 3000            // Délai après la dernière modification
#define SAVE_POLL_MS      100             // Période de scrutation de taskSave

// Redémarrage
#define REBOOT_DELAY_MS   300             // Délai entre "OK REBOOT" et ESP.restart()

// -----------------------------------------------------------------------------
//  Stockage non volatile (Preferences / NVS)
//  Clés : mn0..mn7 (MIN), mx0..mx7 (MAX), rp0..rp7 (RAMP), iv0..iv7 (INV),
//         tm0..tm7 (TMAX en ms), wdt, uwdt, as (autosave), ver (=2)
// -----------------------------------------------------------------------------
#define NVS_NAMESPACE     "robot"
#define NVS_SCHEMA_VER    2

// -----------------------------------------------------------------------------
//  WiFi (point d'accès) et serveur web
// -----------------------------------------------------------------------------
#define WIFI_SSID         "RobotAgricole"
#define WIFI_PASS         "robot1234"     // 8 caractères minimum (WPA2)
#define WIFI_CHANNEL      6
#define WIFI_MAX_CLIENTS  4
#define AP_IP             192, 168, 4, 1  // IP du robot
#define AP_GW             192, 168, 4, 1
#define AP_MASK           255, 255, 255, 0
#define HTTP_PORT         80
#define WEB_POLL_MS       400             // Période d'interrogation /api/state par la page

// -----------------------------------------------------------------------------
//  Tâches FreeRTOS (cœur, priorité, taille de pile en octets)
// -----------------------------------------------------------------------------
#define CORE_CTL          1               // Cœur temps réel (contrôle + UART)
#define CORE_NET          0               // Cœur réseau (WiFi, web, NVS)

#define TASK_CTL_PRIO     5
#define TASK_CTL_STACK    4096
#define TASK_UART_PRIO    4
#define TASK_UART_STACK   6144
#define TASK_WEB_PRIO     2
#define TASK_WEB_STACK    8192
#define TASK_SAVE_PRIO    1
#define TASK_SAVE_STACK   4096

#define UART_POLL_MS      1               // Période de lecture UART/USB
#define DONE_QUEUE_LEN    16              // File des notifications "#D n"
