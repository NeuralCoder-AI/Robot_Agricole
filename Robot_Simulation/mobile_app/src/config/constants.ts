/**
 * mobile_app/src/config/constants.ts
 * Constantes physiques, agronomiques et réseau du Robot Agricole ESP32-S3 (Relevé 26/10/2026).
 */

export const ROBOT_CONFIG = {
  // Dimensions physiques relevé terrain (26/10/2026)
  CHASSIS_LENGTH_TOP_CM: 169.0,
  CHASSIS_LENGTH_BOTTOM_CM: 130.0,
  TOTAL_WIDTH_WITH_WHEELS_CM: 121.0,
  CHASSIS_BODY_WIDTH_CM: 81.0,
  TOTAL_HEIGHT_FOLDED_CM: 88.0,
  GROUND_CLEARANCE_CM: 24.0,
  WHEELBASE_CM: 110.0,
  TRACK_WIDTH_CM: 103.0,

  // Outils agronomiques
  TINES_TOTAL_COUNT: 4,
  TINES_BAR_WIDTH_CM: 84.0,
  FRONT_SMALL_TINE_WIDTH_CM: 13.0,
  REAR_LARGE_TINE_WIDTH_CM: 25.0,
  TILLER_WIDTH_CM: 23.0,
  SEEDER_NOZZLES_COUNT: 2,
  SEEDER_ROW_SPACING_CM: 46.0,
  ROW_SKIP_THRESHOLD_CM: 50.0, // 2 × grosse dent (25 cm)

  // Encombrement hors-tout pour navigation
  FRONT_OVERHANG_M: 1.05,
  REAR_OVERHANG_M: 1.15,
  TOTAL_ROBOT_LENGTH_M: 2.20,
  TOTAL_ROBOT_WIDTH_M: 1.21,
  SAFETY_MARGIN_M: 0.35,

  // Réseau & Hardware
  DEFAULT_WS_PORT: 8000,
  DEFAULT_ESP32_IP: '192.168.4.1',
  DEFAULT_ESP32_PORT: 80,
  DEFAULT_SOCKET_PORT: 8888,
  DEFAULT_BAUD_RATE: 115200,

  // Couleurs UI Dashboard
  COLORS: {
    bgDark: '#0b111e',
    panelBg: '#131f33',
    cardBg: '#1c2d47',
    border: '#2a4365',
    textMain: '#f0f6fc',
    textMuted: '#8b949e',
    accentCyan: '#38bdf8',
    accentGreen: '#22c55e',
    accentAmber: '#f59e0b',
    accentRed: '#ef4444',
  },
} as const;

