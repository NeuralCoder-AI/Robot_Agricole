/**
 * static/js/config/constants.js
 * Constantes partagées synchronisées avec le backend et les specs robot (Relevé 26/10/2026).
 */

export const ROBOT_DIMENSIONS = Object.freeze({
    CHASSIS_LENGTH_TOP_CM: 169.0,
    CHASSIS_LENGTH_BOTTOM_CM: 130.0,
    TOTAL_WIDTH_WITH_WHEELS_CM: 121.0,
    CHASSIS_BODY_WIDTH_CM: 81.0,
    TOTAL_HEIGHT_FOLDED_CM: 88.0,
    GROUND_CLEARANCE_CM: 24.0,
    WHEELBASE_CM: 110.0,
    TRACK_WIDTH_CM: 103.0,
    FRONT_OVERHANG_M: 1.05,
    REAR_OVERHANG_M: 1.15,
    TOTAL_LENGTH_M: 2.20,
    SAFETY_MARGIN_M: 0.35,
});

export const AGRONOMIC_SPECS = Object.freeze({
    PEANUT_ROW_SPACING_CM: 46.0,
    SEED_SPACING_DEFAULT_CM: 20.0,
    ROW_SKIP_THRESHOLD_CM: 50.0, // 2 × grosse dent (25 cm) = 50 cm
    CORRIDORS_DEFAULT_COUNT: 10,
    FIELD_LENGTH_DEFAULT_M: 15.0,
    FIELD_WIDTH_DEFAULT_M: 4.6,
});

export const MOTORS = Object.freeze({
    M1: { id: 1, name: "Roue Avant Gauche", role: "DRIVE_L" },
    M2: { id: 2, name: "Roue Avant Droite", role: "DRIVE_R" },
    M3: { id: 3, name: "Roue Arrière Gauche", role: "DRIVE_L" },
    M4: { id: 4, name: "Roue Arrière Droite", role: "DRIVE_R" },
    M5: { id: 5, name: "Dents Sarcleuses", role: "WEEDER_TINES" },
    M6: { id: 6, name: "Fraise Avant", role: "TILLER" },
    M7: { id: 7, name: "Panneaux Solaires", role: "SOLAR" },
    M8: { id: 8, name: "Semoir Trémie", role: "SEEDER" },
});

export const EXECUTION_MODES = Object.freeze({
    SIMULATION_ONLY: "SIMULATION_ONLY",
    COUPLED: "COUPLED",
});

export const TRANSPORTS = Object.freeze({
    WIFI_HTTP: "WIFI_HTTP",
    SOCKET_TCP_WS: "SOCKET_TCP_WS",
    USB_UART: "USB_UART",
});

