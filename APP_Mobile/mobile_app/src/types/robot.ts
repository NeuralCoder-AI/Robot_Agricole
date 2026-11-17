/**
 * Types et interfaces TypeScript pour l'Application Mobile du Robot Agricole ESP32-S3 (Firmware v2).
 */

export type RobotOperatingMode = 'MANUAL' | 'AUTO';

export type ActiveTab = 'MANUAL' | 'AUTO' | 'TELEMETRY';

export type WorkScreenMode = 'HOME' | 'WEEDING' | 'SEEDING';

export interface TelemetryData {
  armed: number;          // 0 = ESTOP / Désarmé, 1 = Armé (GPIO 38 actif)
  motors: number[];       // M1 à M8 (-100 à +100 %)
  timestamp: number;      // Uptime en ms (#T <armé> <m1..m8> <ms>)
  raw?: string;
}

export interface VisionBoundingBox {
  x: number;
  y: number;
  w: number;
  h: number;
  label: string;
  confidence?: number;
}

export interface VisionAnalysis {
  angular_error_deg: number;
  lateral_offset_cm: number;
  row_detected: boolean;
  left_row_line?: [[number, number], [number, number]];
  right_row_line?: [[number, number], [number, number]];
  ideal_trajectory_line?: [[number, number], [number, number]];
  weed_boxes?: VisionBoundingBox[];
  obstacle_boxes?: VisionBoundingBox[];
  weeds_detected_count?: number;
  obstacle_stop?: boolean;
  recommended_drive?: { a: number; d: number; ms: number };
  recommended_command?: string;
}

export interface AgronomicMetrics {
  crop_type?: string;
  speed_kmh: number;
  pitch: number;          // Tangage (°)
  roll: number;           // Roulis (°)
  weeds_destroyed: number;
  weeds_total: number;
  seeds_sown: number;
  seed_spacing_cm?: number;
  auto_seed_enabled?: boolean;
  seeding_use_tines?: boolean;
  field_length_m?: number;
  field_width_m?: number;
  corridors_total?: number;
  current_corridor?: number;
  distance_traveled_m?: number;
  tines_depth: number;    // 0 à 100 %
  solar_deploy: number;   // 0 à 100 %
  obstacle: boolean;
  distance: number;       // Distance LiDAR (m)
  vision?: VisionAnalysis;
}

export interface DoneEventRecord {
  motor: number;          // 1 à 8
  raw: string;            // ex: "#D 5"
  ts: number;             // Timestamp ms
}

export interface MissionState {
  name: string | null;
  progress: number;       // 0 à 100 %
  step_label: string;
  paused: boolean;
  field_length_m?: number;
  field_width_m?: number;
  corridors_total?: number;
  current_corridor?: number;
  done_history: DoneEventRecord[];
}

export interface MotorNvsConfig {
  motor: number;          // 1 à 8
  inv: number;            // 0 ou 1
  min: number;            // 0 à 90 %
  max: number;            // MIN à 100 %
  ramp: number;           // 1 à 1000 %/s
  tmax: number;           // 0 à 86400 s
}

export interface GlobalNvsConfig {
  wdt: number;            // ms (défaut 2000)
  uwdt: number;           // ms (défaut 1000)
  autosave: number;       // 0 ou 1
  saved: number;          // 1 si sauvegardé dans NVS, 0 si modifié en attente
}

export interface RobotSpecs {
  chassis_length_top_cm: number;
  chassis_length_bottom_cm: number;
  chassis_slope_length_cm: number;
  total_width_with_wheels_cm: number;
  chassis_body_width_cm: number;
  total_height_folded_cm: number;
  ground_clearance_cm: number;
  inner_tunnel_width_cm: number;
  tire_diameter_cm: number;
  tire_circumference_cm: number;
  tire_width_cm: number;
  track_width_cm: number;
  wheelbase_cm: number;
  tines_bar_width_cm: number;
  tines_total_count: number;
  front_small_tine_width_cm: number;
  rear_large_tine_width_cm: number;
  front_to_rear_tines_distance_cm: number;
  rear_large_tines_gap_cm: number;
  front_small_tines_gap_cm: number;
  tines_raised_height_cm: number;
  tines_max_depth_cm: number;
  tines_distance_from_front_cm: number;
  weeder_width_cm: number;
  weeder_rotor_diameter_cm: number;
  tines_to_weeder_distance_cm: number;
  front_axle_to_seeder_cm: number;
  tines_to_seeder_cm: number;
  seed_drop_height_cm: number;
  seeder_nozzles_count: number;
  seeder_row_spacing_cm: number;
  seed_spacing_cm: number;
  seed_with_tines_down: boolean;
  solar_closed_length_cm: number;
  solar_closed_width_cm: number;
  solar_deployed_span_cm: number;
  camera_height_cm: number;
  camera_to_front_axle_cm: number;
  camera_tilt_deg: number;
}

export interface UartLogEntry {
  id: string;
  timestamp: string;
  direction: 'TX' | 'RX' | 'EVENT' | 'SYS';
  line: string;
}
