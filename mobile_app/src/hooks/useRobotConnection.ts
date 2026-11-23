/**
 * mobile_app/src/hooks/useRobotConnection.ts
 * Hook React pour la gestion de la connexion WebSocket, de l'état matériel ESP32,
 * du mode couplé (Hardware-in-the-Loop) et du filtrage strict des logs sans pollution console.
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import { Platform } from 'react-native';
import {
  RobotOperatingMode,
  TelemetryData,
  AgronomicMetrics,
  MissionState,
  RobotSpecs,
} from '../types/robot';
import { ROBOT_CONFIG } from '../config/constants';

export const DEFAULT_ROBOT_SPECS: RobotSpecs = {
  chassis_length_top_cm: ROBOT_CONFIG.CHASSIS_LENGTH_TOP_CM,
  chassis_length_bottom_cm: ROBOT_CONFIG.CHASSIS_LENGTH_BOTTOM_CM,
  chassis_slope_length_cm: 30.0,
  total_width_with_wheels_cm: ROBOT_CONFIG.TOTAL_WIDTH_WITH_WHEELS_CM,
  chassis_body_width_cm: ROBOT_CONFIG.CHASSIS_BODY_WIDTH_CM,
  total_height_folded_cm: ROBOT_CONFIG.TOTAL_HEIGHT_FOLDED_CM,
  ground_clearance_cm: ROBOT_CONFIG.GROUND_CLEARANCE_CM,
  inner_tunnel_width_cm: 79.5,
  tire_diameter_cm: 26.5,
  tire_circumference_cm: 83.25,
  tire_width_cm: 19.0,
  track_width_cm: ROBOT_CONFIG.TRACK_WIDTH_CM,
  wheelbase_cm: ROBOT_CONFIG.WHEELBASE_CM,
  tines_bar_width_cm: ROBOT_CONFIG.TINES_BAR_WIDTH_CM,
  tines_total_count: ROBOT_CONFIG.TINES_TOTAL_COUNT,
  front_small_tine_width_cm: ROBOT_CONFIG.FRONT_SMALL_TINE_WIDTH_CM,
  rear_large_tine_width_cm: ROBOT_CONFIG.REAR_LARGE_TINE_WIDTH_CM,
  front_to_rear_tines_distance_cm: 30.0,
  rear_large_tines_gap_cm: 29.5,
  front_small_tines_gap_cm: 41.5,
  tines_raised_height_cm: 12.0,
  tines_max_depth_cm: 8.0,
  tines_distance_from_front_cm: 170.0,
  weeder_width_cm: ROBOT_CONFIG.TILLER_WIDTH_CM,
  weeder_rotor_diameter_cm: 5.0,
  tines_to_weeder_distance_cm: 196.0,
  front_axle_to_seeder_cm: 165.0,
  tines_to_seeder_cm: 29.0,
  seed_drop_height_cm: 14.0,
  seeder_nozzles_count: ROBOT_CONFIG.SEEDER_NOZZLES_COUNT,
  seeder_row_spacing_cm: ROBOT_CONFIG.SEEDER_ROW_SPACING_CM,
  seed_spacing_cm: 25.0,
  seed_with_tines_down: true,
  solar_closed_length_cm: 173.0,
  solar_closed_width_cm: 106.0,
  solar_deployed_span_cm: 99.0,
  camera_height_cm: 75.0,
  camera_to_front_axle_cm: 25.0,
  camera_tilt_deg: 35.0,
};

export function detectHost(): string {
  try {
    const g = globalThis as unknown as {
      process?: { env?: Record<string, string | undefined> };
    };
    const envIp = g.process?.env?.EXPO_PUBLIC_PC_LAN_IP;
    if (envIp && /^\d{1,3}(\.\d{1,3}){3}$/.test(envIp.trim())) {
      return envIp.trim();
    }
  } catch {
    // fallback
  }
  if (Platform.OS === 'web' && typeof window !== 'undefined' && window.location?.hostname) {
    return window.location.hostname;
  }
  return '192.168.1.194';
}

export function isConsoleNoise(line: string): boolean {
  const s = String(line || '').trim();
  if (!s) return true;
  const u = s.split('*')[0]?.trim().toUpperCase() || '';
  if (u === 'PING' || u === 'PONG') return true;
  if (s.toUpperCase().startsWith('PING*') || s.toUpperCase().startsWith('PONG*')) return true;
  if (s.startsWith('#T ') || s === '#T') return true;
  return false;
}

export function useRobotConnection() {
  const [serverIp, setServerIp] = useState<string>(detectHost);
  const [serverPort, setServerPort] = useState<string>('8000');
  const [wsConnected, setWsConnected] = useState<boolean>(false);
  const [latencyMs, setLatencyMs] = useState<number>(0);

  const [isArmed, setIsArmed] = useState<boolean>(false);
  const [mode, setMode] = useState<RobotOperatingMode>('MANUAL');
  const [telemetry, setTelemetry] = useState<TelemetryData>({
    armed: 0,
    motors: [0, 0, 0, 0, 0, 0, 0, 0],
    timestamp: 0,
  });
  const [agronomics, setAgronomics] = useState<AgronomicMetrics>({
    crop_type: 'ARACHIDE',
    speed_kmh: 0,
    pitch: 0,
    roll: 0,
    weeds_destroyed: 0,
    weeds_total: 180,
    seeds_sown: 0,
    seed_spacing_cm: 25,
    auto_seed_enabled: false,
    seeding_use_tines: true,
    field_length_m: 10,
    field_width_m: 4.6,
    corridors_total: 10,
    current_corridor: 1,
    distance_traveled_m: 0,
    tines_depth: 0,
    solar_deploy: 0,
    obstacle: false,
    distance: 99.0,
  });
  const [missionState, setMissionState] = useState<MissionState>({
    name: null,
    progress: 0,
    step_label: 'Prêt',
    paused: false,
    done_history: [],
  });
  const [robotSpecs, setRobotSpecs] = useState<RobotSpecs>(DEFAULT_ROBOT_SPECS);

  // Mode Couplé Matériel (ESP32-S3 Hardware Bridge)
  const [executionMode, setExecutionMode] = useState<'SIMULATION_ONLY' | 'COUPLED'>('SIMULATION_ONLY');
  const [hwConnected, setHwConnected] = useState<boolean>(false);
  const [simulationLocked, setSimulationLocked] = useState<boolean>(false);
  const [hwTransport, setHwTransport] = useState<'WIFI_HTTP' | 'SOCKET_TCP_WS' | 'USB_UART'>('WIFI_HTTP');
  const [hwTarget, setHwTarget] = useState<string>(ROBOT_CONFIG.DEFAULT_ESP32_IP);
  const [consoleLogs, setConsoleLogs] = useState<string[]>([]);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  const addConsoleLog = useCallback((line: string) => {
    if (isConsoleNoise(line)) return;
    setConsoleLogs((prev) => [...prev.slice(-149), line]);
  }, []);

  const sendRawCommand = useCallback((cmd: string) => {
    const raw = String(cmd || '').trim();
    if (!raw) return;
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ type: 'command', command: raw }));
      addConsoleLog(`TX -> ${raw}`);
    }
  }, [addConsoleLog]);

  const sendJson = useCallback((msg: object) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(msg));
    }
  }, []);

  const connectWebSocket = useCallback(() => {
    if (wsRef.current) {
      try {
        wsRef.current.close();
      } catch {
        // ignore
      }
    }

    const host = serverIp.trim() || detectHost();
    const port = serverPort.trim() || '8000';
    const wsUrl = `ws://${host}:${port}/ws`;

    try {
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setWsConnected(true);
        addConsoleLog(`[CONNEXION] WebSocket connecté sur ${wsUrl}`);
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.type === 'state') {
            setIsArmed(Boolean(data.state?.armed));
            setMode(data.state?.mode || 'MANUAL');
          } else if (data.type === 'telemetry') {
            setTelemetry(data.telemetry);
          } else if (data.type === 'agronomics') {
            setAgronomics((prev) => ({ ...prev, ...data.agronomics }));
          } else if (data.type === 'mission_progress') {
            setMissionState((prev) => ({
              ...prev,
              progress: data.progress ?? prev.progress,
              step_label: data.step_label || prev.step_label,
            }));
          } else if (data.type === 'hardware_status') {
            const hw = data.hardware || {};
            setHwConnected(Boolean(hw.connected));
            setSimulationLocked(Boolean(hw.simulation_locked));
            if (hw.mode) setExecutionMode(hw.mode);
            if (hw.transport) setHwTransport(hw.transport);
            if (hw.target) setHwTarget(hw.target);
          } else if (data.type === 'serial_log' || data.type === 'console_log') {
            const line = data.line || data.message || '';
            addConsoleLog(line);
          } else if (data.type === 'done') {
            addConsoleLog(`RX <- #D ${data.motor}`);
          }
        } catch {
          // ignore non-json
        }
      };

      ws.onclose = () => {
        setWsConnected(false);
        if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current);
        reconnectTimerRef.current = setTimeout(connectWebSocket, 3000);
      };

      ws.onerror = () => {
        setWsConnected(false);
      };
    } catch {
      setWsConnected(false);
    }
  }, [serverIp, serverPort, addConsoleLog]);

  useEffect(() => {
    connectWebSocket();
    return () => {
      if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current);
      if (wsRef.current) {
        try {
          wsRef.current.close();
        } catch {
          // ignore
        }
      }
    };
  }, [connectWebSocket]);

  return {
    serverIp,
    setServerIp,
    serverPort,
    setServerPort,
    wsConnected,
    latencyMs,
    isArmed,
    mode,
    setMode,
    telemetry,
    agronomics,
    setAgronomics,
    missionState,
    robotSpecs,
    executionMode,
    setExecutionMode,
    hwConnected,
    simulationLocked,
    hwTransport,
    setHwTransport,
    hwTarget,
    setHwTarget,
    consoleLogs,
    setConsoleLogs,
    sendRawCommand,
    sendJson,
    connectWebSocket,
  };
}

