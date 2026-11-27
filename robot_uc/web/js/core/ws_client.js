/**
 * static/js/core/ws_client.js
 * Client WebSocket et API REST partagé entre le Dashboard et le Mobile.
 */

import { stateStore } from "./state_store.js";
import { isConsoleNoise } from "../ui/console_logger.js";

export class WsClient {
    constructor(options = {}) {
        this.options = options;
        this.ws = null;
        this.reconnectTimer = null;
        this.onLogHandlers = new Set();
        this.onStatusHandlers = new Set();
        this._seenLogKeys = new Set();
    }

    async bootstrapFromServerState() {
        try {
            const res = await fetch("/api/status", { cache: "no-store" });
            if (!res.ok) return null;
            const data = await res.json();
            this._applyServerPayload(data);
            return data;
        } catch (e) {
            console.warn("[WsClient] Erreur bootstrap /api/status:", e);
            return null;
        }
    }

    onLog(handler) {
        this.onLogHandlers.add(handler);
        return () => this.onLogHandlers.delete(handler);
    }

    onStatusChange(handler) {
        this.onStatusHandlers.add(handler);
        return () => this.onStatusHandlers.delete(handler);
    }

    connect() {
        if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
            return;
        }

        const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
        const url = `${proto}//${window.location.host}/ws`;
        this.ws = new WebSocket(url);

        this.ws.onopen = () => {
            this._notifyStatus("connected", `🟢 Connecté (${window.location.host})`);
            // Hydratation complète depuis le serveur à chaque connexion/reconnexion
            this.bootstrapFromServerState();
        };

        this.ws.onmessage = (ev) => {
            try {
                const msg = JSON.parse(ev.data);
                this._handleMessage(msg);
            } catch (e) {
                console.error("[WsClient] Erreur parsing:", e);
            }
        };

        this.ws.onclose = () => {
            // Jamais de STOP/ESTOP/disconnect sur perte réseau ou fermeture onglet
            this._notifyStatus("disconnected", "🔴 Reconnexion...");
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = setTimeout(() => this.connect(), 1500);
        };

        this.ws.onerror = (err) => {
            console.warn("[WsClient] Erreur socket:", err);
        };
    }

    send(data) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(data));
            return true;
        }
        return false;
    }

    sendCommand(cmdStr) {
        const clean = String(cmdStr).trim();
        if (!clean) return false;
        return this.send({ type: "cmd", command: clean });
    }

    setExecutionMode(mode) {
        return this.send({ type: "set_execution_mode", execution_mode: mode });
    }

    connectHardware(config) {
        return this.send({
            type: "hardware_connect",
            ...config
        });
    }

    disconnectHardware() {
        return this.send({ type: "hardware_disconnect" });
    }

    arm() {
        return this.send({ type: "arm" });
    }

    estop() {
        return this.send({ type: "estop" });
    }

    pauseMission() {
        return this.send({ type: "pause_mission" });
    }

    resumeMission() {
        return this.send({ type: "resume_mission" });
    }

    abortMission() {
        return this.send({ type: "abort_mission" });
    }

    drive(a, d, ms = 400) {
        return this.sendCommand(`DRIVE:${a},${d}:${ms}`);
    }

    tank(l, r, ms = 400) {
        return this.sendCommand(`TANK:${l},${r}:${ms}`);
    }

    _replayRecentLogs(recentLogs) {
        if (!Array.isArray(recentLogs)) return;
        for (const entry of recentLogs) {
            if (!entry || !entry.line || isConsoleNoise(entry.line)) continue;
            const key = `${entry.ts || 0}|${entry.direction || "RX"}|${entry.line}`;
            if (this._seenLogKeys.has(key)) continue;
            this._seenLogKeys.add(key);
            if (this._seenLogKeys.size > 300) {
                const first = this._seenLogKeys.values().next().value;
                this._seenLogKeys.delete(first);
            }
            for (const handler of this.onLogHandlers) {
                try {
                    handler(entry.direction || "RX", entry.line);
                } catch (e) {
                    console.error(e);
                }
            }
        }
    }

    _applyServerPayload(msg) {
        if (!msg || typeof msg !== "object") return;
        const patch = {};
        if (typeof msg.armed === "number") patch.armed = msg.armed;
        if (msg.mode) patch.mode = msg.mode;
        if (msg.hardware) patch.hardware = msg.hardware;
        if (msg.agronomics) patch.agronomics = msg.agronomics;
        if (msg.camera) patch.camera = msg.camera;
        if (Array.isArray(msg.motors)) patch.motors = msg.motors;
        if (msg.telemetry && Array.isArray(msg.telemetry.motors)) patch.motors = msg.telemetry.motors;
        if (msg.data && Array.isArray(msg.data.motors)) patch.motors = msg.data.motors;

        if (msg.mission_snapshot && typeof msg.mission_snapshot === "object") {
            patch.mission_snapshot = msg.mission_snapshot;
            this._replayRecentLogs(msg.mission_snapshot.recent_logs);
        } else if (msg.mission_state && typeof msg.mission_state === "object") {
            const ms = msg.mission_state;
            patch.mission_snapshot = {
                mission_active: Boolean(ms.mission_active ?? ms.mission_running),
                mission_paused: Boolean(ms.mission_paused ?? ms.paused),
                mission_status: ms.mission_status || (ms.paused ? "PAUSED" : (ms.mission_running ? "RUNNING" : "IDLE")),
                mission_type: ms.mission_type || "manual",
                mission_label: ms.mission_label || "4. Conduite Manuelle",
                current_action: ms.current_action || ms.step_label || "En attente",
                phase_label: ms.phase_label || ms.step_label || "En attente",
                progress: ms.progress ?? 0,
                row_index: ms.current_corridor ?? 1,
                total_rows_est: ms.corridors_total ?? 10,
                distance_m: ms.distance_m ?? 0.0,
                elapsed_s: ms.elapsed_s ?? 0.0,
                parcel_length_m: ms.field_length_m ?? 10.0,
                parcel_width_m: ms.field_width_m ?? 4.6,
            };
        }

        if (Object.keys(patch).length > 0) {
            stateStore.update(patch);
        }
    }

    _handleMessage(msg) {
        if (!msg) return;
        this._applyServerPayload(msg);

        if (msg.type === "mission_lock_rejected" && msg.message) {
            for (const handler of this.onLogHandlers) {
                try {
                    handler("MISSION", `🔒 ${msg.message}`);
                } catch (e) {
                    console.error(e);
                }
            }
        }

        if (msg.type === "serial_log" && msg.line) {
            if (!isConsoleNoise(msg.line)) {
                const key = `${msg.ts || Date.now()}|${msg.direction || "RX"}|${msg.line}`;
                this._seenLogKeys.add(key);
                for (const handler of this.onLogHandlers) {
                    try {
                        handler(msg.direction || "RX", msg.line);
                    } catch (e) {
                        console.error(e);
                    }
                }
            }
        }
    }

    _notifyStatus(state, message) {
        for (const handler of this.onStatusHandlers) {
            try {
                handler(state, message);
            } catch (e) {
                console.error(e);
            }
        }
    }
}

export const wsClient = new WsClient();

