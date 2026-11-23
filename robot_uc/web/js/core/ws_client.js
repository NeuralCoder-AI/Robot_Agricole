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

    drive(a, d, ms = 400) {
        return this.sendCommand(`DRIVE:${a},${d}:${ms}`);
    }

    tank(l, r, ms = 400) {
        return this.sendCommand(`TANK:${l},${r}:${ms}`);
    }

    _handleMessage(msg) {
        if (!msg) return;

        if (typeof msg.armed === "number") {
            stateStore.update({ armed: msg.armed });
        }
        if (msg.mode) {
            stateStore.update({ mode: msg.mode });
        }
        if (msg.hardware) {
            stateStore.update({ hardware: msg.hardware });
        }
        if (msg.agronomics) {
            stateStore.update({ agronomics: msg.agronomics });
        }
        if (Array.isArray(msg.motors)) {
            stateStore.update({ motors: msg.motors });
        }

        if (msg.type === "serial_log" && msg.line) {
            if (!isConsoleNoise(msg.line)) {
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

