/**
 * static/js/mobile/mobile_main.js
 * Point d'entrée de la télécommande mobile horizontale.
 */

import { wsClient } from "../core/ws_client.js";
import { stateStore } from "../core/state_store.js";
import { ConsoleLogger } from "../ui/console_logger.js";
import { initTouchControls } from "./touch_controls.js";
import { initMobileMissions } from "./mobile_missions.js";

document.addEventListener("DOMContentLoaded", () => {
    const consoleContainer = document.getElementById("mobConsoleBox");
    const logger = new ConsoleLogger(consoleContainer, 80);

    const btnClearConsole = document.getElementById("mobBtnClearConsole");
    if (btnClearConsole) {
        btnClearConsole.addEventListener("click", () => logger.clear());
    }

    wsClient.onLog((direction, line) => {
        logger.append(direction, line);
    });

    wsClient.onStatusChange((status, message) => {
        const info = document.getElementById("wsInfoText");
        if (info) info.textContent = message;
    });

    stateStore.subscribe((state) => {
        const badge = document.getElementById("statusBadge");
        if (badge) {
            badge.className = state.armed === 1 ? "badge badge-armed" : "badge badge-estop";
            badge.textContent = state.armed === 1 ? "⚡ ARMÉ" : "🛑 ESTOP";
        }

        const bSim = document.getElementById("mobBtnModeSim");
        const bCoup = document.getElementById("mobBtnModeCoupled");
        const lock = document.getElementById("mobCoupledLockBanner");
        const hwBadge = document.getElementById("mobHwStatusBadge");

        const isCoup = state.executionMode === "COUPLED";
        if (bSim && bCoup) {
            bSim.className = "tab-btn" + (!isCoup ? " active-weed" : "");
            bCoup.className = "tab-btn" + (isCoup ? " active-seed" : "");
        }
        if (lock) {
            lock.style.display = state.simulationLocked ? "block" : "none";
        }
        if (hwBadge) {
            hwBadge.textContent = isCoup
                ? (state.hardware.connected ? `ESP32 OK (${state.hardware.latency_ms || 0}ms)` : "Attente ESP32")
                : "Simul.";
        }

        const cmdHud = document.getElementById("hudDriveCmd");
        if (cmdHud && state.agronomics?.vision?.recommended_command) {
            cmdHud.textContent = state.agronomics.vision.recommended_command;
        }
    });

    initTouchControls();
    initMobileMissions();
    wsClient.connect();
});

