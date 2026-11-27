/**
 * static/js/mobile/mobile_main.js
 * Point d'entrée de la télécommande mobile horizontale.
 */

import { wsClient } from "../core/ws_client.js";
import { stateStore } from "../core/state_store.js";
import { ConsoleLogger } from "../ui/console_logger.js";
import { initTouchControls } from "./touch_controls.js";
import { initMobileMissions } from "./mobile_missions.js";

function formatMobileMMSS(totalSec) {
    const sec = Math.max(0, Math.floor(Number(totalSec) || 0));
    const m = Math.floor(sec / 60);
    const s = sec % 60;
    return `${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}`;
}

document.addEventListener("DOMContentLoaded", async () => {
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

    const btnPauseResume = document.getElementById("mobBtnPauseResume");
    if (btnPauseResume) {
        btnPauseResume.addEventListener("click", () => {
            const st = stateStore.getState();
            if (st.missionPaused) {
                wsClient.resumeMission();
            } else {
                wsClient.pauseMission();
            }
        });
    }

    const btnAbort = document.getElementById("mobBtnAbortMission");
    if (btnAbort) {
        btnAbort.addEventListener("click", () => {
            wsClient.abortMission();
        });
    }

    let liveMobElapsed = 0;

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
        const isRealUC = state.executionMode === "REAL_ZERO_SIM";
        if (bSim && bCoup) {
            bSim.className = "tab-btn" + (!isCoup ? " active-weed" : "");
            bCoup.className = "tab-btn" + (isCoup ? " active-seed" : "");
        }
        if (lock) {
            lock.style.display = state.simulationLocked ? "block" : "none";
        }
        if (hwBadge) {
            if (state.hardware.connected) {
                hwBadge.textContent = `ESP32 OK (${state.hardware.latency_ms || 0}ms)`;
                hwBadge.style.background = "#064e3b";
                hwBadge.style.color = "#6ee7b7";
            } else if (isRealUC) {
                hwBadge.textContent = "ROBOT_UC";
            } else if (isCoup) {
                hwBadge.textContent = "Attente ESP32";
            } else {
                hwBadge.textContent = "Simul.";
            }
        }

        const cmdHud = document.getElementById("hudDriveCmd");
        if (cmdHud && state.agronomics?.vision?.recommended_command) {
            cmdHud.textContent = state.agronomics.vision.recommended_command;
        }

        // Synchronisation automatique de la Vue Mission Active (aucun retour à l'écran de connexion si mission active/en pause)
        const snap = state.missionSnapshot || {};
        const isMissionLive = Boolean(snap.mission_active || snap.mission_paused || state.missionActive || state.missionPaused);
        const isPaused = Boolean(snap.mission_paused || state.missionPaused || snap.mission_status === "PAUSED");

        const activeCard = document.getElementById("mobActiveMissionCard");
        const connCard = document.getElementById("mobConnectionCard");
        const tabsBar = document.querySelector(".tabs-bar");

        if (activeCard) {
            activeCard.style.display = isMissionLive ? "block" : "none";
            activeCard.style.borderColor = isPaused ? "#f59e0b" : "#10b981";
        }
        if (connCard) {
            connCard.style.display = isMissionLive ? "none" : "block";
        }
        if (tabsBar) {
            tabsBar.style.opacity = isMissionLive ? "0.45" : "1";
            tabsBar.style.pointerEvents = isMissionLive ? "none" : "auto";
        }

        if (isMissionLive) {
            liveMobElapsed = Number(snap.elapsed_s || liveMobElapsed || 0);
            const statusBadge = document.getElementById("mobLiveMissionStatusBadge");
            if (statusBadge) {
                statusBadge.textContent = isPaused ? "🟠 ROBOT EN PAUSE" : "🟢 ROBOT EN COURS DE TRAVAIL";
                statusBadge.style.color = isPaused ? "#fbbf24" : "#34d399";
            }
            const lblEl = document.getElementById("mobLiveMissionLabel");
            if (lblEl) lblEl.textContent = snap.mission_label || "Mission Active";
            const actEl = document.getElementById("mobLiveMissionAction");
            if (actEl) actEl.textContent = snap.current_action || snap.phase_label || "Exécution autonome...";
            const rowEl = document.getElementById("mobLiveRow");
            if (rowEl) rowEl.textContent = `${snap.row_index || 1}/${snap.total_rows_est || 1}`;
            const distEl = document.getElementById("mobLiveDist");
            if (distEl) distEl.textContent = `${Number(snap.distance_m || 0).toFixed(1)} m`;
            const pitchEl = document.getElementById("mobLivePitch");
            if (pitchEl && typeof snap.camera_pitch_deg === "number") {
                pitchEl.textContent = `${snap.camera_pitch_deg.toFixed(1)}°`;
            }
            const elEl = document.getElementById("mobLiveElapsed");
            if (elEl) elEl.textContent = `⏱ ${formatMobileMMSS(liveMobElapsed)}`;
            if (btnPauseResume) {
                btnPauseResume.textContent = isPaused ? "▶ Reprendre" : "⏸ Pause";
                btnPauseResume.style.background = isPaused ? "#10b981" : "#f59e0b";
            }
            const streamImg = document.getElementById("mobLiveCameraStream");
            if (streamImg && (!streamImg.src || !streamImg.src.includes("/api/camera/stream"))) {
                streamImg.src = "/api/camera/stream";
            }
        }
    });

    setInterval(() => {
        const st = stateStore.getState();
        if (st.missionActive && !st.missionPaused) {
            liveMobElapsed += 1;
            const elEl = document.getElementById("mobLiveElapsed");
            if (elEl) elEl.textContent = `⏱ ${formatMobileMMSS(liveMobElapsed)}`;
        }
    }, 1000);

    initTouchControls();
    initMobileMissions();
    // Hydratation immédiate depuis GET /api/status AVANT l'ouverture WebSocket
    await wsClient.bootstrapFromServerState();
    wsClient.connect();
});

