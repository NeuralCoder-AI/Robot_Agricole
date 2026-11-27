/**
 * static/js/mobile/mobile_missions.js
 * Vues simplifiées Désherbage Arachide, Semence Arachide, Révision et Connexion Robot Réel.
 */

import { wsClient } from "../core/ws_client.js";
import { stateStore } from "../core/state_store.js";

export function initMobileMissions() {
    const tabWeed = document.getElementById("tabBtnWeed");
    const tabSeed = document.getElementById("tabBtnSeed");
    const tabReseed = document.getElementById("tabBtnReseed");
    const viewWeed = document.getElementById("viewWeeding");
    const viewSeed = document.getElementById("viewSeeding");
    const viewReseed = document.getElementById("viewReseeding");

    const isLockedByActiveMission = () => {
        const st = stateStore.getState();
        return Boolean(st.missionActive || st.missionPaused);
    };

    const switchTab = (activeTab, activeView) => {
        if (isLockedByActiveMission()) return;
        [tabWeed, tabSeed, tabReseed].forEach(t => {
            if (t) {
                t.classList.remove("active-weed");
                t.classList.remove("active-seed");
            }
        });
        [viewWeed, viewSeed, viewReseed].forEach(v => {
            if (v) v.style.display = "none";
        });
        if (activeTab) activeTab.classList.add("active-weed");
        if (activeView) activeView.style.display = "block";
    };

    if (tabWeed) tabWeed.addEventListener("click", () => switchTab(tabWeed, viewWeed));
    if (tabSeed) tabSeed.addEventListener("click", () => switchTab(tabSeed, viewSeed));
    if (tabReseed) tabReseed.addEventListener("click", () => switchTab(tabReseed, viewReseed));

    // Abonnement aux métriques de révision
    stateStore.subscribe((state) => {
        if (!state.agronomics) return;
        const gapsEl = document.getElementById("mobMetricGaps");
        const deadEl = document.getElementById("mobMetricDead");
        const seedsEl = document.getElementById("mobMetricSeeds");
        const rowsEl = document.getElementById("mobMetricRows");
        if (gapsEl && typeof state.agronomics.gaps_detected_count === "number") {
            gapsEl.textContent = state.agronomics.gaps_detected_count;
        }
        if (deadEl && typeof state.agronomics.dead_plants_count === "number") {
            deadEl.textContent = state.agronomics.dead_plants_count;
        }
        if (seedsEl && typeof state.agronomics.reseeded_seeds_count === "number") {
            seedsEl.textContent = state.agronomics.reseeded_seeds_count;
        }
        if (rowsEl && typeof state.agronomics.reviewed_rows_count === "number") {
            rowsEl.textContent = state.agronomics.reviewed_rows_count;
        }
    });

    // Commandes d'outils
    const btnTinesDown = document.getElementById("btnTinesDown");
    const btnTinesUp = document.getElementById("btnTinesUp");
    const btnTillerStart = document.getElementById("btnTillerStart");
    const btnTillerStop = document.getElementById("btnTillerStop");
    const btnSeederPulse = document.getElementById("btnSeederPulse");

    if (btnTinesDown) btnTinesDown.addEventListener("click", () => wsClient.sendCommand("PULSE:5:50:4000"));
    if (btnTinesUp) btnTinesUp.addEventListener("click", () => wsClient.sendCommand("PULSE:5:-50:4000"));
    if (btnTillerStart) btnTillerStart.addEventListener("click", () => wsClient.sendCommand("M6:70"));
    if (btnTillerStop) btnTillerStop.addEventListener("click", () => wsClient.sendCommand("STOP:6"));
    if (btnSeederPulse) btnSeederPulse.addEventListener("click", () => wsClient.sendCommand("PULSE:8:60:500"));

    // Liaison physique / mode d'exécution
    const bSim = document.getElementById("mobBtnModeSim");
    const bCoup = document.getElementById("mobBtnModeCoupled");
    const btnConnectHw = document.getElementById("mobBtnConnectHw");

    if (bSim) {
        bSim.addEventListener("click", () => {
            if (isLockedByActiveMission()) return;
            wsClient.setExecutionMode("SIMULATION_ONLY");
        });
    }
    if (bCoup) {
        bCoup.addEventListener("click", () => {
            if (isLockedByActiveMission()) return;
            wsClient.setExecutionMode("COUPLED");
        });
    }

    if (btnConnectHw) {
        btnConnectHw.addEventListener("click", () => {
            if (isLockedByActiveMission()) return;
            const tr = document.getElementById("mobHwTransport")?.value || "WIFI_HTTP";
            const target = document.getElementById("mobHwTarget")?.value.trim() || "192.168.4.1";
            wsClient.connectHardware({
                execution_mode: "COUPLED",
                transport: tr,
                host: tr === "USB_UART" ? "192.168.4.1" : target,
                port: 80,
                serial_port: tr === "USB_UART" ? target : "/dev/ttyACM0",
                baudrate: 115200,
                use_xor_checksum: true,
                username: "admin",
                password: "hyper1234"
            });
        });
    }

    drawPeanutVisionCanvas();
}

export function drawPeanutVisionCanvas() {
    const cvs = document.getElementById("peanutVisionCanvas");
    if (!cvs) return;
    const ctx = cvs.getContext("2d");
    if (!ctx) return;
    const W = cvs.width = cvs.clientWidth || 320;
    const H = cvs.height = cvs.clientHeight || 180;

    ctx.fillStyle = "#8d5b28";
    ctx.fillRect(0, 0, W, H);

    ctx.fillStyle = "#6d3b10";
    ctx.beginPath();
    ctx.moveTo(W * 0.34, 30); ctx.lineTo(W * 0.66, 30);
    ctx.lineTo(W * 0.84, H); ctx.lineTo(W * 0.16, H);
    ctx.fill();

    ctx.strokeStyle = "#00e676";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(W * 0.34, 30); ctx.lineTo(W * 0.16, H);
    ctx.moveTo(W * 0.66, 30); ctx.lineTo(W * 0.84, H);
    ctx.stroke();

    ctx.fillStyle = "#00b0ff";
    ctx.strokeStyle = "#00b0ff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(W / 2, H); ctx.lineTo(W / 2, 32);
    ctx.stroke();

    ctx.fillStyle = "#00e5ff";
    ctx.font = "bold 9px monospace";
    ctx.fillText("COULOIR INTER-RANG 46cm", W / 2 - 62, 20);
}
