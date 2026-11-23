/**
 * static/js/mobile/touch_controls.js
 * Pavé tactile DRIVE/TANK, boutons d'armement et arrêt d'urgence ESTOP.
 */

import { wsClient } from "../core/ws_client.js";

export function initTouchControls() {
    const btnEstop = document.getElementById("btnEstop");
    const btnArm = document.getElementById("btnArm");

    if (btnEstop) {
        btnEstop.addEventListener("click", () => wsClient.estop());
    }
    if (btnArm) {
        btnArm.addEventListener("click", () => wsClient.arm());
    }

    const padButtons = document.querySelectorAll("[data-pad-cmd]");
    padButtons.forEach(btn => {
        const cmd = btn.getAttribute("data-pad-cmd");
        const sendPad = () => {
            if (cmd === "FORWARD") wsClient.drive(30, 0, 300);
            else if (cmd === "BACKWARD") wsClient.drive(-30, 0, 300);
            else if (cmd === "LEFT") wsClient.tank(-25, 25, 250);
            else if (cmd === "RIGHT") wsClient.tank(25, -25, 250);
            else if (cmd === "STOP") wsClient.sendCommand("STOP");
        };

        btn.addEventListener("touchstart", (e) => {
            e.preventDefault();
            sendPad();
        });
        btn.addEventListener("mousedown", (e) => {
            sendPad();
        });
    });
}

