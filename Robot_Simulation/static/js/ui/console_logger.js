/**
 * static/js/ui/console_logger.js
 * Filtrage configurable et formatage horodaté de la console des commandes (TX/RX).
 * Permet d'ignorer ou d'afficher les trames de heartbeat PING / PONG / #T.
 */

export function isConsoleNoise(line) {
    if (!line) return true;
    const s = String(line).trim();
    if (!s) return true;
    const base = s.split("*")[0].trim().toUpperCase();
    if (base === "PING" || base === "PONG") return true;
    if (s.toUpperCase().startsWith("PING*") || s.toUpperCase().startsWith("PONG*")) return true;
    if (s.startsWith("#T ") || s === "#T" || s.toUpperCase().startsWith("#T ")) return true;
    return false;
}

export function formatLogEntry(direction, line) {
    const clean = String(line).trim();
    const now = new Date();
    const timeStr = now.toTimeString().split(" ")[0] + "." + String(now.getMilliseconds()).padStart(3, "0");

    let text = `[${timeStr}] `;
    let color = "#6ee7b7";
    let isError = false;

    if (direction === "TX") {
        text += `📤 [TX] ${clean}`;
        color = "#38bdf8";
    } else if (clean.startsWith("#D")) {
        text += `🏁 [FIN] ${clean} (Action terminée)`;
        color = "#fde047";
    } else if (clean.toUpperCase().startsWith("ERR:")) {
        text += `⚠️ [ERR] ${clean}`;
        color = "#ff1744";
        isError = true;
    } else if (clean.toUpperCase().startsWith("PING") || clean.toUpperCase().startsWith("PONG") || clean.startsWith("#T")) {
        text += `💓 [HEARTBEAT] ${clean}`;
        color = "#64748b";
    } else {
        text += `📥 [RX] ${clean}`;
        color = "#6ee7b7";
    }

    return { text, color, isError, raw: clean, direction, timeStr };
}

export class ConsoleLogger {
    constructor(containerElement, maxEntries = 250) {
        this.container = containerElement;
        this.maxEntries = maxEntries;
        this.ignorePong = true; // Ignorer PING/PONG par défaut
    }

    setIgnorePong(enabled) {
        this.ignorePong = Boolean(enabled);
    }

    clear() {
        if (this.container) {
            this.container.innerHTML = "";
        }
    }

    append(direction, line) {
        if (!line) return;
        if (this.ignorePong && isConsoleNoise(line)) {
            return;
        }
        if (!this.container) return;

        const entry = formatLogEntry(direction, line);
        const row = document.createElement("div");
        row.style.color = entry.color;
        row.style.lineHeight = "1.4";
        row.style.wordBreak = "break-all";
        row.style.marginBottom = "2px";
        if (entry.isError) row.style.fontWeight = "800";
        row.textContent = entry.text;

        this.container.appendChild(row);
        this.container.scrollTop = this.container.scrollHeight;

        while (this.container.childNodes.length > this.maxEntries) {
            this.container.removeChild(this.container.firstChild);
        }
    }
}
