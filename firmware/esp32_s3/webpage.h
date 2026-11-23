// =============================================================================
//  ROBOT AGRICOLE – Firmware ESP32-S3 – webpage.h
//  Page web embarquée (HTML + CSS + JS dans un seul fichier, stockée en flash
//  via PROGMEM). Servie à l'adresse http://192.168.4.1/ par taskWeb.
//
//  API utilisée par la page :
//    GET  /api/state         -> JSON de l'état (interrogé toutes les 400 ms,
//                               ce qui entretient le watchdog web WDT)
//    GET  /api/cmd?c=<cmd>   -> réponse texte du parseur (identique à l'UART)
// =============================================================================
#pragma once
#include <pgmspace.h>

const char WEBPAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Robot Agricole</title>
<style>
  :root{
    --bg:#0f1511;--panel:#172019;--line:#243128;--txt:#e6efe8;--muted:#8fa396;
    --ok:#3ccf6a;--warn:#f0b429;--err:#ef5350;--acc:#4fa3ff;--btn:#223029;
  }
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  html,body{margin:0;padding:0;background:var(--bg);color:var(--txt);
    font:15px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
  header{position:sticky;top:0;z-index:10;background:var(--panel);border-bottom:1px solid var(--line);
    padding:10px 14px;display:flex;align-items:center;justify-content:space-between;gap:10px}
  header h1{font-size:17px;margin:0;font-weight:600}
  .badge{padding:4px 10px;border-radius:999px;font-size:13px;font-weight:600;background:var(--err);color:#fff}
  .badge.on{background:var(--ok);color:#06220f}
  .badge.off{background:var(--muted);color:#06220f}
  main{padding:12px;max-width:900px;margin:0 auto;display:grid;gap:12px}
  section{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px}
  section h2{font-size:14px;margin:0 0 10px;color:var(--muted);text-transform:uppercase;letter-spacing:.06em}
  .row{display:flex;gap:8px;flex-wrap:wrap}
  button{border:0;border-radius:10px;padding:12px 14px;font-size:15px;font-weight:600;color:var(--txt);
    background:var(--btn);cursor:pointer;flex:1;min-width:80px;touch-action:manipulation;user-select:none}
  button:active{filter:brightness(1.3)}
  button.arm{background:#1f6b3a}
  button.estop{background:#a62828;font-size:17px}
  button.stop{background:#7a5a10}
  button.small{padding:8px 10px;font-size:13px;flex:0 0 auto;min-width:0}
  .motor{display:grid;grid-template-columns:1fr auto;gap:4px 10px;padding:8px 0;border-top:1px solid var(--line)}
  .motor:first-of-type{border-top:0}
  .motor .lbl{font-weight:600}
  .motor .role{color:var(--muted);font-size:13px}
  .motor .val{font-variant-numeric:tabular-nums;text-align:right;min-width:56px}
  .motor .val .cur{color:var(--acc);font-weight:600}
  .motor input[type=range]{grid-column:1/2;width:100%;height:34px;accent-color:var(--acc)}
  .motor .zero{grid-column:2/3}
  .pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:360px;margin:0 auto}
  .pad button{aspect-ratio:1.6;font-size:22px}
  .pad .mid{background:#7a5a10}
  .speed{display:flex;align-items:center;gap:10px;margin-top:10px}
  .speed input{flex:1;accent-color:var(--acc)}
  .status{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:13px;
    white-space:pre-wrap;word-break:break-all;color:#cfe2d4}
  .kv{display:grid;grid-template-columns:repeat(auto-fit,minmax(110px,1fr));gap:6px;margin-top:8px}
  .kv div{background:var(--bg);border-radius:8px;padding:6px 8px;font-size:13px}
  .kv b{display:block;color:var(--muted);font-weight:500;font-size:11px;text-transform:uppercase}
  form.cmd{display:flex;gap:8px}
  form.cmd input{flex:1;background:var(--bg);border:1px solid var(--line);border-radius:10px;color:var(--txt);
    padding:12px;font-size:15px;font-family:ui-monospace,Menlo,Consolas,monospace;text-transform:uppercase}
  form.cmd button{flex:0 0 auto}
  #log{height:180px;overflow-y:auto;background:var(--bg);border-radius:8px;padding:8px;margin-top:8px;
    font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12.5px;line-height:1.5}
  #log .tx{color:var(--muted)}
  #log .ok{color:var(--ok)}
  #log .er{color:var(--err)}
  #log .tl{color:#9fb4ff}
  .foot{color:var(--muted);font-size:12px;text-align:center;padding:8px}
  .offline{background:var(--err);color:#fff;text-align:center;padding:6px;border-radius:8px;display:none}
</style>
</head>
<body>
<header>
  <h1>Robot Agricole</h1>
  <span id="badge" class="badge off">connexion…</span>
</header>
<main>
  <div id="offline" class="offline">Liaison perdue avec le robot – les moteurs commandés par le web seront arrêtés par le watchdog.</div>

  <section>
    <h2>Sécurité</h2>
    <div class="row">
      <button class="arm" onclick="cmd('ARM')">ARMER</button>
      <button class="stop" onclick="cmd('STOP')">STOP</button>
      <button class="estop" onclick="cmd('ESTOP')">ARRÊT D'URGENCE</button>
    </div>
  </section>

  <section>
    <h2>Conduite (DRIVE)</h2>
    <div class="pad">
      <button data-d="pivl" title="Pivot gauche">↺</button>
      <button data-d="fwd" title="Avant">▲</button>
      <button data-d="pivr" title="Pivot droite">↻</button>
      <button data-d="left" title="Gauche">◀</button>
      <button class="mid" onclick="cmd('STOP')" title="Stop">■</button>
      <button data-d="right" title="Droite">▶</button>
      <span></span>
      <button data-d="back" title="Arrière">▼</button>
      <span></span>
    </div>
    <div class="speed">
      <span>Vitesse</span>
      <input id="spd" type="range" min="10" max="100" value="50" oninput="spdv.textContent=this.value+' %'">
      <span id="spdv">50 %</span>
    </div>
    <div class="foot">Maintenir un bouton enfoncé pour rouler, relâcher pour arrêter.</div>
  </section>

  <section>
    <h2>Moteurs</h2>
    <div id="motors"></div>
  </section>

  <section>
    <h2>État</h2>
    <div id="status" class="status">—</div>
    <div class="kv">
      <div><b>Watchdog web</b><span id="k_wdt">–</span></div>
      <div><b>Watchdog UART</b><span id="k_uwdt">–</span></div>
      <div><b>Sauvegardé</b><span id="k_saved">–</span></div>
      <div><b>Mémoire libre</b><span id="k_heap">–</span></div>
      <div><b>Clients WiFi</b><span id="k_cli">–</span></div>
      <div><b>Firmware</b><span id="k_fw">–</span></div>
    </div>
  </section>

  <section>
    <h2>Commande libre</h2>
    <form class="cmd" onsubmit="return sendFree()">
      <input id="free" placeholder="ex. M5:60:3000  –  CFG:2  –  HELP" autocomplete="off" autocapitalize="characters">
      <button type="submit">Envoyer</button>
    </form>
    <div class="row" style="margin-top:8px">
      <button class="small" onclick="cmd('STATUS')">STATUS</button>
      <button class="small" onclick="cmd('GCFG')">GCFG</button>
      <button class="small" onclick="cmd('PERF')">PERF</button>
      <button class="small" onclick="cmd('VER')">VER</button>
      <button class="small" onclick="cmd('SAVE')">SAVE</button>
      <button class="small" onclick="clearLog()">Effacer</button>
    </div>
    <div id="log"></div>
  </section>
  <div class="foot">EPI-Niger – Robot agricole – interface locale 192.168.4.1</div>
</main>

<script>
// ---------------------------------------------------------------------------
//  Rôles des 8 moteurs (M1..M8)
// ---------------------------------------------------------------------------
const ROLES = [
  "Roue avant gauche (+ = avant)",
  "Roue avant droite (+ = avant)",
  "Roue arrière gauche (+ = avant)",
  "Roue arrière droite (+ = avant)",
  "Descente des dents (+ = descendre)",
  "Rotation désherbage",
  "Panneaux solaires (+ = déployer)",
  "Semoir (+ = semer)"
];
const POLL_MS = 400;          // période d'interrogation de /api/state (watchdog web)
const SLIDER_DEBOUNCE_MS = 120;

const $ = id => document.getElementById(id);
const log = $('log');
let lastOk = Date.now();

// ---------------------------------------------------------------------------
//  Journal
// ---------------------------------------------------------------------------
function addLog(txt, cls){
  const d = document.createElement('div');
  d.className = cls || '';
  d.textContent = txt;
  log.appendChild(d);
  while (log.children.length > 200) log.removeChild(log.firstChild);
  log.scrollTop = log.scrollHeight;
}
function clearLog(){ log.innerHTML = ''; }

// ---------------------------------------------------------------------------
//  Envoi d'une commande texte (même protocole que l'UART)
// ---------------------------------------------------------------------------
async function cmd(c, quiet){
  c = String(c).trim().toUpperCase();
  if (!c) return '';
  if (!quiet) addLog('> ' + c, 'tx');
  try{
    const r = await fetch('/api/cmd?c=' + encodeURIComponent(c), {cache:'no-store'});
    const t = (await r.text()).trim();
    if (!quiet || t.startsWith('ERR')) addLog(t, t.startsWith('ERR') ? 'er' : 'ok');
    return t;
  }catch(e){
    addLog('ERREUR LIAISON : ' + e.message, 'er');
    return '';
  }
}
function sendFree(){
  const i = $('free');
  cmd(i.value);
  i.value = '';
  return false;
}

// ---------------------------------------------------------------------------
//  Curseurs moteurs
// ---------------------------------------------------------------------------
const timers = {};
function buildMotors(){
  const box = $('motors');
  for (let i = 1; i <= 8; i++){
    const div = document.createElement('div');
    div.className = 'motor';
    div.innerHTML =
      `<div><span class="lbl">M${i}</span> <span class="role">${ROLES[i-1]}</span></div>` +
      `<div class="val"><span class="cur" id="cur${i}">0</span> / <span id="tgt${i}">0</span></div>` +
      `<input type="range" min="-100" max="100" value="0" id="sl${i}">` +
      `<button class="small zero" onclick="zero(${i})">0</button>`;
    box.appendChild(div);
    const sl = div.querySelector('input');
    sl.addEventListener('input', () => {
      $('tgt'+i).textContent = sl.value;
      clearTimeout(timers[i]);
      timers[i] = setTimeout(() => cmd(`M${i}:${sl.value}`, true), SLIDER_DEBOUNCE_MS);
    });
    sl.addEventListener('change', () => { clearTimeout(timers[i]); cmd(`M${i}:${sl.value}`, true); });
  }
}
function zero(i){
  $('sl'+i).value = 0; $('tgt'+i).textContent = '0';
  cmd(`STOP:${i}`, true);
}

// ---------------------------------------------------------------------------
//  Pavé de conduite : appui = DRIVE, relâchement = STOP
// ---------------------------------------------------------------------------
function driveFor(dir){
  const v = parseInt($('spd').value, 10);
  switch(dir){
    case 'fwd':  return `DRIVE:${v},0`;
    case 'back': return `DRIVE:${-v},0`;
    case 'left': return `DRIVE:${v},${-Math.round(v/2)}`;
    case 'right':return `DRIVE:${v},${Math.round(v/2)}`;
    case 'pivl': return `TANK:${-v},${v}`;
    case 'pivr': return `TANK:${v},${-v}`;
  }
  return 'STOP';
}
document.querySelectorAll('.pad button[data-d]').forEach(b => {
  const start = e => { e.preventDefault(); cmd(driveFor(b.dataset.d)); };
  const stop  = e => { e.preventDefault(); cmd('STOP', true); };
  b.addEventListener('pointerdown', start);
  b.addEventListener('pointerup', stop);
  b.addEventListener('pointercancel', stop);
  b.addEventListener('pointerleave', stop);
  b.addEventListener('contextmenu', e => e.preventDefault());
});

// ---------------------------------------------------------------------------
//  État périodique (/api/state) – entretient le watchdog web
// ---------------------------------------------------------------------------
async function poll(){
  try{
    const r = await fetch('/api/state', {cache:'no-store'});
    const s = await r.json();
    lastOk = Date.now();
    $('offline').style.display = 'none';
    const b = $('badge');
    b.textContent = s.armed ? 'ARMÉ' : 'ARRÊT D’URGENCE';
    b.className = 'badge ' + (s.armed ? 'on' : '');
    let st = `STATUS armed=${s.armed}`;
    for (let i = 0; i < 8; i++){
      st += ` M${i+1}=${s.cur[i]}`;
      $('cur'+(i+1)).textContent = s.cur[i];
      // On ne déplace pas un curseur que l'utilisateur manipule
      const sl = $('sl'+(i+1));
      if (document.activeElement !== sl){
        sl.value = s.tgt[i];
        $('tgt'+(i+1)).textContent = s.tgt[i] + (s.left[i] ? ` (${(s.left[i]/1000).toFixed(1)} s)` : '');
      }
    }
    $('status').textContent = st + `\nuptime=${(s.ms/1000).toFixed(0)} s`;
    $('k_wdt').textContent   = s.wdt ? s.wdt + ' ms' : 'inactif';
    $('k_uwdt').textContent  = s.uwdt ? s.uwdt + ' ms' : 'inactif';
    $('k_saved').textContent = s.saved ? 'oui' : 'non';
    $('k_heap').textContent  = Math.round(s.heap/1024) + ' ko';
    $('k_cli').textContent   = s.clients;
    $('k_fw').textContent    = s.fw;
  }catch(e){
    if (Date.now() - lastOk > 1500){
      $('offline').style.display = 'block';
      const b = $('badge'); b.textContent = 'hors ligne'; b.className = 'badge off';
    }
  }
}

buildMotors();
poll();
setInterval(poll, POLL_MS);
cmd('VER');
</script>
</body>
</html>
)rawliteral";
