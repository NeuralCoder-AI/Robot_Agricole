/**
 * node_ws_bridge.js — Relais ultra-léger HTTP & WebSocket (0.0.0.0:8082 -> 127.0.0.1:8000)
 *
 * Exécuté par `node.exe` pour contourner le filtrage du Pare-feu Windows sur `python.exe`
 * lors d'un Partage de connexion mobile (Hotspot Wi-Fi / USB).
 * Redirige de manière transparente :
 *   - WebSocket : ws://<IP_HOTSPOT>:8082/ws       -> ws://127.0.0.1:8000/ws
 *   - REST/HTTP : http://<IP_HOTSPOT>:8082/api/*  -> http://127.0.0.1:8000/api/*
 */

const http = require('http');
const net = require('net');
const path = require('path');

const LISTEN_HOST = '0.0.0.0';
const LISTEN_PORT = parseInt(process.env.BRIDGE_PORT || '8082', 10);
const TARGET_HOST = '127.0.0.1';
const TARGET_PORT = parseInt(process.env.FASTAPI_PORT || '8000', 10);

// Essayer de charger http-proxy depuis mobile_app/node_modules si disponible,
// sinon utiliser le relais natif Node.js http + net (0 dépendance externe requise).
let httpProxyLib = null;
try {
  httpProxyLib = require(path.join(__dirname, 'mobile_app', 'node_modules', 'http-proxy'));
} catch {
  try {
    httpProxyLib = require('http-proxy');
  } catch {
    httpProxyLib = null;
  }
}

let proxy = null;
if (httpProxyLib && typeof httpProxyLib.createProxyServer === 'function') {
  proxy = httpProxyLib.createProxyServer({
    target: `http://${TARGET_HOST}:${TARGET_PORT}`,
    ws: true,
    changeOrigin: false,
    xfwd: true,
  });

  proxy.on('error', (err, req, res) => {
    if (res && typeof res.writeHead === 'function' && !res.headersSent) {
      res.writeHead(502, {
        'Content-Type': 'application/json',
        'Access-Control-Allow-Origin': '*',
      });
      res.end(JSON.stringify({ error: 'FastAPI backend unreachable on 127.0.0.1:8000', detail: err.message }));
    }
  });
}

const server = http.createServer((clientReq, clientRes) => {
  // Gestion CORS universelle pour l'App Mobile & Web
  clientRes.setHeader('Access-Control-Allow-Origin', '*');
  clientRes.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
  clientRes.setHeader('Access-Control-Allow-Headers', '*');

  if (clientReq.method === 'OPTIONS') {
    clientRes.writeHead(204);
    clientRes.end();
    return;
  }

  if (proxy) {
    proxy.web(clientReq, clientRes);
    return;
  }

  // Fallback natif HTTP proxy
  const options = {
    hostname: TARGET_HOST,
    port: TARGET_PORT,
    path: clientReq.url,
    method: clientReq.method,
    headers: {
      ...clientReq.headers,
      host: clientReq.headers.host || `${TARGET_HOST}:${TARGET_PORT}`,
    },
  };

  const proxyReq = http.request(options, (proxyRes) => {
    clientRes.writeHead(proxyRes.statusCode || 200, {
      ...proxyRes.headers,
      'Access-Control-Allow-Origin': '*',
    });
    proxyRes.pipe(clientRes, { end: true });
  });

  proxyReq.on('error', (err) => {
    if (!clientRes.headersSent) {
      clientRes.writeHead(502, { 'Content-Type': 'application/json' });
    }
    clientRes.end(JSON.stringify({ error: 'Bridge proxy error', detail: err.message }));
  });

  clientReq.pipe(proxyReq, { end: true });
});

// Relais WebSocket temps réel bidirectionnel (/ws)
server.on('upgrade', (req, clientSocket, head) => {
  clientSocket.setNoDelay(true);

  if (proxy) {
    proxy.ws(req, clientSocket, head);
    return;
  }

  // Fallback natif TCP/Upgrade WebSocket direct vers 127.0.0.1:8000
  const backendSocket = net.connect(TARGET_PORT, TARGET_HOST, () => {
    backendSocket.setNoDelay(true);

    const reqLines = [`${req.method} ${req.url} HTTP/${req.httpVersion}`];
    for (let i = 0; i < req.rawHeaders.length; i += 2) {
      reqLines.push(`${req.rawHeaders[i]}: ${req.rawHeaders[i + 1]}`);
    }
    reqLines.push('', '');
    backendSocket.write(reqLines.join('\r\n'));
    if (head && head.length > 0) {
      backendSocket.write(head);
    }

    backendSocket.pipe(clientSocket);
    clientSocket.pipe(backendSocket);
  });

  backendSocket.on('error', () => {
    try {
      clientSocket.destroy();
    } catch {}
  });

  clientSocket.on('error', () => {
    try {
      backendSocket.destroy();
    } catch {}
  });
});

server.listen(LISTEN_PORT, LISTEN_HOST, () => {
  console.log('======================================================================');
  console.log(`  [NODE WS BRIDGE] Relais actif sur http://${LISTEN_HOST}:${LISTEN_PORT}`);
  console.log(`  -> Redirection temps reel vers FastAPI http://${TARGET_HOST}:${TARGET_PORT}`);
  console.log(`  -> WebSocket Hotspot : ws://<IP_HOTSPOT>:${LISTEN_PORT}/ws`);
  console.log('======================================================================');
});
