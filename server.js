const http = require('http');
const fs   = require('fs');
const path = require('path');
const os   = require('os');
const { WebSocketServer } = require('ws');

function getLocalIP() {
  for (const ifaces of Object.values(os.networkInterfaces())) {
    for (const iface of ifaces) {
      if (iface.family === 'IPv4' && !iface.internal) return iface.address;
    }
  }
  return '127.0.0.1';
}

const PORT = process.env.PORT || 3000;
const ROOT = __dirname;

// ── Static file MIME map ──────────────────────────────────────────────────────

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.css':  'text/css; charset=utf-8',
  '.js':   'application/javascript; charset=utf-8',
  '.json': 'application/json',
  '.png':  'image/png',
  '.jpg':  'image/jpeg',
  '.svg':  'image/svg+xml',
  '.ico':  'image/x-icon',
  '.woff': 'font/woff',
  '.woff2':'font/woff2',
  '.ttf':  'font/ttf',
  '.bin':  'application/octet-stream',
};

// ── Buddy registry ────────────────────────────────────────────────────────────
// buddyId (string) → { device: WebSocket|null, client: WebSocket|null }

const buddies = new Map();

function getOrCreate(id) {
  if (!buddies.has(id)) buddies.set(id, { device: null, client: null });
  return buddies.get(id);
}

function send(ws, payload) {
  if (ws && ws.readyState === ws.OPEN) ws.send(payload);
}

// ── HTTP server (static files) ────────────────────────────────────────────────

const server = http.createServer((req, res) => {
  let urlPath = req.url.split('?')[0];

  // API: return server's local network IP (used by flash page to pre-fill host field)
  if (urlPath === '/api/ip') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    return res.end(JSON.stringify({ ip: getLocalIP(), port: PORT }));
  }

  if (urlPath === '/') urlPath = '/index.html';

  // Serve bundled libraries from node_modules
  let filePath;
  if (urlPath === '/esptool.js') {
    filePath = path.join(ROOT, 'node_modules/esptool-js/bundle.js');
  } else if (urlPath === '/crypto-js.js') {
    filePath = path.join(ROOT, 'node_modules/crypto-js/crypto-js.js');
  } else {
    filePath = path.join(ROOT, urlPath);
  }

  // Prevent directory traversal (only for user-supplied paths, not lib remaps)
  const isLibRemap = urlPath === '/esptool.js' || urlPath === '/crypto-js.js';
  if (!isLibRemap && !filePath.startsWith(ROOT)) {
    res.writeHead(403);
    return res.end('Forbidden');
  }

  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404, { 'Content-Type': 'text/plain' });
      return res.end('404 — not found');
    }
    const ext  = path.extname(filePath).toLowerCase();
    const mime = MIME[ext] || 'application/octet-stream';
    res.writeHead(200, { 'Content-Type': mime });
    res.end(data);
  });
});

// ── WebSocket server ──────────────────────────────────────────────────────────

const wss = new WebSocketServer({ noServer: true });

server.on('upgrade', (req, socket, head) => {
  // Only handle /ws upgrades
  if (!req.url.startsWith('/ws')) {
    socket.destroy();
    return;
  }
  wss.handleUpgrade(req, socket, head, ws => {
    wss.emit('connection', ws, req);
  });
});

wss.on('connection', (ws, req) => {
  const params = new URLSearchParams(req.url.replace('/ws?', ''));
  const rawId  = (params.get('id') || '').toUpperCase().trim();
  const role   = (params.get('role') || '').toLowerCase();

  if (!rawId || !['device', 'client'].includes(role)) {
    ws.close(4000, 'missing id or invalid role');
    return;
  }

  const id    = rawId.startsWith('BDY-') ? rawId : `BDY-${rawId}`;
  const entry = getOrCreate(id);

  // ── Buddy device connects (Raspberry Pi) ──────────────────────────────────

  if (role === 'device') {
    // Replace any stale device socket
    if (entry.device) entry.device.close(1001, 'replaced by new device');
    entry.device = ws;

    console.log(`[+] device   ${id}`);

    // If a browser client is already waiting, pair immediately
    if (entry.client && entry.client.readyState === entry.client.OPEN) {
      send(entry.client, JSON.stringify({ type: 'paired', id }));
      send(entry.device, JSON.stringify({ type: 'client_connected' }));
      console.log(`[~] paired   ${id}`);
    }

    ws.on('message', (data, isBinary) => {
      // Relay everything to the paired browser client
      if (entry.client && entry.client.readyState === entry.client.OPEN) {
        entry.client.send(data, { binary: isBinary });
      }
    });

    ws.on('close', () => {
      console.log(`[-] device   ${id}`);
      entry.device = null;
      send(entry.client, JSON.stringify({ type: 'buddy_disconnected' }));
      // Clean up entry if both gone
      if (!entry.client) buddies.delete(id);
    });

    ws.on('error', err => console.error(`[!] device ${id}:`, err.message));
    return;
  }

  // ── Browser client connects ───────────────────────────────────────────────

  if (role === 'client') {
    // Replace stale client
    if (entry.client) entry.client.close(1001, 'replaced by new client');
    entry.client = ws;

    console.log(`[+] client   ${id}`);

    if (entry.device && entry.device.readyState === entry.device.OPEN) {
      // Device already registered — pair immediately
      send(entry.client, JSON.stringify({ type: 'paired', id }));
      send(entry.device, JSON.stringify({ type: 'client_connected' }));
      console.log(`[~] paired   ${id}`);
    } else {
      // Device not yet present — tell browser to wait
      send(entry.client, JSON.stringify({ type: 'waiting', msg: 'buddy not online yet — waiting...' }));
    }

    ws.on('message', (data, isBinary) => {
      // Relay everything to the paired device
      if (entry.device && entry.device.readyState === entry.device.OPEN) {
        entry.device.send(data, { binary: isBinary });
      }
    });

    ws.on('close', () => {
      console.log(`[-] client   ${id}`);
      entry.client = null;
      send(entry.device, JSON.stringify({ type: 'client_disconnected' }));
      if (!entry.device) buddies.delete(id);
    });

    ws.on('error', err => console.error(`[!] client ${id}:`, err.message));
    return;
  }
});

// ── Start ─────────────────────────────────────────────────────────────────────

server.listen(PORT, () => {
  console.log(`\n  buddy hub running\n`);
  console.log(`  -> http://localhost:${PORT}\n`);
  console.log(`  ws protocol:`);
  console.log(`     Device : ws://[server-ip]:${PORT}/ws?id=BDY-XXXXX&role=device`);
  console.log(`     Browser: ws://[server-ip]:${PORT}/ws?id=BDY-XXXXX&role=client\n`);
});
