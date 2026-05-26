const { Aedes } = require('aedes');
const aedes = new Aedes();
const net = require('net');
const http = require('http');
const express = require('express');
const cors = require('cors');
const ws = require('websocket-stream');

const app = express();
app.use(cors());
app.use(express.json());

// ==========================================
// MQTT BROKER SETUP
// ==========================================
// 1. MQTT over TCP (For ESP32)
const mqttServer = net.createServer(aedes.handle);
const MQTT_PORT = 1883;

mqttServer.listen(MQTT_PORT, function () {
  console.log(`MQTT Broker (TCP) running on port ${MQTT_PORT}`);
});

// 2. MQTT over WebSockets (For React Frontend)
const httpServer = http.createServer(app);
const WS_PORT = 8883;

ws.createServer({ server: httpServer }, aedes.handle);

httpServer.listen(WS_PORT, function () {
  console.log(`MQTT Broker (WebSocket) running on port ${WS_PORT}`);
});

// ==========================================
// BROKER EVENTS
// ==========================================
aedes.on('client', function (client) {
  console.log(`Client Connected: \x1b[33m${(client ? client.id : client)}\x1b[0m`);
});

aedes.on('clientDisconnect', function (client) {
  console.log(`Client Disconnected: \x1b[31m${(client ? client.id : client)}\x1b[0m`);
});

aedes.on('publish', function (packet, client) {
  if (client && packet.topic !== '$SYS') {
    // console.log(`[${packet.topic}] ${packet.payload.toString()}`);
  }
});

// ==========================================
// EXPRESS API (Optional for Phase 2)
// ==========================================
const HTTP_PORT = 3000;

app.get('/api/status', (req, res) => {
  res.json({ status: 'Online', clients: aedes.connectedClients });
});

app.listen(HTTP_PORT, () => {
  console.log(`Express API running on port ${HTTP_PORT}`);
});
