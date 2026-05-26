const aedes = require('aedes')();
const net = require('net');
const http = require('http');
const ws = require('websocket-stream');

// ==========================================
// MQTT BROKER SETUP
// ==========================================

// 1. MQTT over TCP on port 1883 (For ESP32)
const mqttServer = net.createServer(aedes.handle);
const MQTT_PORT = 1883;

mqttServer.listen(MQTT_PORT, '0.0.0.0', function () {
  console.log(`MQTT Broker (TCP) running on 0.0.0.0:${MQTT_PORT}`);
});

// 2. MQTT over WebSockets on port 8883 (For React Frontend)
const httpServer = http.createServer();
const WS_PORT = 8883;

ws.createServer({ server: httpServer }, aedes.handle);

httpServer.listen(WS_PORT, '0.0.0.0', function () {
  console.log(`MQTT Broker (WebSocket) running on 0.0.0.0:${WS_PORT}`);
});

// ==========================================
// BROKER EVENTS
// ==========================================
aedes.on('client', function (client) {
  console.log(`Client Connected: \x1b[33m${client ? client.id : 'unknown'}\x1b[0m`);
});

aedes.on('clientDisconnect', function (client) {
  console.log(`Client Disconnected: \x1b[31m${client ? client.id : 'unknown'}\x1b[0m`);
});

aedes.on('publish', function (packet, client) {
  if (client && !packet.topic.startsWith('$SYS')) {
    // Uncomment to debug:
    // console.log(`[${packet.topic}] ${packet.payload.toString()}`);
  }
});

aedes.on('subscribe', function (subscriptions, client) {
  if (client) {
    console.log(`\x1b[36m${client.id}\x1b[0m subscribed to: ${subscriptions.map(s => s.topic).join(', ')}`);
  }
});

console.log('MQTT Broker ready. Waiting for connections...');
