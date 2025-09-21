// Simple WebSocket + static server to stream frames from lidar.bin
// Usage: node server.js ../logs_l2/lidar.bin

const fs = require("fs");
const path = require("path");
const express = require("express");

const app = express();
const PORT_HTTP = 8080;

// Serve static files, defaulting to object_detection.html
app.use(express.static(path.join(__dirname, "public"), { index: 'object_detection.html' }));
app.listen(PORT_HTTP, () => {
  console.log(`HTTP server:  http://localhost:${PORT_HTTP}`);
});

// Note: Live app already hosts a WS server on 8081. This dev server only serves HTTP.
// If you want to play a recorded lidar.bin via WS, run a separate script or change port.
