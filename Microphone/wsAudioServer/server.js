const path = require("path");
const express = require("express");
const WebSocket = require("ws");
const fs = require("fs");

const app = express();

const WS_PORT = 8888;
const HTTP_PORT = 8000;

// Directory where recorded WAV files will be saved
const SAVE_DIR = "E:/rec_app";

if (!fs.existsSync(SAVE_DIR)) fs.mkdirSync(SAVE_DIR, { recursive: true });

// WebSocket server — receives raw PCM audio from ESP32 and broadcasts to browser clients
const wsServer = new WebSocket.Server({ port: WS_PORT }, () =>
  console.log(`WS server is listening at ws://localhost:${WS_PORT}`)
);

let connectedClients = [];

// Server-side recording state
let isServerRecording = false;
let recordedBuffers = [];
let recordedLength = 0;

wsServer.on("connection", (ws) => {
  console.log("Client Connected");

  connectedClients.push(ws);

  ws.on("message", (data) => {

    // Convert incoming buffer to string to check for control commands
    const message = data.toString();

    if (message === "START_RECORD") {
      isServerRecording = true;
      recordedBuffers = [];
      recordedLength = 0;
      console.log("Recording Started");
      return;
    }

    if (message === "STOP_RECORD") {
      isServerRecording = false;
      saveWavFile();
      console.log("Recording Saved");
      return;
    }

    // Forward alert notifications (e.g. from the Python detector) to all
    // other clients as text. Return early so they are never treated as audio.
    if (message.startsWith("ALERT:")) {
      console.log("Alert received:", message);
      connectedClients.forEach((client) => {
        if (client !== ws && client.readyState === client.OPEN) {
          client.send(message);
        }
      });
      return;
    }

    // Broadcast raw audio data to all connected browser clients
    connectedClients.forEach((client, i) => {
      if (client.readyState === client.OPEN) {
        client.send(data);
      } else {
        connectedClients.splice(i, 1);
      }
    });

    // Accumulate audio chunks if server recording is active
    if (isServerRecording) {
      recordedBuffers.push(Buffer.from(data));
      recordedLength += data.length;
    }
  });

  // Remove client from list on disconnect
  ws.on("close", () => {
    console.log("Client Disconnected");
    connectedClients = connectedClients.filter((c) => c !== ws);
  });

  // Handle unexpected errors without crashing the server
  ws.on("error", (err) => {
    console.error("WebSocket error:", err.message);
    connectedClients = connectedClients.filter((c) => c !== ws);
  });
});

// Writes accumulated PCM buffers to a WAV file with a proper header
function saveWavFile() {
  const sampleRate = 44100;
  const channels = 1;
  const bitsPerSample = 16;

  const byteRate = sampleRate * channels * bitsPerSample / 8;
  const blockAlign = channels * bitsPerSample / 8;
  const dataSize = recordedLength;

  // Build the 44-byte WAV header
  const buffer = Buffer.alloc(44);
  buffer.write("RIFF", 0);
  buffer.writeUInt32LE(36 + dataSize, 4);
  buffer.write("WAVE", 8);
  buffer.write("fmt ", 12);
  buffer.writeUInt32LE(16, 16);
  buffer.writeUInt16LE(1, 20);          // PCM format
  buffer.writeUInt16LE(channels, 22);
  buffer.writeUInt32LE(sampleRate, 24);
  buffer.writeUInt32LE(byteRate, 28);
  buffer.writeUInt16LE(blockAlign, 32);
  buffer.writeUInt16LE(bitsPerSample, 34);
  buffer.write("data", 36);
  buffer.writeUInt32LE(dataSize, 40);

  const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
  const filePath = path.join(SAVE_DIR, `recording_${timestamp}.wav`);

  const fileData = Buffer.concat([buffer, ...recordedBuffers]);
  fs.writeFileSync(filePath, fileData);

  recordedBuffers = [];
  recordedLength = 0;

  console.log("Saved:", filePath);
}

// HTTP server — serves the audio client page and static assets
app.use("/image", express.static("image"));
app.use("/js", express.static("js"));

app.get("/audio", (req, res) =>
  res.sendFile(path.resolve(__dirname, "./audio_client.html"))
);

app.listen(HTTP_PORT, () =>
  console.log(`HTTP server listening at http://localhost:${HTTP_PORT}`)
);
