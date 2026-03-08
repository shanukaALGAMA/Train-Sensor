const express = require("express");
const bodyParser = require("body-parser");
const mysql = require("mysql2/promise");
require("dotenv").config();

const app = express();
const PORT = 3000;

// Database connection pool
const pool = mysql.createPool({
    host: process.env.DB_HOST,
    user: process.env.DB_USER,
    password: process.env.DB_PASSWORD,
    database: process.env.DB_NAME,
    port: process.env.DB_PORT,
});

app.use(bodyParser.json());

// In-memory state for audio-based logic
const elephantTimers = {}; // { device_code: timestamp }
const recentVehicles = {}; // { device_code: timestamp }
let lastKnownDevice = "ZONE_1"; // Fallback for audio alerts without device specs

const ELEPHANT_TIMEOUT_MS = 10 * 60 * 1000; // 10 minutes
const VEHICLE_TIMEOUT_MS = 15 * 1000;       // 15 seconds

// ✅ ESP32 ALERT ENDPOINT
app.post("/alert", async (req, res) => {
    const { device_code, state } = req.body;

    if (device_code && state) {
        lastKnownDevice = device_code;

        const now = Date.now();
        const isElephantActive = elephantTimers[device_code] && (now - elephantTimers[device_code] < ELEPHANT_TIMEOUT_MS);
        const isVehicleActive = recentVehicles[device_code] && (now - recentVehicles[device_code] < VEHICLE_TIMEOUT_MS);

        if (state === "OCCUPIED") {
            // ALWAYS update OCCUPIED
            try {
                await pool.query("UPDATE zones SET status = 2, checked_at = CURRENT_TIMESTAMP WHERE device_code = ?", [device_code]);
                console.log(`✅ ESP32 updated: ${device_code} -> OCCUPIED (2)`);
            } catch (err) { console.error("❌ DB error:", err); }
        }
        else if (state === "APPROACHING") {
            if (isElephantActive) {
                console.log(`🐘 Ignored APPROACHING on ${device_code}: Elephant timer active (Forced OCCUPIED)`);
            } else if (isVehicleActive) {
                console.log(`🚗 Ignored APPROACHING on ${device_code}: Vehicle detected recently (False Positive Prevention)`);
            } else {
                try {
                    await pool.query("UPDATE zones SET status = 1, checked_at = CURRENT_TIMESTAMP WHERE device_code = ?", [device_code]);
                    console.log(`✅ ESP32 updated: ${device_code} -> APPROACHING (1)`);
                } catch (err) { console.error("❌ DB error:", err); }
            }
        }
        else if (state === "CLEAR" || state === "SAFE") {
            if (isElephantActive) {
                console.log(`🐘 Ignored CLEAR on ${device_code}: Elephant timer active for 10 mins`);
            } else {
                try {
                    await pool.query("UPDATE zones SET status = 0, checked_at = CURRENT_TIMESTAMP WHERE device_code = ?", [device_code]);
                    console.log(`✅ ESP32 updated: ${device_code} -> CLEAR (0)`);
                } catch (err) { console.error("❌ DB error:", err); }
            }
        }
    }
    res.send({ status: "OK" });
});

// ✅ AUDIO DETECTOR ENDPOINT
app.post("/audio-alert", async (req, res) => {
    const { type } = req.body;
    const device_code = req.body.device_code || lastKnownDevice;

    if (type) {
        if (type === "ELEPHANT_DETECTED") {
            elephantTimers[device_code] = Date.now();
            try {
                await pool.query("UPDATE zones SET status = 2, checked_at = CURRENT_TIMESTAMP WHERE device_code = ?", [device_code]);
                console.log(`🐘 AUDIO TRIGGER: Forced ${device_code} to OCCUPIED for 10 minutes`);

                // Set timeout to clear it after 10 minutes if no other events occur
                setTimeout(async () => {
                    const elapsed = Date.now() - (elephantTimers[device_code] || 0);
                    if (elapsed >= ELEPHANT_TIMEOUT_MS - 1000) { // Allow for slight delay in setTimeout execution
                        const [rows] = await pool.query("SELECT status FROM zones WHERE device_code = ?", [device_code]);
                        if (rows.length > 0 && rows[0].status === 2) { // Only clear if still OCCUPIED due to elephant
                            await pool.query("UPDATE zones SET status = 0, checked_at = CURRENT_TIMESTAMP WHERE device_code = ?", [device_code]);
                            console.log(`⏳ 10m Elephant timer expired for ${device_code}. Resetting to SAFE.`);
                        } else {
                            console.log(`🐘 Elephant timer for ${device_code} expired, but zone status is no longer OCCUPIED (2). No action.`);
                        }
                    }
                }, ELEPHANT_TIMEOUT_MS);
            } catch (err) {
                console.error("❌ DB error on elephant trigger:", err);
            }
        } else if (type === "VEHICLE_DETECTED") {
            recentVehicles[device_code] = Date.now();
            console.log(`🎙️ AUDIO: Vehicle/Speech heard near ${device_code}. Approach signals will be ignored for 15s.`);
        }
    }

    res.send({ status: "OK" });
});

app.listen(PORT, () => {
    console.log(`✅ Server running on http://localhost:${PORT}`);
});
