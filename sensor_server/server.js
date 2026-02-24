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

// ✅ ESP32 ALERT ENDPOINT
app.post("/alert", async (req, res) => {
    console.log("🚨 ALERT RECEIVED FROM ESP32:");
    console.log(req.body);

    const { device_code, state } = req.body;

    if (device_code && state) {
        // Map string state to tinyint (0=SAFE/CLEAR, 1=APPROACHING, 2=OCCUPIED)
        let statusInt = 0;
        if (state === "APPROACHING") statusInt = 1;
        if (state === "OCCUPIED") statusInt = 2;

        try {
            await pool.query(
                "UPDATE zones SET status = ?, checked_at = CURRENT_TIMESTAMP WHERE device_code = ?",
                [statusInt, device_code]
            );
            console.log(`✅ Database updated: ${device_code} -> ${state} (${statusInt})`);
        } catch (err) {
            console.error("❌ Database update failed:", err);
        }
    }

    console.log("-----------------------------");

    res.send({ status: "OK" });
});

app.listen(PORT, () => {
    console.log(`✅ Server running on http://localhost:${PORT}`);
});
