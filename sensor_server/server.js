const express = require("express");
const bodyParser = require("body-parser");

const app = express();
const PORT = 3000;

app.use(bodyParser.json());

// ✅ ESP32 ALERT ENDPOINT
app.post("/alert", (req, res) => {
    console.log("🚨 ALERT RECEIVED FROM ESP32:");
    console.log(req.body);
    console.log("-----------------------------");

    res.send({ status: "OK" });
});

app.listen(PORT, () => {
    console.log(`✅ Server running on http://localhost:${PORT}`);
});
