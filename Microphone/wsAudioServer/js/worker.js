// worker.js (FIXED - Level Meter Only)
self.addEventListener("message", function (e) {
  let mean = 0;

  const samples_read = e.data.byteLength / 2; // ✅ 16-bit PCM = 2 bytes
  if (samples_read > 0) {
    const byteArray = new Int16Array(e.data);

    for (let i = 0; i < samples_read; ++i) {
      mean += Math.abs(byteArray[i]); // ✅ Absolute for signal strength
    }

    mean /= samples_read;
    self.postMessage(mean);
  }
});
