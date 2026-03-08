import asyncio
import websockets
import numpy as np
from scipy import signal
import tensorflow_hub as hub
import csv

# ── Audio settings ──────────────────────────────────────────────────────────
SAMPLE_RATE_IN     = 44100   # Rate used by the ESP32 / INMP441
SAMPLE_RATE_YAMNET = 16000   # Rate required by YAMNet
WINDOW_SECONDS     = 3.0     # How many seconds of audio to analyse at once
STEP_SECONDS       = 1.0     # sliding window step size (updates every 1 sec)
WINDOW_SAMPLES     = int(SAMPLE_RATE_IN * WINDOW_SECONDS)
STEP_SAMPLES       = int(SAMPLE_RATE_IN * STEP_SECONDS)

# Minimum composite score to trigger an alert
DETECTION_THRESHOLD = 0.5

WS_URL = "ws://localhost:8888"

# ── Load YAMNet ──────────────────────────────────────────────────────────────
print("Loading YAMNet model (first run downloads ~100 MB)...")
yamnet_model = hub.load("https://tfhub.dev/google/yamnet/1")

# Read the class name CSV that ships with the model
class_map_path = yamnet_model.class_map_path().numpy().decode()
class_names = []
with open(class_map_path) as f:
    for row in csv.DictReader(f):
        class_names.append(row["display_name"].lower())

# Define a "Composite Elephant Signature"
signature_classes = [
    "elephant",
    "animal",
    "roar",
    "cattle, bovinae",
    "wild animals"
]

# We need to actively PENALIZE sounds that are definitely not elephants
# (like you talking near the mic, or a fan blowing)
anti_signature_classes = [
    "speech",
    "human",
    "fan",
    "wind",
    "vehicle"
]

# Find the indices for our signature classes
signature_indices = []
for i, name in enumerate(class_names):
    if any(sig in name for sig in signature_classes):
        signature_indices.append(i)

# Find indices for sounds we want to ignore
anti_signature_indices = []
for i, name in enumerate(class_names):
    if any(anti in name for anti in anti_signature_classes):
        anti_signature_indices.append(i)

print(f"Tracking composite signature across {len(signature_indices)} target classes...")
print(f"Filtering out {len(anti_signature_indices)} background noise classes...")
print("Model ready. Connecting to audio server...\n")


# ── Main detection loop ──────────────────────────────────────────────────────
async def detect():
    audio_buffer = np.array([], dtype=np.int16)

    async with websockets.connect(WS_URL, max_size=10**7) as ws:
        print(f"Connected to {WS_URL}. Listening for elephant sounds...")

        async for message in ws:
            if isinstance(message, str):
                continue

            chunk = np.frombuffer(message, dtype=np.int16)
            audio_buffer = np.concatenate([audio_buffer, chunk])

            if len(audio_buffer) < WINDOW_SAMPLES:
                continue

            window       = audio_buffer[:WINDOW_SAMPLES]
            audio_buffer = audio_buffer[STEP_SAMPLES:]

            audio_float = window.astype(np.float32) / 32768.0

            resampled = signal.resample_poly(
                audio_float,
                SAMPLE_RATE_YAMNET,
                SAMPLE_RATE_IN,
            )

            scores, _, _ = yamnet_model(resampled)
            mean_scores  = np.mean(scores.numpy(), axis=0)

            # Calculate composite score (sum of target classes)
            target_score = sum(mean_scores[i] for i in signature_indices)
            
            # Calculate anti-score (sum of human speech, fans, vehicles)
            noise_score = sum(mean_scores[i] for i in anti_signature_indices)
            
            # Final signature = targets MINUS background noise
            composite_score = target_score - (noise_score * 1.5) # heavily penalise speech
            
            # Keep it between 0.0 and 1.0 (0% - 100%)
            composite_score = max(0.0, min(composite_score, 1.0))
            
            top_class = class_names[int(np.argmax(mean_scores))].title()

            print(f"Top class: {top_class:<35} | Elephant: {composite_score:.3f} | Vehicle/Speech: {noise_score:.3f}")

            # TRUTH CONDITIONS
            if composite_score >= DETECTION_THRESHOLD:
                print(f"🐘 ELEPHANT DETECTED! Signature Match: {composite_score:.1%}")
                await ws.send(f"ALERT:ELEPHANT_DETECTED:{composite_score:.3f}")
            elif noise_score >= 0.3 and composite_score < 0.2:
                print(f"🚗 VEHICLE/SPEECH DETECTED! Signature Match: {noise_score:.1%}")
                await ws.send(f"ALERT:VEHICLE_DETECTED:{noise_score:.3f}")


if __name__ == "__main__":
    asyncio.run(detect())
