#!/usr/bin/env python3
"""驗證 Astra S 麥克風能以節點使用的參數 (16k/mono/512/float32) 開啟並收到音訊。"""
import sys, time
import numpy as np
import sounddevice as sd

def find_astra():
    for i, d in enumerate(sd.query_devices()):
        if d["max_input_channels"] > 0 and "ASTRA" in d["name"].upper():
            return i, d["name"]
    return None, None

idx, name = find_astra()
print(f"ASTRA index={idx} name={name}")
if idx is None:
    sys.exit("找不到 ASTRA S 錄音裝置")

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
frames = []
def cb(indata, n, t, status):
    if status:
        print("status:", status)
    frames.append(indata.flatten().copy())

st = sd.InputStream(device=idx, samplerate=16000, channels=1,
                    blocksize=512, dtype="float32", callback=cb, latency="low")
st.start()
time.sleep(DUR)
st.stop(); st.close()

a = np.concatenate(frames) if frames else np.zeros(1, dtype=np.float32)
rms = float(np.sqrt(np.mean(a**2)))
peak = float(np.max(np.abs(a)))
print(f"blocks={len(frames)} samples={a.size} ({a.size/16000:.2f}s)")
print(f"RMS={rms:.6f} ({20*np.log10(rms+1e-12):.1f} dBFS)  peak={peak:.4f} ({20*np.log10(peak+1e-12):.1f} dBFS)")
np.save(sys.argv[2] if len(sys.argv) > 2 else "/tmp/mic.npy", a)
print("saved raw float32 ->", sys.argv[2] if len(sys.argv) > 2 else "/tmp/mic.npy")
