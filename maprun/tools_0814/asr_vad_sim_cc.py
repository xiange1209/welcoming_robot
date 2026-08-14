#!/usr/bin/env python3
"""離線重現 voice_trigger_node 的 VAD 狀態機，確認「有語音時真的會轉去 COMMAND
並在講完後送 is_final」——不需要有人對麥克風講話。

用法: vad_sim.py <wav 或 .npy>
"""
import sys, wave, struct
import numpy as np
import sherpa_onnx

VAD_MODEL = "/home/user/models/vad/silero_vad.onnx"
SR, CHUNK = 16000, 512
SPEECH_START_FRAMES = 3          # 100 ms
SILENCE_FRAMES = 15              # 500 ms
CMD_INIT_WAIT_FRAMES = 93        # 3000 ms


def load(path):
    if path.endswith(".npy"):
        return np.load(path).astype(np.float32)
    w = wave.open(path, "rb")
    n, ch = w.getnframes(), w.getnchannels()
    d = struct.unpack("<%dh" % (n * ch), w.readframes(n))
    a = np.array(d, dtype=np.float32) / 32768.0
    if ch == 2:
        a = a.reshape(-1, 2).mean(axis=1)
    return a


cfg = sherpa_onnx.VadModelConfig()
cfg.silero_vad.model = VAD_MODEL
cfg.sample_rate = SR
cfg.num_threads = 1
vad = sherpa_onnx.VadModel.create(cfg)

audio = load(sys.argv[1])
print(f"音訊 {len(audio)/SR:.2f}s  RMS={np.sqrt(np.mean(audio**2)):.5f}")

state = "IDLE"
pos = 0
sil = 0
wait = 0
in_wait = False
published = 0
finals = 0
speech_frames = 0
nframes = len(audio) // CHUNK

for i in range(nframes):
    seg = audio[i * CHUNK:(i + 1) * CHUNK]
    is_speech = vad.is_speech(seg.tolist())
    speech_frames += int(is_speech)

    if state == "IDLE":
        if is_speech:
            pos += 1
            if pos >= SPEECH_START_FRAMES:
                state = "COMMAND"
                in_wait, wait, sil = True, 0, 0
                print(f"  [{i*CHUNK/SR:6.2f}s] IDLE -> COMMAND (語音起點)")
                published += 15   # buffer flush
        else:
            pos = 0
    elif state == "COMMAND":
        if in_wait:
            wait += 1
            published += 1
            if wait >= CMD_INIT_WAIT_FRAMES:
                in_wait = False
                sil = 0
                print(f"  [{i*CHUNK/SR:6.2f}s] 初始等待結束，開始監測語音結束")
        else:
            if is_speech:
                sil = 0
                published += 1
            else:
                sil += 1
                published += 1
                if sil >= SILENCE_FRAMES:
                    finals += 1
                    state = "IDLE"
                    pos = 0
                    print(f"  [{i*CHUNK/SR:6.2f}s] 偵測說話結束 -> 送 is_final，回 IDLE")

print(f"VAD 正幀 {speech_frames}/{nframes} ({100*speech_frames/max(nframes,1):.0f}%)")
print(f"發佈音訊塊 ~{published}，送出 is_final 次數 = {finals}，結束狀態 = {state}")
