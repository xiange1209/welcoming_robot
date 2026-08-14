#!/usr/bin/env python3
"""把一個 WAV 當成麥克風，分塊發到 /audio_in，最後一塊帶 is_final=True。
用來在「不需要人講話」的情況下驗證 speech_recognizer -> /user_text 這一段。

用法: inject_audio.py <wav路徑> [chunk_size]
"""
import sys, time, wave, struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from smartnav_msgs.msg import AudioData


def load_mono16k(path):
    w = wave.open(path, "rb")
    n, ch, sr = w.getnframes(), w.getnchannels(), w.getframerate()
    d = struct.unpack("<%dh" % (n * ch), w.readframes(n))
    a = np.array(d, dtype=np.float32) / 32768.0
    if ch == 2:
        a = a.reshape(-1, 2).mean(axis=1)
    return a, sr


def main():
    path = sys.argv[1]
    chunk = int(sys.argv[2]) if len(sys.argv) > 2 else 512
    audio, sr = load_mono16k(path)
    # 尾端補 0.5 秒靜音，讓最後一個字吐得出來
    audio = np.concatenate([audio, np.zeros(int(sr * 0.5), dtype=np.float32)])
    print(f"[inject] {path}  {len(audio)/sr:.2f}s  sr={sr}  chunk={chunk}")

    rclpy.init()
    node = Node("audio_injector")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
    pub = node.create_publisher(AudioData, "/audio_in", qos)

    # 等訂閱者接上
    for _ in range(100):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"[inject] 訂閱者數量 = {pub.get_subscription_count()}")
    time.sleep(0.5)

    nchunks = (len(audio) + chunk - 1) // chunk
    for i in range(nchunks):
        seg = audio[i * chunk:(i + 1) * chunk]
        if seg.size == 0:
            continue
        pcm = np.clip(np.round(seg * 32768.0), -32768, 32767).astype(np.int16)
        m = AudioData()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = "injector"
        m.data = pcm.tobytes()
        m.format = "pcm_s16le"
        m.sample_rate = sr
        m.channels = 1
        m.is_final = (i == nchunks - 1)
        pub.publish(m)
        time.sleep(chunk / sr)   # 依真實時間送，模擬串流
    print(f"[inject] 送完 {nchunks} 塊，最後一塊 is_final=True")
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
