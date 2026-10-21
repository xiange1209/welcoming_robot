# -*- coding: utf-8 -*-
"""2026-10-21：語音辨識（ASR）修補的離線測試。筆電沒有 ROS／sherpa_onnx／sounddevice，全部用替身。

背景（10/20 實車，筆電端從原始檔複算）：10 句參考稿、單一說話者，五輪 CER 42.1~51.3%；
句首 2 字錯誤率 68.8% vs 句尾 2 字 33.3%（48 句）；疊字（「信信用卡」「在在哪裡」）遍布整句；
input overflow 全開 74 次/分；熱詞要手動符號連結才載入；平板朗讀被收進來、文字層回音過濾 0/2。

改了什麼 -> 這裡怎麼測
  1. voice_trigger 觸發那一塊被送兩次   -> 真的 process_audio_chunk（object.__new__ 假 self），VAD 替身照腳本回 True/False
  2. 收音緩衝 latency 寫死 "low"         -> 參數解析＋voice_trigger 真的 __init__ 一遍，看 sd.InputStream 收到什麼
  3. 除錯錄音 debug_wav_dir              -> write_wav_int16 讀回比對；speech_recognizer 真的 __init__＋recognize_audio（辨識器替身）
  4. asr_chain.launch.py 四個參數        -> AST 查預設與接線；_preflight 用假 context／假 arecord 真的跑一遍
  替身 Node 照 rclpy 的規則擋參數型別（沒開 dynamic_typing 時型別不符就丟例外），不是什麼都收。

不測（筆電做不到，要上車看 log）
  ✗ PortAudio 的 "high" 實際是幾毫秒、overflow 有沒有變少 -> log「收音緩衝 latency='high' -> 實際 … ms」＋ input overflow 次數
  ✗ launch_ros 把 dict 轉成節點參數那一步 -> log「ASR 參數：…」＋ 節點的「熱詞已啟用」「除錯錄音已開啟」
  ✗ sherpa-onnx 真的解碼；疊字到底從哪來 -> 這正是除錯錄音要回答的

用法：python -X utf8 maprun/tools_1021/test_asr_fixes.py（環境變數 SMARTNAV_REPO 可覆寫 repo 根目錄）
"""
import ast
import importlib.abc
import importlib.machinery
import importlib.util
import os
import re
import subprocess
import sys
import tempfile
import threading
import traceback
import types
import wave
from collections import deque
from pathlib import Path

sys.dont_write_bytecode = True  # 不在 src/ 底下留 __pycache__

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

import numpy as np  # noqa: E402

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
PKG = CAR / "src/smartnav_audio"
PATHS = {
    "vt": PKG / "smartnav_audio/voice_trigger_node.py",
    "ar": PKG / "smartnav_audio/audio_recorder.py",
    "sr": PKG / "smartnav_audio/speech_recognizer_node.py",
    "launch": PKG / "launch/asr_chain.launch.py",
}
TREE = {k: ast.parse(p.read_text(encoding="utf-8")) for k, p in PATHS.items()}

# launch 四個新參數：(環境變數, 預設值)
EXPECTED = {
    "hotwords_file": ("ASR_HOTWORDS_FILE", ""),
    "mute_during_tts": ("ASR_MUTE_DURING_TTS", "true"),
    "debug_wav_dir": ("ASR_DEBUG_WAV_DIR", ""),
    "audio_latency": ("ASR_AUDIO_LATENCY", "high"),
}
DECLS = {}  # AST 從 launch 讀到的 {參數: (環境變數, 預設)}，第 6 段的 _preflight 實跑拿它當預設

results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


def section(title, fn):
    print(f"── {title} ──")
    try:
        fn()
    except Exception as e:  # noqa: BLE001  一段爆掉不能讓後面的段落沒跑
        check(False, f"這一段丟例外：{e!r}")
        traceback.print_exc(limit=4, file=sys.stdout)


# ═══════════════ 替身 ═══════════════

STUB_PREFIXES = (
    "rclpy", "rcl_interfaces", "std_msgs", "smartnav_msgs", "sherpa_onnx", "sounddevice",
    "ament_index_python", "opencc", "launch", "launch_ros",
)


class _AnyMeta(type):
    """替身類別：ReliabilityPolicy.BEST_EFFORT 這類類別屬性一律回自己。"""

    def __getattr__(cls, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return cls


class StubModule(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        cls = _AnyMeta(name, (), {"__init__": lambda self, *a, **k: None, "__call__": lambda self, *a, **k: self})
        setattr(self, name, cls)
        return cls


class StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, fullname, path, target=None):
        if fullname.split(".")[0] in STUB_PREFIXES:
            return importlib.machinery.ModuleSpec(fullname, self, is_package=True)
        return None

    def create_module(self, spec):
        m = StubModule(spec.name)
        m.__path__ = []
        return m

    def exec_module(self, module):
        pass


class Log:
    def __init__(self):
        self.lines = []

    def debug(self, msg, *a, **k):
        pass

    def info(self, msg, *a, **k):
        self.lines.append(("I", str(msg)))

    def warning(self, msg, *a, **k):
        self.lines.append(("W", str(msg)))

    warn = warning

    def error(self, msg, *a, **k):
        self.lines.append(("E", str(msg)))

    def find(self, level, pattern):
        return [m for lv, m in self.lines if lv == level and re.search(pattern, m)]


class ParameterDescriptor:
    def __init__(self, description="", dynamic_typing=False, **kw):
        self.description = description
        self.dynamic_typing = dynamic_typing


def _ptype(v):
    if isinstance(v, bool):
        return "BOOL"
    if isinstance(v, int):
        return "INTEGER"
    if isinstance(v, float):
        return "DOUBLE"
    if isinstance(v, str):
        return "STRING"
    if isinstance(v, (list, tuple)):
        return "DOUBLE_ARRAY" if all(isinstance(x, float) for x in v) else "ARRAY"
    return type(v).__name__


class FakeParam:
    def __init__(self, value):
        self.value = value

    def get_parameter_value(self):
        t, v = _ptype(self.value), self.value
        return types.SimpleNamespace(
            string_value=v if t == "STRING" else "",
            integer_value=v if t == "INTEGER" else 0,
            double_value=v if t == "DOUBLE" else 0.0,
            bool_value=v if t == "BOOL" else False,
            double_array_value=list(v) if t == "DOUBLE_ARRAY" else [],
        )


class FakeNode:
    """rclpy.node.Node 替身。參數照 rclpy 規則：沒開 dynamic_typing 時，覆寫值型別要跟預設一樣，否則宣告當下丟例外。"""

    overrides = {}

    def __init__(self, name, *a, **k):
        self._params = {}
        self._log = Log()

    def declare_parameter(self, name, value=None, descriptor=None, *a, **k):
        if name in self.overrides:
            ov = self.overrides[name]
            if not getattr(descriptor, "dynamic_typing", False) and _ptype(ov) != _ptype(value):
                raise TypeError(f"InvalidParameterTypeException：{name} 宣告型別 {_ptype(value)}，收到 {_ptype(ov)}")
            value = ov
        self._params[name] = value
        return FakeParam(value)

    def get_parameter(self, name):
        return FakeParam(self._params[name])

    def get_logger(self):
        return self._log

    def create_subscription(self, *a, **k):
        return object()

    def create_publisher(self, *a, **k):
        return types.SimpleNamespace(publish=lambda msg: None)

    def create_timer(self, *a, **k):
        return object()

    def destroy_node(self):
        pass


class FakeInputStream:
    """sd.InputStream 替身：記下收到的參數；latency 照 PortAudio 常見值換成秒數。"""

    made = []

    def __init__(self, **kw):
        self.kw = kw
        lat = kw.get("latency")
        self.latency = {"low": 0.0087, "high": 0.0348}.get(lat, lat)
        FakeInputStream.made.append(self)

    def start(self):
        pass

    def stop(self):
        pass

    def close(self):
        pass


sys.meta_path.insert(0, StubFinder())
_node_mod = types.ModuleType("rclpy.node")
_node_mod.Node = FakeNode
_rcl_msg = types.ModuleType("rcl_interfaces.msg")
_rcl_msg.ParameterDescriptor = ParameterDescriptor
sys.modules["rclpy.node"] = _node_mod
sys.modules["rcl_interfaces.msg"] = _rcl_msg
sys.path.insert(0, str(PKG))

import smartnav_audio.audio_recorder as ar  # noqa: E402
import smartnav_audio.speech_recognizer_node as srn  # noqa: E402
import smartnav_audio.voice_trigger_node as vt  # noqa: E402
from smartnav_audio.voice_utils import AudioCodec  # noqa: E402

ar.sd.InputStream = FakeInputStream


def calls(tree, name):
    """樹裡所有 name(...) 或 x.name(...) 呼叫"""
    out = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            f = n.func
            fname = f.id if isinstance(f, ast.Name) else f.attr if isinstance(f, ast.Attribute) else None
            if fname == name:
                out.append(n)
    return out


def kw(call, name):
    return next((k.value for k in call.keywords if k.arg == name), None)


def const(n):
    return n.value if isinstance(n, ast.Constant) else None


# ═══════════════ 1. voice_trigger 狀態機 ═══════════════


def chunk(k, n=512):
    return np.full(n, k / 1024.0, dtype=np.float32)


def cid(a):
    return int(round(float(a[0]) * 1024))


class ScriptVad:
    def __init__(self):
        self.script = deque()

    def is_speech(self, samples):
        return self.script.popleft()


def make_trigger(start=3, silence=3, initial=5):
    o = object.__new__(vt.VoiceTriggerNode)
    o._log = Log()
    o._lock = threading.Lock()
    o._state = vt.TriggerState.IDLE
    o._vad_positive_frames = 0
    o._silence_frames = 0
    o._command_wait_frames = 0
    o._in_command_initial_wait = False
    o._audio_buffer = deque(maxlen=15)
    o.speech_start_frames, o.silence_frames_threshold, o.command_initial_wait_frames = start, silence, initial
    o.vad = ScriptVad()
    o.sent = []
    o._publish_audio = lambda audio, is_final=False: o.sent.append((cid(audio), bool(is_final)))
    return o


def run(o, first_id, flags):
    """依序餵塊（flags[i] = 第 i 塊的 VAD 判定），回傳每次呼叫新送出的 [(塊號, is_final)]"""
    per_call = []
    for i, flag in enumerate(flags):
        o.vad.script.append(flag)
        n0 = len(o.sent)
        o.process_audio_chunk(chunk(first_id + i))
        per_call.append(o.sent[n0:])
    return per_call


def t_trigger():
    o = make_trigger(start=3, silence=3, initial=5)
    calls_ = run(o, 1, [False] * 20 + [True] * 3)  # 塊 1~20 靜音、21~23 講話 -> 第 23 塊觸發
    check(all(c == [] for c in calls_[:-1]), "IDLE 期間（塊 1~22）一塊都不送")
    trig = calls_[-1]
    ids = [i for i, _ in trig]
    check(ids == list(range(9, 24)), f"觸發那一刻送出緩衝裡最後 15 塊、依序（9~23）：{ids}")
    check(ids.count(23) == 1, f"觸發的那一塊（23）只送 1 次（修正前 2 次）：{ids.count(23)} 次")
    check(not any(f for _, f in trig), "觸發時送出的都不是 is_final")
    check(o._state == vt.TriggerState.COMMAND and o._in_command_initial_wait, "進入 COMMAND、初始等待中")
    check(o._command_wait_frames == 1, f"觸發那塊照樣算進初始等待（只計數不重送，初始等待長度不變）：{o._command_wait_frames}")
    check(len(o._audio_buffer) == 0, "緩衝已清空")

    calls_ = run(o, 24, [False] * 4)  # 初始等待共 5 塊（含觸發那塊）：靜音也照送、不收句
    check(calls_ == [[(24 + i, False)] for i in range(4)], f"初始等待期每塊恰送 1 次（含靜音塊）：{calls_}")
    check(not o._in_command_initial_wait, "第 5 塊後初始等待結束")
    calls_ = run(o, 28, [True, True, False, False, False])
    check(calls_[:4] == [[(28, False)], [(29, False)], [(30, False)], [(31, False)]],
          "說話中每塊恰送 1 次；靜音未滿門檻也照送")
    check(calls_[4] == [(32, True)], f"連續 3 塊靜音 -> 第 3 塊以 is_final 送出：{calls_[4]}")
    check(o._state == vt.TriggerState.IDLE and o._vad_positive_frames == 0, "收句後回到 IDLE、正幀計數歸零")
    check([i for i, _ in o.sent] == list(range(9, 33)), "整段送出的塊號 9~32 連續：無重複、無遺漏、順序不亂")
    check([f for _, f in o.sent].count(True) == 1, "整段只有最後一塊是 is_final")

    calls_ = run(o, 33, [False, False, True, True, True])  # 第二輪：上一輪的塊不能再出現
    check(all(c == [] for c in calls_[:-1]) and calls_[-1] == [(i, False) for i in range(33, 38)],
          f"第二輪觸發只送回到 IDLE 之後的塊（33~37 各一次）：{calls_[-1]}")

    o = make_trigger(start=1, silence=3, initial=5)
    calls_ = run(o, 1, [True, False])
    check(calls_ == [[(1, False)], [(2, False)]], f"speech_start_frames=1（緩衝只有觸發那塊）：每塊仍只送一次：{calls_}")


# ═══════════════ 2. 收音緩衝 audio_latency ═══════════════


def t_latency():
    parse = vt.parse_audio_latency
    cases = [("high", "high"), ("LOW", "low"), (" High ", "high"), ("0.2", 0.2), (0.2, 0.2), (1, 1.0),
             ("", None), ("medium", None), ("-0.1", None), (0, None), ("nan", None), ("inf", None), (True, None)]
    bad = [(v, parse(v), want) for v, want in cases if parse(v) != want]
    check(not bad, f"parse_audio_latency：low／high／正秒數照收，空字串、負數、NaN、打錯字回 None：{bad or '全對'}")

    check(ar.AudioRecorder(logger=Log()).latency == "high", "AudioRecorder 不帶 latency 時預設 'high'")

    def boot(**overrides):
        FakeNode.overrides = overrides
        node = vt.VoiceTriggerNode()
        stream = FakeInputStream.made[-1]
        node._stop_recorder()
        return node, stream

    node, s = boot()
    check(s.kw.get("latency") == "high", f"voice_trigger 預設：sd.InputStream(latency='high')（修正前寫死 'low'）：{s.kw.get('latency')!r}")
    check(node.get_logger().find("I", r"收音緩衝 latency='high' -> 實際 34\.8 ms"), "啟動後 log 印出實際緩衝（秒 -> 毫秒）")
    check(s.kw.get("samplerate") == 16000 and s.kw.get("blocksize") == 512, "其他 InputStream 參數不變（16000 Hz、每塊 512）")
    for given, want in (("low", "low"), ("0.2", 0.2), (0.2, 0.2)):
        node, s = boot(audio_latency=given)
        check(s.kw.get("latency") == want,
              f"audio_latency={given!r}（{type(given).__name__}）-> InputStream(latency={want!r})：{s.kw.get('latency')!r}")
    node, s = boot(audio_latency="bogus")
    check(s.kw.get("latency") == "high" and node.get_logger().find("W", r"audio_latency='bogus' 無法辨識"),
          "看不懂的值 -> 警告並退回 'high'（麥克風照樣開得起來）")
    try:
        FakeNode.overrides = {"chunk_size": "512"}
        vt.VoiceTriggerNode()
        strict = False
    except TypeError:
        strict = True
    check(strict, "替身 Node 真的會擋型別不符（chunk_size 給字串 -> 例外）——上面 0.2（double）過得了是 dynamic_typing 的功勞")


# ═══════════════ 3. write_wav_int16 ═══════════════


def read_wav(path):
    with wave.open(str(path), "rb") as wf:
        meta = (wf.getnchannels(), wf.getsampwidth(), wf.getframerate(), wf.getnframes())
        return meta, np.frombuffer(wf.readframes(wf.getnframes()), dtype="<i2")


def t_wav():
    with tempfile.TemporaryDirectory() as td:
        x = (0.5 * np.sin(2 * np.pi * 440 * np.arange(16000) / 16000)).astype(np.float32)
        p = os.path.join(td, "sine.wav")
        secs = srn.write_wav_int16(p, x, 16000)
        meta, data = read_wav(p)
        check(meta == (1, 2, 16000, 16000), f"wave 讀得回：單聲道、16-bit、16000 Hz、16000 點：{meta}")
        check(abs(secs - 1.0) < 1e-9, f"回傳秒數 = 點數／取樣率：{secs}")
        check(data.tobytes() == AudioCodec.encode_pcm_s16le(x),
              "量化跟 /audio_in 用的 encode_pcm_s16le 逐位元組相同（聽到的就是辨識器吃到的）")

        pcm = np.concatenate([np.arange(-32768, 32768, 7), [32767]]).astype(np.int16)
        p2 = os.path.join(td, "roundtrip.wav")
        srn.write_wav_int16(p2, AudioCodec.decode_pcm_s16le(pcm.tobytes()), 16000)
        check(np.array_equal(read_wav(p2)[1], pcm), "s16 -> 解碼成 float -> 寫檔 -> 讀回：每一點都一樣（含 -32768 與 32767 兩端）")

        y = np.array([-3.0, -1.5, -1.0, -0.5, 0.0, 0.5, 0.999, 1.0, 1.5, 3.0, np.nan], dtype=np.float32)
        p3 = os.path.join(td, "clip.wav")
        srn.write_wav_int16(p3, y, 8000)
        meta, got = read_wav(p3)
        want = [-32768, -32768, -32768, -16384, 0, 16384, 32735, 32767, 32767, 32767, 0]
        check(got.tolist() == want, f"超出 ±1 削波不溢位（+1.0／1.5／3.0 -> 32767，沒有繞回負數）、NaN -> 0：{got.tolist()}")
        check(meta[2] == 8000 and meta[3] == len(y), f"取樣率與點數照參數寫（8000 Hz、{len(y)} 點）：{meta}")


# ═══════════════ 4. 除錯錄音接線 ═══════════════


class FakeAsrStream:
    def __init__(self):
        self.fed = []

    def accept_waveform(self, sample_rate, a):
        self.fed.append(np.asarray(a).copy())

    def input_finished(self):
        pass


class FakeRecognizer:
    def __init__(self):
        self.text, self.endpoint, self.boom, self.resets = "", False, False, 0

    def create_stream(self):
        return FakeAsrStream()

    def is_ready(self, s):
        return False

    def decode_stream(self, s):
        pass

    def is_endpoint(self, s):
        return self.endpoint

    def get_result(self, s):
        if self.boom:
            raise RuntimeError("替身：解碼爆掉")
        return self.text

    def reset(self, s):
        self.resets += 1


def boot_recognizer(**overrides):
    FakeNode.overrides = overrides
    node = srn.SpeechRecognizerNode()
    node.recognizer = FakeRecognizer()  # 筆電沒有模型：換成替身
    node.stream = node.recognizer.create_stream()
    node.opencc = None
    return node


def feed(node, k, *, final=False, text="", endpoint=False, boom=False):
    """餵 0.1 秒、每段內容都不同的音訊；值都是 1/1024 的整數倍，寫成 s16 不會有捨入誤差"""
    a = ((np.arange(1600) % 200 - 100 + k) / 1024.0).astype(np.float32)
    node.recognizer.text, node.recognizer.endpoint, node.recognizer.boom = text, endpoint, boom
    node.recognize_audio(a, is_final=final)
    return a


def as_i16(*arrs):
    return np.clip(np.round(np.concatenate(arrs) * 32768.0), -32768, 32767).astype(np.int16)


def wavs(d):
    return sorted(f for f in os.listdir(d) if f.endswith(".wav"))


def t_debug_wav():
    with tempfile.TemporaryDirectory() as td:
        d = os.path.join(td, "asr_wav")  # 還不存在：節點要自己建
        node = boot_recognizer(debug_wav_dir=d, debug_wav_max_files=3)
        log = node.get_logger()
        check(os.path.isdir(d) and node._dbg_enabled and log.find("I", r"除錯錄音已開啟"), "目錄不存在時自動建立、開啟並 log")

        a1, a2 = feed(node, 1), feed(node, 2)
        check(wavs(d) == [], "還沒斷句時不寫檔")
        a3 = feed(node, 3, final=True, text="你好")
        files = wavs(d)
        check(len(files) == 1 and re.fullmatch(r"asr_\d{8}_\d{6}_0001_VAD\.wav", files[0]),
              f"VAD 斷句寫一檔，檔名 asr_YYYYmmdd_HHMMSS_序號_VAD.wav：{files}")
        meta, data = read_wav(os.path.join(d, files[0]))
        check(meta[:3] == (1, 2, 16000) and np.array_equal(data, as_i16(a1, a2, a3)),
              "內容 = 這個 stream 從頭吃進去的三段，逐點相同")
        check(log.find("I", r"^🎙 除錯錄音 .+0001_VAD\.wav（0\.30 秒，VAD 斷句）辨識：'你好'$"),
              "log：「🎙 除錯錄音 <路徑>（0.30 秒，VAD 斷句）辨識：'你好'」")

        b1, b2 = feed(node, 4), feed(node, 5, endpoint=True, text="")
        new = [f for f in wavs(d) if f not in files]
        check(len(new) == 1 and new[0].endswith("_0002_sherpa.wav"), f"sherpa 端點斷句也寫一檔（沒有辨識結果照寫）：{new}")
        check(new and np.array_equal(read_wav(os.path.join(d, new[0]))[1], as_i16(b1, b2)),
              "內容只有上一句之後的兩段（上一句沒混進來）")
        check(log.find("I", r"0002_sherpa\.wav（0\.20 秒，sherpa 斷句）辨識：''$"), "空結果的 log：辨識：''")
        check(node.recognizer.resets == 1, "端點斷句照樣 reset(stream)（原本行為不變）")

        files = wavs(d)
        feed(node, 6)
        feed(node, 7, boom=True)  # get_result 爆掉 -> 例外路徑重建 stream
        check(node._dbg_chunks == [] and log.find("E", r"語音辨識失敗"), "例外路徑重建 stream 時清掉累積")
        c1 = feed(node, 8, final=True)
        new = [f for f in wavs(d) if f not in files]
        check(len(new) == 1 and np.array_equal(read_wav(os.path.join(d, new[0]))[1], as_i16(c1)),
              "例外之後的下一個檔只有新音訊（爆掉那段沒混進來）")

        feed(node, 9, final=True, text="多一句")
        feed(node, 10, final=True, text="再一句")
        check(len(wavs(d)) == 3, f"上限 3：第 4、5 句不再寫：{len(wavs(d))} 個檔")
        check(len(log.find("W", r"已達上限")) == 1, "上限警告只出現一次")
        check(not node._dbg_enabled and node._dbg_chunks == [], "達上限後連累積都停掉（不吃記憶體）")
        node.destroy_node()

        node2 = boot_recognizer(debug_wav_dir=d, debug_wav_max_files=3)
        feed(node2, 11, final=True, text="哈囉")
        check(len(wavs(d)) == 3 and not node2._dbg_enabled and node2.get_logger().find("W", r"本次不錄"),
              "重開節點時目錄已有 3 個 -> 啟動就說不錄；舊檔一個都沒少（絕不刪檔）")
        node2.destroy_node()

        node3 = boot_recognizer()
        feed(node3, 12)
        feed(node3, 13, final=True, text="你好")
        check(not node3._dbg_enabled and node3._dbg_chunks == [], "debug_wav_dir 預設空字串 = 關，連累積都不做")
        node3.destroy_node()

        d4 = os.path.join(td, "asr_wav_mute")
        node4 = boot_recognizer(debug_wav_dir=d4, mute_during_tts=True)
        feed(node4, 14)
        node4._playback_status_callback(types.SimpleNamespace(data=True))
        check(node4._dbg_chunks == [], "mute 開、平板開始朗讀 -> 重建 stream 時清掉累積")
        feed(node4, 15)  # 朗讀中：recognize_audio 直接 return
        node4._playback_status_callback(types.SimpleNamespace(data=False))
        h1 = feed(node4, 16, final=True, text="我要開戶")
        f4 = wavs(d4)
        check(len(f4) == 1 and np.array_equal(read_wav(os.path.join(d4, f4[0]))[1], as_i16(h1)),
              "朗讀結束後的下一個檔只有新音訊（朗讀前、朗讀中的都不在裡面）")
        node4.destroy_node()


# ═══════════════ 5. AST：launch 參數、節點宣告 ═══════════════


def t_ast():
    la = TREE["launch"]
    for c in calls(la, "DeclareLaunchArgument"):
        dv = kw(c, "default_value")
        if c.args and isinstance(dv, ast.Call) and getattr(dv.func, "id", None) == "EnvironmentVariable":
            DECLS[const(c.args[0])] = (const(dv.args[0]), const(kw(dv, "default_value")))
    for name, want in EXPECTED.items():
        check(DECLS.get(name) == want,
              f"launch 參數 {name} = EnvironmentVariable({want[0]!r}, default_value={want[1]!r})：{DECLS.get(name)}")
    text = PATHS["launch"].read_text(encoding="utf-8")
    check(DECLS.get("mic_gain") == ("ASR_MIC_GAIN", "60") and DECLS.get("mic_mode") == ("ASR_MIC_MODE", "left")
          and 'ASR_MIC_GAIN", default_value="60"' in text and 'ASR_MIC_MODE", default_value="left"' in text,
          "舊參數 mic_gain=60、mic_mode=left 原樣（verify_0／verify_1 用 grep 查這兩行）")

    pf = next(f for f in la.body if isinstance(f, ast.FunctionDef) and f.name == "_preflight")
    used = {const(c.args[0]) for c in calls(pf, "LaunchConfiguration") if c.args}
    check(set(EXPECTED) <= used, f"_preflight 用 perform_substitution 取四個值：缺 {sorted(set(EXPECTED) - used) or '無'}")
    params = {}
    for c in calls(pf, "Node"):
        p = kw(c, "parameters")
        if isinstance(p, ast.List) and p.elts and isinstance(p.elts[0], ast.Dict):
            params[const(kw(c, "executable"))] = {const(k): v for k, v in zip(p.elts[0].keys, p.elts[0].values)}
    rec, trg = params.get("speech_recognizer", {}), params.get("voice_trigger", {})
    names = ("hotwords_file", "mute_during_tts", "debug_wav_dir")
    check(all(isinstance(rec.get(n), ast.Name) and rec[n].id == n for n in names),
          f"recognizer 帶到 {', '.join(names)}：{sorted(k for k in rec if k)}")
    check(isinstance(trg.get("audio_latency"), ast.Name) and trg["audio_latency"].id == "audio_latency",
          f"trigger 帶到 audio_latency：{sorted(k for k in trg if k)}")
    logs = [c for c in calls(pf, "LogInfo") if {n.id for n in ast.walk(c) if isinstance(n, ast.Name)} >= set(EXPECTED)]
    check(len(logs) == 1, "有一行 LogInfo 印出四個實際值")

    ar_tree = TREE["ar"]
    lat = [kw(c, "latency") for c in calls(ar_tree, "InputStream")]
    check(len(lat) == 1 and lat[0] is not None and const(lat[0]) is None,
          f"audio_recorder：sd.InputStream 的 latency 不再寫死字串（{ast.unparse(lat[0]) if lat and lat[0] else '沒有'}）")
    init = next(f for n in ar_tree.body if isinstance(n, ast.ClassDef) and n.name == "AudioRecorder"
                for f in n.body if isinstance(f, ast.FunctionDef) and f.name == "__init__")
    args = [a.arg for a in init.args.args]
    defaults = dict(zip(args[len(args) - len(init.args.defaults):], init.args.defaults))
    check(const(defaults.get("latency")) == "high", "audio_recorder：建構子多了 latency 參數、預設 'high'")

    def decl(tree):
        return {const(c.args[0]): c for c in calls(tree, "declare_parameter") if c.args}

    sd_ = decl(TREE["sr"])
    check("debug_wav_dir" in sd_ and const(sd_["debug_wav_dir"].args[1]) == "", "speech_recognizer 宣告 debug_wav_dir（預設 '' = 關）")
    check("debug_wav_max_files" in sd_ and const(sd_["debug_wav_max_files"].args[1]) == 1000,
          "speech_recognizer 宣告 debug_wav_max_files（預設 1000）")
    ra = next(f for n in TREE["sr"].body if isinstance(n, ast.ClassDef)
              for f in n.body if isinstance(f, ast.FunctionDef) and f.name == "recognize_audio")
    srcs = sorted(const(c.args[0]) for c in calls(ra, "_dump_debug_wav") if c.args)
    check(srcs == ["VAD", "sherpa"], f"recognize_audio 的兩個重設點各寫一次（VAD、sherpa）：{srcs}")

    vd = decl(TREE["vt"])
    al = vd.get("audio_latency")
    desc = al.args[2] if al is not None and len(al.args) > 2 else None
    check(al is not None and const(al.args[1]) == "high" and isinstance(desc, ast.Call)
          and const(kw(desc, "dynamic_typing")) is True,
          "voice_trigger 宣告 audio_latency（預設 'high'、dynamic_typing=True）")
    check(any(kw(c, "latency") is not None for c in calls(TREE["vt"], "AudioRecorder")), "voice_trigger 建 AudioRecorder 時傳 latency")


# ═══════════════ 6. launch：_preflight 實跑 ═══════════════


class LC:
    def __init__(self, name):
        self.name = name


class Ctx:
    def __init__(self, values):
        self.values = values

    def perform_substitution(self, sub):
        return self.values[sub.name]


class Rec:
    made = []

    def __init__(self, *a, **k):
        self.a, self.k = a, k
        Rec.made.append(self)


class NodeRec(Rec):
    pass


class LogRec(Rec):
    pass


def t_launch():
    spec = importlib.util.spec_from_file_location("asr_chain_launch_t", PATHS["launch"])
    lm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lm)
    lm.generate_launch_description()  # 只確認跑得完（替身 launch 不會真的宣告）
    lm.LaunchConfiguration, lm.Node, lm.LogInfo = LC, NodeRec, LogRec
    lm._device_index = lambda: 1
    lm.time = types.SimpleNamespace(sleep=lambda s: None)

    def fake_run(cmd, **k):
        out = "card 3: S [ASTRA S], device 0: USB Audio [USB Audio]\n" if cmd[:2] == ["arecord", "-l"] else ""
        return types.SimpleNamespace(returncode=0, stdout=out, stderr="")

    lm.subprocess = types.SimpleNamespace(run=fake_run, TimeoutExpired=subprocess.TimeoutExpired)

    def preflight(**over):
        Rec.made.clear()
        vals = {"mic_gain": "60", "mic_mode": "left"}
        vals.update({k: DECLS.get(k, EXPECTED[k])[1] for k in EXPECTED})  # 用 launch 檔裡真正的預設
        vals.update(over)
        lm._preflight(Ctx(vals))
        nodes = {r.k["executable"]: r.k["parameters"][0] for r in Rec.made if isinstance(r, NodeRec)}
        logs = [str(r.k.get("msg", "")) for r in Rec.made if isinstance(r, LogRec)]
        return nodes.get("speech_recognizer", {}), nodes.get("voice_trigger", {}), logs

    with tempfile.TemporaryDirectory() as share:
        hw = os.path.join(share, "config", "hotwords.txt")
        os.makedirs(os.path.dirname(hw))
        with open(hw, "w", encoding="utf-8") as f:
            f.write("开 户\n")
        lm.get_package_share_directory = lambda pkg: share

        rec, trg, logs = preflight()
        check(rec.get("hotwords_file") == hw, f"預設：hotwords_file = share/config/hotwords.txt（存在才用）：{rec.get('hotwords_file')!r}")
        check(rec.get("mute_during_tts") is True, f"預設：mute_during_tts = True（bool，不是字串）：{rec.get('mute_during_tts')!r}")
        check(rec.get("debug_wav_dir") == "" and rec.get("num_threads") == 2, "預設：debug_wav_dir 空 = 關；num_threads 仍是 2")
        check(trg.get("audio_latency") == "high", f"預設：trigger 收到 audio_latency='high'：{trg.get('audio_latency')!r}")
        check(trg.get("device") == 1 and trg.get("device_name_match") == "ASTRA" and trg.get("mic_mode") == "left",
              "trigger 原有參數不變（device、device_name_match、mic_mode）")
        check(any("ASR 參數" in m and hw in m and "mute_during_tts=True" in m and "audio_latency=high" in m for m in logs),
              f"LogInfo 印出實際值：{[m for m in logs if 'ASR 參數' in m]}")

        os.remove(hw)
        rec, _, _ = preflight()
        check(rec.get("hotwords_file") == "", "share 裡沒有 hotwords.txt -> 傳空字串（交給節點自己找）")

        def boom(pkg):
            raise KeyError(pkg)

        lm.get_package_share_directory = boom
        rec, _, _ = preflight()
        check(rec.get("hotwords_file") == "", "找不到套件 -> 傳空字串，不讓 launch 掛掉")

    rec, trg, logs = preflight(hotwords_file="~/hw.txt", mute_during_tts="false", debug_wav_dir="~/asr_wav_1021",
                               audio_latency="0.2")
    check(rec.get("hotwords_file") == os.path.expanduser("~/hw.txt"), f"指定熱詞檔 -> 展開 ~：{rec.get('hotwords_file')!r}")
    check(rec.get("mute_during_tts") is False, "ASR_MUTE_DURING_TTS=false -> False")
    check(rec.get("debug_wav_dir") == os.path.expanduser("~/asr_wav_1021"), f"debug_wav_dir 展開 ~：{rec.get('debug_wav_dir')!r}")
    check(trg.get("audio_latency") == 0.2, f"audio_latency='0.2' -> 0.2（float）：{trg.get('audio_latency')!r}")
    rec, trg, _ = preflight(audio_latency="  ")
    check(trg.get("audio_latency") == "high", "audio_latency 空白 -> 'high'")
    on = [v for v in ("true", "TRUE", "1", "yes", "on") if lm._truthy(v) is not True]
    off = [v for v in ("false", "0", "", "no", "off", "abc") if lm._truthy(v) is not False]
    check(not on and not off, f"mute 字串判讀：true／1／yes／on 為開、其他為關：{(on, off) if on or off else '全對'}")


section("1. voice_trigger：觸發那一塊只送一次（真的 process_audio_chunk，VAD 照腳本）", t_trigger)
section("2. 收音緩衝 audio_latency（解析＋voice_trigger 真的 __init__）", t_latency)
section("3. write_wav_int16", t_wav)
section("4. 除錯錄音接線（speech_recognizer 真的 __init__＋recognize_audio，辨識器替身）", t_debug_wav)
section("5. AST：launch 四個參數與接線、節點宣告", t_ast)
section("6. launch：_preflight 用假 context／假 arecord 實跑", t_launch)

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if results and all(results) else 1)
