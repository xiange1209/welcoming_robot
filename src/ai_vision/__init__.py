"""Vision AI 模組。

避免在 package import 階段就強制載入所有推理依賴，
讓 help、資料庫初始化等輕量命令可以在部分依賴尚未安裝時先執行。
"""

from importlib import import_module

_EXPORTS = {
	"InferenceEngine": (".inference_engine", "InferenceEngine"),
	"FaceDetector": (".face_detector", "FaceDetector"),
	"FaceRecognizer": (".face_recognizer", "FaceRecognizer"),
	"LivenessDetector": (".liveness_detector", "LivenessDetector"),
}

__all__ = list(_EXPORTS)


def __getattr__(name):
	if name not in _EXPORTS:
		raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

	module_name, attr_name = _EXPORTS[name]
	module = import_module(module_name, __name__)
	return getattr(module, attr_name)
