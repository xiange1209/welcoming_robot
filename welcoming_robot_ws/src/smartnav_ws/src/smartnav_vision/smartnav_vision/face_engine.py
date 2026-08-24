#!/usr/bin/env python3
"""臉部識別引擎核心類別

封裝 InsightFace 模型的臉部偵測、特徵提取功能
"""

import os
import numpy as np
import insightface
from pathlib import Path
from typing import List, Optional, Any
from dataclasses import dataclass

from smartnav_vision.face_utils import get_default_logger


@dataclass
class FaceDetectResult:
    """臉部偵測結果資料類別"""

    bbox: np.ndarray
    embedding: np.ndarray
    age: Optional[float] = None
    gender: Optional[str] = None


class FaceEngine:
    """臉部識別引擎 - 封裝 InsightFace 模型與特徵處理"""

    def __init__(
        self,
        model_name: str = "buffalo_sc",
        ctx_id: int = 0,
        det_thresh: float = 0.5,
        enable_gpu: bool = True,
        logger: Optional[Any] = None,
        det_size: int = 640,
        cpu_cores: int = 2,
    ):
        """初始化臉部引擎

        Args:
            model_name: InsightFace 模型名稱 (buffalo_sc, buffalo_m, buffalo_l 等)
            ctx_id: GPU 設備 ID (0 為第一個 GPU，-1 為 CPU)
            det_thresh: 偵測信心閾值 (0.0-1.0)
            enable_gpu: 若可用，啟用 GPU 加速
            logger: 日誌記錄器，若無則使用預設記錄器
            det_size: 偵測輸入邊長。★ 一定要明確給，見 _init_model 的說明
            cpu_cores: 最多用幾顆核心。0 = 不限制
        """
        self.model_name = model_name
        self.det_thresh = det_thresh
        self.enable_gpu = enable_gpu
        self.ctx_id = ctx_id if enable_gpu else -1
        self.det_size = int(det_size)
        self.cpu_cores = int(cpu_cores)
        self.logger = logger or get_default_logger(__name__)

        self.face_model: Optional[insightface.app.FaceAnalysis] = None
        self._init_model()

    def _init_model(self) -> None:
        """初始化 InsightFace FaceAnalysis 模型

        ★★ 2026-08-24：兩個「不改會白白多花一倍」的地方 ★★

        1. **`det_size` 一定要明確給。**
           `FaceAnalysis.prepare()` 的 `det_size` 預設是 `None`，而 None 會被
           換成 `DEFAULT_DET_SIZES = [(128, 128), (640, 640)]` ——
           `SCRFD.detect()` 對這個清單的迴圈**沒有 early break**，
           兩個尺寸都跑完才 vstack + NMS 合併。也就是**每一幀都偵測兩次**。
           實測佐證：`det_500m @640` 單獨 651 ms，串聯的 `all_auto` 1148 ms。
           而 128x128 對本專案毫無貢獻——50 cm 的臉在 640 寬的畫面裡是 70 px，
           縮到 128 只剩 14 px，已經在 SCRFD 偵測得到的邊緣。
           明確給 640 等於「拿掉一次沒有用的偵測」，召回率不變。

        2. **執行緒要限制。**
           onnxruntime 的 CPU EP 預設 intra-op 執行緒 = 核心數 = 4，
           而 Pi 4 就只有 4 顆。這是全系統唯一沒設上限的重推論
           （ASR 有 num_threads:=2、VAD 有 vad_num_threads:=1），
           它會直接把導航控制迴圈的核心搶走 —— 8/24 實測控制迴圈只有
           15.4 Hz 而不是設定的 20 Hz。

           insightface 的 `get_model()` 只轉送 `providers` 與
           `provider_options`，**沒有 `session_options`**，所以執行緒數
           傳不進去。這裡改用兩個一定有效的手段：
             - 建立 session 前設 `OMP_NUM_THREADS` 等環境變數
             - `sched_setaffinity` 直接把行程綁在部分核心上
               （不管 ORT 開幾條執行緒，作業系統都不會讓它跑滿全部核心）
        """
        try:
            os.environ["INSIGHTFACE_HOME"] = str(Path.home() / ".insightface")

            if self.cpu_cores > 0:
                n = str(self.cpu_cores)
                # 這幾個要在 session 建立**之前**設才有效
                for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS",
                            "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
                    os.environ[var] = n
                try:
                    total = os.cpu_count() or 4
                    keep = set(range(max(0, total - self.cpu_cores), total))
                    os.sched_setaffinity(0, keep)
                    self.logger.info(
                        f"  人臉推論限制在 {len(keep)} 顆核心 {sorted(keep)}"
                        f"（共 {total} 顆）—— 把其餘核心留給導航控制迴圈")
                except (AttributeError, OSError) as exc:
                    self.logger.warning(f"  設定 CPU 親和性失敗（不影響功能）: {exc}")

            providers = self._get_providers()

            self.face_model = insightface.app.FaceAnalysis(
                name=self.model_name,
                providers=providers,
            )

            # ★ 明確給 det_size，不要讓它走 [(128,128),(640,640)] 的串聯
            self.face_model.prepare(
                ctx_id=self.ctx_id,
                det_thresh=self.det_thresh,
                det_size=(self.det_size, self.det_size),
            )
            self.logger.info(
                f"✓ InsightFace {self.model_name} 模型載入成功"
                f"（偵測輸入 {self.det_size}x{self.det_size}，單一尺寸不串聯）")
        except Exception as e:
            raise RuntimeError(f"InsightFace 初始化失敗: {e}")

    def _get_providers(self) -> List[str]:
        """取得 ONNX Runtime 的可用執行提供者

        Returns:
            List[str]: 按優先順序的提供者清單
        """
        providers = []

        if self.enable_gpu:
            try:
                import onnxruntime

                available_providers = onnxruntime.get_available_providers()

                if "CUDAExecutionProvider" in available_providers:
                    providers.append("CUDAExecutionProvider")
                    self.logger.info("✓ GPU 加速 (CUDA) 可用")
                else:
                    self.logger.warning("⚠ GPU 加速 (CUDA) 不可用，將使用 CPU")
            except ImportError:
                self.logger.warning("⚠ onnxruntime 未找到，將使用 CPU")

        providers.append("CPUExecutionProvider")
        return providers

    def detect_and_extract(self, image: np.ndarray) -> Optional[FaceDetectResult]:
        """偵測影像中的臉部並提取特徵

        Args:
            image: BGR 格式的輸入影像 (OpenCV 格式)

        Returns:
            Optional[FaceDetectResult]: 提取到的臉部結果，若沒有提取到則返回 None
        """
        if image is None or image.size == 0:
            return None

        try:
            if self.face_model is None:
                self.logger.error("臉部模型未初始化")
                return None

            faces = self.face_model.get(image, max_num=1)

            if len(faces) == 0:
                return None

            face = FaceDetectResult(
                # ★ 2026-08-10：原本是 .astype(int)，但 FaceEmbedding.msg 的
                #   欄位是 `float32[4] bbox`。rosidl 產生的 setter 會 assert
                #   每個值 isinstance(v, float)，而 Python int 不是 float 的
                #   instance -> 每偵測到一張臉就 AssertionError，被上層的
                #   `except Exception` 吃掉，只在 log 印一行「處理影像時發生錯誤」，
                #   節點照跑、話題存在、但 /face_embedding 一則都發不出去。
                #   同一個寫法也讓 ExtractFaceEmbedding 服務永遠回 success=False
                #   （HMI 的照片註冊 /register_face_photo 一起壞）。
                bbox=faces[0].bbox.astype(np.float32),
                embedding=faces[0].embedding.astype(np.float32),
            )

            age = getattr(faces[0], "age", None)
            if age is not None:
                face.age = float(age)

            gender = getattr(faces[0], "gender", None)
            if gender is not None:
                face.gender = "M" if gender == 1 else "F"
        except Exception as e:
            self.logger.error(f"臉部偵測並提取錯誤: {e}")
            return None

        return face
