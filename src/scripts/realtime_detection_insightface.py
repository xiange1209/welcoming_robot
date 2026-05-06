#!/usr/bin/env python3
"""
即時人臉檢測 - InsightFace 版本
支持：Raspberry Pi 4 + Picamera2
特性：
  - InsightFace RetinaFace ONNX 模型（內置，~6MB）
  - 高準確度人臉檢測 + 側臉支持
  - 同時生成人臉 embedding（後續用於識別）
  - 跳幀優化（提高 FPS）
  - 按 Q 退出，S 保存，空格暫停
"""

import cv2
import numpy as np
import sys
import time
from pathlib import Path
import logging
import os

try:
    from PIL import Image, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False


def _find_cjk_font() -> str | None:
    """動態搜尋系統中支援中文的字型檔"""
    # 先試常見固定路徑
    from pathlib import Path as _P
    candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJKtc-Regular.otf",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    ]
    for p in candidates:
        if _P(p).exists():
            return p
    # fc-list 動態搜尋
    try:
        import subprocess
        out = subprocess.run(
            ["fc-list", ":lang=zh", "-f", "%{file}\\n"],
            capture_output=True, text=True, timeout=3
        ).stdout
        for line in out.splitlines():
            f = line.strip()
            if f and _P(f).exists():
                return f
    except Exception:
        pass
    # glob 最後手段
    try:
        for p in _P("/usr/share/fonts").rglob("*"):
            n = p.name.lower()
            if ("cjk" in n or "wqy" in n or "noto" in n) and p.suffix in (".ttc", ".ttf", ".otf"):
                return str(p)
    except Exception:
        pass
    return None


_CJK_FONT = _find_cjk_font() if PIL_AVAILABLE else None
_FONT_CACHE: dict = {}  # {font_size: ImageFont}


def _get_font(font_size: int):
    if font_size not in _FONT_CACHE:
        _FONT_CACHE[font_size] = ImageFont.truetype(_CJK_FONT, font_size)
    return _FONT_CACHE[font_size]


# 添加 src/ 到路徑
_SRC_DIR = Path(__file__).parent.parent
_PROJECT_ROOT = _SRC_DIR.parent
sys.path.insert(0, str(_SRC_DIR))

DB_PATH = str(_PROJECT_ROOT / "database" / "ai_bank_robot.db")

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False

from ai_vision.face_recognizer import FaceRecognizer
from ai_vision.liveness_detector import LivenessDetector
from database.schema import DatabaseSchema

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

if _CJK_FONT:
    logger.info(f"✓ 中文字型: {_CJK_FONT}")
else:
    logger.warning("⚠ 未找到中文字型，標籤將顯示英文。安裝：sudo apt install fonts-noto-cjk")

try:
    import onnxruntime as _ort
    _ort.set_default_logger_severity(3)
except Exception:
    pass

try:
    from insightface.app import FaceAnalysis
    INSIGHTFACE_AVAILABLE = True
except ImportError:
    INSIGHTFACE_AVAILABLE = False


class RealtimeFaceDetectorInsightFace:
    """使用 Picamera2 + InsightFace 的即時人臉檢測"""

    def __init__(self):
        """初始化檢測器"""
        if not INSIGHTFACE_AVAILABLE:
            logger.error("✗ InsightFace 未安裝。請執行：pip install insightface")
            sys.exit(1)

        self.picam2 = None
        self.cap = None

        if PICAMERA2_AVAILABLE:
            logger.info("初始化 Picamera2...")
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"size": (640, 480), "format": "XBGR8888"}
            )
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(1)
            logger.info("✓ Picamera2 已初始化 (640x480)")
        else:
            logger.info("Picamera2 不可用，使用 USB 攝像頭...")
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            logger.info("✓ USB 攝像頭已初始化 (640x480)")

        # 初始化 InsightFace
        logger.info("初始化 InsightFace...")
        try:
            self.face_app = FaceAnalysis(
                name='buffalo_sc',
                root='./insightface_models',
                providers=['CPUExecutionProvider']
            )
            self.face_app.prepare(ctx_id=-1, det_size=(320, 320))
            logger.info("✓ InsightFace 已初始化 (RetinaFace + ArcFace)")
        except Exception as e:
            logger.error(f"✗ InsightFace 初始化失敗: {e}")
            sys.exit(1)

        # 初始化人臉識別模組
        logger.info("初始化人臉識別模組...")
        try:
            db_schema = DatabaseSchema(DB_PATH)
            if not Path(DB_PATH).exists():
                logger.info("初始化數據庫...")
                db_schema.initialize()

            self.face_recognizer = FaceRecognizer(
                db_path=DB_PATH,
                vip_threshold=0.25,
                blacklist_threshold=0.20
            )
            logger.info(f"✓ 人臉識別已初始化 ({len(self.face_recognizer.vip_cache)} VIP, "
                       f"{len(self.face_recognizer.blacklist_cache)} 黑名單)")
        except Exception as e:
            logger.error(f"✗ 人臉識別初始化失敗: {e}")
            self.face_recognizer = None

        # 初始化活體檢測模組
        logger.info("初始化活體檢測模組...")
        try:
            self.liveness_detector = LivenessDetector(
                detection_threshold=3,
                eye_ar_threshold=0.2,
                mouth_ar_threshold=0.5,
                head_rotation_threshold=15.0
            )
            logger.info("✓ 活體檢測已初始化")
        except Exception as e:
            logger.warning(f"⚠ 活體檢測初始化失敗: {e}（將跳過活體檢測）")
            self.liveness_detector = None

        # 跳幀設置
        self.detect_interval = 5
        self.frame_since_detect = 0
        self.last_detections = []

        # 連續確認（避免單次高分誤判）
        self._confirm_last = None   # (name, person_type)
        self._confirm_count = 0

        # 統計信息
        self.frame_count = 0
        self.max_faces = 0
        self.start_time = time.time()
        self.save_dir = Path("/tmp")
        self.paused = False

    def _apply_temporal_filter(self, result: dict) -> dict:
        """需要連續 2 次偵測才確認 VIP/黑名單，防止單次誤判"""
        ptype = result['person_type']
        if ptype in ('vip', 'blacklist'):
            key = (result['name'], ptype)
            if key == self._confirm_last:
                self._confirm_count += 1
            else:
                self._confirm_count = 1
                self._confirm_last = key
            if self._confirm_count >= 2:
                return result
            # 尚未確認，顯示為訪客
            return {'person_type': 'visitor', 'name': None, 'gender': None,
                    'confidence': result['confidence'], 'vip_level': None,
                    'risk_level': None, 'details': {}}
        else:
            self._confirm_last = None
            self._confirm_count = 0
            return result

    def detect_faces_insightface(self, frame):
        """使用 InsightFace 檢測人臉並進行身份識別"""
        detections = []

        try:
            # 執行人臉檢測
            faces = self.face_app.get(frame)

            for face in faces:
                # 獲取邊界框
                bbox = face.bbox.astype(np.int32)  # [x1, y1, x2, y2]
                x1, y1, x2, y2 = bbox

                # 轉換為 (x, y, w, h) 格式
                x = x1
                y = y1
                w = x2 - x1
                h = y2 - y1

                # 獲取置信度
                confidence = float(face.det_score)

                # 獲取 512D embedding
                embedding = face.embedding.astype(np.float32)

                # 執行人臉識別（如果識別模組可用）
                recognition_result = None
                if self.face_recognizer:
                    raw = self.face_recognizer.recognize_face(embedding)
                    recognition_result = self._apply_temporal_filter(raw)

                # 執行活體檢測
                liveness_result = None
                if self.liveness_detector:
                    face_roi = frame[max(0, y1):min(frame.shape[0], y2),
                                     max(0, x1):min(frame.shape[1], x2)]
                    if face_roi.size > 0:
                        liveness_result = self.liveness_detector.check_liveness(face_roi)

                detection = {
                    'box': (x, y, w, h),
                    'detection_confidence': confidence,
                    'embedding': embedding,
                    'type': 'face',
                    'recognition': recognition_result,
                    'liveness': liveness_result
                }

                detections.append(detection)

        except Exception as e:
            logger.error(f"檢測失敗: {e}")

        return detections

    def draw_detections(self, frame, detections):
        """繪製檢測結果（包括識別、性別和活體檢測）"""
        result = frame.copy()
        text_items = []  # [(text, pos, font_size, color)]

        for det in detections:
            x, y, w, h = det['box']
            detection_conf = det['detection_confidence']

            color = (0, 255, 0)
            label_text = f"Visitor ({detection_conf:.2f})"

            if det['recognition']:
                rec = det['recognition']
                person_type = rec['person_type']
                name = rec['name']
                gender = rec.get('gender', 'Other')
                rec_conf = rec['confidence']

                if person_type == 'vip':
                    color = (0, 255, 255)
                    label_text = f"VIP {name} ({gender}) {rec_conf:.2f}"
                elif person_type == 'blacklist':
                    color = (0, 0, 255)
                    label_text = f"黑名單 {name} ({gender}) {rec_conf:.2f}"
                else:
                    label_text = f"訪客 {rec_conf:.2f}"

            cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
            text_items.append((label_text, (x, max(0, y - 30)), 20, color))

            if det['liveness']:
                liveness = det['liveness']
                is_text = "Alive" if liveness['is_alive'] else "Spoofing?"
                lv_color = (0, 255, 0) if liveness['is_alive'] else (0, 0, 255)
                text_items.append((is_text, (x, max(0, y - 5)), 16, lv_color))

        # 所有文字一次 PIL round-trip（避免每張臉各做一次）
        if text_items:
            if PIL_AVAILABLE and _CJK_FONT:
                pil = Image.fromarray(cv2.cvtColor(result, cv2.COLOR_BGR2RGB))
                draw = ImageDraw.Draw(pil)
                for text, pos, font_size, color in text_items:
                    draw.text(pos, text, font=_get_font(font_size),
                              fill=(color[2], color[1], color[0]))
                result = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
            else:
                for text, pos, font_size, color in text_items:
                    cv2.putText(result, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                                font_size / 30, color, 2)

        return result

    def run(self):
        """運行即時檢測"""
        logger.info("=" * 70)
        logger.info("🎥 即時人臉檢測 (Picamera2 + InsightFace)")
        logger.info("=" * 70)
        logger.info("控制：")
        logger.info("  Q - 退出")
        logger.info("  S - 保存當前幀")
        logger.info("  空格 - 暫停/繼續")
        logger.info("=" * 70)

        try:
            while True:
                # 讀取幀
                if self.picam2:
                    frame_xbgr = self.picam2.capture_array()
                    if frame_xbgr is None:
                        logger.error("✗ 讀取幀失敗")
                        break
                    frame_bgr = cv2.cvtColor(frame_xbgr, cv2.COLOR_RGBA2BGR)
                else:
                    ret, frame_bgr = self.cap.read()
                    if not ret:
                        logger.error("✗ 讀取幀失敗")
                        break

                if not self.paused:
                    self.frame_count += 1
                    self.frame_since_detect += 1

                    # 每 N 幀進行一次檢測
                    if self.frame_since_detect >= self.detect_interval:
                        self.last_detections = self.detect_faces_insightface(frame_bgr)
                        self.frame_since_detect = 0
                        self.max_faces = max(self.max_faces, len(self.last_detections))

                    detections = self.last_detections
                    result = self.draw_detections(frame_bgr, detections)
                else:
                    result = frame_bgr
                    detections = []

                # 添加統計信息
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                status = " PAUSED" if self.paused else " RUNNING"
                info_text = f"Frame: {self.frame_count:4d} | FPS: {fps:5.1f} | Faces: {len(detections)} | {status}"
                cv2.putText(result, info_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # 控制提示
                hint_text = "Press [Q]uit [S]ave [Space]Pause"
                cv2.putText(result, hint_text, (10, 460),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                # 嘗試顯示（如果環境支援）
                try:
                    cv2.imshow("Face Detection (InsightFace)", result)
                    key = cv2.waitKey(1) & 0xFF
                except cv2.error:
                    # 不支援 GUI，模擬按鍵檢查用其他方法
                    key = -1
                    # 每 30 幀輸出一次日誌
                    if self.frame_count % 30 == 0:
                        logger.info(f"[Frame {self.frame_count:4d}] Detected {len(detections)} face(s)")

                # 按鍵處理（如果支援 GUI）
                if key == ord('q') or key == ord('Q'):
                    logger.info("\n< 退出中... >")
                    break
                elif key == ord('s') or key == ord('S'):
                    filename = self.save_dir / f"face_detection_{self.frame_count:05d}.jpg"
                    cv2.imwrite(str(filename), result)
                    logger.info(f"✓ 幀已保存：{filename}")

        except KeyboardInterrupt:
            logger.info("\n< 收到中斷信號 >")
        finally:
            self.cleanup()

    def cleanup(self):
        """清理資源"""
        if self.picam2:
            self.picam2.stop()
            self.picam2.close()
        if self.cap:
            self.cap.release()

        # 關閉識別和檢測模組
        if self.face_recognizer:
            self.face_recognizer.close()
        if self.liveness_detector:
            # MediaPipe 的資源由垃圾回收處理
            pass

        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

        logger.info("=" * 70)
        logger.info("✓ 程序已結束")
        logger.info(f"  總幀數: {self.frame_count}")
        logger.info(f"  最多人臉數: {self.max_faces}")
        if self.frame_count > 0:
            logger.info(f"  平均 FPS: {self.frame_count / (time.time() - self.start_time):.1f}")
        logger.info(f"  運行時長: {time.time() - self.start_time:.1f}s")
        logger.info("=" * 70)


def main():
    """主程序"""
    detector = RealtimeFaceDetectorInsightFace()
    detector.run()


if __name__ == "__main__":
    main()
