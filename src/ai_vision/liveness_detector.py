"""
活體檢測模組 - 防止照片/視頻欺騙
使用 MediaPipe FaceMesh 檢測面部關鍵點，計算：
  1. 眨眼 (Eye Aspect Ratio)
  2. 摇头 (頭部旋轉角度)
  3. 張嘴 (Mouth Aspect Ratio)
"""

import cv2
import numpy as np
from collections import deque
from typing import Dict, Tuple, Optional
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False


class LivenessDetector:
    """基於 MediaPipe 的活體檢測"""

    # MediaPipe FaceMesh 關鍵點索引
    # 眼睛：左眼 33, 128, 33, 160 和右眼 133, 159, 159, 362
    LEFT_EYE_INDICES = [33, 160, 158, 133, 155, 154]
    RIGHT_EYE_INDICES = [362, 385, 387, 398, 384, 381]

    # 嘴巴：53, 62, 70, 83
    MOUTH_INDICES = [61, 146, 91, 181, 84, 17, 314, 405, 321, 375]

    # 面部輪廓（用於頭部姿態估計）
    FACE_CONTOUR = [10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288, 397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136, 172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109]

    # 鼻子和眼睛（用於頭部姿態估計）
    NOSE_INDEX = 1
    LEFT_EYE_CENTER = 159
    RIGHT_EYE_CENTER = 386

    def __init__(self, detection_threshold: float = 3,
                 eye_ar_threshold: float = 0.2,
                 mouth_ar_threshold: float = 0.5,
                 head_rotation_threshold: float = 15.0):
        """
        初始化活體檢測器

        Args:
            detection_threshold: 需要偵測到的連續幀數（防止閃爍）
            eye_ar_threshold: 眨眼的 Eye Aspect Ratio 閾值
            mouth_ar_threshold: 張嘴的 Mouth Aspect Ratio 閾值
            head_rotation_threshold: 頭部旋轉角度閾值（度）
        """
        if not MEDIAPIPE_AVAILABLE:
            logger.error("✗ MediaPipe 未安裝。請執行：pip install mediapipe")
            raise ImportError("MediaPipe not available")

        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.detection_threshold = detection_threshold
        self.eye_ar_threshold = eye_ar_threshold
        self.mouth_ar_threshold = mouth_ar_threshold
        self.head_rotation_threshold = head_rotation_threshold

        # 追蹤狀態
        self.blink_count = 0
        self.mouth_open_count = 0
        self.head_rotation_count = 0

        # 歷史記錄
        self.eye_ar_history = deque(maxlen=20)
        self.mouth_ar_history = deque(maxlen=20)
        self.head_pose_history = deque(maxlen=20)

    @staticmethod
    def distance(pt1: np.ndarray, pt2: np.ndarray) -> float:
        """計算兩點之間的距離"""
        return float(np.linalg.norm(pt1 - pt2))

    @staticmethod
    def eye_aspect_ratio(eye: np.ndarray) -> float:
        """
        計算 Eye Aspect Ratio (EAR)
        EAR = (||p2 - p6|| + ||p3 - p5||) / (2 * ||p1 - p4||)
        ref: https://www.pyimagesearch.com/2017/04/24/eye-blink-detection-opencv-python-dlib/
        """
        if len(eye) < 6:
            return 0.0

        # 計算垂直距離
        vertical_dist_1 = LivenessDetector.distance(eye[1], eye[5])
        vertical_dist_2 = LivenessDetector.distance(eye[2], eye[4])

        # 計算水平距離
        horizontal_dist = LivenessDetector.distance(eye[0], eye[3])

        # 計算 EAR
        ear = (vertical_dist_1 + vertical_dist_2) / (2.0 * horizontal_dist + 1e-6)
        return ear

    @staticmethod
    def mouth_aspect_ratio(mouth: np.ndarray) -> float:
        """
        計算 Mouth Aspect Ratio (MAR)
        MAR = (||p2 - p10|| + ||p3 - p9|| + ||p4 - p8||) / (3 * ||p1 - p7||)
        """
        if len(mouth) < 10:
            return 0.0

        # 計算垂直距離（open/close）
        vertical_dist_1 = LivenessDetector.distance(mouth[1], mouth[9])
        vertical_dist_2 = LivenessDetector.distance(mouth[2], mouth[8])
        vertical_dist_3 = LivenessDetector.distance(mouth[4], mouth[6])

        # 計算水平距離
        horizontal_dist = LivenessDetector.distance(mouth[0], mouth[6])

        # 計算 MAR
        mar = (vertical_dist_1 + vertical_dist_2 + vertical_dist_3) / (3.0 * horizontal_dist + 1e-6)
        return mar

    def detect_head_rotation(self, landmarks: np.ndarray) -> float:
        """
        偵測頭部旋轉（Yaw 角度）
        ref: https://github.com/google/mediapipe/wiki/Face%20Mesh%20Left%20and%20Right%20Flipped

        Returns:
            head_rotation: 頭部 Yaw 角度（度）
        """
        try:
            # 提取關鍵點
            nose = landmarks[self.NOSE_INDEX]
            left_eye = landmarks[self.LEFT_EYE_CENTER]
            right_eye = landmarks[self.RIGHT_EYE_CENTER]

            # 計算眼睛中線和鼻子的距離（用於估計 Yaw 角）
            eye_center = (left_eye + right_eye) / 2.0
            nose_to_eye_vector = eye_center - nose

            # 簡單的 Yaw 角度估計：基於鼻子相對於眼睛的位置
            # 如果鼻子在眼睛左邊，yaw 為負（向左轉）
            # 如果鼻子在眼睛右邊，yaw 為正（向右轉）
            yaw_angle = np.degrees(np.arctan2(nose_to_eye_vector[0], nose_to_eye_vector[2]))

            return yaw_angle
        except:
            return 0.0

    def check_liveness(self, frame: np.ndarray) -> Dict:
        """
        檢測人臉活體（需要多幀交互）

        Args:
            frame: BGR 圖像幀

        Returns:
            {
                'is_alive': bool,  # 是否通過活體檢測
                'blink_detected': bool,
                'head_rotation_detected': bool,
                'mouth_open_detected': bool,
                'eye_ar': float,
                'mouth_ar': float,
                'head_rotation': float,
                'status': str  # 詳細狀態信息
            }
        """
        result = {
            'is_alive': False,
            'blink_detected': False,
            'head_rotation_detected': False,
            'mouth_open_detected': False,
            'eye_ar': 0.0,
            'mouth_ar': 0.0,
            'head_rotation': 0.0,
            'status': 'Processing...'
        }

        try:
            # 轉換為 RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(rgb_frame)

            if not results.multi_face_landmarks:
                result['status'] = 'No face detected'
                return result

            # 獲取關鍵點
            landmarks = results.multi_face_landmarks[0].landmark
            h, w = frame.shape[:2]
            landmarks_np = np.array([[lm.x * w, lm.y * h] for lm in landmarks])

            # 1. 計算 EAR（眨眼）
            left_eye = landmarks_np[self.LEFT_EYE_INDICES]
            right_eye = landmarks_np[self.RIGHT_EYE_INDICES]

            left_ear = self.eye_aspect_ratio(left_eye)
            right_ear = self.eye_aspect_ratio(right_eye)
            ear = (left_ear + right_ear) / 2.0

            self.eye_ar_history.append(ear)
            result['eye_ar'] = ear

            # 檢測眨眼（EAR 下降到閾值以下）
            if ear < self.eye_ar_threshold:
                self.blink_count += 1
            else:
                if self.blink_count > 0:
                    result['blink_detected'] = True
                self.blink_count = 0

            # 2. 計算 MAR（張嘴）
            mouth = landmarks_np[self.MOUTH_INDICES]
            mar = self.mouth_aspect_ratio(mouth)

            self.mouth_ar_history.append(mar)
            result['mouth_ar'] = mar

            if mar > self.mouth_ar_threshold:
                self.mouth_open_count += 1
            else:
                if self.mouth_open_count > 0:
                    result['mouth_open_detected'] = True
                self.mouth_open_count = 0

            # 3. 檢測頭部旋轉
            head_rotation = self.detect_head_rotation(landmarks_np)
            self.head_pose_history.append(head_rotation)
            result['head_rotation'] = head_rotation

            # 計算旋轉變化
            if len(self.head_pose_history) > 5:
                rotation_change = abs(
                    max(self.head_pose_history) - min(self.head_pose_history)
                )
                if rotation_change > self.head_rotation_threshold:
                    result['head_rotation_detected'] = True
                    self.head_rotation_count = 0  # 重置計數器
            else:
                rotation_change = 0.0

            # 4. 判斷是否通過活體檢測
            # 需要檢測到至少 2 種動作
            actions_detected = sum([
                result['blink_detected'],
                result['mouth_open_detected'],
                result['head_rotation_detected']
            ])

            if actions_detected >= 2:
                result['is_alive'] = True
                result['status'] = f'✓ Alive (detected {actions_detected} actions)'
            else:
                result['status'] = f'Liveness check: {actions_detected}/3 actions detected'

            return result

        except Exception as e:
            logger.error(f"活體檢測失敗: {e}")
            result['status'] = f'Error: {str(e)}'
            return result

    def draw_landmarks(self, frame: np.ndarray, result: Dict) -> np.ndarray:
        """在幀上繪製活體檢測信息"""
        output = frame.copy()
        h, w = frame.shape[:2]

        # 狀態文字
        status_color = (0, 255, 0) if result['is_alive'] else (0, 0, 255)
        cv2.putText(output, result['status'], (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        # 詳細信息
        info_text = f"EAR: {result['eye_ar']:.3f} | MAR: {result['mouth_ar']:.3f} | Yaw: {result['head_rotation']:.1f}°"
        cv2.putText(output, info_text, (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # 動作指示器
        actions_text = f"Blink: {'✓' if result['blink_detected'] else '✗'} | " \
                      f"Mouth: {'✓' if result['mouth_open_detected'] else '✗'} | " \
                      f"Head: {'✓' if result['head_rotation_detected'] else '✗'}"
        cv2.putText(output, actions_text, (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        return output


if __name__ == "__main__":
    # 測試代碼
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("✗ 無法打開攝像頭")
        exit(1)

    detector = LivenessDetector()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 檢測活體
        result = detector.check_liveness(frame)

        # 繪製結果
        output = detector.draw_landmarks(frame, result)

        cv2.imshow("Liveness Detection", output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
