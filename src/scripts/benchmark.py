"""
性能基準測試工具
測量推理延遲、CPU/內存使用率、FPS
"""

import sys
import time
import psutil
import numpy as np
import cv2
from pathlib import Path
from typing import Dict
import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

# 設置 Python 路徑以導入項目模組
sys.path.insert(0, str(Path(__file__).parent.parent))

from vision_ai.inference_engine import InferenceEngine
from vision_ai.face_detector import FaceDetector
from database.schema import DatabaseSchema


class PerformanceBenchmark:
    """性能基準測試"""

    def __init__(self, config_path: str = "config/inference_config.yaml"):
        self.config_path = config_path
        self.engine = InferenceEngine(config_path)
        self.process = psutil.Process()

    def benchmark_inference_engine(self, num_runs: int = 20) -> Dict:
        """
        測試推理引擎性能
        Args:
            num_runs: 運行次數
        Returns:
            性能指標字典
        """
        logger.info("=" * 50)
        logger.info("🧪 推理引擎性能測試")
        logger.info("=" * 50)

        self.engine.load_all_models()

        results = {}
        for model_name in self.engine.models.keys():
            # 獲取模型輸入形狀
            model_info = self.engine.get_model_info(model_name)
            if not model_info:
                continue

            input_shape = tuple(model_info['input_shape'])
            logger.info(f"\n📊 {model_name}")
            logger.info(f"   輸入形狀: {input_shape}")

            # 執行基準測試
            latencies = []
            for _ in range(num_runs):
                dummy_input = np.random.randn(*input_shape).astype(np.float32)
                start = time.perf_counter()
                self.engine.predict(model_name, dummy_input)
                latency_ms = (time.perf_counter() - start) * 1000
                latencies.append(latency_ms)

            avg_latency = np.mean(latencies)
            results[model_name] = {
                'min_ms': float(np.min(latencies)),
                'max_ms': float(np.max(latencies)),
                'avg_ms': float(avg_latency),
                'fps': float(1000 / avg_latency),
                'std_ms': float(np.std(latencies))
            }

            logger.info(f"   最小延遲: {results[model_name]['min_ms']:.2f}ms")
            logger.info(f"   最大延遲: {results[model_name]['max_ms']:.2f}ms")
            logger.info(f"   平均延遲: {results[model_name]['avg_ms']:.2f}ms")
            logger.info(f"   FPS: {results[model_name]['fps']:.1f}")
            logger.info(f"   標準差: {results[model_name]['std_ms']:.2f}ms")

        return results

    def benchmark_face_detector(self, num_runs: int = 10) -> Dict:
        """
        測試人臉檢測完整管線
        Args:
            num_runs: 運行次數
        Returns:
            性能指標
        """
        logger.info("\n" + "=" * 50)
        logger.info("🧪 人臉檢測管線性能測試")
        logger.info("=" * 50)

        detector = FaceDetector(self.config_path)

        # 生成虛擬測試圖像
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        latencies = []
        cpu_usages = []
        memory_usages = []

        for i in range(num_runs):
            # 記錄資源使用
            cpu_before = self.process.cpu_percent()
            mem_before = self.process.memory_info().rss / 1024 / 1024  # MB

            start = time.perf_counter()
            detections = detector.detect(test_image)
            latency_ms = (time.perf_counter() - start) * 1000

            cpu_after = self.process.cpu_percent()
            mem_after = self.process.memory_info().rss / 1024 / 1024  # MB

            latencies.append(latency_ms)
            cpu_usages.append((cpu_before + cpu_after) / 2)
            memory_usages.append(mem_after)

            logger.info(f"   運行 {i+1}/{num_runs}: {latency_ms:.2f}ms, "
                       f"檢測到 {len(detections)} 個人臉")

        logger.info(f"\n📊 結果統計：")
        logger.info(f"   平均延遲: {np.mean(latencies):.2f}ms")
        logger.info(f"   最小/最大: {np.min(latencies):.2f}ms / {np.max(latencies):.2f}ms")
        logger.info(f"   FPS: {1000 / np.mean(latencies):.1f}")
        logger.info(f"   平均 CPU 使用率: {np.mean(cpu_usages):.1f}%")
        logger.info(f"   平均內存使用: {np.mean(memory_usages):.1f}MB")
        logger.info(f"   內存峰值: {np.max(memory_usages):.1f}MB")

        return {
            'avg_latency_ms': float(np.mean(latencies)),
            'max_latency_ms': float(np.max(latencies)),
            'fps': float(1000 / np.mean(latencies)),
            'avg_cpu_percent': float(np.mean(cpu_usages)),
            'avg_memory_mb': float(np.mean(memory_usages)),
            'peak_memory_mb': float(np.max(memory_usages))
        }

    def benchmark_system_resources(self, duration_sec: int = 10) -> Dict:
        """
        監控系統資源使用（CPU、內存、溫度）
        Args:
            duration_sec: 監控時長（秒）
        Returns:
            資源統計
        """
        logger.info("\n" + "=" * 50)
        logger.info(f"🧪 系統資源監控 ({duration_sec}秒)")
        logger.info("=" * 50)

        cpu_values = []
        memory_values = []
        start_time = time.time()

        while time.time() - start_time < duration_sec:
            cpu = psutil.cpu_percent(interval=0.5)
            memory = psutil.virtual_memory().percent

            cpu_values.append(cpu)
            memory_values.append(memory)

            if len(cpu_values) % 4 == 0:
                logger.info(f"   時點 {len(cpu_values)}s: CPU {cpu:.1f}%, 內存 {memory:.1f}%")

        logger.info(f"\n📊 資源統計：")
        logger.info(f"   CPU 平均: {np.mean(cpu_values):.1f}%, "
                   f"峰值: {np.max(cpu_values):.1f}%")
        logger.info(f"   內存平均: {np.mean(memory_values):.1f}%, "
                   f"峰值: {np.max(memory_values):.1f}%")

        # 嘗試讀取溫度
        try:
            import subprocess
            result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True)
            if result.returncode == 0:
                logger.info(f"   {result.stdout.strip()}")
        except:
            pass

        return {
            'avg_cpu_percent': float(np.mean(cpu_values)),
            'max_cpu_percent': float(np.max(cpu_values)),
            'avg_memory_percent': float(np.mean(memory_values)),
            'max_memory_percent': float(np.max(memory_values))
        }

    def benchmark_database(self) -> Dict:
        """
        測試資料庫操作性能
        Returns:
            資料庫操作耗時
        """
        logger.info("\n" + "=" * 50)
        logger.info("🧪 資料庫操作性能測試")
        logger.info("=" * 50)

        schema = DatabaseSchema()
        schema.initialize()

        conn = schema.connect()

        # 測試寫入
        logger.info("   測試 1000 次插入...")
        start = time.perf_counter()
        for i in range(1000):
            schema.log_visit(conn, 'visitor', f'visitor_{i}', emotion='neutral', confidence=0.95)
        insert_time = (time.perf_counter() - start) * 1000

        logger.info(f"   平均單次插入: {insert_time / 1000:.2f}ms")

        # 測試查詢
        logger.info("   測試 100 次查詢...")
        start = time.perf_counter()
        for i in range(100):
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM visit_logs LIMIT 10")
            cursor.fetchall()
        query_time = (time.perf_counter() - start) * 1000

        logger.info(f"   平均單次查詢: {query_time / 100:.2f}ms")

        conn.close()

        return {
            'insert_time_ms': float(insert_time),
            'query_time_ms': float(query_time)
        }


def main():
    """執行所有基準測試"""
    logger.info("\n")
    logger.info("🤖 樹莓派 4 AI 銀行機器人 - 性能基準測試")
    logger.info("=" * 50)

    benchmark = PerformanceBenchmark()

    # 運行測試
    results = {}

    try:
        results['inference_engine'] = benchmark.benchmark_inference_engine()
    except Exception as e:
        logger.error(f"✗ 推理引擎測試失敗：{e}")

    try:
        results['face_detector'] = benchmark.benchmark_face_detector()
    except Exception as e:
        logger.error(f"✗ 人臉檢測測試失敗：{e}")

    try:
        results['system_resources'] = benchmark.benchmark_system_resources(duration_sec=10)
    except Exception as e:
        logger.error(f"✗ 系統資源監控失敗：{e}")

    try:
        results['database'] = benchmark.benchmark_database()
    except Exception as e:
        logger.error(f"✗ 資料庫測試失敗：{e}")

    # 輸出總結
    logger.info("\n" + "=" * 50)
    logger.info("✅ 基準測試完成")
    logger.info("=" * 50)

    return results


if __name__ == "__main__":
    main()
