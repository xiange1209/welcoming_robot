"""
推理引擎抽象層
- ONNX Runtime 配置
- 模型加載與推理
- 自動平台切換（RPi4 ↔ Jetson Orin）
"""

import yaml
import onnxruntime as ort
import numpy as np
from pathlib import Path
from typing import Dict, Any, Tuple
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class InferenceEngine:
    """ONNX 推理引擎 - 配置驅動，支持多平台"""

    def __init__(self, config_path: str = "config/inference_config.yaml"):
        """
        初始化推理引擎
        Args:
            config_path: 配置檔路徑
        """
        self.config = self._load_config(config_path)
        self.platform = self.config['platform']
        self.precision = self.config['model_precision']
        self.models = {}
        self._initialize_ort_session()

    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """加載 YAML 配置"""
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        logger.info(f"✓ 配置已加載 ({config['platform']}, {config['model_precision']})")
        return config

    def _initialize_ort_session(self):
        """初始化 ONNX Runtime 會話"""
        if self.platform == "rpi4":
            # RPi4：使用 CPU，禁用 GPU（無 GPU）
            self.sess_options = ort.SessionOptions()
            self.sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.sess_options.intra_op_num_threads = self.config['num_threads']
            self.providers = ['CPUExecutionProvider']
            logger.info(f"✓ ONNX Runtime 已配置（RPi4 CPU，{self.config['num_threads']} 線程）")

        elif self.platform == "jetson_orin":
            # Jetson Orin：優先使用 GPU
            self.sess_options = ort.SessionOptions()
            self.sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            logger.info("✓ ONNX Runtime 已配置（Jetson Orin GPU）")

    def load_model(self, model_name: str, model_path: str) -> ort.InferenceSession:
        """
        加載 ONNX 模型
        Args:
            model_name: 模型標識名稱（e.g. 'face_detector'）
            model_path: 模型檔路徑
        Returns:
            ONNX InferenceSession
        """
        if not Path(model_path).exists():
            logger.error(f"✗ 模型不存在：{model_path}")
            return None

        try:
            session = ort.InferenceSession(
                model_path,
                sess_options=self.sess_options,
                providers=self.providers
            )
            self.models[model_name] = session
            logger.info(f"✓ 模型已加載：{model_name}")
            return session
        except Exception as e:
            logger.error(f"✗ 模型加載失敗 ({model_name})：{e}")
            return None

    def load_all_models(self):
        """一次性加載所有配置中的模型"""
        model_paths = self.config['model_paths']
        for model_name, model_path in model_paths.items():
            self.load_model(model_name, model_path)

    def predict(self, model_name: str, input_data: np.ndarray) -> np.ndarray:
        """
        運行推理
        Args:
            model_name: 模型標識名稱
            input_data: 輸入數據 (numpy array)
        Returns:
            模型輸出 (numpy array)
        """
        if model_name not in self.models:
            logger.error(f"✗ 模型未加載：{model_name}")
            return None

        session = self.models[model_name]
        input_name = session.get_inputs()[0].name
        output_names = [output.name for output in session.get_outputs()]

        try:
            outputs = session.run(output_names, {input_name: input_data})
            return outputs[0] if len(outputs) == 1 else outputs
        except Exception as e:
            logger.error(f"✗ 推理失敗 ({model_name})：{e}")
            return None

    def get_model_info(self, model_name: str) -> Dict[str, Any]:
        """獲取模型詳細信息"""
        if model_name not in self.models:
            return None

        session = self.models[model_name]
        input_info = session.get_inputs()[0]
        output_info = session.get_outputs()

        return {
            'model_name': model_name,
            'input_shape': input_info.shape,
            'input_type': input_info.type,
            'output_shapes': [o.shape for o in output_info],
            'output_types': [o.type for o in output_info]
        }

    def benchmark(self, model_name: str, input_shape: Tuple, num_runs: int = 10) -> Dict[str, float]:
        """
        模型基準測試（測量推理延遲）
        Args:
            model_name: 模型標識名稱
            input_shape: 輸入形狀
            num_runs: 測試運行次數
        Returns:
            {'min_ms': ..., 'max_ms': ..., 'avg_ms': ..., 'fps': ...}
        """
        import time

        if model_name not in self.models:
            logger.error(f"✗ 模型未加載：{model_name}")
            return None

        # 建立虛擬輸入數據
        dummy_input = np.random.randn(*input_shape).astype(np.float32)
        latencies = []

        for _ in range(num_runs):
            start = time.perf_counter()
            self.predict(model_name, dummy_input)
            latency_ms = (time.perf_counter() - start) * 1000
            latencies.append(latency_ms)

        avg_latency = np.mean(latencies)
        return {
            'min_ms': min(latencies),
            'max_ms': max(latencies),
            'avg_ms': avg_latency,
            'fps': 1000 / avg_latency
        }


if __name__ == "__main__":
    # 測試推理引擎
    engine = InferenceEngine()
    print(f"平台：{engine.platform}")
    print(f"精度：{engine.precision}")
    print(f"已配置的模型路徑：{engine.config['model_paths']}")
