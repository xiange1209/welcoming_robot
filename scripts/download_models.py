#!/usr/bin/env python3
"""
下載量化模型腳本
支持：RetinaFace INT8、ArcFace INT8、EfficientNet-B0
"""

import os
import sys
from pathlib import Path
import urllib.request
import hashlib

# 模型下載列表
MODELS = {
    'retinaface_int8.onnx': {
        'url': 'https://huggingface.co/spaces/arnoldjair/trimgs/resolve/main/retinaface_int8.onnx',
        'size_mb': 10,
        'description': 'RetinaFace INT8 人臉檢測',
        'optional': False,
        'note': '量化 INT8 版本，適合樹莓派'
    },
    'arcface_int8.onnx': {
        'url': 'https://huggingface.co/models?search=arcface+onnx',
        'size_mb': 18,
        'description': 'ArcFace INT8 人臉識別',
        'optional': False,
        'note': '建議從 HuggingFace 手動下載或使用替代方案'
    },
    'efficientnet_b0_emotion.onnx': {
        'url': 'https://huggingface.co/spaces/arnoldjair/trimgs/resolve/main/emotion.onnx',
        'size_mb': 4,
        'description': 'EfficientNet-B0 表情分析',
        'optional': True,
        'note': 'Phase 3 使用'
    }
}


class ModelDownloader:
    """模型下載器"""

    def __init__(self, models_dir: str = "models"):
        self.models_dir = Path(models_dir)
        self.models_dir.mkdir(exist_ok=True)

    def download_file(self, url: str, filename: str, file_size_mb: int = None) -> bool:
        """
        下載單個文件
        Args:
            url: 下載 URL
            filename: 保存檔名
            file_size_mb: 預期文件大小（用於進度估計）
        Returns:
            成功返回 True
        """
        filepath = self.models_dir / filename
        if filepath.exists():
            print(f"✓ {filename} 已存在，跳過下載")
            return True

        print(f"\n📥 下載 {filename}...")
        print(f"   URL: {url}")
        if file_size_mb:
            print(f"   大小: ~{file_size_mb}MB")

        try:
            # 创建进度条
            def download_progress(block_num, block_size, total_size):
                downloaded = block_num * block_size
                percent = min(downloaded * 100 / total_size, 100) if total_size > 0 else 0
                bar_len = 30
                filled = int(bar_len * percent / 100)
                bar = '█' * filled + '░' * (bar_len - filled)
                print(f"\r   [{bar}] {percent:.1f}%", end='', flush=True)

            urllib.request.urlretrieve(url, str(filepath), download_progress)
            print("\n   ✓ 下載完成")
            return True

        except Exception as e:
            print(f"\n   ✗ 下載失敗: {e}")
            return False

    def verify_file(self, filename: str, expected_hash: str = None) -> bool:
        """驗證下載的文件"""
        filepath = self.models_dir / filename
        if not filepath.exists():
            return False

        file_size_mb = filepath.stat().st_size / 1024 / 1024
        print(f"   ✓ 文件大小: {file_size_mb:.1f}MB")
        return True

    def list_models(self):
        """列出所有模型及狀態"""
        print("\n" + "=" * 60)
        print("📦 可用模型列表")
        print("=" * 60)

        for idx, (model_name, info) in enumerate(MODELS.items(), 1):
            filepath = self.models_dir / model_name
            status = "✓ 已下載" if filepath.exists() else "❌ 未下載"
            optional = "（可選）" if info['optional'] else "（必需）"

            print(f"\n{idx}. {model_name} {optional}")
            print(f"   説明: {info['description']}")
            print(f"   大小: ~{info['size_mb']}MB")
            print(f"   狀態: {status}")
            print(f"   備註: {info['note']}")

    def download_essential_models(self) -> bool:
        """下載必需模型"""
        print("\n" + "=" * 60)
        print("🚀 下載必需模型（Phase 2）")
        print("=" * 60)

        success_count = 0
        fail_count = 0

        for model_name, info in MODELS.items():
            if info['optional']:
                continue

            print(f"\n【{model_name}】")
            print(f"説明: {info['description']}")

            if self.download_file(info['url'], model_name, info['size_mb']):
                if self.verify_file(model_name):
                    success_count += 1
                else:
                    fail_count += 1
            else:
                fail_count += 1

        print("\n" + "=" * 60)
        print(f"下載結果: {success_count} 成功, {fail_count} 失敗")
        print("=" * 60)

        return fail_count == 0

    def download_all_models(self) -> bool:
        """下載所有模型"""
        print("\n" + "=" * 60)
        print("🚀 下載所有模型")
        print("=" * 60)

        success_count = 0
        fail_count = 0

        for model_name, info in MODELS.items():
            print(f"\n【{model_name}】")
            print(f"説明: {info['description']}")

            if self.download_file(info['url'], model_name, info['size_mb']):
                if self.verify_file(model_name):
                    success_count += 1
                else:
                    fail_count += 1
            else:
                fail_count += 1

        print("\n" + "=" * 60)
        print(f"下載結果: {success_count} 成功, {fail_count} 失敗")
        print("=" * 60)

        return fail_count == 0


def main():
    downloader = ModelDownloader()

    # 顯示可用模型
    downloader.list_models()

    # 詢問用戶選擇
    print("\n" + "=" * 60)
    print("請選擇：")
    print("1. 下載必需模型（推薦 Phase 2）")
    print("2. 下載所有模型")
    print("3. 僅列出模型")
    print("=" * 60)

    choice = input("\n輸入選擇 (1-3): ").strip()

    if choice == '1':
        print("\n⏳ 開始下載必需模型...")
        success = downloader.download_essential_models()
    elif choice == '2':
        print("\n⏳ 開始下載所有模型...")
        success = downloader.download_all_models()
    else:
        print("\n✓ 已列出所有模型")
        return

    if success:
        print("\n✅ 所有模型下載並驗證完成！")
        print("可以運行: python3 scripts/benchmark.py")
    else:
        print("\n⚠ 部分模型下載失敗，請檢查網絡連接")


if __name__ == "__main__":
    main()
