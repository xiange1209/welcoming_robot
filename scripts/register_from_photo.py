#!/usr/bin/env python3
"""從照片檔註冊人臉（不需要相機、不需要啟動 ROS 節點）。

一般註冊流程要人站在相機前採集 N 張樣本（`/face_registration/register` 服務）；
本工具改成直接吃現成的照片檔，方便：
  ・遠端／沒有相機時先把人建進資料庫
  ・用同一張照片重複實驗，讓 E1／E2 的比對條件可重現

用法：
    python3 scripts/register_from_photo.py --name 陳佳憲 --gender M --type VIP photo.jpg
    python3 scripts/register_from_photo.py --name 王小明 --type BLACKLIST a.jpg b.jpg c.jpg

多張照片會全部存成同一個人的樣本（辨識時取平均，樣本越多越穩）。
註冊完若辨識節點正在跑，記得刷新快取：
    ros2 service call /face_recognition/refresh_cache std_srvs/srv/Empty
"""

import argparse
import sys
from pathlib import Path

import cv2

from smartnav_vision.database_manager import DatabaseManager
from smartnav_vision.face_engine import FaceEngine


def main() -> int:
    ap = argparse.ArgumentParser(description="從照片檔註冊人臉到 smartnav 人臉資料庫")
    ap.add_argument("images", nargs="+", help="照片檔路徑（可多張，都算同一個人）")
    ap.add_argument("--name", required=True, help="人員名稱")
    ap.add_argument("--gender", default="Other", choices=["M", "F", "Other"], help="性別")
    ap.add_argument("--type", dest="person_type", default="VIP", choices=["VIP", "BLACKLIST"], help="人員類型")
    ap.add_argument("--model", default="buffalo_sc", help="InsightFace 模型名稱")
    ap.add_argument("--gpu", action="store_true", help="啟用 GPU（RPi4 沒有 CUDA，不要開）")
    args = ap.parse_args()

    engine = FaceEngine(model_name=args.model, enable_gpu=args.gpu)
    db = DatabaseManager()

    # 先掃過所有照片，確定每張都抓得到臉，再真的建人——
    # 免得建了人卻一張樣本都存不進去，在資料庫留下空殼。
    prepared = []
    for path in args.images:
        p = Path(path)
        if not p.exists():
            print(f"✗ 找不到檔案: {p}")
            return 1
        image = cv2.imread(str(p))
        if image is None:
            print(f"✗ 無法讀取（非影像檔或格式不支援）: {p}")
            return 1
        faces = engine.detect_and_extract(image)
        if not faces:
            print(f"✗ 這張照片偵測不到人臉: {p}")
            return 1
        if len(faces) > 1:
            # 多張臉時取面積最大的那張（通常是主角）
            faces.sort(key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]), reverse=True)
            print(f"⚠ {p.name} 偵測到 {len(faces)} 張臉，取面積最大的那張")
        f = faces[0]
        x1, y1, x2, y2 = (int(v) for v in f.bbox[:4])
        print(f"✓ {p.name}: 偵測到人臉，信心度 {f.confidence:.3f}，臉部尺寸 {x2 - x1}×{y2 - y1} px")
        prepared.append((image, f.embedding))

    person_uuid = db.register_person(args.name, args.gender, args.person_type)
    ok = sum(1 for image, emb in prepared if db.add_face_sample(person_uuid, image, emb))

    print()
    print(f"註冊完成：{args.name}（{args.person_type}）")
    print(f"  UUID     : {person_uuid}")
    print(f"  樣本數   : {ok}/{len(prepared)}")
    print(f"  資料庫   : {db.database_dir}")
    if ok < len(prepared):
        print("⚠ 有樣本寫入失敗，請看上方錯誤訊息")
        return 1
    print()
    print("辨識節點若正在運行，執行以下指令讓它重新載入資料庫：")
    print("  ros2 service call /face_recognition/refresh_cache std_srvs/srv/Empty")
    return 0


if __name__ == "__main__":
    sys.exit(main())
