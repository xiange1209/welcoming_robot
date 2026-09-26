#!/usr/bin/env python3

"""把 map_server 存的地圖（.yaml + .pgm）轉成 Gazebo 世界

用途：拿實車建好的地圖當模擬場地 —— 參數在「我們真的會跑的地方」先驗一輪。

    ros2 run smartnav_sim pgm_to_world <地圖.yaml> -o <輸出.sdf>
    # 或讓 launch 當場轉：
    ros2 launch smartnav_sim sim_explore.launch.py map_yaml:=<地圖.yaml>

★ 世界座標 = 地圖座標。地圖原點 (0, 0) 就是當初 slam_toolbox 開始建圖時車子的位置，
  一定是空地，所以模擬車直接放在 (0, 0) 就好。

轉換規則（跟 map_server 讀圖的規則一致）：
    佔據機率 p = (255 − 像素) / 255（negate: 0 時）
    p > occupied_thresh → 牆；p < free_thresh → 空地；其餘 → 未知

兩個清理步驟（都可以關）：
    --min-blob N   小於 N 格的孤立牆點丟掉（建圖時掃到的人腳、椅腳雜點）
    封邊（預設開） 跟空地相鄰的未知格補成牆 —— 實車地圖的牆常有缺口
                   （玻璃、掃描角度沒掃到），不補的話模擬車會從缺口開出地圖外

牆以「整面長方形」輸出（同一列連續的牆格併成一段，上下相同的段再併成一塊），
一張 8 × 7 m 的地圖大約幾百塊，Gazebo 載入很快。
"""

import argparse
import math
import os
import sys
from collections import deque

import yaml

FREE, WALL, UNKNOWN = 0, 1, -1


# ──────────────────────────────────────────────────────────────
# 世界檔（bottlenecks.sdf 也用這個產生，兩種世界的物理設定一致）
# ──────────────────────────────────────────────────────────────
def world_sdf(world_name, boxes, header_comment=""):
    """boxes: [(標籤, cx, cy, yaw, sx, sy, 高), ...] → 完整 SDF 字串

    ★ 物理步長 1 ms：前輪轉向連桿只有 0.05 kg，步長放大會抖。
      2026-09-26 在這台筆電的 WSL 實測 gpu_lidar＋1 ms 即時率 1.00。
    ★ Sensors 不寫死 render_engine：讓 `gz sim --render-engine ogre` 蓋得過去
      （WSL 上 ogre2 視窗有時會崩，要能退回 ogre）。
    """
    elems = []
    for label, cx, cy, yaw, sx, sy, h in boxes:
        pose = f"{cx:.4f} {cy:.4f} {h / 2:.4f} 0 0 {yaw:.6f}"
        size = f"{sx:.4f} {sy:.4f} {h:.4f}"
        elems.append(
            f"        <!-- {label} -->\n"
            f"        <collision name=\"c_{len(elems)}\"><pose>{pose}</pose>"
            f"<geometry><box><size>{size}</size></box></geometry></collision>\n"
            f"        <visual name=\"v_{len(elems)}\"><pose>{pose}</pose>"
            f"<geometry><box><size>{size}</size></box></geometry>"
            f"<material><ambient>0.7 0.7 0.72 1</ambient><diffuse>0.7 0.7 0.72 1</diffuse></material>"
            f"</visual>")
    comment = "".join(f"  {line}\n" for line in header_comment.strip().splitlines())
    return f"""<?xml version="1.0"?>
<!--
{comment}-->
<sdf version="1.10">
  <world name="{world_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.85 0.83 0.78 1</ambient><diffuse>0.85 0.83 0.78 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="walls">
      <static>true</static>
      <link name="link">
{chr(10).join(elems)}
      </link>
    </model>
  </world>
</sdf>
"""


# ──────────────────────────────────────────────────────────────
# 讀圖
# ──────────────────────────────────────────────────────────────
def read_pgm(path):
    """讀 P5（二進位）或 P2（文字）PGM，回傳 (寬, 高, 最大值, 像素串列)。不依賴 PIL。"""
    with open(path, "rb") as f:
        raw = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b"#":           # 註解到行尾
            while pos < len(raw) and raw[pos:pos + 1] not in (b"\n", b"\r"):
                pos += 1
            continue
        start = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace():
            pos += 1
        tokens.append(raw[start:pos])
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b"P5":
        pos += 1                                # 表頭後恰好一個空白字元
        if maxval < 256:
            pix = list(raw[pos:pos + w * h])
        else:
            pix = [int.from_bytes(raw[pos + 2 * i:pos + 2 * i + 2], "big") for i in range(w * h)]
    elif magic == b"P2":
        pix = [int(t) for t in raw[pos:].split()][: w * h]
    else:
        raise ValueError(f"{path} 不是 PGM（開頭是 {magic!r}）")
    if len(pix) != w * h:
        raise ValueError(f"{path} 像素數 {len(pix)} ≠ {w}×{h}")
    return w, h, maxval, pix


def classify(meta, w, h, maxval, pix):
    """像素 → grid[列][行]，列 0 是圖片最上面（y 最大）"""
    occ = float(meta.get("occupied_thresh", 0.65))
    free = float(meta.get("free_thresh", 0.196))
    negate = int(meta.get("negate", 0))
    grid = []
    for r in range(h):
        row = []
        for c in range(w):
            v = pix[r * w + c] / maxval
            p = v if negate else 1.0 - v
            row.append(WALL if p > occ else FREE if p < free else UNKNOWN)
        grid.append(row)
    return grid


NEIGHBORS8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def remove_small_blobs(grid, min_cells):
    """小於 min_cells 格的牆塊（8 連通）改成空地，回傳丟掉幾格"""
    if min_cells <= 1:
        return 0
    h, w = len(grid), len(grid[0])
    seen = [[False] * w for _ in range(h)]
    removed = 0
    for r in range(h):
        for c in range(w):
            if grid[r][c] != WALL or seen[r][c]:
                continue
            blob, q = [], deque([(r, c)])
            seen[r][c] = True
            while q:
                y, x = q.popleft()
                blob.append((y, x))
                for dy, dx in NEIGHBORS8:
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < h and 0 <= nx < w and not seen[ny][nx] and grid[ny][nx] == WALL:
                        seen[ny][nx] = True
                        q.append((ny, nx))
            if len(blob) < min_cells:
                for y, x in blob:
                    grid[y][x] = FREE
                removed += len(blob)
    return removed


def seal_unknown(grid):
    """跟空地 8 連通相鄰的未知格補成牆，回傳補了幾格"""
    h, w = len(grid), len(grid[0])
    to_seal = []
    for r in range(h):
        for c in range(w):
            if grid[r][c] != UNKNOWN:
                continue
            for dy, dx in NEIGHBORS8:
                ny, nx = r + dy, c + dx
                if 0 <= ny < h and 0 <= nx < w and grid[ny][nx] == FREE:
                    to_seal.append((r, c))
                    break
    for r, c in to_seal:
        grid[r][c] = WALL
    # 空地貼著圖片邊緣的話，邊緣外面什麼都沒有，也補一圈
    for r in range(h):
        for c in (0, w - 1):
            if grid[r][c] == FREE:
                grid[r][c] = WALL
                to_seal.append((r, c))
    for c in range(w):
        for r in (0, h - 1):
            if grid[r][c] == FREE:
                grid[r][c] = WALL
                to_seal.append((r, c))
    return len(to_seal)


def merge_rectangles(grid):
    """牆格 → 長方形 [(r0, c0, r1, c1)]（含端點）。同列連續段，上下同寬再合併。"""
    h, w = len(grid), len(grid[0])
    done, open_rects = [], {}               # open_rects[(c0, c1)] = r0
    for r in range(h + 1):
        runs = set()
        if r < h:
            c = 0
            while c < w:
                if grid[r][c] == WALL:
                    c0 = c
                    while c < w and grid[r][c] == WALL:
                        c += 1
                    runs.add((c0, c - 1))
                else:
                    c += 1
        for key in list(open_rects):
            if key not in runs:              # 這一列斷了 → 收尾
                done.append((open_rects.pop(key), key[0], r - 1, key[1]))
        for key in runs:
            open_rects.setdefault(key, r)
    return done


def grid_to_boxes(grid, meta, height):
    res = float(meta["resolution"])
    ox, oy, oyaw = (list(meta["origin"]) + [0.0, 0.0, 0.0])[:3]
    h = len(grid)
    cos_t, sin_t = math.cos(oyaw), math.sin(oyaw)
    boxes = []
    for r0, c0, r1, c1 in merge_rectangles(grid):
        # 圖片座標（未旋轉）：左下角是原點，列 0 在最上面
        px = (c0 + c1 + 1) / 2.0 * res
        py = (2 * h - r0 - r1 - 1) / 2.0 * res
        cx = ox + cos_t * px - sin_t * py
        cy = oy + sin_t * px + cos_t * py
        boxes.append((f"列 {r0}-{r1} 行 {c0}-{c1}", cx, cy, oyaw,
                      (c1 - c0 + 1) * res, (r1 - r0 + 1) * res, height))
    return boxes


def _xy_to_rc(grid, meta, x, y):
    res = float(meta["resolution"])
    ox, oy, oyaw = (list(meta["origin"]) + [0.0, 0.0, 0.0])[:3]
    dx, dy = x - ox, y - oy
    px = math.cos(oyaw) * dx + math.sin(oyaw) * dy
    py = -math.sin(oyaw) * dx + math.cos(oyaw) * dy
    return len(grid) - 1 - int(math.floor(py / res)), int(math.floor(px / res))


def _rc_to_xy(grid, meta, r, c):
    """格子中心的地圖座標"""
    res = float(meta["resolution"])
    ox, oy, oyaw = (list(meta["origin"]) + [0.0, 0.0, 0.0])[:3]
    px, py = (c + 0.5) * res, (len(grid) - 1 - r + 0.5) * res
    return (ox + math.cos(oyaw) * px - math.sin(oyaw) * py,
            oy + math.sin(oyaw) * px + math.cos(oyaw) * py)


def cell_at(grid, meta, x, y):
    """地圖座標 (x, y) 落在哪一格、那格是什麼（地圖外回傳 None）"""
    r, c = _xy_to_rc(grid, meta, x, y)
    if 0 <= r < len(grid) and 0 <= c < len(grid[0]):
        return grid[r][c]
    return None


def wall_clearance(grid):
    """每一格到最近牆格的距離（格數，8 方向 BFS 近似）。牆本身是 0。"""
    h, w = len(grid), len(grid[0])
    INF = 10 ** 9
    dist = [[INF] * w for _ in range(h)]
    q = deque()
    for r in range(h):
        for c in range(w):
            if grid[r][c] == WALL:
                dist[r][c] = 0
                q.append((r, c))
    while q:
        r, c = q.popleft()
        for dy, dx in NEIGHBORS8:
            ny, nx = r + dy, c + dx
            if 0 <= ny < h and 0 <= nx < w and dist[ny][nx] > dist[r][c] + 1:
                dist[ny][nx] = dist[r][c] + 1
                q.append((ny, nx))
    return dist


def suggest_start(grid, meta, clearance_m=0.35):
    """離 (0, 0) 最近、且離牆至少 clearance_m 的空地中心 → (x, y, 是否就是原點那格)

    車寬 0.37、後軸到車頭 0.40：0.35 m 的淨空讓車子放下去不會卡進牆裡。
    找不到（整張圖都太窄）就退回最寬敞的那格。
    """
    res = float(meta["resolution"])
    need = int(math.ceil(clearance_m / res))
    dist = wall_clearance(grid)
    r0, c0 = _xy_to_rc(grid, meta, 0.0, 0.0)
    best, best_d2, widest = None, None, None
    for r, row in enumerate(grid):
        for c, v in enumerate(row):
            if v != FREE:
                continue
            if widest is None or dist[r][c] > dist[widest[0]][widest[1]]:
                widest = (r, c)
            if dist[r][c] >= need:
                d2 = (r - r0) ** 2 + (c - c0) ** 2
                if best_d2 is None or d2 < best_d2:
                    best, best_d2 = (r, c), d2
    pick = best or widest
    if pick is None:
        return 0.0, 0.0, False
    if pick == (r0, c0):
        return 0.0, 0.0, True
    x, y = _rc_to_xy(grid, meta, *pick)
    # + 0.0 把 round 產生的 -0.0 變回 0.0（不然 log 會印「起點 (-0.0, …)」）
    return round(x, 2) + 0.0, round(y, 2) + 0.0, False


def convert(map_yaml, height=1.0, min_blob=3, seal=True, world_name=None):
    """回傳 (SDF 字串, 摘要字串, 建議起點 (x, y))。launch 也直接呼叫這個。"""
    with open(map_yaml, encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    img = meta["image"]
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), img)
    w, h, maxval, pix = read_pgm(img)
    grid = classify(meta, w, h, maxval, pix)
    n_wall0 = sum(row.count(WALL) for row in grid)
    removed = remove_small_blobs(grid, min_blob)
    sealed = seal_unknown(grid) if seal else 0
    boxes = grid_to_boxes(grid, meta, height)
    origin_cell = {FREE: "空地", WALL: "牆", UNKNOWN: "未知", None: "地圖外"}[cell_at(grid, meta, 0.0, 0.0)]
    sx, sy, at_origin = suggest_start(grid, meta)
    why = "離牆不到 0.35 m" if origin_cell == "空地" else f"是「{origin_cell}」"
    start_txt = "(0, 0) ✓" if at_origin else f"({sx}, {sy}) —— 原點那格{why}，改放最近的寬敞空地"
    name = world_name or os.path.splitext(os.path.basename(map_yaml))[0]
    summary = (
        f"地圖 {os.path.basename(map_yaml)}：{w}×{h} 格、解析度 {meta['resolution']} m"
        f"（{w * float(meta['resolution']):.1f} × {h * float(meta['resolution']):.1f} m）\n"
        f"  牆 {n_wall0} 格、去掉雜點 {removed} 格、封邊補 {sealed} 格 → 合併成 {len(boxes)} 塊\n"
        f"  起點：{start_txt}")
    header = (
        f"由 smartnav_sim/pgm_to_world 從 {os.path.basename(map_yaml)} 產生（請勿手改，重新轉就好）。\n"
        f"世界座標 = 地圖座標；(0, 0) 通常是當初建圖的起點。建議起點 x:={sx} y:={sy}。\n"
        f"牆高 {height} m、去雜點門檻 {min_blob} 格、封邊 {'開' if seal else '關'}。")
    return world_sdf(name, boxes, header), summary, (sx, sy)


def main(argv=None):
    ap = argparse.ArgumentParser(description="map_server 地圖（.yaml + .pgm）→ Gazebo 世界（.sdf）")
    ap.add_argument("map_yaml")
    ap.add_argument("-o", "--output", help="輸出 .sdf（預設：目前目錄下的 <地圖名>.sdf）")
    ap.add_argument("--wall-height", type=float, default=1.0)
    ap.add_argument("--min-blob", type=int, default=3, help="小於這麼多格的孤立牆點丟掉（1 = 全留）")
    ap.add_argument("--no-seal", action="store_true", help="不要把空地旁的未知格補成牆")
    ap.add_argument("--name", help="世界名稱（預設用地圖檔名）")
    a = ap.parse_args(argv)
    sdf, summary, _ = convert(a.map_yaml, a.wall_height, a.min_blob, not a.no_seal, a.name)
    out = a.output or os.path.splitext(os.path.basename(a.map_yaml))[0] + ".sdf"
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(sdf)
    print(summary)
    print(f"  → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
