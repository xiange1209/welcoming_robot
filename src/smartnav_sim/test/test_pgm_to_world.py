"""pgm_to_world：讀圖、分類、清理、合併、座標（不需要 ROS：python -m pytest test/）"""

import os
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from smartnav_sim import pgm_to_world as p2w  # noqa: E402

W, F, U = 0, 254, 205     # map_saver 的三種像素：牆 0、空地 254、未知 205


def write_map(tmp_path, rows, res=0.5, origin=(-1.0, -1.0, 0.0), fmt="P5"):
    h, w = len(rows), len(rows[0])
    pgm = tmp_path / "m.pgm"
    if fmt == "P5":
        pgm.write_bytes(f"P5\n# CREATOR: test\n{w} {h}\n255\n".encode() + bytes(v for r in rows for v in r))
    else:
        pgm.write_text(f"P2\n{w} {h}\n255\n" + "\n".join(" ".join(map(str, r)) for r in rows) + "\n")
    y = tmp_path / "m.yaml"
    y.write_text(
        f"image: m.pgm\nmode: trinary\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, {origin[2]}]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return str(y)


def load(yaml_path):
    import yaml
    meta = yaml.safe_load(open(yaml_path, encoding="utf-8"))
    img = os.path.join(os.path.dirname(yaml_path), meta["image"])
    return meta, p2w.classify(meta, *p2w.read_pgm(img))


@pytest.mark.parametrize("fmt", ["P5", "P2"])
def test_read_and_classify(tmp_path, fmt):
    y = write_map(tmp_path, [[W, F, U]], fmt=fmt)
    _, grid = load(y)
    assert grid == [[p2w.WALL, p2w.FREE, p2w.UNKNOWN]]


def test_merge_covers_each_wall_cell_exactly_once():
    grid = [[1, 1, 0, 1],
            [1, 1, 0, 1],
            [0, 1, 1, 1],
            [1, 0, 0, 0]]
    covered = {}
    for r0, c0, r1, c1 in p2w.merge_rectangles(grid):
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                covered[(r, c)] = covered.get((r, c), 0) + 1
    walls = {(r, c) for r in range(4) for c in range(4) if grid[r][c] == 1}
    assert set(covered) == walls
    assert all(n == 1 for n in covered.values())
    # 左上 2×2 應該併成一塊
    assert (0, 0, 1, 1) in p2w.merge_rectangles(grid)


def test_box_coordinates_follow_map_server_convention(tmp_path):
    # 3×3、解析度 0.5、原點 (-1,-1)：左下角格子中心 = (-0.75, -0.75)
    y = write_map(tmp_path, [[F, F, F],
                             [F, F, F],
                             [W, F, F]])
    meta, grid = load(y)
    boxes = p2w.grid_to_boxes(grid, meta, 1.0)
    assert len(boxes) == 1
    _, cx, cy, yaw, sx, sy, h = boxes[0]
    assert (cx, cy, sx, sy) == pytest.approx((-0.75, -0.75, 0.5, 0.5))
    # 圖片最上面那列是 y 最大
    assert p2w.cell_at(grid, meta, -0.75, -0.75) == p2w.WALL
    assert p2w.cell_at(grid, meta, -0.75, 0.25) == p2w.FREE
    assert p2w.cell_at(grid, meta, 5.0, 5.0) is None


def test_small_blobs_removed_big_kept():
    grid = [[1, 0, 0, 0, 0],
            [0, 0, 0, 1, 1],
            [0, 0, 0, 1, 1]]
    removed = p2w.remove_small_blobs(grid, 3)
    assert removed == 1
    assert grid[0][0] == p2w.FREE and grid[1][3] == p2w.WALL


def test_seal_closes_gaps_next_to_free_space():
    grid = [[-1, -1, -1, -1],
            [-1, 0, 0, -1],
            [-1, -1, -1, -1],
            [-1, -1, -1, -1]]
    p2w.seal_unknown(grid)
    # 空地旁一圈變牆，遠處的未知不動
    assert grid[0][0] == p2w.WALL and grid[2][3] == p2w.WALL
    assert grid[3][0] == p2w.UNKNOWN


def test_seal_closes_free_cells_on_image_border():
    grid = [[0, 0], [0, 0]]
    p2w.seal_unknown(grid)
    assert all(v == p2w.WALL for row in grid for v in row)


def test_start_is_origin_when_roomy(tmp_path):
    rows = [[F] * 21 for _ in range(21)]
    y = write_map(tmp_path, rows, res=0.05, origin=(-0.525, -0.525, 0.0))
    meta, grid = load(y)
    assert p2w.suggest_start(grid, meta) == (0.0, 0.0, True)


def test_start_moves_away_from_wall(tmp_path):
    rows = [[F] * 40 for _ in range(20)]
    for r in range(20):
        rows[r][10] = W              # 原點正好在一道牆上
    y = write_map(tmp_path, rows, res=0.05, origin=(-0.525, -0.5, 0.0))
    meta, grid = load(y)
    x, yy, at_origin = p2w.suggest_start(grid, meta)
    assert not at_origin
    assert p2w.cell_at(grid, meta, x, yy) == p2w.FREE
    dist = p2w.wall_clearance(grid)
    r, c = p2w._xy_to_rc(grid, meta, x, yy)
    assert dist[r][c] * 0.05 >= 0.35


def test_world_sdf_is_valid_xml_and_counts_boxes(tmp_path):
    rows = [[W, W, W], [W, F, W], [W, W, W]]
    y = write_map(tmp_path, rows)
    sdf, summary, start = p2w.convert(y)
    root = ET.fromstring(sdf.encode("utf-8"))
    walls = [m for m in root.iter("model") if m.get("name") == "walls"][0]
    assert len(list(walls.iter("collision"))) == len(list(walls.iter("visual"))) >= 3
    assert "合併成" in summary


def test_bundled_worlds_are_valid_xml():
    # XML 註解裡不能出現 "--"，手改 worlds/*.sdf 時最容易踩到
    wdir = os.path.join(os.path.dirname(__file__), "..", "worlds")
    for name in os.listdir(wdir):
        if name.endswith(".sdf"):
            ET.parse(os.path.join(wdir, name))
