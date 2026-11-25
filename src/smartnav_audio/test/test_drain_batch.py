"""語音辨識積壓時合併音訊塊（drain_batch，2026-11-24）。"""
import queue

import numpy as np

from smartnav_audio.speech_recognizer_node import drain_batch


def _chunk(v):
    return np.full(512, v, dtype=np.float32)


def test_no_backlog_takes_only_one():
    q = queue.Queue()
    parts, fin, taken, stop = drain_batch(q, (_chunk(1), False), 16)
    assert len(parts) == 1 and not fin and taken == 1 and not stop


def test_merges_backlog_up_to_limit_in_order():
    q = queue.Queue()
    for i in range(2, 30):
        q.put((_chunk(i), False))
    parts, fin, taken, stop = drain_batch(q, (_chunk(1), False), 16)
    assert taken == 16 and len(parts) == 16 and not fin and not stop
    assert [float(p[0]) for p in parts] == list(range(1, 17))      # 順序不變
    assert q.qsize() == 30 - 2 - 15                                 # 剩下的留給下一批


def test_stops_at_final_chunk():
    q = queue.Queue()
    q.put((_chunk(2), False))
    q.put((_chunk(3), True))       # 句尾
    q.put((_chunk(4), False))      # 下一句，不能混進來
    parts, fin, taken, stop = drain_batch(q, (_chunk(1), False), 16)
    assert fin and taken == 3 and [float(p[0]) for p in parts] == [1, 2, 3]
    assert q.qsize() == 1


def test_first_chunk_final_is_not_merged():
    q = queue.Queue()
    q.put((_chunk(2), False))
    parts, fin, taken, stop = drain_batch(q, (_chunk(1), True), 16)
    assert fin and taken == 1 and len(parts) == 1 and q.qsize() == 1


def test_sentinel_stops_and_is_counted():
    q = queue.Queue()
    q.put((_chunk(2), False))
    q.put(None)
    parts, fin, taken, stop = drain_batch(q, (_chunk(1), False), 16)
    assert stop and taken == 3 and len(parts) == 2


def test_batch_one_disables_merging():
    q = queue.Queue()
    q.put((_chunk(2), False))
    parts, fin, taken, stop = drain_batch(q, (_chunk(1), False), 1)
    assert taken == 1 and q.qsize() == 1
