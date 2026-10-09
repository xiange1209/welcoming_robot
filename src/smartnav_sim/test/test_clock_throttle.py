"""clock_throttle 的轉發決策（不需要 rclpy 執行環境）。"""
from smartnav_sim.clock_throttle_node import should_forward

MS = 1_000_000


def test_first_message_always_forwarded():
    assert should_forward(None, 123, 10 * MS)


def test_drops_messages_closer_than_min_dt():
    last = 1_000 * MS
    assert not should_forward(last, last + 1 * MS, 10 * MS)
    assert not should_forward(last, last + 9 * MS, 10 * MS)


def test_forwards_at_or_after_min_dt():
    last = 1_000 * MS
    assert should_forward(last, last + 10 * MS, 10 * MS)
    assert should_forward(last, last + 25 * MS, 10 * MS)


def test_zero_min_dt_forwards_everything():
    last = 1_000 * MS
    assert should_forward(last, last, 0)
    assert should_forward(last, last + 1, 0)


def test_time_going_backwards_is_always_forwarded():
    # 模擬重置：時間回到 0，下游不能卡在舊的未來
    assert should_forward(5_000 * MS, 2 * MS, 10 * MS)


def test_rate_100hz_over_1khz_stream_keeps_about_a_tenth():
    last, kept = None, 0
    for i in range(10_000):          # 10 秒、1 kHz
        ns = i * MS
        if should_forward(last, ns, 10 * MS):
            last, kept = ns, kept + 1
    assert 990 <= kept <= 1010
