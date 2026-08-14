#!/usr/bin/env python3
"""量任意話題的實際頻率 —— 取代 `ros2 topic hz`。

★ 2026-08-14：這台機器上 `ros2 topic hz` / `topic info` 會逾時而印不出東西，
  而「印不出東西」跟「沒有資料」長得一模一樣。我今天為此誤判四次，
  其中一次還害使用者白插拔了一次相機（當時實際是 6.40 Hz）。

  **CLI 逾時不是證據。** 這支自己開訂閱者，並且一律用 BEST_EFFORT
  （相容 RELIABLE 發布者；反過來則收不到，/scan 就是 BEST_EFFORT 的）。

用法：
    python3 rate.py /scan /odom /imu/data_raw
    python3 rate.py --sec 20 /camera/color/image_raw
"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def resolve(node, topic, tries=8):
    """從 graph 查話題型別，不用使用者自己指定。"""
    for _ in range(tries):
        for name, types in node.get_topic_names_and_types():
            if name == topic and types:
                return types[0]
        rclpy.spin_once(node, timeout_sec=0.4)
    return None


def load(type_str):
    pkg, kind, msg = type_str.split("/")
    mod = __import__(f"{pkg}.{kind}", fromlist=[msg])
    return getattr(mod, msg)


def main():
    args = sys.argv[1:]
    sec = 10.0
    if "--sec" in args:
        i = args.index("--sec")
        sec = float(args[i + 1])
        del args[i:i + 2]
    if not args:
        print(__doc__)
        return 1

    rclpy.init()
    n = Node("rate_probe")
    counts = {}
    missing = []
    for t in args:
        ts = resolve(n, t)
        if ts is None:
            missing.append(t)
            continue
        counts[t] = 0

        def mk(name):
            def cb(_):
                counts[name] += 1
            return cb

        n.create_subscription(load(ts), t, mk(t), qos_profile_sensor_data)

    print(f"  量 {sec:.0f} 秒…")
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)
    dt = time.time() - t0
    print()
    for t, c in counts.items():
        print(f"  {'✓' if c else '✗'} {t:34s} {c:5d} 則  {c/dt:7.2f} Hz")
    for t in missing:
        print(f"  ? {t:34s} 話題不存在（沒有任何發布者）")
    rclpy.try_shutdown()
    return 0 if counts and all(counts.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
