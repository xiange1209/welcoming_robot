"""把 Gazebo 的 /clock 限流後再發給所有節點。

## 為什麼要這個（2026-10-08 量到的）

Gazebo 的 /clock **每個物理步都發、不節流**。物理步長 1 ms、即時率 0.5 時，
/clock 是 **574.7 Hz**（實測，`ros2 topic hz /clock`）。模擬裡約 25 個節點都用 use_sim_time，
每個都要處理這 575 則/秒：

  - rclpy 每收到一則 /clock 就更新時鐘、喚醒執行器重算所有計時器 —— Python 節點特別貴。
    閒置（車子沒動、沒導航）時 path_teach 74%、map_service 65%、waypoint_service 58%、
    navigation_action 51%、stuck_detector 50%、steering_trim 47%（1 個核心 = 100%）；
    同一時間 nav2 的 C++ 節點只有 4~11%。
  - 全部加起來把筆電的 WSL（8 核）吃到 load average 約 7~16，**即時率掉到 0.3~0.55**
    （9/26 較輕的世界是 1.00）。

即時率掉下來有兩個後果：(1) 實驗慢兩倍；(2) path_teach 的計時（「5 秒沒動」「等 8 秒」「偏離 2 秒」）
用的是牆鐘 time.monotonic()，而物理世界只走了一半 —— 「5 秒沒動」只對應 2.5 秒的模擬時間，
模擬比真車嚴苛，失敗率被放大。把 /clock 降到 100 Hz 就能把負載砍掉大半、即時率拉回接近 1。

## 做法

ros_gz_bridge 把 Gazebo 的 /clock 橋成 **/clock_raw**（見 config/bridge.yaml），
本節點以 rate_hz 限流後發成 /clock。rate_hz <= 0 = 不限流（逐則轉發，等於舊行為，
用來重現 10/08 以前的實驗）。

★ 本節點自己**不能**用 use_sim_time（它就是 /clock 的來源）。
★ 100 Hz = 模擬時間每 10 ms 一格：夠 20 Hz 的控制迴圈與 30 Hz 的里程計用，tf 查詢也不受影響。
★ 時間倒退（模擬被重置）一定放行，免得下游卡在「未來」。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def should_forward(last_ns, ns, min_dt_ns) -> bool:
    """這一則 /clock 要不要轉發。

    - 第一則一定轉發
    - 離上一則轉發的時間不到 min_dt_ns（模擬時間）→ 丟掉
    - min_dt_ns <= 0（不限流）→ 全部轉發
    - 時間倒退（模擬被重置）一定放行，免得下游卡在「未來」
    """
    if last_ns is None or ns < last_ns:
        return True
    return ns - last_ns >= min_dt_ns


class ClockThrottle(Node):
    def __init__(self):
        super().__init__("clock_throttle")
        self.declare_parameter("rate_hz", 100.0)
        rate = float(self.get_parameter("rate_hz").value)
        self._min_dt_ns = int(1e9 / rate) if rate > 0.0 else 0
        self._last_ns = None
        self._in = 0
        self._out = 0
        # 訂閱用 BEST_EFFORT：與 RELIABLE／BEST_EFFORT 的發布者都接得上。
        # 發布用 RELIABLE：與 RELIABLE／BEST_EFFORT 的訂閱者都接得上（rclpy／rclcpp 的 use_sim_time 兩種都有）。
        self._pub = self.create_publisher(Clock, "/clock", QoSProfile(depth=10))
        self.create_subscription(
            Clock, "/clock_raw", self._cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
        self.create_timer(30.0, self._report)
        self.get_logger().info(
            "時鐘限流：/clock_raw -> /clock，" +
            (f"{rate:.0f} Hz" if rate > 0.0 else "不限流（逐則轉發）"))

    def _cb(self, msg: Clock) -> None:
        self._in += 1
        ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if not should_forward(self._last_ns, ns, self._min_dt_ns):
            return
        self._last_ns = ns
        self._out += 1
        self._pub.publish(msg)

    def _report(self) -> None:
        # 只在有流量時印；比例就是這個節點幫全部下游省下的處理量
        if self._in:
            self.get_logger().info(
                f"近 30 秒：收 {self._in} 則 / 發 {self._out} 則（省下 {100 - 100 * self._out / self._in:.0f}%）")
        self._in = self._out = 0


def main(args=None):
    rclpy.init(args=args)
    node = ClockThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
