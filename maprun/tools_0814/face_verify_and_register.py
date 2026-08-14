#!/usr/bin/env python3
"""一次站定就把「同步修好了沒」與「註冊」兩件事做完。

★ 2026-08-14 背景
`user_auth_node.py` 的 ApproximateTimeSynchronizer 原本 `queue_size=10` 寫死。
10 格 ÷ 影像速率 = 佇列涵蓋的時間，而向量要等 InsightFace CPU 推論（落後約 1.1 秒）：

    影像 6.4 Hz -> 1.56 秒 > 1.1  配得到
    影像  25 Hz -> 0.40 秒 < 1.1  **永遠配不到**（/user_identity 全靜默、註冊收 0/10）

已改成參數 `sync_queue_size`，預設 60。這支先確認同步真的活了，再送註冊，
避免使用者站兩次。

流程：
  1. 等 /user_identity 出現   -> 同步活著的直接證據
  2. 立刻送 register_face      -> 盯 registration_progress 的 collected 逐張跳
  3. 註冊完再等辨識            -> 確認認得出來、相似度多少
"""
import sys, time
import rclpy
from rclpy.node import Node
from smartnav_msgs.msg import UserIdentity, RegistrationProgress, UserType
from smartnav_msgs.srv import RegisterFace

NAME = sys.argv[1] if len(sys.argv) > 1 else "陳佳憲"
N = int(sys.argv[2]) if len(sys.argv) > 2 else 10


class Flow(Node):
    def __init__(self):
        super().__init__("face_verify_register")
        self.t0 = time.time()
        self.ident = []
        self.prog = []
        self.create_subscription(UserIdentity, "user_identity", self._id, 10)
        self.create_subscription(RegistrationProgress, "registration_progress", self._pg, 10)
        self.cli = self.create_client(RegisterFace, "register_face")

    def _t(self):
        return time.time() - self.t0

    def _id(self, m):
        self.ident.append((self._t(), m.user_name, m.similarity, m.recognized,
                           getattr(m.user_type, "type", -1)))

    def _pg(self, m):
        c = getattr(m, "collected_samples", 0)
        if not self.prog or self.prog[-1][1] != c or m.status != self.prog[-1][2]:
            self.prog.append((self._t(), c, m.status))
            print(f"    [{self._t():5.1f}s] {m.status:10s} {c}/{getattr(m,'target_samples',0)}  {m.message[:44]}")

    def spin(self, sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init(); f = Flow()

    print("  ── 步驟 1：確認同步回呼活著（等 /user_identity）──")
    t = time.time()
    while time.time() - t < 30.0 and not f.ident:
        rclpy.spin_once(f, timeout_sec=0.05)
    if not f.ident:
        # ★ 沒有 user_identity 有兩個完全不同的原因，不要混為一談：
        #   (a) 鏡頭前沒有人 -> face_embedding 根本不會發
        #   (b) 同步壞掉     -> face_embedding 有發，但配不到影像
        # 2026-08-14 我第一版直接印「同步仍然沒修好」，而當時只是使用者走開了，
        # 差點又把人導向錯的方向。要分開就看 face_embedding 有沒有在發。
        from smartnav_msgs.msg import FaceEmbedding
        cnt = [0]
        f.create_subscription(FaceEmbedding, "face_embedding", lambda _: cnt.__setitem__(0, cnt[0] + 1), 10)
        f.spin(8.0)
        if cnt[0] == 0:
            print("  ⚠ 30 秒沒有 /user_identity，且 face_embedding 也沒有 ——"
                  " **鏡頭前沒有人臉**，不是同步的問題。請站到鏡頭前再跑一次。")
        else:
            print(f"  ✗✗ face_embedding 有在發（8 秒 {cnt[0]} 則）卻沒有 /user_identity ——"
                  " 這才是同步真的壞了。查 sync_queue_size 與影像速率。")
        rclpy.try_shutdown(); return 1
    _, nm, sim, rec, ut = f.ident[0]
    print(f"  ✓ 同步活著：{nm!r} similarity={sim:.3f} recognized={rec} type={ut}\n")

    print(f"  ── 步驟 2：註冊 {NAME}（VIP，{N} 張）──")
    if not f.cli.wait_for_service(timeout_sec=12.0):
        print("  ✗ register_face 沒回應"); rclpy.try_shutdown(); return 1
    req = RegisterFace.Request()
    req.user_name = NAME
    req.user_type = UserType(type=1)
    req.description = "VIP客戶"
    req.num_samples = N
    fut = f.cli.call_async(req)
    rclpy.spin_until_future_complete(f, fut, timeout_sec=15.0)
    r = fut.result()
    if r is None or not r.success:
        print(f"  ✗ {r.message if r else '無回應'}"); rclpy.try_shutdown(); return 1
    f.spin(30.0)

    done = [p for p in f.prog if p[2] in ("succeeded", "timeout")]
    if not done:
        print("\n  ⚠ 30 秒內沒有結束狀態")
    elif done[-1][2] == "timeout":
        print(f"\n  ✗ 仍然逾時，只收到 {done[-1][1]}/{N} 張")
        rclpy.try_shutdown(); return 1
    else:
        print(f"\n  ★★ 註冊成功，共 {done[-1][1]} 張\n")

    print("  ── 步驟 3：確認認得出來 ──")
    f.ident.clear()
    f.spin(20.0)
    if not f.ident:
        print("  ⚠ 20 秒內沒有辨識結果")
    else:
        for t_, nm, sim, rec, ut in f.ident[:6]:
            tag = {0: "GUEST", 1: "VIP", 2: "ADMIN", 3: "BLACKLIST"}.get(ut, str(ut))
            print(f"    [{t_:5.1f}s] {nm!r:14s} {tag:9s} similarity={sim:.3f} recognized={rec}")
        sims = [s for _, _, s, r_, _ in f.ident if r_]
        if sims:
            print(f"\n  ★ 認出 {len(sims)}/{len(f.ident)} 次，相似度 "
                  f"{min(sims):.3f}~{max(sims):.3f}")
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
