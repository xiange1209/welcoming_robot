"""使用者類型（與 smartnav_msgs/UserType.msg 的數值一致；要改先改 .msg）。

不依賴 numpy／cv2，讓只需要類型名稱的節點（nav_arbiter）不必載入影像套件。
"""

from enum import IntEnum


class UserType(IntEnum):
    """使用者類型枚舉

    BLACKLIST 是 2026-08-01 補上的 —— 專題的後半段是「安全通報」，
    但先前的資料模型裡沒有黑名單這個概念，等於通報永遠沒有觸發條件。
    數值接在 ADMIN 之後而不是插在中間：改動既有值會讓已註冊的
    使用者型別全部錯位（SQLite 裡存的是名稱字串，但 msg 用的是 uint8）。
    """

    GUEST = 0
    VIP = 1
    ADMIN = 2
    BLACKLIST = 3
