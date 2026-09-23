"""網路位址偵測。"""

import socket
from typing import List


def detect_lan_ips() -> List[str]:
    """列出本機可供平板連線的 IPv4 位址（排除 loopback）

    刻意不依賴 netifaces／psutil 之類的額外套件——Pi 上多裝一個 pip 套件
    就多一個部署時會忘記裝的東西。兩段式偵測：

      1. 開一個「連到外部位址」的 UDP socket，問路由表該走哪張網卡。
         UDP 的 connect 不會真的送出封包，**沒有網路也能用**，
         拿到的是預設路由那張網卡的位址——通常就是平板連得到的那個。
      2. 再從主機名稱解析補上其他網卡：機器人常同時接有線與 WiFi，
         平板可能只連得到其中一邊。

    Returns:
        List[str]: 依可用性排序的 IPv4 位址，偵測不到時為空 list
    """
    ips: List[str] = []

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        ips.append(probe.getsockname()[0])
    except OSError:
        pass  # 沒有預設路由（例如只接了一條沒設閘道的網線）
    finally:
        probe.close()

    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            # Debian 系的 /etc/hosts 常把主機名指到 127.0.1.1，那個對平板沒用
            if ip not in ips and not ip.startswith("127."):
                ips.append(ip)
    except OSError:
        pass

    return ips
