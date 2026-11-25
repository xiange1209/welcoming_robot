# smartnav-hmi 服務的附加設定

`20-asr-priority.conf`：讓 HMI 與它啟動的單元可以調高優先權。沒有它，處理器分時整個不作用（log 會有「處理器分時沒有作用」）。

安裝（換卡、重裝後都要做一次；重啟 HMI 會停掉它開著的單元，挑車子停著的時候做）：

```bash
sudo mkdir -p /etc/systemd/system/smartnav-hmi.service.d
sudo install -m 644 20-asr-priority.conf /etc/systemd/system/smartnav-hmi.service.d/
sudo systemctl daemon-reload && sudo systemctl restart smartnav-hmi
systemctl show -p LimitNICE -p LimitRTPRIO smartnav-hmi    # 要看到 LimitNICE=30（＝nice -10）、LimitRTPRIO=30
```

展場要關掉收音距離閘門時，另建 `30-demo.conf`：

```ini
[Service]
Environment=SMARTNAV_PRESENCE_GATE=0
```
