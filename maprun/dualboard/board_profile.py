#!/usr/bin/env python3
"""板子設定（single／A／B）的檢查、預覽、套用。所有值都在同目錄的 board_profiles.ini（每一項的說明也在那裡）。

  python3 board_profile.py status                這塊板子現在符合哪一種、逐項 ✓／✗（不用 sudo）
  python3 board_profile.py plan single|A|B       改成那一種會動到什麼（只列出、不改）
  sudo python3 board_profile.py apply single|A|B [--force-mem] [--restart-hmi]
        會動到的檔案先備份 → 逐項套用 → 再檢查一次；備份與紀錄在 ~/maprun/logs/board_profile/。

測試：--root DIR ＝ 系統檔全換成 DIR 底下的檔，指令不真的跑（改寫 DIR/state.json 模擬結果）。
      test_board_profile.py 用它跑 single→A→single、single→B→single、B→A 來回，最後要和原檔一字不差。
★ 2026-10-23 車上 agent 寫（取代 set_board_name.sh／nm_eth0_setup.sh／dualboard_mode.sh 各做各的；舊腳本留著沒改）。
"""
import argparse
import configparser
import datetime
import json
import os
import pwd
import re
import shutil
import signal
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
INI = os.path.join(HERE, "board_profiles.ini")
LOG_DIR = "/home/user/maprun/logs/board_profile"   # collect_data.sh 會把 ~/maprun/logs 帶回筆電（這裡沒有機密）
PROFILES = ("single", "A", "B")

# 系統檔
P_HOSTNAME = "/etc/hostname"
P_HOSTS = "/etc/hosts"
P_HOSTS_TMPL = "/etc/cloud/templates/hosts.debian.tmpl"
P_USERDATA = "/boot/firmware/user-data"
P_CLOUD_OVR = "/etc/cloud/cloud.cfg.d/99-smartnav-hostname.cfg"
P_NM_CONF = "/etc/NetworkManager/conf.d/60-dualboard-eth0.conf"
P_FB_UNIT = "/etc/systemd/system/smartnav-eth0.service"
P_FB_CONF = "/etc/NetworkManager/conf.d/61-dualboard-eth0-unmanaged.conf"
P_HMI_DROPIN = "/etc/systemd/system/smartnav-hmi.service.d/50-dualboard.conf"
P_CHRONY_CONF = "/etc/chrony/conf.d/50-dualboard-server.conf"
P_TSYNC_CONF = "/etc/systemd/timesyncd.conf.d/50-dualboard.conf"
NM_CONN = "dualboard-eth0"

# 範本（repo 裡的檔，A／B 照抄進系統）
T_NM_CONF = os.path.join(HERE, "nm", "60-dualboard-eth0.conf")
T_HMI_DROPIN = os.path.join(HERE, "systemd", "smartnav-hmi.service.d", "50-dualboard.conf")
T_CHRONY = os.path.join(HERE, "chrony", "50-dualboard-server.conf")
T_TSYNC = os.path.join(HERE, "timesyncd", "50-dualboard.conf")
UNIT_TEMPLATES = {
    "smartnav-perception.service": os.path.join(HERE, "systemd", "smartnav-perception.service"),
    "smartnav-b-powerlog.service": os.path.join(HERE, "systemd", "smartnav-b-powerlog.service"),
}

CLOUD_OVR_TEXT = """\
# board_profile.py：保留手動設定的 hostname（smartnav-a／smartnav-b），不要每次開機改回 user-data 的名字
# 復原：python3 ~/maprun/dualboard/board_profile.py apply single（或直接刪掉這個檔）
# （不寫 manage_etc_hosts: false——user-data 的 true 會蓋過它；/etc/hosts 改在 cloud-init 範本）
preserve_hostname: true
"""
HOSTS_BEGIN = "# >>> smartnav 雙板 peer（board_profile.py 管理，不要手改）>>>"
HOSTS_END = "# <<< smartnav 雙板 peer <<<"
# 與 set_board_name.sh 加的區塊一字不差（兩支工具互認；拿掉後 .bashrc 回到原樣）
BASHRC_BEGIN = "# >>> 2026-10-22 板子標示"
BASHRC_END = "# <<< 2026-10-22 板子標示 <<<"
BASHRC_BLOCK = r"""
# >>> 2026-10-22 板子標示（雙板：A＝8 GB、B＝4 GB）>>>
# 依「實際記憶體」判斷是哪塊板子：SD 卡換到另一塊板子也不會標錯。hostname 跟著 SD 卡走，
# 提示字元的 [A·8GB]／[B·4GB] 和 hostname（smartnav-a／smartnav-b）對不起來，就是卡插錯板了。
# 移除：刪掉這兩行標記之間的內容（備份 ~/.bashrc.bak_<日期>）
__smartnav_board() {
  case "$(awk '/MemTotal/{printf "%d", $2/1048576 + 0.5}' /proc/meminfo)" in
    8) echo "A·8GB" ;;
    4) echo "B·4GB" ;;
    *) echo "?GB" ;;
  esac
}
case $- in *i*)
  PS1="[\$(__smartnav_board)] $PS1"
  echo "★ 這是 $(__smartnav_board) 板（hostname：$(hostname)）"
  ;;
esac
# <<< 2026-10-22 板子標示 <<<
"""


# ───────────────────────── 系統存取（真的／沙盒） ─────────────────────────

class RealSystem:
    sandbox = False
    root = ""

    def __init__(self, log):
        self.log = log
        self.backup_dir = None
        self._backed_up = set()

    def path(self, p):
        return self.root + p

    def read(self, p):
        try:
            with open(self.path(p), encoding="utf-8") as f:
                return f.read()
        except (FileNotFoundError, PermissionError, IsADirectoryError):
            return None

    def exists(self, p):
        return os.path.lexists(self.path(p))

    def backup(self, p):
        """每個檔只在第一次被動到之前備份一次（同一次 apply 裡改兩次也留原檔）。"""
        if not self.backup_dir or p in self._backed_up or not self.exists(p):
            return
        os.makedirs(self.backup_dir, exist_ok=True)
        shutil.copy2(self.path(p), os.path.join(self.backup_dir, p.strip("/").replace("/", "__")))
        self._backed_up.add(p)

    def write(self, p, text, owner=None, mode=0o644):
        self.backup(p)
        os.makedirs(os.path.dirname(self.path(p)), exist_ok=True)
        with open(self.path(p), "w", encoding="utf-8") as f:
            f.write(text)
        os.chmod(self.path(p), mode)
        if owner and not self.sandbox:
            pw = pwd.getpwnam(owner)
            os.chown(self.path(p), pw.pw_uid, pw.pw_gid)

    def remove(self, p):
        if self.exists(p):
            self.backup(p)
            os.remove(self.path(p))

    def run(self, cmd, check=False):
        r = subprocess.run(cmd, capture_output=True, text=True)
        out = (r.stdout + r.stderr).strip()
        self.log(f"    $ {' '.join(cmd)}  → {r.returncode}" + (f"\n      {out[-300:]}" if out else ""))
        if check and r.returncode != 0:
            raise RuntimeError(f"指令失敗（{r.returncode}）：{' '.join(cmd)}")
        return r.returncode

    def _query(self, cmd):
        r = subprocess.run(cmd, capture_output=True, text=True)
        return r.returncode, r.stdout

    # ── 探測 ──
    def mem_gb(self):
        with open("/proc/meminfo") as f:
            kb = int(re.search(r"MemTotal:\s+(\d+)", f.read()).group(1))
        return int(kb / 1048576 + 0.5)   # 與 set_board_name.sh、.bashrc 標示同一個算法

    def nm_conn(self, name):
        rc, out = self._query(["nmcli", "-g", "ipv4.addresses,connection.uuid", "connection", "show", name])
        if rc != 0:
            return None
        lines = out.split("\n")
        return {"ip": lines[0].strip(), "uuid": lines[1].strip() if len(lines) > 1 else ""}

    def unit_enabled(self, unit):
        _, out = self._query(["systemctl", "is-enabled", unit])
        return out.strip() or "not-found"

    def pkg_installed(self, pkg):
        rc, out = self._query(["dpkg-query", "-W", "-f=${Status}", pkg])
        return rc == 0 and out.strip().endswith(" installed")


class SandboxSystem(RealSystem):
    """路徑全部接在 root 底下；指令只記錄，並照指令改寫 root/state.json（測試用）。"""
    sandbox = True

    def __init__(self, root, log):
        super().__init__(log)
        self.root = os.path.abspath(root)
        self.state_path = os.path.join(self.root, "state.json")
        self.commands = []

    def _state(self):
        with open(self.state_path, encoding="utf-8") as f:
            return json.load(f)

    def _save(self, st):
        with open(self.state_path, "w", encoding="utf-8") as f:
            json.dump(st, f, ensure_ascii=False, indent=1)

    def run(self, cmd, check=False):
        self.commands.append(cmd)
        self.log(f"    $ {' '.join(cmd)}  （沙盒）")
        st = self._state()
        if cmd[:2] == ["hostnamectl", "set-hostname"]:
            with open(self.path(P_HOSTNAME), "w") as f:
                f.write(cmd[2] + "\n")
        elif cmd[:3] == ["nmcli", "con", "delete"]:
            st["nm"].pop(cmd[3], None)
        elif cmd[:3] == ["nmcli", "con", "add"]:
            st["nm"][cmd[cmd.index("con-name") + 1]] = {"ip": cmd[cmd.index("ipv4.addresses") + 1], "uuid": "sandbox"}
        elif cmd[:2] in (["systemctl", "enable"], ["systemctl", "disable"]):
            st["units"][cmd[-1]] = "enabled" if cmd[1] == "enable" else "disabled"
        elif cmd[:2] == ["apt-get", "install"]:
            st["pkgs"][cmd[-1]] = True
        self._save(st)
        return 0

    def mem_gb(self):
        return self._state()["mem_gb"]

    def nm_conn(self, name):
        return self._state()["nm"].get(name)

    def unit_enabled(self, unit):
        st = self._state()["units"].get(unit)
        if st is None:
            return "disabled" if self.exists(f"/etc/systemd/system/{unit}") else "not-found"
        return st

    def pkg_installed(self, pkg):
        return bool(self._state()["pkgs"].get(pkg))


# ───────────────────────── 共用小工具 ─────────────────────────

def read_template(p):
    with open(p, encoding="utf-8") as f:
        return f.read()


def render_hosts(tmpl, fqdn, hostname):
    """照 cloud-init 的方式把 hosts.debian.tmpl 變成 /etc/hosts（10/23 對原檔與 B 開機後的檔都一字不差）。
    cloud-init 填的是 fqdn＝現在的 hostname、hostname＝user-data 的名字。"""
    first, _, rest = tmpl.partition("\n")
    body = rest if first.startswith("## template:") else tmpl
    body = re.sub(r"\{#.*?-#\}\s*", "", body, flags=re.S)
    body = re.sub(r"\{#.*?#\}", "", body, flags=re.S)
    body = re.sub(r"\{\{\s*hostname\s*\}\}", hostname, body)
    return re.sub(r"\{\{\s*fqdn\s*\}\}", fqdn, body)


def strip_block(text, begin, end):
    """拿掉「換行＋begin…end＋換行」。區塊一律用「原文＋\\n＋區塊」加上去，所以拿掉後與原文一字不差。"""
    return re.sub(r"\n?" + re.escape(begin) + r".*?" + re.escape(end) + r"\n?", "", text, count=1, flags=re.S)


def hosts_block(cfg):
    lines = [ln.strip() for ln in cfg["common"]["peer_hosts"].strip().splitlines() if ln.strip()]
    return "\n".join([HOSTS_BEGIN] + lines + [HOSTS_END]) + "\n"


def userdata_hostname(S, cfg):
    m = re.search(r"^hostname:\s*(\S+)", S.read(P_USERDATA) or "", flags=re.M)
    return m.group(1) if m else cfg["common"]["userdata_hostname"]


def etc_hosts_managed(S):
    """user-data 的 manage_etc_hosts 是 true＝cloud-init 每次開機依範本重寫 /etc/hosts（這張卡就是）。"""
    ud = S.read(P_USERDATA)
    if ud is None:
        return True
    m = re.search(r"^manage_etc_hosts:\s*(\S+)", ud, flags=re.M)
    return bool(m) and m.group(1).strip("'\"").lower() in ("true", "yes", "1", "template")


class Step:
    def __init__(self, desc, fn, needs_hmi_restart=False):
        self.desc, self.fn, self.needs_hmi_restart = desc, fn, needs_hmi_restart


def _fail(msg):
    def fn():
        raise RuntimeError(msg)
    return fn


# ───────────────────────── 每一項 ─────────────────────────
# 每項回傳 (ini 的 key, 要的值, 現在的樣子, 要做的步驟)；沒有步驟＝符合。status 與 plan 用同一份判斷，不會互相矛盾。

def item_hosts(S, cfg, want):
    hosts_now = S.read(P_HOSTS) or ""
    tmpl_now = S.read(P_HOSTS_TMPL)
    ud = userdata_hostname(S, cfg)
    peers = want["hosts_peers"] == "yes"
    steps = []
    if tmpl_now is not None and etc_hosts_managed(S):
        tmpl_want = strip_block(tmpl_now, HOSTS_BEGIN, HOSTS_END) + ("\n" + hosts_block(cfg) if peers else "")
        hosts_want = render_hosts(tmpl_want, want["hostname"], ud)
        if tmpl_now != tmpl_want:
            steps.append(Step(f"改 cloud-init 範本 {P_HOSTS_TMPL}：peer 行{'加上' if peers else '拿掉'}"
                              "（開機時 /etc/hosts 依它重寫）", lambda: S.write(P_HOSTS_TMPL, tmpl_want)))
        tmpl_peers = HOSTS_BEGIN in tmpl_now
    else:   # cloud-init 不管 /etc/hosts 的卡：直接改
        base = strip_block(hosts_now, HOSTS_BEGIN, HOSTS_END)
        base = re.sub(r"^127\.0\.1\.1\s.*$", f"127.0.1.1 {want['hostname']} {want['hostname']}", base,
                      count=1, flags=re.M)
        hosts_want = base + ("\n" + hosts_block(cfg) if peers else "")
        tmpl_peers = None
    if hosts_now != hosts_want:
        steps.append(Step(f"重寫 {P_HOSTS}（127.0.1.1 {want['hostname']} {ud}{'＋peer 行' if peers else ''}；"
                          "和開機後 cloud-init 寫的一樣，先改好 sudo 才解析得到新名字）",
                          lambda: S.write(P_HOSTS, hosts_want)))
    shown = "yes" if HOSTS_BEGIN in hosts_now else "no"
    if "smartnav-a car-a" in hosts_now and HOSTS_BEGIN not in hosts_now:
        shown += "（set_board_name.sh 加的舊行，重開機會被 cloud-init 洗掉）"
    if tmpl_peers is not None and tmpl_peers != (HOSTS_BEGIN in hosts_now):
        shown += f"（範本 {'yes' if tmpl_peers else 'no'}：重開機後會變）"
    return "hosts_peers", want["hosts_peers"], shown, steps


def item_hostname(S, cfg, want):
    cur = (S.read(P_HOSTNAME) or "").strip()
    ovr = S.read(P_CLOUD_OVR)
    ud = userdata_hostname(S, cfg)
    need_ovr = want["hostname"] != ud
    steps = []
    if need_ovr and ovr != CLOUD_OVR_TEXT:
        steps.append(Step(f"寫 {P_CLOUD_OVR}（preserve_hostname: true；不然開機時 cloud-init 改回 {ud}）",
                          lambda: S.write(P_CLOUD_OVR, CLOUD_OVR_TEXT)))
    if not need_ovr and ovr is not None:
        steps.append(Step(f"刪 {P_CLOUD_OVR}（hostname 交回 cloud-init：{ud}）", lambda: S.remove(P_CLOUD_OVR)))
    if cur != want["hostname"]:
        steps.append(Step(f"hostnamectl set-hostname {want['hostname']}",
                          lambda: S.run(["hostnamectl", "set-hostname", want["hostname"]], check=True)))
    shown = cur + ("" if ovr is None else "（有 cloud-init 覆寫）" if ovr == CLOUD_OVR_TEXT else "（有舊版 cloud-init 覆寫）")
    return "hostname", want["hostname"], shown, steps


def _remove_fallback_step(S):
    return Step("拿掉備案 smartnav-eth0.service（停用、刪單元與 61-dualboard-eth0-unmanaged.conf、reload NM）",
                lambda: (S.run(["systemctl", "disable", "--now", "smartnav-eth0.service"]),
                         S.remove(P_FB_UNIT), S.remove(P_FB_CONF),
                         S.run(["systemctl", "daemon-reload"]), S.run(["systemctl", "reload", "NetworkManager"])))


def item_eth0(S, cfg, want):
    conn = S.nm_conn(NM_CONN)
    has_conf = S.exists(P_NM_CONF)
    fb = S.read(P_FB_UNIT)
    fb_ip = None
    if fb is not None:
        m = re.search(r"ip addr replace (\S+) dev eth0", fb)
        fb_ip = m.group(1) if m else "?"
    has_fb = fb is not None or S.exists(P_FB_CONF)
    shown = "、".join(x for x in (f"nm {conn['ip']}" if conn else "", "ignore-carrier 檔" if has_conf else "",
                                   f"fallback {fb_ip}" if fb is not None else "") if x) or "none"
    ip = want["eth0_ip"]
    method = cfg["common"].get("eth0_method", "nm")
    steps = []
    if ip == "none":
        if conn:
            def _del(uuid=conn.get("uuid", "")):
                S.backup(f"/etc/netplan/90-NM-{uuid}.yaml")   # NM 把連線存成 netplan 檔
                S.run(["nmcli", "con", "delete", NM_CONN], check=True)
            steps.append(Step(f"刪 NM 連線 {NM_CONN}（{conn['ip']}；只動 eth0，不碰 Wi-Fi）", _del))
        if has_conf:
            steps.append(Step(f"刪 {P_NM_CONF}、reload NetworkManager",
                              lambda: (S.remove(P_NM_CONF), S.run(["systemctl", "reload", "NetworkManager"]))))
        if has_fb:
            steps.append(_remove_fallback_step(S))
    elif method == "fallback":
        if fb_ip != ip or conn:
            role = {"10.10.10.1/24": "A", "10.10.10.2/24": "B"}.get(ip)
            if role is None:
                steps.append(Step(f"✗ eth0_method=fallback 只支援 eth0_static_fallback.sh 寫死的 10.10.10.1／.2（ini 是 {ip}）",
                                  _fail("fallback 不支援這個 IP")))
            else:
                steps.append(Step(f"跑 eth0_static_fallback.sh {role}",
                                  lambda: S.run([os.path.join(HERE, "eth0_static_fallback.sh"), role], check=True)))
    else:
        tmpl = read_template(T_NM_CONF)
        if has_fb:
            steps.append(_remove_fallback_step(S))
        if S.read(P_NM_CONF) != tmpl:
            steps.append(Step(f"裝 {P_NM_CONF}（ignore-carrier：沒插線也保有 IP）、reload NetworkManager",
                              lambda: (S.write(P_NM_CONF, tmpl), S.run(["systemctl", "reload", "NetworkManager"]))))
        if not conn or conn["ip"] != ip:
            def _add():
                if S.nm_conn(NM_CONN):
                    S.run(["nmcli", "con", "delete", NM_CONN], check=True)
                S.run(["nmcli", "con", "add", "type", "ethernet", "ifname", "eth0", "con-name", NM_CONN,
                       "ipv4.method", "manual", "ipv4.addresses", ip, "ipv4.never-default", "yes",
                       "ipv4.ignore-auto-dns", "yes", "ipv6.method", "disabled", "connection.autoconnect", "yes",
                       "connection.autoconnect-priority", "100"], check=True)
                S.run(["nmcli", "con", "up", NM_CONN])   # 沒插線可能失敗；ignore-carrier 生效的話位址仍在
            steps.append(Step(f"建 NM 連線 {NM_CONN}：eth0 {ip}（不設閘道、不設 DNS）", _add))
    return "eth0_ip", ip, shown, steps


def item_role(S, cfg, want):
    p = os.path.join(cfg["common"]["home"], ".smartnav", "dualboard_role")
    cur = (S.read(p) or "").strip() or "none"
    steps = []
    if cur != want["role"]:
        if want["role"] == "none":
            steps.append(Step(f"刪 {p}（env.sh 回到單板 DDS：domain 0、LOCALHOST）", lambda: S.remove(p),
                              needs_hmi_restart=True))
        else:
            steps.append(Step(f"寫 {p} ＝ {want['role']}（env.sh 改吃雙板 DDS：overlay.sh）",
                              lambda: S.write(p, want["role"] + "\n", owner=cfg["common"]["user"]),
                              needs_hmi_restart=True))
    return "role", want["role"], cur, steps


def item_hmi_dropin(S, cfg, want):
    text = S.read(P_HMI_DROPIN)
    tmpl = read_template(T_HMI_DROPIN)
    cur = "no" if text is None else "yes" if text == tmpl else "yes（內容與範本不同）"
    steps = []
    if want["hmi_dualboard_dropin"] == "yes" and text != tmpl:
        steps.append(Step(f"裝 {P_HMI_DROPIN}、daemon-reload",
                          lambda: (S.write(P_HMI_DROPIN, tmpl), S.run(["systemctl", "daemon-reload"])),
                          needs_hmi_restart=True))
    if want["hmi_dualboard_dropin"] == "no" and text is not None:
        steps.append(Step(f"刪 {P_HMI_DROPIN}、daemon-reload",
                          lambda: (S.remove(P_HMI_DROPIN), S.run(["systemctl", "daemon-reload"])),
                          needs_hmi_restart=True))
    return "hmi_dualboard_dropin", want["hmi_dualboard_dropin"], cur, steps


def item_hmi_service(S, cfg, want):
    cur = S.unit_enabled("smartnav-hmi.service")
    steps = []
    if cur != want["hmi_service"]:
        if want["hmi_service"] == "enabled":
            steps.append(Step("systemctl enable --now smartnav-hmi",
                              lambda: S.run(["systemctl", "enable", "--now", "smartnav-hmi.service"], check=True)))
        else:
            steps.append(Step("systemctl disable --now smartnav-hmi（★ 平板會連不到這塊板子的 HMI）",
                              lambda: S.run(["systemctl", "disable", "--now", "smartnav-hmi.service"], check=True)))
    return "hmi_service", want["hmi_service"], cur, steps


def _unit_item(key, unit):
    def item(S, cfg, want):
        p = f"/etc/systemd/system/{unit}"
        tmpl = read_template(UNIT_TEMPLATES[unit])
        text = S.read(p)
        en = S.unit_enabled(unit)
        cur = "absent" if text is None else en if text == tmpl else f"{en}（單元檔與範本不同）"
        steps = []
        if want[key] == "enabled" and (text != tmpl or en != "enabled"):
            steps.append(Step(f"裝 {p}、daemon-reload、enable --now {unit}",
                              lambda: (S.write(p, tmpl), S.run(["systemctl", "daemon-reload"]),
                                       S.run(["systemctl", "enable", "--now", unit], check=True))))
        if want[key] == "absent" and text is not None:
            steps.append(Step(f"停用並移除 {unit}",
                              lambda: (S.run(["systemctl", "disable", "--now", unit]), S.remove(p),
                                       S.run(["systemctl", "daemon-reload"]))))
        return key, want[key], cur, steps
    return item


def item_time_sync(S, cfg, want):
    w = want["time_sync"]
    chrony_conf, tsync_conf = S.read(P_CHRONY_CONF), S.read(P_TSYNC_CONF)
    wait = S.unit_enabled("systemd-time-wait-sync.service")
    chrony = S.pkg_installed("chrony")
    t_chrony, t_tsync = read_template(T_CHRONY), read_template(T_TSYNC)
    steps = []
    if w != "chrony_server" and chrony_conf is not None:
        steps.append(Step(f"刪 {P_CHRONY_CONF}、重啟 chrony",
                          lambda: (S.remove(P_CHRONY_CONF), S.run(["systemctl", "restart", "chrony"]))))
    if w != "timesyncd_from_a" and tsync_conf is not None:
        steps.append(Step(f"刪 {P_TSYNC_CONF}、重啟 systemd-timesyncd",
                          lambda: (S.remove(P_TSYNC_CONF), S.run(["systemctl", "restart", "systemd-timesyncd"]))))
    if w != "timesyncd_from_a" and wait == "enabled":
        steps.append(Step("disable systemd-time-wait-sync（只有 B 要等校時）",
                          lambda: S.run(["systemctl", "disable", "systemd-time-wait-sync.service"])))
    if w == "chrony_server":
        if not chrony:
            steps.append(Step("apt-get install -y chrony（要網路；會取代 systemd-timesyncd）",
                              lambda: S.run(["apt-get", "install", "-y", "chrony"], check=True)))
        if chrony_conf != t_chrony:
            steps.append(Step(f"裝 {P_CHRONY_CONF}（當 B 的校時伺服器）、重啟 chrony",
                              lambda: (S.write(P_CHRONY_CONF, t_chrony), S.run(["systemctl", "restart", "chrony"]))))
    if w == "timesyncd_from_a":
        if chrony:
            steps.append(Step("✗ 這塊板子裝了 chrony，timesyncd 不會跑：先接 Wi-Fi 執行 "
                              "sudo apt-get install -y systemd-timesyncd（會移除 chrony）再 apply",
                              _fail("B 上裝了 chrony")))
        if tsync_conf != t_tsync:
            steps.append(Step(f"裝 {P_TSYNC_CONF}（NTP＝A 10.10.10.1）、重啟 systemd-timesyncd",
                              lambda: (S.write(P_TSYNC_CONF, t_tsync),
                                       S.run(["systemctl", "restart", "systemd-timesyncd"]))))
        if wait != "enabled":
            steps.append(Step("enable systemd-time-wait-sync（感知服務等校時完成才開）",
                              lambda: S.run(["systemctl", "enable", "systemd-time-wait-sync.service"])))
    name = "chrony_server" if chrony_conf is not None else "timesyncd_from_a" if tsync_conf is not None else "default"
    notes = [x for x in ("chrony 已裝" if chrony else "", "time-wait-sync 開著" if wait == "enabled" else "",
                         "兩種設定都有" if chrony_conf is not None and tsync_conf is not None else "") if x]
    return "time_sync", w, name + (f"（{'、'.join(notes)}）" if notes else ""), steps


def item_bashrc(S, cfg, want):
    p = os.path.join(cfg["common"]["home"], ".bashrc")
    text = S.read(p) or ""
    has = BASHRC_BEGIN in text
    steps = []
    if want["bashrc_label"] == "yes" and not has:
        steps.append(Step(f"{p} 加板子標示區塊（提示字元 [A·8GB]／[B·4GB]）",
                          lambda: S.write(p, text + BASHRC_BLOCK, owner=cfg["common"]["user"])))
    if want["bashrc_label"] == "no" and has:
        steps.append(Step(f"{p} 拿掉板子標示區塊",
                          lambda: S.write(p, strip_block(text, BASHRC_BEGIN, BASHRC_END), owner=cfg["common"]["user"])))
    return "bashrc_label", want["bashrc_label"], "yes" if has else "no", steps


# 順序＝套用順序：/etc/hosts 要在 hostnamectl 之前改好（sudo 才解析得到新名字）
ITEMS = [item_hosts, item_hostname, item_eth0, item_role, item_hmi_dropin, item_hmi_service,
         _unit_item("perception_service", "smartnav-perception.service"),
         _unit_item("powerlog_service", "smartnav-b-powerlog.service"),
         item_time_sync, item_bashrc]


# ───────────────────────── 指令 ─────────────────────────

def load_cfg():
    cfg = configparser.ConfigParser(interpolation=None)
    cfg.optionxform = str
    with open(INI, encoding="utf-8") as f:
        cfg.read_file(f)
    missing = [p for p in PROFILES if p not in cfg]
    if missing:
        raise SystemExit(f"{INI} 少了 {missing}")
    return cfg


def evaluate(S, cfg, profile):
    return [it(S, cfg, cfg[profile]) for it in ITEMS]


def mem_ok(S, cfg, profile):
    m = cfg[profile]["mem_gb"]
    return m == "any" or int(m) == S.mem_gb()


def print_rows(rows, out):
    for key, want, cur, steps in rows:
        out(f"  {'✗' if steps else '✓'} {key:<21} 要 {want:<15} 現在 {cur}")


def cmd_status(S, cfg, out):
    results = {p: evaluate(S, cfg, p) for p in PROFILES}
    scores = {p: sum(bool(r[3]) for r in results[p]) for p in PROFILES}
    best = min(PROFILES, key=lambda p: (scores[p], PROFILES.index(p)))
    out(f"這塊板子：{S.mem_gb()} GB、hostname {(S.read(P_HOSTNAME) or '').strip()}")
    out(f"最接近 {best}（{cfg[best]['description']}）：不符 {scores[best]} 項"
        f"（{'、'.join(f'{p} 不符 {scores[p]} 項' for p in PROFILES if p != best)}）")
    if not mem_ok(S, cfg, best):
        out(f"  ⚠ 記憶體 {S.mem_gb()} GB，{best} 應是 {cfg[best]['mem_gb']} GB（SD 卡插錯板？）")
    print_rows(results[best], out)
    return best, scores[best]


def cmd_plan(S, cfg, profile, out):
    steps = [s for r in evaluate(S, cfg, profile) for s in r[3]]
    out(f"改成 {profile}（{cfg[profile]['description']}）：{len(steps)} 步")
    if not mem_ok(S, cfg, profile):
        out(f"  ⚠ 記憶體 {S.mem_gb()} GB，{profile} 應是 {cfg[profile]['mem_gb']} GB——apply 會拒絕（確定要改加 --force-mem）")
    for i, s in enumerate(steps, 1):
        out(f"  {i}. {s.desc}")
    if not steps:
        out("  （已經符合，不用改）")
    return steps


def cmd_apply(S, cfg, profile, out, stamp, force_mem=False, restart_hmi=False):
    if not mem_ok(S, cfg, profile) and not force_mem:
        out(f"✗ 拒絕：記憶體 {S.mem_gb()} GB，{profile} 應是 {cfg[profile]['mem_gb']} GB（確定要改加 --force-mem）")
        return 2
    S.backup_dir = os.path.join(S.root + LOG_DIR if S.sandbox else LOG_DIR, "backups", f"{stamp}_{profile}")
    hmi_restart, failed = False, 0
    for it in ITEMS:   # 一項一項重新評估再做：前一項的結果（例如 /etc/hosts 已改）會影響後一項
        for s in it(S, cfg, cfg[profile])[3]:
            out(f"▶ {s.desc}")
            try:
                s.fn()
                hmi_restart |= s.needs_hmi_restart
            except Exception as e:  # noqa: BLE001 —— 一項失敗不擋其他項，最後的檢查會標出來
                failed += 1
                out(f"  ✗ 失敗：{e}")
    out(f"備份：{S.backup_dir}" if os.path.isdir(S.backup_dir) else "（沒有檔案需要備份）")
    out("── 套用後 ──")
    rows = evaluate(S, cfg, profile)
    print_rows(rows, out)
    if hmi_restart and cfg[profile]["hmi_service"] == "enabled":
        if restart_hmi:
            out("▶ 重啟 HMI（--restart-hmi）")
            S.run(["systemctl", "restart", "smartnav-hmi.service"])
        else:
            out("★ HMI 的 DDS 環境變了：平板先「全部關閉」→ sudo systemctl restart smartnav-hmi（會停掉 HMI 開著的單元）")
    if profile in ("A", "B"):
        out("★ 兩塊都設好、插上網路線後：~/maprun/dualboard/check_dualboard.sh；開新終端機才看得到 [A·8GB]／[B·4GB]")
    out("★ 重開機後再跑一次 status（cloud-init、NetworkManager 開機時會重讀設定）")
    return 1 if (failed or any(r[3] for r in rows)) else 0


def main():
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)   # 接 | head 時安靜結束
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("status", "plan", "apply"))
    ap.add_argument("profile", nargs="?", choices=PROFILES)
    ap.add_argument("--root", help="沙盒根目錄（測試用）")
    ap.add_argument("--force-mem", action="store_true", help="記憶體和 profile 對不上也照改")
    ap.add_argument("--restart-hmi", action="store_true", help="HMI 的 DDS 環境變了就直接重啟 HMI")
    a = ap.parse_args()
    if a.cmd != "status" and not a.profile:
        ap.error(f"{a.cmd} 要指定 profile：{'|'.join(PROFILES)}")

    lines = []

    def out(msg):
        print(msg, flush=True)
        lines.append(msg)

    S = SandboxSystem(a.root, out) if a.root else RealSystem(out)
    cfg = load_cfg()
    if a.cmd == "status":
        cmd_status(S, cfg, out)
        return 0
    if a.cmd == "plan":
        cmd_plan(S, cfg, a.profile, out)
        return 0
    if not a.root and os.geteuid() != 0:
        print(f"要用 sudo：sudo python3 {os.path.abspath(__file__)} apply {a.profile}")
        return 1
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out(f"# board_profile.py apply {a.profile} —— {datetime.datetime.now():%Y-%m-%d %H:%M:%S}，"
        f"改之前 hostname {(S.read(P_HOSTNAME) or '').strip()}")
    rc = cmd_apply(S, cfg, a.profile, out, stamp, a.force_mem, a.restart_hmi)
    if not a.root:
        os.makedirs(LOG_DIR, exist_ok=True)
        lp = os.path.join(LOG_DIR, f"apply_{stamp}_{a.profile}.txt")
        with open(lp, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        pw = pwd.getpwnam(cfg["common"]["user"])
        for dp, _, fns in os.walk(LOG_DIR):   # sudo 寫出來的東西還給使用者
            os.chown(dp, pw.pw_uid, pw.pw_gid)
            for fn in fns:
                os.chown(os.path.join(dp, fn), pw.pw_uid, pw.pw_gid)
        print(f"紀錄：{lp}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
