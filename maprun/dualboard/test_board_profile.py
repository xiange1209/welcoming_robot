#!/usr/bin/env python3
"""board_profile.py 的測試（沙盒，不動系統；真系統只跑 status／plan 唯讀）。
用法：python3 ~/maprun/dualboard/test_board_profile.py
  沙盒的起點是這塊板子真的檔（/etc/hostname、/etc/hosts、cloud-init 範本、~/.bashrc）；user-data 用假的
  （真的 user-data 可能有密碼，不複製）。每一種來回最後都要和起點一字不差。
  ★ 2026-10-23 另有一組用 10/23 改回 r680 前備份下來的 B 狀態（revert_r680_20261023/）重跑一次「apply single」，
    結果要和當天手動還原的一樣。
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "board_profile.py")
sys.path.insert(0, HERE)
import board_profile as bp  # noqa: E402

REVERT = os.path.join(HERE, "revert_r680_20261023")
HOME = "/home/user"
FAKE_USERDATA = "#cloud-config\nhostname: wheeltec-r680\nmanage_etc_hosts: true\n"
TRACKED = ["/etc/hostname", "/etc/hosts", bp.P_HOSTS_TMPL, HOME + "/.bashrc"]

passed = failed = 0


def check(name, cond, detail=""):
    global passed, failed
    if cond:
        passed += 1
        print(f"  ✓ {name}")
    else:
        failed += 1
        print(f"  ✗ {name}  {detail}")


def put(root, p, text):
    os.makedirs(os.path.dirname(root + p), exist_ok=True)
    with open(root + p, "w", encoding="utf-8") as f:
        f.write(text)


def get(root, p):
    try:
        with open(root + p, encoding="utf-8") as f:
            return f.read()
    except FileNotFoundError:
        return None


def real(p):
    with open(p, encoding="utf-8") as f:
        return f.read()


def sandbox(base, name, mem_gb, files=None, nm=None):
    root = os.path.join(base, name)
    os.makedirs(root)
    src = files or {p: real(p) for p in TRACKED}
    for p, text in src.items():
        put(root, p, text)
    if bp.P_HOSTS_TMPL not in src:
        put(root, bp.P_HOSTS_TMPL, real(bp.P_HOSTS_TMPL))
    put(root, bp.P_USERDATA, FAKE_USERDATA)
    st = {"mem_gb": mem_gb, "nm": nm or {},
          "units": {"smartnav-hmi.service": "enabled", "systemd-time-wait-sync.service": "disabled"},
          "pkgs": {"chrony": False}}
    with open(os.path.join(root, "state.json"), "w") as f:
        json.dump(st, f)
    return root


def state(root):
    with open(os.path.join(root, "state.json")) as f:
        return json.load(f)


def set_mem(root, gb):
    st = state(root)
    st["mem_gb"] = gb
    with open(os.path.join(root, "state.json"), "w") as f:
        json.dump(st, f)


def tool(root, *args):
    cmd = [sys.executable, TOOL, *args] + (["--root", root] if root else [])
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    return r.returncode, r.stdout + r.stderr


def snapshot(root):
    return {p: get(root, p) for p in TRACKED}


def status_best(root):
    _, out = tool(root, "status")
    m = re.search(r"最接近 (\S+?)（.*：不符 (\d+) 項", out)
    return (m.group(1), int(m.group(2))) if m else (None, None), out


def assert_clean_single(root, orig, label):
    check(f"{label}：四個檔與起點一字不差", snapshot(root) == orig,
          str([p for p in TRACKED if get(root, p) != orig[p]]))
    for p in (bp.P_CLOUD_OVR, bp.P_NM_CONF, bp.P_HMI_DROPIN, bp.P_CHRONY_CONF, bp.P_TSYNC_CONF,
              HOME + "/.smartnav/dualboard_role", "/etc/systemd/system/smartnav-perception.service",
              "/etc/systemd/system/smartnav-b-powerlog.service"):
        if get(root, p) is not None:
            check(f"{label}：{p} 應該不存在", False)
    st = state(root)
    check(f"{label}：沒有 NM 連線 dualboard-eth0", "dualboard-eth0" not in st["nm"])
    check(f"{label}：HMI 開機自啟", st["units"].get("smartnav-hmi.service") == "enabled")
    check(f"{label}：time-wait-sync 沒開", st["units"].get("systemd-time-wait-sync.service") != "enabled")
    best, _ = status_best(root)
    check(f"{label}：status＝single 不符 0 項", best == ("single", 0), str(best))


def main():
    base = tempfile.mkdtemp(prefix="board_profile_test_")
    try:
        print("① 渲染與區塊（對真的檔）")
        tmpl = real(bp.P_HOSTS_TMPL)
        check("render（r680, r680）＝10/22 原檔 /etc/hosts.bak_20261022",
              bp.render_hosts(tmpl, "wheeltec-r680", "wheeltec-r680") == real("/etc/hosts.bak_20261022"))
        if os.path.exists(os.path.join(REVERT, "hosts.before_revert")):
            check("render（smartnav-b, r680）＝B 開機後 cloud-init 寫的檔",
                  bp.render_hosts(tmpl, "smartnav-b", "wheeltec-r680") == real(os.path.join(REVERT, "hosts.before_revert")))
            check("strip_block（B 的 .bashrc）＝10/22 原檔 .bashrc.bak_20261022",
                  bp.strip_block(real(os.path.join(REVERT, "bashrc.before_revert")), bp.BASHRC_BEGIN, bp.BASHRC_END)
                  == real(HOME + "/.bashrc.bak_20261022"))
        for t in ("a\n", "a\n\n", "a", ""):
            check(f"區塊加了再拿掉回到原文（{t!r}）",
                  bp.strip_block(t + "\n" + bp.HOSTS_BEGIN + "\nx\n" + bp.HOSTS_END + "\n", bp.HOSTS_BEGIN, bp.HOSTS_END) == t)

        print("② 真系統（唯讀）")
        best, out = status_best(None)
        check("status＝single 不符 0 項", best == ("single", 0), out[-400:])
        rc, out = tool(None, "plan", "single")
        check("plan single＝0 步", "：0 步" in out, out[-300:])
        rc, out = tool(None, "plan", "A")
        check("plan A 在 4 GB 板上警告記憶體", "apply 會拒絕" in out)

        print("③ 8 GB：single → A → A → single")
        r = sandbox(base, "a8", 8)
        orig = snapshot(r)
        rc, out = tool(r, "apply", "A")
        check("apply A 成功（rc 0）", rc == 0, out[-600:])
        check("hostname＝smartnav-a", get(r, "/etc/hostname") == "smartnav-a\n")
        hosts = get(r, "/etc/hosts")
        check("/etc/hosts 有 peer 行與 127.0.1.1 smartnav-a wheeltec-r680",
              bp.HOSTS_BEGIN in hosts and "10.10.10.2   smartnav-b car-b" in hosts
              and "127.0.1.1 smartnav-a wheeltec-r680\n" in hosts)
        check("/etc/hosts＝開機後 cloud-init 依範本寫的（重開機不會掉）",
              bp.render_hosts(get(r, bp.P_HOSTS_TMPL), "smartnav-a", "wheeltec-r680") == hosts)
        check("cloud-init 覆寫是新版（只有 preserve_hostname）", get(r, bp.P_CLOUD_OVR) == bp.CLOUD_OVR_TEXT)
        check("NM 連線 10.10.10.1/24＋ignore-carrier 檔",
              state(r)["nm"].get("dualboard-eth0", {}).get("ip") == "10.10.10.1/24"
              and get(r, bp.P_NM_CONF) == real(bp.T_NM_CONF))
        check("role＝A、HMI drop-in＝範本", get(r, HOME + "/.smartnav/dualboard_role") == "A\n"
              and get(r, bp.P_HMI_DROPIN) == real(bp.T_HMI_DROPIN))
        check("chrony 已裝＋伺服器設定", state(r)["pkgs"].get("chrony") and get(r, bp.P_CHRONY_CONF) == real(bp.T_CHRONY))
        check(".bashrc 有板子標示", bp.BASHRC_BEGIN in get(r, HOME + "/.bashrc"))
        check("提示要重啟 HMI", "sudo systemctl restart smartnav-hmi" in out)
        check("有備份原檔", os.path.isfile(os.path.join(r + bp.LOG_DIR, "backups",
                                                   sorted(os.listdir(r + bp.LOG_DIR + "/backups"))[0], "etc__hosts")))
        rc, out = tool(r, "plan", "A")
        check("再 plan A＝0 步（可重複執行）", "：0 步" in out, out[-300:])
        best, _ = status_best(r)
        check("status＝A 不符 0 項", best == ("A", 0), str(best))
        rc, out = tool(r, "apply", "single")
        check("apply single 成功", rc == 0, out[-600:])
        assert_clean_single(r, orig, "A→single")

        print("④ 4 GB 板套 A 要被擋")
        r = sandbox(base, "a4", 4)
        orig = snapshot(r)
        rc, out = tool(r, "apply", "A")
        check("rc 2 且沒動任何檔", rc == 2 and snapshot(r) == orig and "拒絕" in out, out[-300:])

        print("⑤ 4 GB：single → B → B → single")
        r = sandbox(base, "b4", 4)
        orig = snapshot(r)
        rc, out = tool(r, "apply", "B")
        check("apply B 成功", rc == 0, out[-600:])
        st = state(r)
        check("HMI 停用、感知與電源紀錄服務啟用",
              st["units"].get("smartnav-hmi.service") == "disabled"
              and st["units"].get("smartnav-perception.service") == "enabled"
              and st["units"].get("smartnav-b-powerlog.service") == "enabled"
              and get(r, "/etc/systemd/system/smartnav-perception.service") == real(bp.UNIT_TEMPLATES["smartnav-perception.service"]))
        check("校時：timesyncd→A＋time-wait-sync", get(r, bp.P_TSYNC_CONF) == real(bp.T_TSYNC)
              and st["units"].get("systemd-time-wait-sync.service") == "enabled")
        check("NM 10.10.10.2/24、沒有 HMI drop-in", st["nm"]["dualboard-eth0"]["ip"] == "10.10.10.2/24"
              and get(r, bp.P_HMI_DROPIN) is None)
        check("B 不提示重啟 HMI（HMI 已停用）", "sudo systemctl restart smartnav-hmi" not in out)
        rc, out = tool(r, "plan", "B")
        check("再 plan B＝0 步", "：0 步" in out, out[-300:])
        rc, out = tool(r, "apply", "single")
        check("apply single 成功", rc == 0, out[-600:])
        assert_clean_single(r, orig, "B→single")

        print("⑥ 同一張卡 B → 換到 8 GB 板 → A → single")
        r = sandbox(base, "ba", 4)
        orig = snapshot(r)
        tool(r, "apply", "B")
        set_mem(r, 8)
        rc, out = tool(r, "apply", "A")
        check("B→A 成功", rc == 0, out[-600:])
        best, _ = status_best(r)
        st = state(r)
        check("status＝A 不符 0 項、感知服務移除、timesyncd 設定移除",
              best == ("A", 0) and get(r, "/etc/systemd/system/smartnav-perception.service") is None
              and get(r, bp.P_TSYNC_CONF) is None and st["units"]["smartnav-hmi.service"] == "enabled", str(best))
        rc, out = tool(r, "apply", "single")
        assert_clean_single(r, orig, "B→A→single")
        set_mem(r, 4)
        st = state(r)
        st["pkgs"]["chrony"] = True
        with open(os.path.join(r, "state.json"), "w") as f:
            json.dump(st, f)
        rc, out = tool(r, "apply", "B")
        check("裝過 chrony 的卡套 B：rc 1 並說明要先裝回 timesyncd", rc == 1 and "systemd-timesyncd（會移除 chrony）" in out)

        print("⑦ 重現 10/23 改回 r680 之前的 B 狀態（set_board_name.sh＋nm_eth0_setup.sh 做的）→ apply single")
        if os.path.isdir(REVERT):
            files = {"/etc/hostname": real(os.path.join(REVERT, "hostname.before_revert")),
                     "/etc/hosts": real(os.path.join(REVERT, "hosts.before_revert")),
                     bp.P_HOSTS_TMPL: real(bp.P_HOSTS_TMPL),
                     HOME + "/.bashrc": real(os.path.join(REVERT, "bashrc.before_revert")),
                     bp.P_CLOUD_OVR: real(os.path.join(REVERT, "99-smartnav-hostname.cfg")),
                     bp.P_NM_CONF: real(os.path.join(REVERT, "60-dualboard-eth0.conf"))}
            r = sandbox(base, "legacy", 4, files=files,
                        nm={"dualboard-eth0": {"ip": "10.10.10.2/24", "uuid": "d5396160-900d-44dc-8770-43d109ed01fe"}})
            _, out = tool(r, "status")
            check("status 看得出舊版覆寫", "舊版 cloud-init 覆寫" in out, out[-500:])
            rc, out = tool(r, "apply", "single")
            check("apply single 成功", rc == 0, out[-600:])
            orig = {"/etc/hostname": "wheeltec-r680\n", "/etc/hosts": real("/etc/hosts.bak_20261022"),
                    bp.P_HOSTS_TMPL: real(bp.P_HOSTS_TMPL), HOME + "/.bashrc": real(HOME + "/.bashrc.bak_20261022")}
            assert_clean_single(r, orig, "舊 B→single（＝今天手動還原的結果）")
        else:
            print("  （沒有 revert_r680_20261023/，略過）")
    finally:
        shutil.rmtree(base, ignore_errors=True)
    print(f"\n{passed}/{passed + failed} 通過")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
