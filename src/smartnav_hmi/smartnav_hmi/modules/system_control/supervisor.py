"""受管 ROS launch 的程序群組與非同步 operation。

這層刻意不依賴 rclpy，讓 HMI、CLI 與測試都能使用同一套安全規則。
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path
from typing import Callable, Optional


SIGINT_GRACE_SEC = 20.0
SIGTERM_GRACE_SEC = 5.0
SESSION_READY_SEC = 0.5


class LaunchSupervisor:
    """只管理由統一 system_control.launch.py 建立的 launch session。"""

    def __init__(self, logger: Callable[[str], None] | None = None):
        self._log = logger or (lambda _msg: None)
        root = Path.home() / ".smartnav"
        self._state_dir = root / "system_control"
        self._log_dir = root / "logs" / "system_control"
        self._state_dir.mkdir(parents=True, exist_ok=True)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._operations: dict[str, dict] = {}
        self._unit_ops: dict[str, str] = {}

    # ---- public operation API -------------------------------------------------
    def request_unit(self, unit: str, action: str, variant: str | None = None) -> tuple[bool, dict]:
        """排入單元操作；呼叫者不需等待 SIGINT 的 grace period。"""
        with self._lock:
            if any(
                op.get("scope") == "scenario" and op.get("status") in ("queued", "running")
                for op in self._operations.values()
            ):
                return False, {"message": "情境套用中，暫時不能手動啟停"}
            active_id = self._unit_ops.get(unit)
            if active_id and self._operations.get(active_id, {}).get("status") in ("queued", "running"):
                return False, {"message": f"{unit} 正在處理中", "operation_id": active_id}
            op = self._new_operation("unit", unit, action, variant)
            self._unit_ops[unit] = op["operation_id"]
            thread = threading.Thread(target=self._run_unit_operation, args=(op["operation_id"],), daemon=True)
            thread.start()
            return True, op.copy()

    def request_scenario(self, key: str, steps: list[tuple[str, str, str | None]]) -> tuple[bool, dict]:
        """情境同樣是背景 operation；步驟在 worker 內同步收斂。"""
        with self._lock:
            if any(o.get("scope") == "scenario" and o.get("status") in ("queued", "running") for o in self._operations.values()):
                return False, {"message": "另一個情境正在套用"}
            if any(o.get("scope") == "unit" and o.get("status") in ("queued", "running") for o in self._operations.values()):
                return False, {"message": "有單元正在啟停，請完成後再套用情境"}
            op = self._new_operation("scenario", key, "apply", None)
            thread = threading.Thread(target=self._run_scenario, args=(op["operation_id"], steps), daemon=True)
            thread.start()
            return True, op.copy()

    def operation(self, operation_id: str) -> Optional[dict]:
        with self._lock:
            op = self._operations.get(operation_id)
            return dict(op) if op else None

    def managed_state(self, unit: str, variant: str | None = None) -> Optional[dict]:
        """供 CLI/診斷用；只回傳已通過完整身分驗證的 state。"""
        data = self._read_state(unit) or self._find_adoptable(unit, variant)
        if not data:
            return None
        try:
            self._validate(data, unit, variant)
        except (OSError, PermissionError, ProcessLookupError, RuntimeError):
            self._clear_state(unit)
            return None
        return data

    def transition(self, unit: str) -> Optional[dict]:
        with self._lock:
            op_id = self._unit_ops.get(unit)
            op = self._operations.get(op_id or "")
            if not op or op["status"] not in ("queued", "running"):
                return None
            return {
                "operation_id": op["operation_id"],
                "action": op["action"],
                "phase": op["phase"],
                "started_at": op["started_at"],
                "message": op["message"],
            }

    def run_unit_sync(self, unit: str, action: str, variant: str | None = None) -> tuple[bool, str]:
        """供 scenario worker 使用；不要直接從 HTTP request 呼叫。"""
        op = self._new_operation("unit", unit, action, variant)
        with self._lock:
            self._unit_ops[unit] = op["operation_id"]
        self._run_unit_operation(op["operation_id"])
        done = self.operation(op["operation_id"]) or {}
        return done.get("status") == "succeeded", done.get("message", "操作失敗")

    # ---- worker implementation ------------------------------------------------
    def _new_operation(self, scope: str, target: str, action: str, variant: str | None) -> dict:
        op = {
            "operation_id": uuid.uuid4().hex,
            "scope": scope,
            "target": target,
            "action": action,
            "variant": variant,
            "status": "queued",
            "phase": "queued",
            "message": "已排入處理",
            "started_at": time.time(),
            "steps": [],
        }
        self._operations[op["operation_id"]] = op
        return op

    def _set(self, op_id: str, **fields) -> None:
        with self._lock:
            op = self._operations[op_id]
            op.update(fields)
            if fields.get("status") in ("succeeded", "failed"):
                op["finished_at"] = time.time()

    def _run_unit_operation(self, op_id: str) -> None:
        with self._lock:
            op = self._operations[op_id]
            unit, action, variant = op["target"], op["action"], op.get("variant")
        self._set(op_id, status="running", phase="launching" if action == "start" else "sigint")
        try:
            if action == "start":
                message = self._start(unit, variant, op_id)
            else:
                message = self._stop(unit, op_id)
        except Exception as exc:  # noqa: BLE001
            self._log(f"system-control {unit}/{action} failed: {exc}")
            self._set(op_id, status="failed", phase="failed", message=f"操作失敗：{exc}")
        else:
            self._set(op_id, status="succeeded", phase="ready" if action == "start" else "stopped", message=message)
        finally:
            with self._lock:
                if self._unit_ops.get(unit) == op_id:
                    self._unit_ops.pop(unit, None)

    def _run_scenario(self, op_id: str, steps: list[tuple[str, str, str | None]]) -> None:
        self._set(op_id, status="running", phase="running", message="情境套用中")
        outcomes = []
        ok_all = True
        for unit, action, variant in steps:
            ok, msg = self.run_unit_sync(unit, action, variant)
            outcomes.append({"unit": unit, "action": action, "variant": variant, "success": ok, "message": msg})
            with self._lock:
                self._operations[op_id]["steps"] = outcomes
            ok_all = ok_all and ok
        self._set(
            op_id,
            status="succeeded" if ok_all else "failed",
            phase="completed" if ok_all else "failed",
            message="情境已套用" if ok_all else "情境部分步驟失敗",
        )

    # ---- process identity ------------------------------------------------------
    @staticmethod
    def _boot_id() -> str:
        return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()

    @staticmethod
    def _starttime(pid: int) -> str:
        # comm 在括號中且可含空白；從最後一個 ')' 後再數 Linux stat 欄位。
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        fields = raw.rsplit(")", 1)[1].split()
        return fields[19]  # field 22, fields[0] is stat field 3

    @staticmethod
    def _argv(pid: int) -> list[str]:
        return [x.decode("utf-8", "replace") for x in Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0") if x]

    @staticmethod
    def _namespace(pid: int) -> str:
        return os.readlink(f"/proc/{pid}/ns/pid")

    def _identity(self, pid: int) -> dict:
        return {
            "pid": pid,
            "pgid": os.getpgid(pid),
            "sid": os.getsid(pid),
            "uid": os.stat(f"/proc/{pid}").st_uid,
            "starttime": self._starttime(pid),
            "boot_id": self._boot_id(),
            "pid_namespace": self._namespace(pid),
            "argv": self._argv(pid),
        }

    @staticmethod
    def _matches_command(argv: list[str], unit: str, variant: str | None) -> bool:
        required = ["launch", "smartnav_hmi", "system_control.launch.py", f"unit:={unit}"]
        if variant:
            required.append(f"variant:={variant}")
        return all(token in argv for token in required)

    def _validate(self, data: dict, unit: str, variant: str | None) -> dict:
        pid = int(data["pid"])
        identity = self._identity(pid)
        mine_ns = self._namespace(os.getpid())
        if identity["uid"] != os.geteuid():
            raise PermissionError("launch 與 HMI 不是同一 Linux 使用者")
        if identity["pid_namespace"] != mine_ns:
            raise PermissionError("launch 與 HMI 不在相同 PID namespace")
        if identity["pgid"] != pid or identity["sid"] != pid:
            raise RuntimeError("目標不是獨立的 launch process-group leader")
        if identity["pgid"] == os.getpgrp():
            raise RuntimeError("拒絕對 HMI 自身 process group 操作")
        if not self._matches_command(identity["argv"], unit, variant):
            raise RuntimeError("launch argv 與受管單元不符")
        for field in ("starttime", "boot_id", "pid_namespace", "uid"):
            if str(data.get(field)) != str(identity[field]):
                raise RuntimeError(f"launch 身分驗證失敗：{field}")
        return identity

    def _state_path(self, unit: str) -> Path:
        return self._state_dir / f"{unit}.json"

    def _read_state(self, unit: str) -> Optional[dict]:
        try:
            return json.loads(self._state_path(unit).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None

    def _write_state(self, unit: str, data: dict) -> None:
        path = self._state_path(unit)
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
        os.replace(temp, path)

    def _clear_state(self, unit: str) -> None:
        try:
            self._state_path(unit).unlink()
        except FileNotFoundError:
            pass

    def _find_adoptable(self, unit: str, variant: str | None) -> Optional[dict]:
        # /proc 掃描只接受完整 launch argv 與 group leader，絕不按節點名稱猜測。
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                pid = int(entry.name)
                identity = self._identity(pid)
                if not self._matches_command(identity["argv"], unit, variant):
                    continue
                if identity["pid"] != identity["pgid"] or identity["pid"] != identity["sid"]:
                    continue
                data = {k: identity[k] for k in ("pid", "pgid", "sid", "uid", "starttime", "boot_id", "pid_namespace", "argv")}
                self._validate(data, unit, variant)
                return data
            except (OSError, PermissionError, ProcessLookupError, RuntimeError):
                continue
        return None

    # ---- launch / stop ---------------------------------------------------------
    def _environment(self) -> dict[str, str]:
        env = os.environ.copy()
        env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        env.setdefault("ROS_DOMAIN_ID", "0")
        env.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
        dds = Path.home() / "cyclonedds.xml"
        if dds.is_file():
            env.setdefault("CYCLONEDDS_URI", f"file://{dds}")
        else:
            env.pop("CYCLONEDDS_URI", None)
        return env

    def _start(self, unit: str, variant: str | None, op_id: str) -> str:
        existing = self._read_state(unit)
        if existing:
            try:
                self._validate(existing, unit, variant)
            except (OSError, PermissionError, ProcessLookupError, RuntimeError):
                self._clear_state(unit)
                existing = None
        existing = existing or self._find_adoptable(unit, variant)
        if existing:
            self._validate(existing, unit, variant)
            raise RuntimeError("單元已由受管 launch 執行中")
        command = ["ros2", "launch", "smartnav_hmi", "system_control.launch.py", f"unit:={unit}"]
        if variant:
            command.append(f"variant:={variant}")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = self._log_dir / f"{unit}_{stamp}.log"
        with log_path.open("ab", buffering=0) as log:
            proc = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=self._environment(),
            )
        deadline = time.monotonic() + SESSION_READY_SEC
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                tail = log_path.read_text(encoding="utf-8", errors="replace")[-800:].strip()
                raise RuntimeError(f"launch 提前退出 ({proc.returncode})" + (f"：{tail}" if tail else ""))
            try:
                identity = self._identity(proc.pid)
                if identity["pgid"] == proc.pid and identity["sid"] == proc.pid:
                    data = {k: identity[k] for k in ("pid", "pgid", "sid", "uid", "starttime", "boot_id", "pid_namespace", "argv")}
                    self._validate(data, unit, variant)
                    self._write_state(unit, data)
                    return f"已啟動 {unit}"
            except (OSError, PermissionError, ProcessLookupError, RuntimeError) as exc:
                last_error = exc
            time.sleep(0.02)
        proc.terminate()  # 只殺 PID；尚未確認 group 前絕不 killpg。
        raise RuntimeError(f"launch 未能建立安全 session：{last_error or '逾時'}")

    def _stop(self, unit: str, op_id: str) -> str:
        with self._lock:
            variant = self._operations[op_id].get("variant")
        data = self._read_state(unit) or self._find_adoptable(unit, variant)
        if not data:
            raise RuntimeError(f"{unit} 沒有可安全接管的統一 launch")
        identity = self._validate(data, unit, variant)
        self._write_state(unit, data)
        for phase, sig, grace in (("sigint", signal.SIGINT, SIGINT_GRACE_SEC), ("sigterm", signal.SIGTERM, SIGTERM_GRACE_SEC), ("sigkill", signal.SIGKILL, 0.0)):
            self._set(op_id, phase=phase, message={"sigint": "正在優雅關閉中", "sigterm": "仍在關閉，正在終止殘留程序", "sigkill": "正在強制結束殘留程序"}[phase])
            # 每次訊號前重新驗證，不讓 PID/namespace 變化變成誤殺。
            identity = self._validate(data, unit, variant)
            os.killpg(identity["pgid"], sig)
            until = time.monotonic() + grace
            while time.monotonic() < until:
                if not self._alive(data, unit, variant):
                    self._clear_state(unit)
                    return f"已停止 {unit}"
                time.sleep(0.1)
            if not self._alive(data, unit, variant):
                self._clear_state(unit)
                return f"已停止 {unit}"
        for _ in range(20):
            if not self._alive(data, unit, variant):
                self._clear_state(unit)
                return f"已強制停止 {unit}"
            time.sleep(0.1)
        raise RuntimeError(f"{unit} 在 SIGKILL 後仍存活")

    def _alive(self, data: dict, unit: str, variant: str | None) -> bool:
        try:
            self._validate(data, unit, variant)
            return True
        except (OSError, PermissionError, ProcessLookupError, RuntimeError):
            return False
