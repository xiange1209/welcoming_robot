"""命令列入口：與 HMI 共用受管 launch metadata，而不是重建 shell kill 邏輯。"""

from __future__ import annotations

import argparse
import json

from .modules.system_control.supervisor import LaunchSupervisor


def main() -> int:
    parser = argparse.ArgumentParser(description="SmartNav 受管系統控制")
    parser.add_argument("action", choices=("start", "stop", "status"))
    parser.add_argument("unit")
    parser.add_argument("--variant")
    args = parser.parse_args()
    supervisor = LaunchSupervisor(print)
    if args.action == "status":
        state = supervisor.managed_state(args.unit, args.variant)
        print(json.dumps(state or {"running": False}, ensure_ascii=False))
        return 0 if state else 1
    ok, message = supervisor.run_unit_sync(args.unit, args.action, args.variant)
    print(message)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
