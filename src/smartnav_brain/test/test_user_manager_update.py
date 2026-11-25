"""update_user 要改到「指定的那個人」（2026-11-24：以前迴圈變數蓋掉參數，會改到最後一位註冊的人）。"""
import json

from smartnav_brain.user_manager import UserManager
from smartnav_brain.user_types import UserType


def _mgr(tmp_path):
    reg = {
        "uuid-a": {"user_name": "甲", "user_type": "GUEST", "description": "", "created_at": "1", "samples": []},
        "uuid-b": {"user_name": "乙", "user_type": "GUEST", "description": "", "created_at": "2", "samples": []},
        "uuid-c": {"user_name": "丙", "user_type": "GUEST", "description": "", "created_at": "3", "samples": []},
    }
    (tmp_path / "user_registry.json").write_text(json.dumps(reg, ensure_ascii=False), encoding="utf-8")
    return UserManager(tmp_path)


def test_update_changes_only_the_target(tmp_path):
    m = _mgr(tmp_path)
    vip = [t for t in UserType if t.name == "VIP"][0]
    assert m.update_user("uuid-a", user_name="甲", user_type=vip, description="新描述")
    assert m.user_registry["uuid-a"] == {**m.user_registry["uuid-a"], "user_name": "甲", "user_type": "VIP", "description": "新描述"}
    for other in ("uuid-b", "uuid-c"):           # 以前最後一位（丙）會被改成「甲／VIP」
        assert m.user_registry[other]["user_type"] == "GUEST"
        assert m.user_registry[other]["description"] == ""
    assert m.user_registry["uuid-c"]["user_name"] == "丙"
    saved = json.loads((tmp_path / "user_registry.json").read_text(encoding="utf-8"))
    assert saved["uuid-a"]["user_type"] == "VIP" and saved["uuid-c"]["user_name"] == "丙"


def test_rename_to_existing_name_is_rejected(tmp_path):
    m = _mgr(tmp_path)
    assert not m.update_user("uuid-a", user_name="乙")
    assert m.user_registry["uuid-a"]["user_name"] == "甲"


def test_rename_and_unknown_uuid(tmp_path):
    m = _mgr(tmp_path)
    assert m.update_user("uuid-b", user_name="丁")
    assert m.user_registry["uuid-b"]["user_name"] == "丁" and m.user_registry["uuid-c"]["user_name"] == "丙"
    assert not m.update_user("uuid-x", user_name="戊")
