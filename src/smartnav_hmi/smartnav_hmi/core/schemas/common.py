"""HTTP 請求的資料格式。

只描述「前端會送什麼」，不含任何業務邏輯——驗證交給 pydantic，
預設值就是「前端沒帶這個欄位時該怎麼辦」的唯一定義。
"""

from typing import Dict, List

from pydantic import BaseModel


class LoginRequest(BaseModel):
    """POST /api/login"""

    username: str = ""
    password: str = ""


class RegisterPhotoRequest(BaseModel):
    """POST /api/users/register-photo

    照片用 base64 傳而不是 multipart，因為 multipart 需要額外安裝
    python-multipart，而這個專案刻意不在 Pi 上多裝 pip 套件。
    photos 接受純 base64 或 "data:image/jpeg;base64,..." 兩種寫法。
    """

    user_name: str
    user_type: int = 0
    description: str = ""
    photos: List[str] = []


class RegisterRequest(BaseModel):
    """POST /api/users/register"""

    user_name: str
    user_type: int = 0
    description: str = ""
    num_samples: int = 10


class UpdateUserRequest(BaseModel):
    """PUT /api/users/{uuid}"""

    user_name: str = ""
    user_type: int = 0
    description: str = ""


class SayRequest(BaseModel):
    """POST /api/say (丟給 LLM) 與 /api/speak (直接播報)"""

    text: str


class TeleopRequest(BaseModel):
    # 正值前進、負值後退 (m/s)。上限由 hmi_server 端再夾一次，
    # 不信任前端傳來的數值。
    linear: float = 0.0
    # 正值左轉 (rad/s)
    angular: float = 0.0


class RearMaskRequest(BaseModel):
    enabled: bool = True
    # 以車尾正後方為中心、往兩側各遮這麼多度
    half_angle_deg: float = 35.0


class SwitchMapRequest(BaseModel):
    """POST /api/maps/switch"""

    map_id: str


class CreateMapRequest(BaseModel):
    """POST /api/maps/create"""

    map_name: str


class CreateWaypointRequest(BaseModel):
    """POST /api/waypoints

    x/y/yaw 皆為 map frame 座標。use_given_pose=false 時忽略座標，
    改用機器人當前位置建點。
    """

    waypoint_name: str
    use_given_pose: bool = True
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class SetPoseRequest(BaseModel):
    """POST /api/localize/here

    把定位設到一個已知位置
    """

    waypoint_id: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    align: bool = True  # 設完之後是否呼叫 /align_pose 做掃描對齊


class NavigateRequest(BaseModel):
    """POST /api/navigate

    waypoint_id 有值時導航到既有地點；否則導航到 x/y/yaw 指定的座標。
    """

    waypoint_id: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    target_name: str = ""


class RecordPathRequest(BaseModel):
    """POST /api/paths/record —— 教導路徑的錄製控制"""

    action: str = "start"  # start / stop / cancel
    name: str = ""  # stop 時必填


class FollowPathRequest(BaseModel):
    """POST /api/paths/follow —— 重播教導路徑"""

    path_id: str = ""
    name: str = ""  # 只用於作業標籤的顯示
    reverse: bool = False  # 反向走完整條路徑（原路折返回起點）
    speed_scale: float = 0.0  # 0 或負值 = 用節點的預設速度


class PlanPathRequest(BaseModel):
    """POST /api/paths/plan —— 把地圖上點選的位置規劃成教導路徑"""

    name: str = ""
    start_from_robot: bool = True
    points: List[Dict[str, float]] = []  # [{"x":.., "y":.., "yaw":..}, ...]
