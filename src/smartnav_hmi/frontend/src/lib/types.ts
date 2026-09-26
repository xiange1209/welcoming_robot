/**
 * 後端契約。欄位名稱與 hmi_server_node.py 的 HmiState.snapshot() 一致。
 *
 * 全部標成 optional 不是偷懶——推播是「有變才帶」的增量快照
 * （messages 在對話沒變時整個欄位不會出現），而節點沒啟動時
 * map_meta / robot_pose 本來就是 null。把它當成必填會在展示現場
 * 炸在最不該炸的地方。
 */

export type UserTypeName = "GUEST" | "VIP" | "ADMIN" | "BLACKLIST";

export interface Identity {
  user_name?: string;
  user_type?: UserTypeName | string;
  recognized?: boolean;
  similarity?: number;
  description?: string;
  updated_at?: number;
  /** [x1, y1, x2, y2]。無人時 user_auth_node 明確寫死 [0,0,0,0] */
  bbox?: number[];
}

export interface ChatStats {
  total?: number;
  think?: number;
  generate?: number;
  cps?: number;
  chars?: number;
}

export interface ChatMessage {
  role: "user" | "robot";
  text: string;
  ts?: number;
  stats?: ChatStats;
  /** 串流中的暫時訊息，前端自己塞的，不會從後端來 */
  ghost?: boolean;
}

export interface SystemStats {
  camera_fps?: number;
  voltage?: number;
  charging?: boolean;
  charge_current?: number;
  speed?: number;
  cpu_temp?: number;
  cpu_load?: number;
  map_id?: string;
  llm_model?: string;
}

export interface MapMeta {
  version: number;
  width: number;
  height: number;
  resolution: number;
  origin_x: number;
  origin_y: number;
}

export interface RobotPose {
  x: number;
  y: number;
  yaw: number;
}

export interface NavPath {
  /** taught = 純追蹤（主力）；其餘為 nav2 + MPPI（備援） */
  source?: "taught" | "nav2" | string;
  points?: [number, number][];
}

export type JobStatus =
  | "pending"
  | "running"
  | "cancelling"
  | "succeeded"
  | "failed"
  | "cancelled";

export interface Job {
  job_id: string;
  label: string;
  status: JobStatus | string;
  message?: string;
}

export interface Registration {
  status: "running" | "succeeded" | "failed" | string;
  user_name?: string;
  message?: string;
  collected?: number;
  num_samples?: number;
  /** 伺服器時鐘。要取 deadline - started_at 的差值，不能直接拿來跟本地時間比 */
  started_at?: number;
  deadline?: number;
  finished_at?: number;
}

/** WebSocket /ws 推播的一則快照 */
export interface Snapshot {
  version: number;
  /** 伺服器送出當下的時間，用來換算 identity 有多舊 */
  now: number;
  identity?: Identity;
  messages_version?: number;
  messages?: ChatMessage[];
  partial_text?: string;
  llm_streaming?: string;
  system?: SystemStats;
  map_meta?: MapMeta | null;
  robot_pose?: RobotPose | null;
  nav_path?: NavPath | null;
  jobs?: Job[];
  registration?: Registration | null;
}

/** 後端一律用 success/message 表達成敗，HTTP 狀態碼只用來分類錯誤來源 */
export interface ApiResult {
  success?: boolean;
  message?: string;
  [key: string]: unknown;
}

export interface Waypoint {
  waypoint_id: string;
  waypoint_name: string;
  x: number;
  y: number;
  yaw?: number;
}

export interface MapEntry {
  map_id: string;
  map_name: string;
}

export interface MapStatus extends ApiResult {
  mode?: "mapping" | "localization" | "unknown" | string;
  current_map?: string;
  map_meta?: MapMeta;
  mapping_job?: { label?: string } | null;
  last_failed_job?: { message?: string } | null;
}

export interface TaughtPath {
  path_id: string;
  name: string;
  length_m: number;
  num_points: number;
  num_cusps: number;
  source?: "plan" | "drive" | string;
}

export interface UserRecord {
  user_uuid: string;
  user_name: string;
  user_type: number;
  user_type_name: UserTypeName | string;
  description?: string;
  num_samples: number;
}

export interface SysVariant {
  key: string;
  label: string;
  /** 會讓車子自己跑的選項，按下前要確認 */
  warn?: string;
}

export interface SysTransition {
  operation_id: string;
  action: "start" | "stop" | string;
  phase: "launching" | "sigint" | "sigterm" | "sigkill" | string;
  started_at: number;
  message?: string;
}

export interface SysUnit {
  key: string;
  label: string;
  hint?: string;
  running?: boolean;
  /** 部分節點在跑，狀態不完整——卡在一半時最需要能把它收乾淨 */
  partial?: boolean;
  /** 前置條件不足的說明。有值代表現在按下去一定失敗 */
  blocked?: string;
  variants?: SysVariant[];
  transition?: SysTransition | null;
}

/** 測試情境：一鍵把該開的開、該關的關（後端 SystemControlManager.SCENARIOS） */
export interface Scenario {
  key: string;
  label: string;
  why: string;
  note?: string;
  /** 會啟動的單元（後端已換成看得懂的名稱） */
  start: string[];
  /** 會先停掉的單元 */
  stop: string[];
}

/** 一鍵驗證項目（後端 SystemControlManager.VERIFY_SCRIPTS） */
export interface VerifyItem {
  key: string;
  label: string;
  why: string;
  note?: string;
  /** false＝要讀鍵盤輸入（捲尺、量角器讀數），網頁跑不了，只能給指令 */
  runnable: boolean;
  script: string;
}

export interface HealthNode {
  name: string;
  ok: boolean;
  state?: string;
  group?: string;
  device?: "pi" | "edge" | "down" | "unknown" | string;
}

export interface HealthResult extends ApiResult {
  host?: string;
  camera?: boolean;
  map?: boolean;
  nodes?: HealthNode[];
  services?: Record<string, boolean>;
  actions?: Record<string, boolean>;
}

export interface HardwareItem {
  key: string;
  label: string;
  present: boolean;
  detail?: string;
  hint?: string;
}

export interface StatsResult extends ApiResult {
  available?: boolean;
  reason?: string;
  today?: { count?: number; by_type?: Record<string, number> };
  unique_people?: number;
  total?: number;
  by_hour?: number[];
  by_day?: { date: string; count: number }[];
  recent?: { name: string; type: string; type_label?: string; time: string; confidence: number }[];
}
