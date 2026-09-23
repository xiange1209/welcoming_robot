import { authHeaders, setAdmin } from "./session";
import { toast } from "./toast";
import type { ApiResult } from "./types";

/**
 * 統一的 API 呼叫。
 *
 * 後端一律用 success/message 兩個欄位表達成敗，HTTP 狀態碼只用來分類
 * 錯誤來源。但 FastAPI 自己產生的錯誤不長那樣：
 *     422 驗證失敗   -> {"detail": [{...}]}   沒有 message
 *     500 未捕捉例外 -> {"detail": "..."} 或空 沒有 message
 * 少了下面那段轉譯，這些情況畫面上不會有任何反應，使用者只會覺得
 * 「按了沒事發生」——這是整站最大的靜默失敗來源。
 */
export async function api<T extends ApiResult = ApiResult>(
  path: string,
  options: RequestInit = {},
): Promise<T> {
  try {
    const res = await fetch(path, {
      ...options,
      headers: authHeaders({
        "Content-Type": "application/json",
        ...(options.headers as Record<string, string>),
      }),
    });
    const data = (await res.json().catch(() => ({}))) as T;

    if (res.status === 401) {
      // 權杖過期或被撤銷：收起管理分頁，別讓畫面停在一個會一直失敗的地方
      toast.error("管理者登入已失效，請重新登入");
      setAdmin(false);
      return { success: false, message: "需要管理者登入" } as T;
    }

    if (!res.ok) {
      let m = data.message;
      if (!m && data.detail) {
        const detail = data.detail as unknown;
        if (typeof detail === "string") {
          m = detail;
        } else if (
          Array.isArray(detail) &&
          detail[0] &&
          (detail[0] as { msg?: string }).msg
        ) {
          m = "資料格式錯誤：" + (detail[0] as { msg: string }).msg;
        } else {
          m = "資料格式錯誤";
        }
      }
      toast.error(m || `機器人回報錯誤 (HTTP ${res.status})`);
      if (!data.message) data.message = m || `HTTP ${res.status}`;
      if (data.success === undefined) data.success = false;
    }
    return data;
  } catch (e) {
    toast.error("無法連線到機器人：" + (e as Error).message);
    return { success: false, message: String(e) } as T;
  }
}

export const post = <T extends ApiResult = ApiResult>(
  path: string,
  body?: unknown,
) => api<T>(path, { method: "POST", body: JSON.stringify(body ?? {}) });

export const put = <T extends ApiResult = ApiResult>(
  path: string,
  body?: unknown,
) => api<T>(path, { method: "PUT", body: JSON.stringify(body ?? {}) });

export const del = <T extends ApiResult = ApiResult>(path: string) =>
  api<T>(path, { method: "DELETE" });

/** 送出並把結果用 toast 報出來。清單類操作到處都要這三行，抽出來省得漏掉錯誤提示 */
export async function run(
  call: Promise<ApiResult>,
  onSuccess?: () => void,
): Promise<boolean> {
  const r = await call;

  if (r.message) {
    if (r.success) {
      toast.success(r.message);
    } else {
      toast.error(r.message);
    }
  }

  if (r.success) onSuccess?.();
  return !!r.success;
}
