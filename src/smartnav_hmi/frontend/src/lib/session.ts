import { createStore } from "./store";

/**
 * 管理者工作階段。
 *
 * 權杖放 sessionStorage 而不是 localStorage：重新整理還在、關掉分頁就失效。
 * 這台平板是共用的，展示結束把分頁關掉就等於登出。
 */
const TOKEN_KEY = "hmiToken";

let token = sessionStorage.getItem(TOKEN_KEY) || "";

export const sessionStore = createStore<{ admin: boolean }>({ admin: false });

export const getToken = (): string => token;

/** 要帶給受保護端點的標頭。沒登入時回空物件，不要送空的 Bearer */
export function authHeaders(base: Record<string, string> = {}): Record<string, string> {
  return token ? { ...base, Authorization: `Bearer ${token}` } : { ...base };
}

export function setAdmin(on: boolean, newToken?: string): void {
  if (on) {
    if (newToken) {
      token = newToken;
      sessionStorage.setItem(TOKEN_KEY, newToken);
    }
  } else {
    token = "";
    sessionStorage.removeItem(TOKEN_KEY);
  }
  sessionStore.set({ admin: on });
}

export interface LoginOutcome {
  ok: boolean;
  message?: string;
}

/**
 * 登入。
 *
 * ★ 這支刻意不走 api()：尚未登入時後端回的 401 是「帳密錯誤」，
 *   而 api() 會把任何 401 解讀成「權杖失效」並跳出「請重新登入」的提示——
 *   在登入框裡跳這句話只會讓人以為系統壞了。
 */
export async function login(username: string, password: string): Promise<LoginOutcome> {
  try {
    const res = await fetch("/api/login", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ username, password }),
    });
    const data = (await res.json().catch(() => ({}))) as { success?: boolean; message?: string; token?: string };
    if (!data.success || !data.token) {
      return { ok: false, message: data.message || "登入失敗" };
    }
    setAdmin(true, data.token);
    return { ok: true };
  } catch (e) {
    return { ok: false, message: "無法連線：" + (e as Error).message };
  }
}

export async function logout(): Promise<void> {
  try {
    await fetch("/api/logout", { method: "POST", headers: authHeaders() });
  } catch {
    // 送不出去也照樣在本地登出——權杖留著只會讓後續請求一直吃 401
  }
  setAdmin(false);
}

/**
 * 開頁時確認手上的權杖還有效。
 * 地圖與地點都是管理端點，沒先確認就打只會拿到一串 401。
 */
export async function restoreSession(): Promise<boolean> {
  if (!token) {
    setAdmin(false);
    return false;
  }
  let ok: boolean;
  try {
    const res = await fetch("/api/session", { headers: authHeaders() });
    ok = ((await res.json().catch(() => ({}))) as { admin?: boolean }).admin === true;
  } catch {
    ok = false;
  }
  setAdmin(ok);
  return ok;
}
