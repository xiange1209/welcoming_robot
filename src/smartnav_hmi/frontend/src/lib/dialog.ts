import { createStore } from "./store";

/**
 * 以 Promise 表達的確認／輸入對話框，取代原生的 confirm() 與 prompt()。
 *
 * 換掉原生對話框有兩個實際理由，不只是為了好看：
 *   1. iOS 在「加入主畫面」的全螢幕模式下，原生對話框會把網址列拉回來，
 *      整個版面跳一次。
 *   2. 原生 prompt() 在部分平板瀏覽器上被當成彈出視窗擋掉，而它是新增
 *      地點的唯一入口——被擋掉時完全沒有徵兆。
 *
 * 做成 store 而不是 React context，是為了讓非元件的模組也能叫得動。
 */

export interface ConfirmSpec {
  kind: "confirm";
  title: string;
  message?: string;
  confirmLabel?: string;
  cancelLabel?: string;
  /** 破壞性操作：確認鈕轉紅。刪除、放棄錄製這類無法復原的動作都要標 */
  destructive?: boolean;
}

export interface PromptSpec {
  kind: "prompt";
  title: string;
  message?: string;
  placeholder?: string;
  defaultValue?: string;
  confirmLabel?: string;
}

type Spec = ConfirmSpec | PromptSpec;

export interface DialogRequest {
  id: number;
  spec: Spec;
  resolve: (value: boolean | string | null) => void;
}

export const dialogStore = createStore<DialogRequest | null>(null);

let nextId = 1;

function open(spec: Spec): Promise<boolean | string | null> {
  return new Promise((resolve) => {
    // 同時只會有一個。前一個還開著就先把它當成取消收掉，
    // 免得兩層對話框疊在一起而底下那個永遠沒人回答。
    const current = dialogStore.get();
    if (current) current.resolve(current.spec.kind === "confirm" ? false : null);

    dialogStore.set({ id: nextId++, spec, resolve });
  });
}

export function closeDialog(id: number, value: boolean | string | null): void {
  const current = dialogStore.get();
  if (!current || current.id !== id) return;
  dialogStore.set(null);
  current.resolve(value);
}

export async function confirmDialog(spec: Omit<ConfirmSpec, "kind">): Promise<boolean> {
  return (await open({ ...spec, kind: "confirm" })) === true;
}

export async function promptDialog(spec: Omit<PromptSpec, "kind">): Promise<string | null> {
  const value = await open({ ...spec, kind: "prompt" });
  return typeof value === "string" ? value : null;
}
