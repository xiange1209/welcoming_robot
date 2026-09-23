import { useEffect, useRef, useState } from "react";
import { Toaster } from "sonner";
import { useStoreValue } from "../hooks/useStore";
import { closeDialog, dialogStore, type DialogRequest } from "../lib/dialog";
import { Sheet } from "./Sheet";
import { Button, Field } from "./ui";

export function Toasts() {
  return (
    <Toaster
      position="top-center"
      offset={{ top: "calc(env(safe-area-inset-top) + 10px)" }}
      gap={16}
      style={{ zIndex: 55 }}
      toastOptions={{
        unstyled: true,
        classNames: {
          toast:
            "surface-strong pointer-events-auto flex items-center gap-3 rounded-2xl px-5 py-4 text-left text-[14px] leading-snug text-label shadow-md backdrop-blur-md transition-all" +
            "w-[calc(100vw-32px)] sm:w-[380px]",
          success:
            "bg-[color-mix(in_srgb,var(--color-green)_18%,var(--color-surface-strong))] border border-green/40 text-green",
          warning:
            "bg-[color-mix(in_srgb,var(--color-yellow)_18%,var(--color-surface-strong))] border border-yellow/40 text-yellow",
          error:
            "bg-[color-mix(in_srgb,var(--color-red)_18%,var(--color-surface-strong))] border border-red/40 text-red",
          info: "bg-[color-mix(in_srgb,var(--color-blue)_18%,var(--color-surface-strong))] border border-blue/40 text-blue",
        },
      }}
    />
  );
}

/** confirm()／prompt() 的替代品。實際內容由 lib/dialog.ts 推進來 */
export function Dialogs() {
  const request = useStoreValue(dialogStore);
  if (!request) return null;

  // key 讓每一次新的請求都拿到全新的元件實例，輸入值自然是預設值——
  // 比在 effect 裡重置乾淨，也不會先繪一幀舊值。
  return <DialogBody key={request.id} request={request} />;
}

function DialogBody({ request }: { request: DialogRequest }) {
  const { spec, id } = request;
  const [value, setValue] = useState(
    spec.kind === "prompt" ? (spec.defaultValue ?? "") : "",
  );
  const inputRef = useRef<HTMLInputElement>(null);

  const isPrompt = spec.kind === "prompt";

  useEffect(() => {
    if (!isPrompt) return;
    // 自動聚焦，平板上順便把鍵盤叫起來，少一次點擊。
    // 延遲一下是等進場動畫開始，否則 iOS 會把鍵盤和動畫疊在一起跳。
    const timer = setTimeout(() => inputRef.current?.focus(), 60);
    return () => clearTimeout(timer);
  }, [isPrompt]);

  const cancel = () => closeDialog(id, spec.kind === "confirm" ? false : null);
  const submit = () =>
    closeDialog(id, spec.kind === "confirm" ? true : value.trim());

  return (
    <Sheet
      open
      onClose={cancel}
      title={spec.title}
      subtitle={spec.message}
      footer={
        <>
          <Button variant="ghost" className="flex-1" onClick={cancel}>
            {spec.kind === "confirm" ? (spec.cancelLabel ?? "取消") : "取消"}
          </Button>
          <Button
            variant={
              spec.kind === "confirm" && spec.destructive ? "danger" : "primary"
            }
            className="flex-1"
            onClick={submit}
            // 輸入框空白時不給送出：新增地點沒有名稱是後端會拒絕的請求
            disabled={isPrompt && !value.trim()}
          >
            {spec.confirmLabel ?? "確定"}
          </Button>
        </>
      }
    >
      {isPrompt && (
        <Field
          ref={inputRef}
          value={value}
          placeholder={spec.placeholder}
          onChange={(e) => setValue(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === "Enter" && value.trim()) submit();
          }}
        />
      )}
    </Sheet>
  );
}
