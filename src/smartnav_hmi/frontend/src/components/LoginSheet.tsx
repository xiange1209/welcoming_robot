import { useRef, useState } from "react";
import { login } from "../lib/session";
import { toast } from "../lib/toast";
import { Sheet } from "./Sheet";
import { Button, Field } from "./ui";

export function LoginSheet({
  onClose,
  onSuccess,
}: {
  onClose: () => void;
  onSuccess: () => void;
}) {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const passRef = useRef<HTMLInputElement>(null);

  const submit = async () => {
    if (!username.trim() || !password) {
      setError("請輸入帳號與密碼");
      return;
    }
    setBusy(true);
    const r = await login(username.trim(), password);
    setBusy(false);
    if (!r.ok) {
      setError(r.message || "登入失敗");
      return;
    }
    setPassword("");
    toast.success("管理者登入成功");
    onSuccess();
  };

  return (
    <Sheet
      open
      onClose={onClose}
      title="管理者登入"
      subtitle="登入後才會出現地圖導航、遙控建圖、系統開關與使用者管理"
      footer={
        <>
          <Button variant="ghost" className="flex-1" onClick={onClose}>
            取消
          </Button>
          <Button
            variant="primary"
            className="flex-1"
            disabled={busy}
            onClick={() => void submit()}
          >
            {busy ? "登入中…" : "登入"}
          </Button>
        </>
      }
    >
      <div className="flex flex-col gap-2.5">
        <Field
          placeholder="帳號"
          autoComplete="username"
          value={username}
          onChange={(e) => setUsername(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === "Enter") passRef.current?.focus();
          }}
        />
        <Field
          ref={passRef}
          type="password"
          placeholder="密碼"
          autoComplete="current-password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === "Enter") void submit();
          }}
        />
        {/* 高度固定，錯誤訊息出現時不要把按鈕往下推 */}
        <p className="min-h-[18px] text-[13px] text-red">{error}</p>
      </div>
    </Sheet>
  );
}
