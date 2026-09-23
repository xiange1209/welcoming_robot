import { useEffect, useRef, useState } from "react";
import { Sheet } from "../components/Sheet";
import {
  Badge,
  Button,
  Card,
  Empty,
  Field,
  Hint,
  Row,
  Segmented,
  Select,
} from "../components/ui";
import { cx } from "../lib/cx";
import { useStoreSelector, useStoreValue } from "../hooks/useStore";
import { useVideoStream } from "../hooks/useVideoStream";
import { del, post, put, run } from "../lib/api";
import { confirmDialog } from "../lib/dialog";
import { presenceStore, telemetryStore } from "../lib/telemetry";
import { toast } from "../lib/toast";
import { refreshUsers, usersStore } from "../lib/users";
import type { UserRecord } from "../lib/types";

const USER_TYPES = [
  { value: 0, label: "訪客 GUEST" },
  { value: 1, label: "貴賓 VIP" },
  { value: 2, label: "管理員 ADMIN" },
  /* ★ BLACKLIST=3 從一開始就存在於 UserType.msg，但平板一直沒得選
     ——等於**沒辦法在現場註冊黑名單**，而黑名單通報是專題三個角色之一。 */
  { value: 3, label: "黑名單 BLACKLIST" },
];

/**
 * 註冊頁的人臉對位提示。
 *
 * ★ 必須同時看「新鮮度」與「有沒有 bbox」。少了 bbox 那一半，收到後端主動
 *   發出的「無人」訊息之後的數秒內，畫面會顯示「偵測到人臉——可以開始註冊」
 *   並把虛線框轉成綠色，對著空無一人的鏡頭給註冊者「已經對好位」的錯誤訊號。
 */
function FaceGuide() {
  const { present, boxed } = useStoreValue(presenceStore);
  const identity = useStoreSelector(
    telemetryStore,
    (t) => ({
      recognized: !!t.snap.identity?.recognized,
      name: t.snap.identity?.user_name || "",
    }),
    (a, b) => a.recognized === b.recognized && a.name === b.name,
  );
  /* 註冊狀態與剩餘秒數。
     deadline 與 snap.now **都是伺服器時鐘**，兩者相減得到的秒數不受平板與
     機器人的時鐘偏移影響——所以倒數直接在 selector 裡算完，不需要本地錨點、
     計時器或額外的 state。推播是 10 Hz，但 left 取整到秒，所以這個元件
     一秒只重繪一次。
     照片註冊沒有時限（做完就回），deadline 會是 undefined，此時不倒數。 */
  const reg = useStoreSelector(
    telemetryStore,
    (t) => {
      const r = t.snap.registration;
      if (r?.status !== "running") return null;
      return {
        message: r.message || "註冊進行中…",
        left: r.deadline ? Math.max(0, Math.ceil(r.deadline - t.snap.now)) : 0,
      };
    },
    (a, b) =>
      a === b || (!!a && !!b && a.message === b.message && a.left === b.left),
  );
  const left = reg?.left ?? 0;

  const registering = !!reg;
  const seen = present && boxed;
  const ok = registering || seen;

  const text = registering
    ? // 文字統一由 user_auth_node 產生，前端只補上倒數，免得即時採樣與
      // 照片註冊兩種流程各寫一份措辭
      reg.message + (left > 0 ? `　剩 ${left} 秒` : "")
    : seen
      ? identity.recognized
        ? `偵測到人臉：${identity.name}`
        : "偵測到人臉（未在資料庫中）——可以開始註冊"
      : "未偵測到人臉——請正對鏡頭，臉部對準虛線框";

  return (
    <>
      <div
        className={cx(
          "pointer-events-none absolute top-1/2 left-1/2 aspect-[3/4] max-h-[76%] w-[42%] -translate-x-1/2 -translate-y-1/2 rounded-[50%] border-2 transition-colors duration-300",
          ok ? "border-green" : "border-dashed border-white/50",
        )}
      />
      <span
        className={cx(
          // 浮貼在即時攝影機畫面上，底色跟預設文字色故意跟主題脫鉤
          // （淺色模式的黑底配深色文字幾乎看不見）——只有 ok 的綠色沿用主題。
          "pointer-events-none absolute bottom-2 left-2 rounded-full bg-black px-2.5 py-1 text-[12px]",
          ok ? "text-green" : "text-white/70",
        )}
      >
        {text}
      </span>
    </>
  );
}

function EditUserSheet({
  user,
  onClose,
}: {
  user: UserRecord;
  onClose: (changed: boolean) => void;
}) {
  // 初始值直接從 props 來：這個元件由呼叫端用 key 重建，不會換 user
  const [name, setName] = useState(user.user_name);
  const [type, setType] = useState(user.user_type);
  const [desc, setDesc] = useState(user.description || "");

  return (
    <Sheet
      open
      onClose={() => onClose(false)}
      title="編輯使用者"
      subtitle={user.user_uuid}
      footer={
        <>
          <Button
            variant="ghost"
            className="flex-1"
            onClick={() => onClose(false)}
          >
            取消
          </Button>
          <Button
            variant="primary"
            className="flex-1"
            disabled={!name.trim()}
            onClick={async () => {
              const ok = await run(
                put(`/api/users/${user.user_uuid}`, {
                  user_name: name.trim(),
                  user_type: type,
                  description: desc,
                }),
              );
              onClose(ok);
            }}
          >
            儲存
          </Button>
        </>
      }
    >
      <div className="flex flex-col gap-2.5">
        <Field
          placeholder="姓名"
          value={name}
          onChange={(e) => setName(e.target.value)}
        />
        <Select
          value={type}
          onChange={(e) => setType(parseInt(e.target.value, 10))}
        >
          {USER_TYPES.map((t) => (
            <option key={t.value} value={t.value}>
              {t.label}
            </option>
          ))}
        </Select>
        <Field
          placeholder="備註"
          value={desc}
          onChange={(e) => setDesc(e.target.value)}
        />
      </div>
    </Sheet>
  );
}

export function UsersPage({ active }: { active: boolean }) {
  const videoRef = useVideoStream(active);
  const users = useStoreValue(usersStore);

  const [mode, setMode] = useState<"live" | "photo">("live");
  const [name, setName] = useState("");
  const [type, setType] = useState(0);
  const [desc, setDesc] = useState("");
  const [samples, setSamples] = useState(10);
  const [shots, setShots] = useState<string[]>([]);
  const [submitting, setSubmitting] = useState(false);
  const [editing, setEditing] = useState<UserRecord | null>(null);

  const fileRef = useRef<HTMLInputElement>(null);

  // 註冊期間所有裝置都鎖住按鈕，避免兩台同時按
  const running = useStoreSelector(
    telemetryStore,
    (t) => t.snap.registration?.status === "running",
  );

  useEffect(() => {
    if (active) void refreshUsers();
  }, [active]);

  const clearForm = () => {
    setName("");
    setDesc("");
  };

  const registerLive = async () => {
    if (!name.trim()) {
      toast.error("請輸入姓名");
      return;
    }
    await run(
      post("/api/users/register", {
        user_name: name.trim(),
        user_type: type,
        description: desc,
        num_samples: samples,
      }),
      clearForm,
      // 進度、完成提示與名單刷新都由 startRegistrationWatch 依後端廣播處理，
      // 這裡不自己算時間——否則只有這台裝置看得到
    );
  };

  const registerPhotos = async () => {
    if (!name.trim()) {
      toast.error("請輸入姓名");
      return;
    }
    if (!shots.length) {
      toast.error("請先選擇或擷取照片");
      return;
    }
    setSubmitting(true);
    try {
      await run(
        post("/api/users/register-photo", {
          user_name: name.trim(),
          user_type: type,
          description: desc,
          photos: shots,
        }),
        () => {
          clearForm();
          setShots([]);
          void refreshUsers();
        },
      );
    } finally {
      setSubmitting(false);
    }
  };

  const snapshot = async () => {
    try {
      const res = await fetch(`/api/frame.jpg?t=${Date.now()}`);
      if (!res.ok) {
        toast.error("目前沒有相機影像");
        return;
      }
      const blob = await res.blob();
      const reader = new FileReader();
      reader.onload = () => {
        if (typeof reader.result === "string") {
          setShots((prev) => [...prev, reader.result as string]);
          toast.success("已擷取 1 張");
        } else {
          toast.error("影像格式轉換失敗");
        }
      };
      reader.readAsDataURL(blob);
    } catch (e) {
      toast.error("擷取失敗：" + (e as Error).message);
    }
  };

  return (
    <div className="grid min-h-0 flex-1 gap-3 xl:grid-cols-[1.3fr_1fr]">
      <Card
        title="已註冊使用者"
        actions={<Button onClick={() => void refreshUsers()}>重新整理</Button>}
      >
        <div className="flex min-h-0 flex-1 flex-col gap-1.5 overflow-y-auto">
          {users.length === 0 && <Empty>尚無註冊使用者。</Empty>}
          {users.map((u) => (
            <Row key={u.user_uuid}>
              <Badge tone={u.user_type_name}>{u.user_type_name}</Badge>
              <span className="min-w-0 flex-1">
                <span className="font-semibold">{u.user_name}</span>
                <br />
                <span className="text-[12px] text-label-3">
                  {u.description || "無備註"}　{u.num_samples} 個樣本
                </span>
              </span>
              <Button onClick={() => setEditing(u)}>編輯</Button>
              <Button
                variant="danger"
                onClick={async () => {
                  const ok = await confirmDialog({
                    title: `刪除「${u.user_name}」？`,
                    message: `連同 ${u.num_samples} 個人臉樣本一起移除，無法復原。`,
                    confirmLabel: "刪除",
                    destructive: true,
                  });
                  if (!ok) return;
                  await run(
                    del(`/api/users/${u.user_uuid}`),
                    () => void refreshUsers(),
                  );
                }}
              >
                刪除
              </Button>
            </Row>
          ))}
        </div>
      </Card>

      <Card
        title="註冊新使用者"
        className="overflow-y-auto"
        bodyClassName="gap-2.5"
      >
        <div className="relative h-[min(34vh,260px)] flex-none overflow-hidden rounded-ios bg-black">
          <img
            ref={videoRef}
            alt="註冊預覽"
            className="size-full object-contain"
          />
          <FaceGuide />
        </div>

        <div className="flex gap-2">
          <Field
            className="min-w-0 flex-1"
            placeholder="姓名"
            value={name}
            onChange={(e) => setName(e.target.value)}
          />
          <Select
            value={type}
            onChange={(e) => setType(parseInt(e.target.value, 10))}
          >
            {USER_TYPES.map((t) => (
              <option key={t.value} value={t.value}>
                {t.label}
              </option>
            ))}
          </Select>
        </div>

        <Field
          placeholder="備註（例如：分行經理）"
          value={desc}
          onChange={(e) => setDesc(e.target.value)}
        />

        <Segmented
          value={mode}
          options={[
            { value: "live" as const, label: "即時採樣" },
            { value: "photo" as const, label: "照片註冊" },
          ]}
          onChange={setMode}
        />

        {mode === "live" ? (
          <>
            <div className="flex items-center gap-2">
              <span className="text-[12px] text-label-3">採樣張數</span>
              <Field
                type="number"
                min={1}
                max={30}
                className="w-20"
                value={samples}
                onChange={(e) => setSamples(parseInt(e.target.value, 10) || 10)}
              />
              <Button
                variant="primary"
                className="ml-auto"
                disabled={running}
                onClick={registerLive}
              >
                {running ? "註冊進行中…" : "開始註冊"}
              </Button>
            </div>
            <Hint>
              按下後<b className="text-label-2">立刻站到相機前並持續站著</b>
              ，稍微轉頭讓樣本多樣化。 上方會即時顯示已採集張數。採樣有{" "}
              <b className="text-label-2">20 秒</b>上限，
              時間內沒收滿會自動取消並移除該筆使用者，需要重新註冊。
            </Hint>
          </>
        ) : (
          <>
            <div className="flex flex-wrap items-center gap-2">
              <Button onClick={() => fileRef.current?.click()}>選擇照片</Button>
              <Button onClick={() => void snapshot()}>從畫面拍照</Button>
              <span className="ml-auto text-[12px] text-label-3">
                {shots.length ? `已選 ${shots.length} 張` : "尚未選擇"}
              </span>
            </div>

            <input
              ref={fileRef}
              type="file"
              accept="image/*"
              multiple
              hidden
              onChange={(e) => {
                const files = Array.from(e.target.files || []);
                files.forEach((f) => {
                  const reader = new FileReader();
                  reader.onload = () =>
                    setShots((prev) => [...prev, reader.result as string]);
                  reader.readAsDataURL(f);
                });
                e.target.value = ""; // 同一個檔案可以再選一次
              }}
            />

            {shots.length > 0 && (
              <div className="flex gap-2 overflow-x-auto py-1">
                {shots.map((src, i) => (
                  <div key={i} className="relative flex-none">
                    <img
                      src={src}
                      alt=""
                      className="size-14 rounded-ios object-cover"
                    />
                    {/* 移除鈕是覆蓋層，不能套 44px 的可點目標下限——被撐高之後會蓋掉
                        縮圖右側一大半，橫向撥動瀏覽時手指碰到就是靜默刪掉那張 */}
                    <button
                      type="button"
                      title="移除"
                      className="absolute -top-1.5 -right-1.5 size-6 rounded-full bg-red text-[14px] leading-6 text-on-red"
                      onClick={() =>
                        setShots((prev) => prev.filter((_, j) => j !== i))
                      }
                    >
                      ×
                    </button>
                  </div>
                ))}
              </div>
            )}

            <Button
              variant="primary"
              block
              disabled={submitting}
              onClick={registerPhotos}
            >
              {submitting ? `分析 ${shots.length} 張照片中…` : "用這些照片註冊"}
            </Button>
            <Hint>
              適合替不在現場的人建檔。每張照片會各自抽一次人臉特徵，
              <b className="text-label-2">沒偵測到人臉的會被略過</b>。
              實測：單張的置信度距門檻只有 0.002，10 張則有 0.071 的安全邊際（約
              35 倍）。
              <b className="text-label-2">
                正式展示請放 10 張以上不同角度的照片。
              </b>
            </Hint>
          </>
        )}
      </Card>

      {/* 條件掛載 + key：換一個使用者就換一個元件實例，表單欄位自然帶到新的值，
          不必在 effect 裡把 props 複製進 state */}
      {editing && (
        <EditUserSheet
          key={editing.user_uuid}
          user={editing}
          onClose={(changed) => {
            setEditing(null);
            if (changed) void refreshUsers();
          }}
        />
      )}
    </div>
  );
}
