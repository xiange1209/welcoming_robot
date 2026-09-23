import { useEffect, useState } from "react";
import { MapCanvas } from "../components/MapCanvas";
import { Badge, Button, Card, Empty, Field, Hint, Row } from "../components/ui";
import { cx } from "../lib/cx";
import { useStoreSelector, useStoreValue } from "../hooks/useStore";
import { useFetch } from "../hooks/useFetch";
import { useVisibleInterval } from "../hooks/useVisibleInterval";
import { api, del, post, run } from "../lib/api";
import { confirmDialog } from "../lib/dialog";
import * as teleop from "../lib/teleop";
import { toast } from "../lib/toast";
import type { ApiResult, MapEntry, MapStatus, TaughtPath } from "../lib/types";

/* ── 方向鍵 ────────────────────────────────────────────────
   九宮格排法對照 teleop_twist_keyboard 的 u/i/o j/k/l m/,/.：
   四個角是「前進＋轉向」的複合鍵，這是阿克曼車真正能執行的動作。
   純轉向的 ◀ ▶ 要搭配 ▲ 或 ▼ 一起按才有效（可以多指同時按）。 */
const PAD: ({ label: string; lin: number; ang: number } | "stop")[] = [
  { label: "◤", lin: 1, ang: 1 },
  { label: "▲", lin: 1, ang: 0 },
  { label: "◥", lin: 1, ang: -1 },
  { label: "◀", lin: 0, ang: 1 },
  "stop",
  { label: "▶", lin: 0, ang: -1 },
  { label: "◣", lin: -1, ang: 1 },
  { label: "▼", lin: -1, ang: 0 },
  { label: "◢", lin: -1, ang: -1 },
];

/** 鍵盤操作（筆電上比較好用）。四個角是複合動作，對應方向鍵無法表達的組合 */
const KEY_MAP: Record<string, [number, number]> = {
  arrowup: [1, 0],
  arrowdown: [-1, 0],
  arrowleft: [0, 1],
  arrowright: [0, -1],
  w: [1, 0],
  s: [-1, 0],
  a: [0, 1],
  d: [0, -1],
};

function DirectionPad() {
  const held = useStoreValue(teleop.heldKeysStore);

  return (
    <div
      className="mx-auto grid max-w-[330px] grid-cols-3 gap-2.5"
      style={{ touchAction: "none", userSelect: "none" }}
    >
      {PAD.map((cell, i) => {
        if (cell === "stop") {
          return (
            <button
              key="stop"
              type="button"
              className="pressable h-[68px] rounded-ios border border-red/40 bg-red/20 text-[24px] text-red"
              // 急停綁 pointerdown 而不是 click：click 要等 pointerup 才觸發，
              // 而且在觸控裝置上還有延遲。急停不能等。
              onPointerDown={(e) => {
                e.preventDefault();
                teleop.stop();
              }}
            >
              ■
            </button>
          );
        }

        const key = `pad${i}`;
        const on = [...held].some((k) => k.startsWith(`p${key}:`));

        return (
          <button
            key={key}
            type="button"
            className={cx(
              "pressable h-[68px] rounded-ios border text-[26px] transition-colors",
              on
                ? "border-blue bg-blue text-on-blue"
                : "border-[var(--color-outline-variant)] bg-[var(--color-surface)] text-label",
            )}
            // touch-action:none 是必要的——否則長按會被瀏覽器判成捲動或選取，
            // 手指還按著卻不再送指令，車子就會被看門狗停掉。
            style={{ touchAction: "none" }}
            onPointerDown={(e) => {
              e.preventDefault();
              // 手指按著時把後續事件都導到這個按鈕，滑出邊界也不會漏掉 pointerup
              try {
                e.currentTarget.setPointerCapture(e.pointerId);
              } catch {
                // 少數瀏覽器在多指同按時會丟例外，捕捉不到就照常送指令
              }
              teleop.press(`p${key}:${e.pointerId}`, cell.lin, cell.ang);
            }}
            // 只移除這一根手指，其他還按著的照常生效
            onPointerUp={(e) => teleop.release(`p${key}:${e.pointerId}`)}
            onPointerCancel={(e) => teleop.release(`p${key}:${e.pointerId}`)}
            onLostPointerCapture={(e) =>
              teleop.release(`p${key}:${e.pointerId}`)
            }
          >
            {cell.label}
          </button>
        );
      })}
    </div>
  );
}

function TeleopStatus() {
  const status = useStoreSelector(teleop.teleopStore, (t) => t.status);
  const moving = useStoreSelector(teleop.teleopStore, (t) => t.moving);
  return (
    <div
      className={cx(
        "mb-2 rounded-ios px-3 py-2 text-[13px] transition-colors",
        moving ? "bg-blue/15 text-blue" : "bg-[var(--color-well)] text-label-3",
      )}
    >
      {status}
    </div>
  );
}

const MODE_LABEL: Record<string, string> = {
  mapping: "建圖中",
  localization: "定位／導航",
  unknown: "未知（導航堆疊沒啟動？）",
};

function MappingStatus({ status }: { status: MapStatus | null }) {
  if (!status) return <Hint>讀取狀態失敗</Hint>;

  const mode = MODE_LABEL[status.mode || ""] || status.mode || "未知";
  const tone =
    status.mode === "mapping"
      ? "text-green"
      : status.mode === "localization"
        ? "text-blue"
        : "text-label-3";
  const m = status.map_meta;
  const size = m
    ? `${m.width}×${m.height} 格 @${m.resolution}m（${(m.width * m.resolution).toFixed(1)}×${(m.height * m.resolution).toFixed(1)} 公尺）`
    : "尚未收到地圖";

  return (
    <div className="mt-2 rounded-ios border border-[var(--color-outline-variant)] bg-[var(--color-well)] px-3 py-2.5 text-[13px] leading-7">
      <div>
        模式：<b className={tone}>{mode}</b>
      </div>
      <div>
        目前地圖：<b>{status.current_map || "（無）"}</b>
      </div>
      <div className="text-label-3">尺寸：{size}</div>

      {/* 「沒有在跑」和「跑了但失敗」是完全不同的兩件事。只顯示前者的話，
          使用者按了開始建圖、後端 15 秒後失敗，畫面卻還是那句「沒有建圖作業在跑」
          ——他會以為是自己還沒按。 */}
      {status.mapping_job ? (
        <div className="text-green">
          ● 建圖作業進行中：{status.mapping_job.label || ""}
        </div>
      ) : status.last_failed_job ? (
        <div className="text-red">
          ✗ 上次建圖失敗：{status.last_failed_job.message || ""}
          <div className="text-[12px] text-label-3">
            按「開始建圖」重試前請先看這則訊息
          </div>
        </div>
      ) : (
        <div className="text-label-3">
          沒有建圖作業在跑 —— 要建圖請先按「開始建圖」
        </div>
      )}
    </div>
  );
}

export function TeleopPage({ active }: { active: boolean }) {
  const [mapName, setMapName] = useState("");
  const [pathName, setPathName] = useState("");
  const [recording, setRecording] = useState(false);
  const [recSeconds, setRecSeconds] = useState(0);
  const [pathSpeed, setPathSpeed] = useState(100);
  const [rearMask, setRearMask] = useState(false);

  const speed = useStoreSelector(teleop.teleopStore, (t) => t.speed);

  const [status, refreshStatus] = useFetch<MapStatus | null>(
    () => api<MapStatus>("/api/maps/status"),
    null,
  );

  const [maps, refreshMapList] = useFetch<MapEntry[]>(
    async () => (await api<{ maps?: MapEntry[] }>("/api/maps")).maps || [],
    [],
  );

  const [pathState, refreshPaths] = useFetch<{
    paths: TaughtPath[];
    error: string | null;
  }>(
    async () => {
      const r = await api<ApiResult & { paths?: TaughtPath[] }>("/api/paths");
      // 失敗時要說「壞了」而不是留一個空清單假裝「沒有資料」
      return r.success === false
        ? { paths: [], error: r.message || "無回應" }
        : { paths: r.paths || [], error: null };
    },
    { paths: [], error: null },
  );
  const { paths, error: pathsError } = pathState;

  // 離開這一頁一定要停車：使用者可能按著方向鍵就切走了。
  // App 是「切走即卸載」，所以這一條掛在卸載時。
  useEffect(() => () => teleop.stop(), []);

  /* 鍵盤遙控。只在這一頁生效，而且焦點在輸入框時完全不攔截——
     同一頁就有「地圖名稱」欄位，不擋的話打 "warehouse" 的每個 w/a/s/d 都會
     開走車子，而且 preventDefault 會讓字元根本打不進去，看起來像輸入框壞掉。 */
  useEffect(() => {
    if (!active) return;

    const isTyping = (t: EventTarget | null) => {
      const el = t as HTMLElement | null;
      return (
        !!el &&
        (el.tagName === "INPUT" ||
          el.tagName === "TEXTAREA" ||
          el.tagName === "SELECT" ||
          el.isContentEditable)
      );
    };

    const onKeyDown = (e: KeyboardEvent) => {
      if (isTyping(e.target)) return;
      if (e.ctrlKey || e.metaKey || e.altKey) return; // Ctrl+S 想存檔不該變成倒車
      if (e.repeat) return; // 按住時瀏覽器會連發，心跳已經在跑了
      const v = KEY_MAP[e.key.toLowerCase()];
      if (!v) return;
      e.preventDefault();
      teleop.press(`k${e.key.toLowerCase()}`, v[0], v[1]);
    };

    const onKeyUp = (e: KeyboardEvent) => {
      if (!KEY_MAP[e.key.toLowerCase()]) return;
      teleop.release(`k${e.key.toLowerCase()}`);
    };

    document.addEventListener("keydown", onKeyDown);
    document.addEventListener("keyup", onKeyUp);
    return () => {
      document.removeEventListener("keydown", onKeyDown);
      document.removeEventListener("keyup", onKeyUp);
    };
  }, [active]);

  /* 錄製計時器是錄製中唯一的「還活著」訊號。沒有它，使用者無從判斷
     錄製是真的在跑、還是後端早就掛了。 */
  useVisibleInterval(
    () => setRecSeconds((s) => s + 1),
    1000,
    active && recording,
    false,
  );

  const startMapping = async () => {
    const name = mapName.trim() || `map_${Date.now()}`;
    await run(post("/api/maps/create", { map_name: name }), () => {
      setTimeout(refreshStatus, 4000);
    });
  };

  const finishMapping = async () => {
    teleop.stop(); // 存檔前先確保車子停著
    await run(post("/api/maps/finish", {}), () => {
      setTimeout(() => {
        refreshStatus();
        refreshMapList();
      }, 5000);
    });
  };

  const followPath = async (p: TaughtPath, reverse: boolean) => {
    const word = reverse ? "反向重播" : "重播";
    const ok = await confirmDialog({
      title: `${word}「${p.name}」？`,
      message: `車子會立刻開始移動（${p.length_m} 公尺）。上方列的「緊急停止」隨時可以中止。`,
      confirmLabel: word,
    });
    if (!ok) return;
    await run(
      post("/api/paths/follow", {
        path_id: p.path_id,
        name: p.name,
        reverse,
        speed_scale: pathSpeed / 100,
      }),
    );
  };

  return (
    <div className="grid min-h-0 flex-1 gap-3 xl:grid-cols-[1.4fr_1fr]">
      {/* ── 左：即時地圖 + 建圖 + 教導路徑 ───────────────── */}
      <Card title="即時地圖" className="overflow-y-auto" bodyClassName="gap-3">
        <MapCanvas
          active={active}
          minimal
          className="h-[min(46vh,360px)] flex-none"
        />

        <MappingStatus status={status} />

        <div className="flex flex-wrap gap-2">
          <Field
            className="min-w-[140px] flex-1"
            placeholder="地圖名稱"
            value={mapName}
            onChange={(e) => setMapName(e.target.value)}
          />
          <Button variant="primary" onClick={startMapping}>
            開始建圖
          </Button>
          <Button onClick={finishMapping}>完成存檔</Button>
        </div>

        <div>
          <h3 className="mb-2 text-[13px] font-semibold text-label-2 uppercase">
            已存地圖
          </h3>
          <div className="flex max-h-[190px] flex-col gap-1.5 overflow-y-auto">
            {maps.length === 0 && <Empty>尚無地圖</Empty>}
            {maps.map((mp) => (
              <Row key={mp.map_id}>
                <span className="flex-1 truncate">
                  {mp.map_name || mp.map_id}
                </span>
                <span className="truncate text-[11px] text-label-3">
                  {mp.map_id}
                </span>
                <Button
                  onClick={async () => {
                    await run(
                      post("/api/maps/switch", { map_id: mp.map_id }),
                      refreshStatus,
                    );
                  }}
                >
                  切換
                </Button>
                <Button
                  variant="danger"
                  onClick={async () => {
                    // 帶上名稱與 ID：同名地圖很常見，只顯示名稱會刪錯
                    const ok = await confirmDialog({
                      title: `刪除地圖「${mp.map_name || ""}」？`,
                      message: `ID: ${mp.map_id}\n此動作無法復原。`,
                      confirmLabel: "刪除",
                      destructive: true,
                    });
                    if (!ok) return;
                    await run(
                      del(`/api/maps/${encodeURIComponent(mp.map_id)}`),
                      () => {
                        refreshMapList();
                        refreshStatus();
                      },
                    );
                  }}
                >
                  刪除
                </Button>
              </Row>
            ))}
          </div>
        </div>

        {/* ── 教導路徑 ─────────────────────────────────────
            放在遙控頁而不是地圖頁：錄製時使用者就是在這裡開車，
            「按開始 → 開一遍 → 按結束」要在同一個畫面完成，
            中途切分頁會讓人以為錄製中斷了。 */}
        <div>
          <h3 className="mb-1.5 text-[13px] font-semibold text-label-2 uppercase">
            教導路徑
          </h3>
          <Hint className="mb-2">
            錄下你開過的路線，之後可以原樣重播。
            <b className="text-label-2">路徑是你親手開過的，所以一定開得過去</b>
            ——比自動規劃可靠，展示時建議用這個。
          </Hint>

          {!recording ? (
            <Button
              variant="primary"
              onClick={async () => {
                await run(
                  post("/api/paths/record", { action: "start" }),
                  () => {
                    setRecSeconds(0); // 在按下的當下歸零，不要靠 effect 追 recording
                    setRecording(true);
                  },
                );
              }}
            >
              ● 開始錄製路徑
            </Button>
          ) : (
            <div className="rounded-ios border border-orange/40 bg-orange/12 p-3">
              <div className="flex items-center gap-2 font-semibold text-orange">
                <span className="anim-pulse size-2.5 rounded-full bg-orange" />
                錄製中
              </div>
              <Hint className="tnum mt-1">
                已錄製 {Math.floor(recSeconds / 60)} 分 {recSeconds % 60}{" "}
                秒　請用方向鍵把車開一遍
              </Hint>
              <Field
                className="mt-2"
                placeholder="路徑名稱（例：大廳到接待點）"
                value={pathName}
                onChange={(e) => setPathName(e.target.value)}
              />
              <div className="mt-2 flex gap-2">
                <Button
                  variant="primary"
                  className="flex-1"
                  onClick={async () => {
                    const name = pathName.trim();
                    if (!name) {
                      toast.error("請先輸入路徑名稱");
                      return;
                    }
                    // 存檔前先停車：錄製中車子可能還在動，最後幾個點會是雜訊
                    teleop.stop();
                    await run(
                      post("/api/paths/record", { action: "stop", name }),
                      () => {
                        /* 只有 success === true 才會執行這裡。
                           失敗時不關掉錄製介面，留著讓使用者改名再存一次 */
                        setRecording(false);
                        setPathName("");
                        refreshPaths();
                      },
                    );
                  }}
                >
                  ■ 結束並存檔
                </Button>
                <Button
                  onClick={async () => {
                    const ok = await confirmDialog({
                      title: "放棄這次錄製？",
                      message: "已錄的內容會被丟棄，無法復原。",
                      confirmLabel: "放棄",
                      destructive: true,
                    });
                    if (!ok) return;
                    await run(
                      post("/api/paths/record", { action: "cancel" }),
                      () => setRecording(false),
                    );
                  }}
                >
                  放棄
                </Button>
              </div>
            </div>
          )}

          <div className="mt-3 flex items-center gap-2">
            <span className="w-16 flex-none text-[12px] text-label-3">
              重播速度
            </span>
            <input
              type="range"
              min={30}
              max={100}
              value={pathSpeed}
              className="flex-1"
              onChange={(e) => setPathSpeed(parseInt(e.target.value, 10))}
            />
            <span className="tnum w-11 text-right text-[12px] text-label-3">
              {pathSpeed}%
            </span>
          </div>

          <div className="mt-2 flex max-h-[240px] flex-col gap-1.5 overflow-y-auto">
            {pathsError && (
              <div className="text-[13px] text-red">讀取失敗：{pathsError}</div>
            )}
            {!pathsError && paths.length === 0 && (
              <Empty>
                尚無教導路徑。按上面的「開始錄製路徑」，把車開一遍就會存下來。
              </Empty>
            )}
            {paths.map((p) => (
              <Row key={p.path_id} className="flex-wrap">
                <span className="min-w-[120px] flex-1">
                  <span className="font-semibold">{p.name}</span>
                  <br />
                  <span className="text-[12px] text-label-3">
                    {p.length_m} 公尺　{p.num_points} 點
                    {p.num_cusps > 0 ? `　折返 ${p.num_cusps} 次` : ""}
                    {p.source === "plan" ? "地圖規劃" : "開車錄製"}
                  </span>
                </span>
                <Button
                  variant="primary"
                  onClick={() => void followPath(p, false)}
                >
                  重播
                </Button>
                {/* 反向重播 = 原路折返回起點。展示時「送客人過去再自己回來」就靠這個，
                    而且不需要掉頭（阿克曼車在 0.99 m 走廊掉不了頭）。 */}
                <Button
                  title="原路折返回起點（不需要掉頭）"
                  onClick={() => void followPath(p, true)}
                >
                  反向
                </Button>
                <Button
                  variant="danger"
                  onClick={async () => {
                    const ok = await confirmDialog({
                      title: `刪除路徑「${p.name}」？`,
                      message: `${p.length_m} 公尺，刪除後無法復原。`,
                      confirmLabel: "刪除",
                      destructive: true,
                    });
                    if (!ok) return;
                    await run(
                      del(`/api/paths/${encodeURIComponent(p.path_id)}`),
                      () => void refreshPaths(),
                    );
                  }}
                >
                  刪除
                </Button>
              </Row>
            ))}
          </div>
        </div>
      </Card>

      {/* ── 右：遙控 ───────────────────────────────────── */}
      <Card title="遙控" className="overflow-y-auto">
        <TeleopStatus />
        <DirectionPad />

        <Hint className="mt-2.5">
          這是<b className="text-label-2">阿克曼轉向</b>車，和汽車一樣
          <b className="text-label-2">無法原地旋轉</b>
          ——只按 ◀ ▶ 車子不會動。請用四個角的複合鍵，或按住 ▲ 再加按 ◀
          ▶。最小轉彎半徑 0.8 m（掉頭需要 1.6 m 寬）。
        </Hint>

        <div className="mt-4 flex items-center gap-2">
          <span className="w-12 flex-none text-[12px] text-label-3">速度</span>
          <input
            type="range"
            min={20}
            max={100}
            value={Math.round(speed * 100)}
            className="flex-1"
            onChange={(e) =>
              teleop.setTeleopSpeed(parseInt(e.target.value, 10) / 100)
            }
          />
          <span className="tnum w-14 text-right text-[12px] text-label-3">
            {(speed * 0.18).toFixed(2)} m/s
          </span>
        </div>

        <label className="mt-4 flex cursor-pointer items-center gap-3">
          <span
            className="ios-switch"
            data-on={rearMask}
            onPointerDown={async (e) => {
              e.preventDefault();
              const next = !rearMask;
              setRearMask(next);
              const ok = await run(
                post("/api/teleop/rearmask", {
                  enabled: next,
                  half_angle_deg: 35,
                }),
              );
              if (!ok) {
                setRearMask(!next);
              }
            }}
          />
          <span className="text-[14px]">遮蔽車尾雷達（我站在車後）</span>
        </label>

        <Hint className="mt-2 text-red/85">
          注意：雷達驅動只在<b>啟動時</b>讀這個設定，勾選後<b>本次不會生效</b>
          （已實測確認）。 要真的遮蔽必須重啟感測器。在那之前，請盡量走在車子
          <b>側邊或前方</b>，不要待在正後方。
        </Hint>

        <Hint className="mt-3">
          放開按鈕即停。若連線中斷（走出 WiFi、鎖螢幕、切換 App），車子會在 0.6
          秒內自動停止。
        </Hint>

        <div className="mt-3">
          <Badge tone="muted">鍵盤：W A S D 或方向鍵，空白鍵急停</Badge>
        </div>
      </Card>
    </div>
  );
}
