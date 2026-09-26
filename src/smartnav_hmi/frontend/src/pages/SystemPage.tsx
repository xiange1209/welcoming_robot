import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Button, Card, Hint } from "../components/ui";
import { cx } from "../lib/cx";
import { useVisibleInterval } from "../hooks/useVisibleInterval";
import { api, post } from "../lib/api";
import { confirmDialog } from "../lib/dialog";
import type { SysUnit } from "../lib/types";

/* 十個開關擠成一列很難找，照迎賓流程的順序分三組。

   ★ `bank_reception` 一定要在「迎賓互動」組裡而且排在前面：它是整條故事線
     的核心（認出人之後要做什麼全在那個節點）。沒分組的話它會被追加到最後，
     排在四個語音開關**後面**——發表當天在平板上往下滑找按鈕，是最不該發生的事。

   ★ `asr_chain` 是 voice_trigger / speech_recognizer 兩個 key 合併後的名字
     （後端把它們改成同一支 run_asr_cc.sh 啟動）。**前端寫死 key、後端改 key**
     是這個檔案踩過兩次的坑，所以下面 SYS_GROUPS 與後端不一致時會主動喊一聲。 */
const SYS_GROUPS = [
  { title: "移動與感測", keys: ["sensors", "camera", "nav"] },
  { title: "迎賓互動", keys: ["face", "user_auth", "bank_reception", "llm"] },
  { title: "語音", keys: ["asr_chain", "speech_synthesizer", "voice_playback"] },
];

/** 導航堆疊要 60 秒才會全部起來，沒有這個回饋操作者只會看到一顆沒反應的
 *  按鈕，然後再按一次——曾經就這樣同時跑起兩套 Nav2。 */
export function SystemPage({ active }: { active: boolean }) {
  const [units, setUnits] = useState<SysUnit[]>([]);
  const [message, setMessage] = useState("");
  // 「已等 N 秒」要有個純粹的時間來源。算繪期間直接呼叫 Date.now()
  // 會讓同一份狀態算出不同結果（非純函式），把它收成 state。
  const [now, setNow] = useState(() => Date.now());

  const warnedKeys = useRef(false);
  const warnedUngrouped = useRef(false);
  const refreshInFlight = useRef(false);

  const refresh = useCallback(async () => {
    if (refreshInFlight.current) return;
    refreshInFlight.current = true;
    try {
      const r = await api<{ units?: SysUnit[] }>("/api/system/status");
      const all = r.units || [];
      setUnits(all);

    /* 後端改了單元 key、這裡忘了跟著改時，舊 key 會被靜默丟掉，畫面上看起來
       就是「那個功能不見了」，而且不會有任何錯誤。已經發生過兩次，
       所以主動喊一聲。只印一次，免得輪詢把 console 洗版。 */
      if (!warnedKeys.current && all.length) {
        warnedKeys.current = true;
        const known = new Set(all.map((u) => u.key));
        const missing = SYS_GROUPS.flatMap((g) => g.keys).filter((k) => !known.has(k));
        if (missing.length) {
          console.warn("[HMI] SYS_GROUPS 裡有後端不認得的單元 key（會被靜默略過）:", missing);
        }
      }
    } finally {
      refreshInFlight.current = false;
    }
  }, []);

  /* 只在這一頁而且分頁沒被隱藏時輪詢。
     ★ 間隔是 4 秒不是 2 秒：/api/system/status 是本站**最貴的一支端點**
       （掃過整個 /proc、查 ROS 圖，再對每個 lifecycle 節點做一次阻塞式
       get_state 服務呼叫，nav2 全開時是 8~10 個）。後端 _nodes_cache 的 TTL
       是 2.0 秒，跟 2000 ms 的輪詢打平，加上網路來回一定超過——那個快取
       **每一次都 miss**，等於沒有。4 秒讓它真的擋得住，而節點上下線這種事
       延遲 4 秒看到完全不影響操作。 */
  const hasTransition = units.some((unit) => !!unit.transition);
  useVisibleInterval(refresh, hasTransition ? 1000 : 4000, active);

  // 「已等 N 秒」要自己跳，不能等下一輪輪詢（那是 4 秒一次，看起來像卡住）
  // immediate 必須是 true：等待一開始就要把 now 對齊到現在，否則第一秒內
  //（now 還停在上一次的值）算出來的「已等 N 秒」會是負數。
  useVisibleInterval(() => setNow(Date.now()), 1000, active && hasTransition, true);

  const fire = async (unit: SysUnit, action: string, label: string) => {
    setMessage(`${label} 執行中…`);
    const r = await post(`/api/system/${unit.key}/${action}`);
    setMessage(r.message || "操作失敗");
    void refresh();
  };

  // 依分組排序；不在任何分組裡的（之後新增的）排最後，不會消失
  const { rows, ungrouped } = useMemo(() => {
    const seen = new Set<string>();
    const out: ({ header: string } | SysUnit)[] = [];
    for (const g of SYS_GROUPS) {
      const inGroup = g.keys
        .map((k) => units.find((u) => u.key === k))
        .filter((u): u is SysUnit => !!u);
      if (!inGroup.length) continue;
      out.push({ header: g.title });
      for (const u of inGroup) {
        out.push(u);
        seen.add(u.key);
      }
    }
    const rest = units.filter((u) => !seen.has(u.key));
    out.push(...rest);
    return { rows: out, ungrouped: rest };
  }, [units]);

  // 警告是副作用，要放 effect 裡而不是算繪期間。只印一次，免得 4 秒一輪洗版。
  useEffect(() => {
    if (!ungrouped.length || warnedUngrouped.current) return;
    warnedUngrouped.current = true;
    console.warn("[HMI] 這些單元沒有分組，被排在最後:", ungrouped.map((u) => u.key));
  }, [ungrouped]);

  return (
    <Card
      title="節點開關"
      className="min-h-0 flex-1"
      actions={<Button onClick={() => void refresh()}>重新整理狀態</Button>}
    >
      <Hint className="mb-3 flex-none">
        綠燈＝執行中。啟動需要時間（導航堆疊約 60 秒），按下後請等狀態變綠再操作下一項。
        <br />
        一般順序：<b className="text-label-2">底盤 → 導航堆疊</b>；要相機避障就在中間加開相機。
      </Hint>

      <div className="flex min-h-0 flex-1 flex-col gap-1 overflow-y-auto">
        {rows.map((row, i) => {
          if ("header" in row) {
            return (
              <div
                key={`h-${row.header}`}
                className={cx(
                  "border-t border-[var(--color-outline-variant)] pt-2 pb-0.5 text-[12px] font-semibold text-label-2",
                  i === 0 && "border-t-0 pt-0",
                )}
              >
                {row.header}
              </div>
            );
          }

          const u = row;
          const p = u.transition;
          const waited = p ? Math.max(0, Math.round(now / 1000 - p.started_at)) : 0;
          const dot = p ? "bg-blue" : u.running ? "bg-green" : u.partial ? "bg-orange" : "bg-[var(--color-label-3)]";

          return (
            <div key={u.key} className="flex items-center gap-2.5 rounded-ios px-1 py-2">
              <span className={cx("size-2.5 flex-none rounded-full", dot, p && "anim-pulse")} />

              <div className="min-w-0 flex-1">
                <div className="text-[14px]">{u.label}</div>
                {u.hint && <div className="text-[12px] text-label-3">{u.hint}</div>}
                {p && (
                  <div className="tnum text-[12px] text-blue">
                    {p.action === "stop" ? "正在關閉中" : "正在啟動中"}… 已等 {waited} 秒
                    {p.action === "stop"
                      ? p.phase === "sigint"
                        ? "（正在優雅關閉，通常需 20–25 秒）"
                        : "（正在收斂殘留程序）"
                      : "（請不要重複按）"}
                  </div>
                )}
                {u.partial && !p && (
                  <div className="text-[12px] text-orange">部分節點在跑，狀態不完整</div>
                )}
                {/* 前置條件不足時直接講清楚，不要讓人按了沒反應才去翻 log */}
                {u.blocked && <div className="text-[12px] text-red">⚠ {u.blocked} —— 啟動會失敗</div>}
              </div>

              <div className="flex flex-none flex-wrap justify-end gap-1.5">
                {/* 執行中或半死狀態都給「停止」：卡在一半時最需要的就是能把它收乾淨 */}
                {p ? (
                  <Button variant="danger" disabled>
                    {p.action === "stop" ? "關閉中…" : "啟動中…"}
                  </Button>
                ) : u.running || u.partial ? (
                  <Button variant="danger" disabled={!!p} onClick={() => void fire(u, "stop", `停止 ${u.label}`)}>
                    停止
                  </Button>
                ) : u.variants?.length ? (
                  u.variants.map((v) => (
                    <Button
                      key={v.key}
                      variant="primary"
                      disabled={!!p}
                      onClick={async () => {
                        // 會讓車子自己跑的選項要先確認，避免誤觸
                        if (v.warn) {
                          const ok = await confirmDialog({
                            title: `${v.label}？`,
                            message: v.warn,
                            confirmLabel: "啟動",
                          });
                          if (!ok) return;
                        }
                        void fire(u, `start:${v.key}`, `啟動 ${v.label}`);
                      }}
                    >
                      {v.label}
                    </Button>
                  ))
                ) : (
                  <Button variant="primary" disabled={!!p} onClick={() => void fire(u, "start", `啟動 ${u.label}`)}>
                    啟動
                  </Button>
                )}
              </div>
            </div>
          );
        })}

        {units.length === 0 && <Hint>讀不到節點狀態。確認 hmi_server 有在跑。</Hint>}
      </div>

      {message && <p className="mt-2 flex-none text-[12px] text-label-3">{message}</p>}
    </Card>
  );
}
