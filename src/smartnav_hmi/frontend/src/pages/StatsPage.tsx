import { Button, Card, Empty, Hint } from "../components/ui";
import { useFetch } from "../hooks/useFetch";
import { api } from "../lib/api";
import type { StatsResult } from "../lib/types";

/* 資料來自 bank_reception_node 寫的 visit_log.db，這一頁只讀不寫。
   ★ 圖表刻意用純 div 畫，不引入圖表函式庫——HMI 是離線服務，
     外部 CDN 在展示現場會直接失敗，而那時你沒有時間查。 */

const TYPE_META: Record<string, { label: string; color: string }> = {
  VIP: { label: "貴賓", color: "var(--color-gold)" },
  GUEST: { label: "訪客", color: "var(--color-blue)" },
  ADMIN: { label: "管理者", color: "var(--color-purple)" },
  BLACKLIST: { label: "黑名單", color: "var(--color-red)" },
};

function Bars({
  values,
  labels,
  color,
  height,
}: {
  values: number[];
  labels: string[];
  color: string;
  height: number;
}) {
  // ★ 尺規至少為 1：全部是 0 時若用 max=0 會除以零，畫出 NaN% 高度的方塊
  const max = Math.max(1, ...values);

  return (
    <div>
      <div className="flex items-end gap-[3px]" style={{ height }}>
        {values.map((v, i) => {
          // 有資料時至少留 3%，否則「1 人次」跟「0 人次」在畫面上看起來一樣
          const h = v > 0 ? Math.max(3, Math.round((v / max) * 100)) : 0;
          return (
            <div key={i} className="flex min-w-[6px] flex-1 flex-col justify-end" title={`${labels[i]}：${v} 人次`}>
              <div
                className="rounded-t-[3px] transition-[height] duration-500"
                style={{
                  height: v > 0 ? `${h}%` : "2px",
                  background: v > 0 ? color : "var(--color-well)",
                  transitionTimingFunction: "var(--ease-ios)",
                }}
              />
            </div>
          );
        })}
      </div>
      <div className="mt-1 flex gap-[3px] text-[10px] text-label-3">
        {labels.map((l, i) => (
          <div key={i} className="min-w-[6px] flex-1 truncate text-center">
            {l}
          </div>
        ))}
      </div>
    </div>
  );
}

export function StatsPage() {
  const [data, refresh] = useFetch<StatsResult | null>(
    () => api<StatsResult>("/api/stats?days=7"),
    null,
  );

  const available = !!data?.available;
  const today = data?.today || {};
  const byType = today.by_type || {};

  const cards = [
    { n: today.count || 0, t: "今日人次", c: "var(--color-blue)" },
    { n: byType.VIP || 0, t: "其中貴賓", c: "var(--color-gold)" },
    { n: data?.unique_people || 0, t: "累計不重複人數", c: "var(--color-label-2)" },
    { n: data?.total || 0, t: "累計總人次", c: "var(--color-label-2)" },
  ];

  const typeNote = Object.entries(byType)
    .map(([k, v]) => `${TYPE_META[k]?.label || k} ${v}`)
    .join("　");

  const days = data?.by_day || [];

  return (
    <div className="grid min-h-0 flex-1 gap-3 lg:grid-cols-2">
      <Card
        title="今日接待"
        className="overflow-y-auto"
        actions={<Button onClick={refresh}>重新整理</Button>}
      >
        {!available ? (
          <>
            <p className="text-[15px] text-label-3">尚無資料</p>
            <Hint className="mt-1">{data?.reason || "讀不到到訪記錄"}</Hint>
          </>
        ) : (
          <>
            <div className="mb-1 flex flex-wrap gap-5">
              {cards.map((c) => (
                <div key={c.t} className="min-w-[96px]">
                  <div className="tnum text-[34px] leading-tight font-bold" style={{ color: c.c }}>
                    {c.n}
                  </div>
                  <div className="text-[12px] text-label-3">{c.t}</div>
                </div>
              ))}
            </div>

            <Hint className="mt-1 mb-4">
              {typeNote
                ? `今日分類：${typeNote}`
                : "今天還沒有人來 —— 到迎賓頁讓相機看到一張已註冊的臉就會出現。"}
            </Hint>

            <h3 className="mb-2 text-[13px] font-semibold text-label-2 uppercase">今日每小時分布</h3>
            <Bars
              values={data?.by_hour || new Array(24).fill(0)}
              labels={[...Array(24).keys()].map((h) => (h % 3 === 0 ? String(h) : ""))}
              color="var(--color-blue)"
              height={110}
            />

            <h3 className="mt-5 mb-2 text-[13px] font-semibold text-label-2 uppercase">近 7 日</h3>
            <Bars
              values={days.map((d) => d.count)}
              labels={days.map((d) => d.date)}
              color="var(--color-teal)"
              height={90}
            />
          </>
        )}
      </Card>

      <Card title="最近到訪" className="overflow-y-auto">
        <div className="flex min-h-0 flex-1 flex-col gap-1.5 overflow-y-auto">
          {!data?.recent?.length && <Empty>還沒有記錄</Empty>}
          {data?.recent?.map((r, i) => {
            const meta = TYPE_META[r.type] || { label: r.type_label || "—", color: "var(--color-label-3)" };
            return (
              <div key={i} className="flex items-center justify-between gap-2.5 px-1 py-1.5">
                <span className="min-w-0 truncate">
                  <b>{r.name}</b>{" "}
                  <span className="text-[12px]" style={{ color: meta.color }}>
                    {meta.label}
                  </span>
                </span>
                <span className="tnum flex-none text-[12px] text-label-3">
                  {r.time}　{r.confidence.toFixed(3)}
                </span>
              </div>
            );
          })}
        </div>

        <Hint className="mt-3 flex-none">
          ★ 相似度是人臉比對的餘弦相似度。實測同一人落在 0.671–0.875、不同人的基線是 0.515
          —— 兩者之間的距離就是這套辨識的安全邊際。
        </Hint>
      </Card>
    </div>
  );
}
