import { Badge, Button, Card, Hint, Row } from "../components/ui";
import { useFetch } from "../hooks/useFetch";
import { api } from "../lib/api";
import type { HardwareItem, HealthNode, HealthResult } from "../lib/types";

function StatusRow({ label, ok, note }: { label: string; ok: boolean; note?: string }) {
  return (
    <Row>
      {/* 「節點存在」不代表能用——lifecycle 節點停在 inactive／unconfigured 時
          節點照樣在圖上，但功能是壞的，所以這裡顯示的是 lifecycle 狀態 */}
      <Badge tone={ok ? "ok" : "bad"}>{ok ? "就緒" : "未就緒"}</Badge>
      <span className="min-w-0 flex-1 truncate font-medium">{label}</span>
      {note && <span className="flex-none text-[12px] text-label-3">{note}</span>}
    </Row>
  );
}

function Heading({ children }: { children: React.ReactNode }) {
  return <div className="mt-2 px-0.5 text-[12px] font-semibold text-label-2">{children}</div>;
}

/** 依 group 分段輸出節點列 */
function NodeGroups({ nodes }: { nodes: HealthNode[] }) {
  const out: React.ReactNode[] = [];
  let group: string | undefined;

  nodes.forEach((n, i) => {
    if (n.group !== group) {
      group = n.group;
      const rows = nodes.filter((x) => x.group === group);
      const ok = rows.filter((x) => x.ok).length;
      out.push(
        <Heading key={`g-${group}-${i}`}>
          {group}（{ok}/{rows.length}）
        </Heading>,
      );
    }
    out.push(<StatusRow key={`${n.name}-${i}`} label={n.name} ok={n.ok} note={n.state} />);
  });

  return <>{out}</>;
}

interface HealthSnapshot {
  health: HealthResult | null;
  hardware: HardwareItem[];
}

export function HealthPage() {
  // 硬體偵測與節點狀態一起更新：它們回答的是同一個問題（這台車現在到底
  // 有什麼在跑），分兩次抓會出現一半新一半舊的畫面
  const [{ health, hardware }, refresh] = useFetch<HealthSnapshot>(async () => {
    const [h, hw] = await Promise.all([
      api<HealthResult>("/api/health"),
      api<{ items?: HardwareItem[] }>("/api/hardware"),
    ]);
    return { health: h, hardware: hw.items || [] };
  }, { health: null, hardware: [] });

  const nodes = health?.nodes || [];
  const piNodes = nodes.filter((n) => n.device === "pi");
  const edgeNodes = nodes.filter((n) => n.device === "edge");
  // 沒在跑的節點談不上在哪台機器，另外列，不塞進任一欄誤導
  const downNodes = nodes.filter((n) => n.device === "down");
  const unknownNodes = nodes.filter((n) => n.device === "unknown");

  return (
    <div className="grid min-h-0 flex-1 gap-3 lg:grid-cols-2">
      <Card
        title={`Pi 4（本機${health?.host ? "：" + health.host : ""}）`}
        className="overflow-y-auto"
        actions={<Button onClick={refresh}>重新整理</Button>}
      >
        <div className="flex flex-col gap-1.5">
          <NodeGroups nodes={piNodes} />

          {/* HMI 自身的能力也算在本機這欄——這個網頁就是這台 Pi 提供的 */}
          <Heading>HMI 介面自身</Heading>
          <StatusRow label="相機影像" ok={!!health?.camera} />
          <StatusRow label="地圖資料" ok={!!health?.map} />
          {Object.entries(health?.services || {}).map(([k, v]) => (
            <StatusRow key={`s-${k}`} label={`服務 ${k}`} ok={v} />
          ))}
          {Object.entries(health?.actions || {}).map(([k, v]) => (
            <StatusRow key={`a-${k}`} label={`動作 ${k}`} ok={v} />
          ))}
        </div>
      </Card>

      <Card title="周邊硬體" className="overflow-y-auto">
        <div className="flex flex-col gap-1.5">
          {hardware.map((h) => {
            // 喇叭缺席是設計，不是故障——用灰色不用紅色，否則操作者會跑去「修」它
            const expectedMissing = h.key === "speaker";
            const dot = h.present ? "🟢" : expectedMissing ? "⚪" : "🔴";
            return (
              <div key={h.key} className="px-0.5 py-1">
                <b className="text-[14px]">
                  {dot} {h.label}
                </b>
                {h.detail && <div className="text-[12px] text-label-3">{h.detail}</div>}
                {h.hint && !h.present && (
                  <div className="text-[12px] text-label-3 opacity-75">{h.hint}</div>
                )}
              </div>
            );
          })}
        </div>

        <Hint className="mt-2 mb-4">
          直接問核心有沒有抓到裝置（相當於 <code>lsusb</code> / <code>arecord -l</code>），不是看設定檔。
          <b className="text-label-2">驅動建置過 ≠ 硬體在車上。</b>
          <br />
          「喇叭」灰色是<b className="text-label-2">預期的</b>——車上刻意不裝，語音走平板瀏覽器。
        </Hint>

        <h3 className="mb-2 text-[13px] font-semibold text-label-2 uppercase">邊緣裝置（筆電）</h3>
        <div className="flex flex-col gap-1.5">
          {edgeNodes.length ? (
            <NodeGroups nodes={edgeNodes} />
          ) : (
            <Heading>目前沒有偵測到其他機器上的節點</Heading>
          )}

          {downNodes.length > 0 && (
            <>
              <Heading>尚未啟動（{downNodes.length}，無法判定裝置）</Heading>
              {downNodes.map((n, i) => (
                <StatusRow key={`d-${i}`} label={n.name} ok={false} note={n.group} />
              ))}
            </>
          )}

          {unknownNodes.length > 0 && (
            <>
              <Heading>內部節點（{unknownNodes.length}，由其他節點建立）</Heading>
              {unknownNodes.map((n, i) => (
                <StatusRow key={`u-${i}`} label={n.name} ok={n.ok} note={n.state} />
              ))}
            </>
          )}
        </div>

        <Hint className="mt-3">
          節點歸屬是比對本機行程判定的：在這台 Pi 找得到對應行程就歸左欄，其餘視為在另一台機器上。
          判定不準的節點可用 <code>pi_nodes</code> / <code>edge_nodes</code> 參數釘死。
          <br />
          顯示「未就緒」代表對應節點沒啟動，不是網頁壞掉——地圖與導航需要
          <code>map_service_node</code>、<code>waypoint_service_node</code> 與 nav2；
          使用者管理需要 <code>user_auth_node</code>，照片註冊另外需要
          <code>face_embedding_node</code>。
        </Hint>
      </Card>
    </div>
  );
}
