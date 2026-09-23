import { useEffect, useState } from "react";
import { MapCanvas, type PickedPose } from "../components/MapCanvas";
import {
  Badge,
  Button,
  Card,
  Empty,
  Field,
  List,
  Row,
  Segmented,
  Select,
} from "../components/ui";
import { useStoreSelector, useStoreValue } from "../hooks/useStore";
import { del, post, run } from "../lib/api";
import { confirmDialog, promptDialog } from "../lib/dialog";
import {
  mapsStore,
  refreshMaps,
  refreshWaypoints,
  reloadMapImage,
  waypointsStore,
} from "../lib/mapdata";
import { telemetryStore } from "../lib/telemetry";
import { toast } from "../lib/toast";
import type { Job } from "../lib/types";

type Mode = "view" | "waypoint" | "navigate";

const MODES: { value: Mode; label: string }[] = [
  { value: "view", label: "檢視" },
  { value: "waypoint", label: "建立地點" },
  { value: "navigate", label: "導航到此" },
];

/** 進行中的作業。由推播驅動，所以取消之後會自己更新，不必手動重整 */
function JobList() {
  const jobs = useStoreSelector(
    telemetryStore,
    (t) => t.snap.jobs ?? EMPTY_JOBS,
    (a, b) =>
      a.length === b.length &&
      a.every(
        (j, i) =>
          j.job_id === b[i].job_id &&
          j.status === b[i].status &&
          j.message === b[i].message,
      ),
  );

  // 新的排前面
  const items = [...jobs].reverse();
  if (!items.length) return <Empty>目前沒有作業。</Empty>;

  return (
    <>
      {items.map((j) => {
        const running =
          j.status === "running" ||
          j.status === "pending" ||
          j.status === "cancelling";
        return (
          <Row key={j.job_id}>
            <Badge
              tone={running ? "run" : j.status === "succeeded" ? "ok" : "bad"}
            >
              {j.status}
            </Badge>
            <span className="min-w-0 flex-1">
              <span className="font-semibold">{j.label}</span>
              <br />
              <span className="text-[12px] text-label-3">
                {j.message || ""}
              </span>
            </span>
            {running && (
              <Button
                variant="danger"
                onClick={() => void run(del(`/api/jobs/${j.job_id}`))}
              >
                取消
              </Button>
            )}
          </Row>
        );
      })}
    </>
  );
}

const EMPTY_JOBS: Job[] = [];

export function MapPage({ active }: { active: boolean }) {
  const [mode, setMode] = useState<Mode>("view");
  const [picked, setPicked] = useState<PickedPose | null>(null);
  const [newMapName, setNewMapName] = useState("");
  const [pickedMap, setSelectedMap] = useState("");

  const waypoints = useStoreValue(waypointsStore);
  const maps = useStoreValue(mapsStore);

  /* 清單是非同步載入的，所以「還沒選」與「選了但那張圖被刪了」都要退回第一張。
     用衍生值而不是 effect：effect 會先繪一次空的下拉選單再補上。 */
  const selectedMap =
    pickedMap && maps.some((m) => m.map_id === pickedMap)
      ? pickedMap
      : (maps[0]?.map_id ?? "");

  useEffect(() => {
    if (!active) return;
    void refreshMaps();
    void refreshWaypoints();
  }, [active]);

  /** 放開手指時才動作：拖曳的過程是在指定朝向，中途不該送出任何請求 */
  const commitPick = async () => {
    if (!picked || mode === "view") return;

    if (mode === "waypoint") {
      const name = await promptDialog({
        title: "這個地點叫什麼名字？",
        message: `座標 (${picked.x.toFixed(2)}, ${picked.y.toFixed(2)})`,
        placeholder: "例如：理財專區",
      });
      if (!name) return;
      await run(
        post("/api/waypoints", {
          waypoint_name: name,
          use_given_pose: true,
          x: picked.x,
          y: picked.y,
          yaw: picked.yaw,
        }),
        () => void refreshWaypoints(),
      );
      return;
    }

    const ok = await confirmDialog({
      title: "確定導航到這個位置？",
      message: `(${picked.x.toFixed(2)}, ${picked.y.toFixed(2)}) 朝向 ${((picked.yaw * 180) / Math.PI).toFixed(0)}°`,
      confirmLabel: "出發",
    });
    if (!ok) return;
    await run(
      post("/api/navigate", {
        x: picked.x,
        y: picked.y,
        yaw: picked.yaw,
        target_name: "指定位置",
      }),
    );
  };

  return (
    <div className="grid min-h-0 flex-1 gap-3 xl:grid-cols-[1.5fr_1fr]">
      <Card
        title="地圖"
        actions={
          <Button
            onClick={() => {
              void refreshWaypoints();
              reloadMapImage();
            }}
          >
            重新整理
          </Button>
        }
      >
        <div className="mb-2.5 flex flex-none flex-wrap items-center gap-2">
          <Segmented
            className="min-w-[260px] flex-1"
            value={mode}
            options={MODES}
            onChange={setMode}
          />
          <span className="tnum text-[12px] text-label-3">
            {picked
              ? `已選 (${picked.x.toFixed(2)}, ${picked.y.toFixed(2)}) 朝向 ${((picked.yaw * 180) / Math.PI).toFixed(0)}°`
              : "尚未選點"}
          </span>
        </div>

        <MapCanvas
          active={active}
          className="min-h-[240px] flex-1"
          picked={picked}
          // 檢視模式下不給選點：地圖頁最常做的事是「看車子在哪」，
          // 手指碰一下就跳出導航確認會嚇到人
          onPick={
            mode === "view" ? undefined : (pose) => pose && setPicked(pose)
          }
          onPickEnd={() => void commitPick()}
        />

        {mode !== "view" && (
          <p className="mt-2 flex-none text-[12px] text-label-3">
            點一下選位置，按住拖曳（超過 10 公分）可設定朝向。
          </p>
        )}
      </Card>

      <Card title="地點" className="overflow-y-auto" bodyClassName="gap-4">
        <div className="flex min-h-[120px] flex-col gap-1.5">
          {waypoints.length === 0 && (
            <Empty>
              尚無地點。切到「建立地點」模式後在地圖上點一下即可新增。
            </Empty>
          )}
          {waypoints.map((w) => (
            <Row key={w.waypoint_id}>
              <span className="min-w-0 flex-1">
                <span className="font-semibold">{w.waypoint_name}</span>
                <br />
                <span className="tnum text-[12px] text-label-3">
                  ({w.x.toFixed(2)}, {w.y.toFixed(2)})
                </span>
              </span>
              <Button
                variant="primary"
                onClick={() =>
                  void run(
                    post("/api/navigate", {
                      waypoint_id: w.waypoint_id,
                      target_name: w.waypoint_name,
                    }),
                  )
                }
              >
                導航
              </Button>
              {/* 刪除要跟「導航」隔開來看：這兩顆並排、誤觸的代價差很多，
                  所以刪除走確認並把座標一起寫進提示，讓人確認刪的是不是同名的另一個點。 */}
              <Button
                variant="danger"
                onClick={async () => {
                  const ok = await confirmDialog({
                    title: `刪除地點「${w.waypoint_name}」？`,
                    message: `(${w.x.toFixed(2)}, ${w.y.toFixed(2)})　刪除後無法復原。`,
                    confirmLabel: "刪除",
                    destructive: true,
                  });
                  if (!ok) return;
                  await run(
                    del(`/api/waypoints/${encodeURIComponent(w.waypoint_id)}`),
                    () =>
                      // 重新拉一次，順便把地圖上的標記清掉
                      void refreshWaypoints(),
                  );
                }}
              >
                刪除
              </Button>
            </Row>
          ))}
        </div>

        <div>
          <h3 className="mb-2 text-[13px] font-semibold text-label-2 uppercase">
            地圖管理
          </h3>
          <div className="flex flex-col gap-2">
            <div className="flex gap-2">
              <Select
                className="min-w-0 flex-1"
                value={selectedMap}
                onChange={(e) => setSelectedMap(e.target.value)}
              >
                {maps.map((m) => (
                  <option key={m.map_id} value={m.map_id}>
                    {m.map_name}
                  </option>
                ))}
              </Select>
              <Button
                onClick={() => {
                  if (!selectedMap) {
                    toast.error("沒有可切換的地圖");
                    return;
                  }
                  void run(
                    post("/api/maps/switch", { map_id: selectedMap }),
                    () => void refreshWaypoints(),
                  );
                }}
              >
                切換
              </Button>
            </div>

            <div className="flex gap-2">
              <Field
                className="min-w-0 flex-1"
                placeholder="新地圖名稱"
                value={newMapName}
                onChange={(e) => setNewMapName(e.target.value)}
              />
              <Button
                onClick={() => {
                  const name = newMapName.trim();
                  if (!name) {
                    toast.error("請先輸入地圖名稱");
                    return;
                  }
                  void run(post("/api/maps/create", { map_name: name }), () =>
                    setNewMapName(""),
                  );
                }}
              >
                開始建圖
              </Button>
            </div>

            <Button block onClick={() => void run(post("/api/localize"))}>
              全域定位
            </Button>
          </div>
        </div>

        <div>
          <h3 className="mb-2 text-[13px] font-semibold text-label-2 uppercase">
            進行中的作業
          </h3>
          <List className="max-h-[180px] flex-none">
            <JobList />
          </List>
        </div>
      </Card>
    </div>
  );
}
