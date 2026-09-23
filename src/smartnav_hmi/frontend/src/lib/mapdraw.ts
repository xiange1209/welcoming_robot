import type { MapMeta, NavPath, RobotPose, Waypoint } from "./types";

/* ══════════════════════════════════════════════════════════════
   地圖繪製。純 Canvas，完全不經過 React。

   位姿是 10 Hz 進來的，走 React 等於每秒重建十次元件樹；這裡改成
   直接訂閱 + requestAnimationFrame 重畫，畫面更新率由瀏覽器決定，
   而且分頁被隱藏時 rAF 自動停——連算都不算。

   ★ 這裡的顏色（車身綠、taught 藍／nav2 橘、地點藍…）刻意不接主題引擎
     （lib/theme.ts）。這些是固定圖例，不是介面色：藍／橘代表兩種完全不同
     的執行方式，讓它們跟著種子色變會讓「現在走哪條路徑」這個除錯時第一個
     要看的線索失效。地圖底圖本身也是後端算好的固定灰階 PNG，不受前端主題
     影響。看到這裡沒有 var(--color-*) 不是漏改，是刻意的。
   ══════════════════════════════════════════════════════════════ */

export interface Viewport {
  w: number;
  h: number;
}

export interface Transform {
  scale: number;
  offX: number;
  offY: number;
}

/** 把畫布依 devicePixelRatio 調到實際像素，回傳 CSS 尺寸 */
export function fitCanvas(canvas: HTMLCanvasElement, ctx: CanvasRenderingContext2D): Viewport {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { w, h };
}

/** 目前地圖圖片在畫布上的縮放與位移（等比置中） */
export function mapTransform(view: Viewport, img: HTMLImageElement): Transform {
  const scale = Math.min(view.w / img.width, view.h / img.height);
  return {
    scale,
    offX: (view.w - img.width * scale) / 2,
    offY: (view.h - img.height * scale) / 2,
  };
}

/* 世界座標 ↔ 圖片像素。
   地圖 PNG 在後端已上下翻轉（ROS 柵格 row 0 在下、影像 row 0 在上），
   所以 y 要再翻一次。
   註：假設地圖 origin 的 yaw 為 0——map_saver 產出的地圖一律如此。 */
export function worldToPixel(meta: MapMeta, wx: number, wy: number) {
  return {
    px: (wx - meta.origin_x) / meta.resolution,
    py: meta.height - (wy - meta.origin_y) / meta.resolution,
  };
}

export function pixelToWorld(meta: MapMeta, px: number, py: number) {
  return {
    x: meta.origin_x + px * meta.resolution,
    y: meta.origin_y + (meta.height - py) * meta.resolution,
  };
}

export function worldToCanvas(t: Transform, meta: MapMeta, wx: number, wy: number) {
  const p = worldToPixel(meta, wx, wy);
  return { x: t.offX + p.px * t.scale, y: t.offY + p.py * t.scale };
}

/* ── 車輛圖示 ──────────────────────────────────────────────
   機器人畫成「真實比例的四輪俯視車」而不是箭頭。目的不是好看，是讓
   使用者能**對照現實判斷會不會撞牆**，所以圖示的外框必須是真的車身
   邊界，絕不能為了造型縮小。

   幾何常數（公尺，base_footprint 座標系：x 向前、y 向左）。出處：
     footprint / padding … smartnav_navigation_cc/config/nav2_senior_akm_cc.yaml:491,492
     軸距 / 輪距 / 輪徑  … 廠商韌體 robot_select_init.h:26,31,54

   ★ base_footprint 在**後輪軸心**，不是車體中心：
     車頭在原點前方 0.40 m，車尾只在後方 0.09 m。
     舊的箭頭是「固定 16 px、以原點前後對稱」，既不隨地圖縮放、也沒有這個
     偏移，所以它代表的長度完全不對應任何真實尺寸——而那正好是使用者要
     拿來判斷撞牆的那段距離。 */
export const CAR = {
  rear: -0.09, // 車尾
  front: 0.4, // 車頭（保險桿最前緣）
  halfW: 0.185, // 半車寬
  pad: 0.02, // costmap footprint_padding：規劃器實際多留的餘裕
  wheelbase: 0.322, // 後軸（x=0）→ 前軸
  track: 0.322, // 左右輪中心距
  wheelD: 0.125, // 輪徑 = 俯視圖上輪子的長度
  wheelW: 0.045, // 胎寬：目視估值，只影響外觀，不影響碰撞邊界
} as const;

/** 圓角矩形路徑。刻意不用 ctx.roundRect()——平板瀏覽器版本不確定，arcTo 才是到處都有的 */
function carRRect(
  c2d: CanvasRenderingContext2D,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
  r: number,
): void {
  const xa = Math.min(x0, x1);
  const xb = Math.max(x0, x1);
  const ya = Math.min(y0, y1);
  const yb = Math.max(y0, y1);
  const rr = Math.max(0, Math.min(r, (xb - xa) / 2, (yb - ya) / 2));
  c2d.beginPath();
  c2d.moveTo(xa + rr, ya);
  c2d.arcTo(xb, ya, xb, yb, rr);
  c2d.arcTo(xb, yb, xa, yb, rr);
  c2d.arcTo(xa, yb, xa, ya, rr);
  c2d.arcTo(xa, ya, xb, ya, rr);
  c2d.closePath();
}

/**
 * 畫一台真實比例的俯視車輛。主地圖與遙控頁小地圖共用。
 *
 *   cx,cy  base_footprint 在畫布上的像素位置
 *   yaw    世界座標朝向（rad）
 *   ppm    每公尺對應幾個畫布像素 = 地圖縮放 scale ÷ mapMeta.resolution
 *          （地圖影像 1 px 就是 1 格 = resolution 公尺，所以圖示會跟著
 *            地圖一起縮放，不是固定像素大小）
 *
 * 座標換算：畫布 y 軸向下、世界 y 軸向上，所以旋轉取 -yaw。
 * 在旋轉後的座標系裡，車體局部座標（前 f、左 l）要畫成 (f*ppm, -l*ppm)。
 */
export function drawRobotCar(
  c2d: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  yaw: number,
  ppm: number,
): void {
  const X = (m: number) => m * ppm; // 前後：局部 +x（前）→ 畫布 +x
  const Y = (m: number) => -m * ppm; // 左右：局部 +y（左）→ 畫布 -y（螢幕上方）
  const len = (CAR.front - CAR.rear) * ppm; // 車身在畫面上的長度（像素）

  c2d.save();
  c2d.translate(cx, cy);
  c2d.rotate(-yaw);

  // ① 規劃器真正在用的安全邊界（footprint + padding）。
  //    畫成淡虛線：車身碰到牆之前，是這條線先碰到。
  if (len >= 14) {
    c2d.setLineDash([3, 3]);
    c2d.strokeStyle = "rgba(48,209,88,0.45)";
    c2d.lineWidth = 1;
    carRRect(
      c2d,
      X(CAR.rear - CAR.pad),
      Y(CAR.halfW + CAR.pad),
      X(CAR.front + CAR.pad),
      Y(-(CAR.halfW + CAR.pad)),
      X(0.03),
    );
    c2d.stroke();
    c2d.setLineDash([]);
  }

  // ② 車身：填滿**真實 footprint**，圓角純粹是造型（只切掉四角各數公釐）
  c2d.fillStyle = "rgba(48,209,88,0.88)";
  carRRect(c2d, X(CAR.rear), Y(CAR.halfW), X(CAR.front), Y(-CAR.halfW), X(0.05));
  c2d.fill();

  // ③ 座艙：往車頭收窄的梯形，讓人一眼看出哪一頭是車頭
  if (len >= 22) {
    c2d.fillStyle = "rgba(9,32,23,0.55)";
    c2d.beginPath();
    c2d.moveTo(X(0.27), Y(0.095));
    c2d.lineTo(X(0.27), Y(-0.095));
    c2d.lineTo(X(0.11), Y(-0.145));
    c2d.lineTo(X(0.11), Y(0.145));
    c2d.closePath();
    c2d.fill();
  }

  // ④ 四個輪子。後軸就在原點（base_footprint = 後輪軸心），前軸在 wheelbase。
  //    輪距 0.322 < 車寬 0.37，所以輪子整個在車身底下——畫在車身之上才看得見。
  //    轉向角前端拿不到（後端 state 沒有這個欄位），一律畫直的。
  if (len >= 18) {
    c2d.fillStyle = "#0e1418";
    for (const ax of [0, CAR.wheelbase]) {
      for (const sy of [1, -1]) {
        carRRect(
          c2d,
          X(ax - CAR.wheelD / 2),
          Y((sy * CAR.track) / 2 + CAR.wheelW / 2),
          X(ax + CAR.wheelD / 2),
          Y((sy * CAR.track) / 2 - CAR.wheelW / 2),
          X(0.012),
        );
        c2d.fill();
      }
    }
  }

  // ⑤ 真實 footprint 的精確矩形外框（直角，不圓角）。
  //    ② 的圓角會把四個角各切掉幾公釐，判斷貼牆時一律以這條線為準。
  c2d.strokeStyle = "#eafff4";
  c2d.lineWidth = Math.max(1, X(0.012));
  c2d.beginPath();
  c2d.rect(X(CAR.rear), Y(CAR.halfW), X(CAR.front - CAR.rear), X(2 * CAR.halfW));
  c2d.stroke();

  // ⑥ 車頭橫桿：位置就是保險桿最前緣，撞牆時第一個碰到的地方
  const noseT = Math.max(1, X(0.035));
  c2d.fillStyle = "#ffffff";
  c2d.beginPath();
  c2d.rect(X(CAR.front) - noseT, Y(CAR.halfW * 0.78), noseT, X(2 * CAR.halfW * 0.78));
  c2d.fill();

  // ⑦ base_footprint 原點（後輪軸心）。狀態列那組座標指的就是這一點，
  //    不是車體中心——標出來使用者才對得上。
  if (len >= 18) {
    c2d.fillStyle = "#0b3d28";
    c2d.beginPath();
    c2d.arc(0, 0, Math.max(1.5, X(0.028)), 0, Math.PI * 2);
    c2d.fill();
    c2d.strokeStyle = "rgba(255,255,255,0.85)";
    c2d.lineWidth = 1;
    c2d.stroke();
  }

  c2d.restore();
}

/** 代表位置與朝向的箭頭。畫布 y 軸向下，所以角度取負。現在只給導航目標點用 */
export function drawArrow(
  c2d: CanvasRenderingContext2D,
  x: number,
  y: number,
  yaw: number,
  color: string,
  size: number,
): void {
  c2d.save();
  c2d.translate(x, y);
  c2d.rotate(-yaw);
  c2d.fillStyle = color;
  c2d.beginPath();
  c2d.moveTo(size, 0);
  c2d.lineTo(-size * 0.6, size * 0.55);
  c2d.lineTo(-size * 0.25, 0);
  c2d.lineTo(-size * 0.6, -size * 0.55);
  c2d.closePath();
  c2d.fill();
  c2d.restore();
}

export interface SceneInput {
  image: HTMLImageElement | null;
  meta: MapMeta | null;
  pose: RobotPose | null;
  navPath: NavPath | null;
  waypoints: Waypoint[];
  picked: { x: number; y: number; yaw: number } | null;
  /** 小地圖不畫地點名稱與目標點，只要「地圖 + 車在哪」 */
  minimal?: boolean;
}

/**
 * 畫一整幀，回傳要顯示在角落的提示文字。
 * 回傳字串而不是自己寫 DOM，是為了讓呼叫端決定要不要更新（省一次 layout）。
 */
export function drawScene(
  canvas: HTMLCanvasElement,
  ctx: CanvasRenderingContext2D,
  input: SceneInput,
): string {
  const view = fitCanvas(canvas, ctx);
  ctx.clearRect(0, 0, view.w, view.h);

  const { image, meta } = input;
  if (!image || !meta) {
    return "等待地圖…（需啟動 SLAM 建圖，或切換到已存在的地圖）";
  }

  const t = mapTransform(view, image);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(image, t.offX, t.offY, image.width * t.scale, image.height * t.scale);

  // 地點標記
  if (!input.minimal) {
    ctx.font = '12px -apple-system, "PingFang TC", sans-serif';
    for (const w of input.waypoints) {
      const c = worldToCanvas(t, meta, w.x, w.y);
      ctx.fillStyle = "#0a84ff";
      ctx.beginPath();
      ctx.arc(c.x, c.y, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "#f2f5f8";
      ctx.fillText(w.waypoint_name, c.x + 9, c.y + 4);
    }
  }

  /* ── 規劃路徑 ──────────────────────────────────────────
     按下導航或開始重播時，車子打算怎麼走就畫在這裡。
     畫在地點與機器人**之前**，這樣路徑不會蓋住那兩個更重要的標記。

     兩個來源用不同顏色，因為它們代表完全不同的執行方式，而「現在到底
     走哪一條」是除錯時第一個要確認的事：
         taught（藍）= nav2 規劃 + 純追蹤執行  ← 主力
         nav2 （橘）= nav2 規劃 + MPPI 執行    ← 備援 */
  const path = input.navPath;
  if (path?.points && path.points.length > 1) {
    const isTaught = path.source === "taught";
    ctx.save();
    // 先畫一條半透明粗線當底，再畫細實線——地圖是灰階的，
    // 單一細線在深色格子上會看不見。
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.strokeStyle = isTaught ? "rgba(10,132,255,0.35)" : "rgba(255,159,10,0.35)";
    ctx.lineWidth = 7;
    ctx.beginPath();
    path.points.forEach((p, i) => {
      const c = worldToCanvas(t, meta, p[0], p[1]);
      if (i === 0) ctx.moveTo(c.x, c.y);
      else ctx.lineTo(c.x, c.y);
    });
    ctx.stroke();
    ctx.strokeStyle = isTaught ? "#0a84ff" : "#ff9f0a";
    ctx.lineWidth = 2;
    ctx.stroke();

    // 終點畫一個小圈：路徑末端在轉折多的地方很難一眼看出是哪一頭
    const last = path.points[path.points.length - 1];
    const lc = worldToCanvas(t, meta, last[0], last[1]);
    ctx.fillStyle = isTaught ? "#0a84ff" : "#ff9f0a";
    ctx.beginPath();
    ctx.arc(lc.x, lc.y, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  // 已選的點（含朝向）
  if (input.picked && !input.minimal) {
    const c = worldToCanvas(t, meta, input.picked.x, input.picked.y);
    drawArrow(ctx, c.x, c.y, input.picked.yaw, "#ffd60a", 20);
  }

  // 機器人。每公尺像素數 = 地圖縮放 ÷ 解析度
  // （t.scale 是「1 個地圖影像像素佔幾個畫布像素」，而 1 個影像像素 = 1 格 = resolution 公尺）
  if (input.pose) {
    const c = worldToCanvas(t, meta, input.pose.x, input.pose.y);
    drawRobotCar(ctx, c.x, c.y, input.pose.yaw, t.scale / meta.resolution);
  }

  const meters = (meta.width * meta.resolution).toFixed(1);
  const pathHint =
    path?.points && path.points.length > 1
      ? `　路徑 ${path.points.length} 點（${path.source === "taught" ? "純追蹤" : "MPPI"}）`
      : "";
  return (
    `${meta.width}×${meta.height} @ ${meta.resolution} m/px（約 ${meters} m 寬）　` +
    (input.pose
      ? `機器人 (${input.pose.x.toFixed(2)}, ${input.pose.y.toFixed(2)})`
      : "機器人位置未知") +
    pathHint
  );
}

/** 指標事件座標 → 世界座標（超出地圖範圍回 null） */
export function eventToWorld(
  canvas: HTMLCanvasElement,
  ctx: CanvasRenderingContext2D,
  image: HTMLImageElement | null,
  meta: MapMeta | null,
  ev: { clientX: number; clientY: number },
): { x: number; y: number } | null {
  if (!image || !meta) return null;
  const view = fitCanvas(canvas, ctx);
  const t = mapTransform(view, image);
  const rect = canvas.getBoundingClientRect();
  const px = (ev.clientX - rect.left - t.offX) / t.scale;
  const py = (ev.clientY - rect.top - t.offY) / t.scale;
  if (px < 0 || py < 0 || px > image.width || py > image.height) return null;
  return pixelToWorld(meta, px, py);
}
