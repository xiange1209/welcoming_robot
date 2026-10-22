// HMI 視覺檢查（筆電端）：用 headless Chrome（CDP）以 iPad 尺寸渲染各頁並截圖，後端用假資料頂替——沒有 iPad、沒有 Pi 也能看版面。
// 2026-10-22 寫成，用來驗 iPad 直放版面、訪客視角（未登入的 VIP／訪客／黑名單）、版本橫幅與閒置自動重載。
//
// 用法（要 Node 22+ 與 Chrome；先 npm run build 讓 frontend/dist 是新的）：
//   node maprun/tools_1021/hmi_visual_check.mjs <frontend/dist> <輸出資料夾> <chrome.exe> [選項]
// 選項：
//   --landscape        另外渲染橫放 1180×820（確認沒有退步）
//   --wide             另外渲染「寬直放」1100×1400（確認 max-lg／max-xl 沒有把兩欄壓壞）
//   --visitor=TYPE     未登入的訪客視角，TYPE＝VIP|GUEST|BLACKLIST（身分遮蔽與帶位按鈕分級）
//   --only=greet,stats 只截指定分頁（greet|stats|map|teleop|system|users|health|voice）
//   --banner           實測版本橫幅：換版後出現、閒置約 45 秒自動重載、同版本不重複重載、簽章還原後消失（約 2.5 分鐘）
//   --banner-only      只做橫幅測試、不截各頁
// ★ 看到的是假資料版面：影像是空白圖、地圖是產生的示意圖。看比例、捲動、遮蔽與按鈕分級，不是看真實內容。
// ★ 除錯埠固定 9333；腳本異常中斷時可能留下 headless Chrome，用 --remote-debugging-port=9333 比對後關掉那一個。
// 用 headless Chrome（CDP）以 iPad 直放尺寸（820×1180）渲染 HMI 各頁並截圖；後端用假資料頂替。
// 用法：node hmi_shots.mjs <frontend/dist> <輸出資料夾> <chrome.exe> [--banner] [--landscape]
import http from "node:http";
import fs from "node:fs";
import path from "node:path";
import zlib from "node:zlib";
import os from "node:os";
import { spawn } from "node:child_process";

const [DIST, OUT, CHROME] = process.argv.slice(2, 5);
const BANNER = process.argv.includes("--banner");
const LANDSCAPE = process.argv.includes("--landscape");
const WIDE = process.argv.includes("--wide");
const VISITOR = (process.argv.find((a) => a.startsWith("--visitor=")) || "").split("=")[1] || "";
const ONLY = ((process.argv.find((a) => a.startsWith("--only=")) || "").split("=")[1] || "").split(",").filter(Boolean);
fs.mkdirSync(OUT, { recursive: true });

/* ── 假圖片（純 JS 產生 PNG） ─────────────────────────────── */
function png(w, h, paint) {
  const raw = Buffer.alloc((w * 3 + 1) * h);
  for (let y = 0; y < h; y++) {
    raw[y * (w * 3 + 1)] = 0;
    for (let x = 0; x < w; x++) {
      const [r, g, b] = paint(x, y);
      const o = y * (w * 3 + 1) + 1 + x * 3;
      raw[o] = r; raw[o + 1] = g; raw[o + 2] = b;
    }
  }
  const chunk = (type, data) => {
    const len = Buffer.alloc(4); len.writeUInt32BE(data.length);
    const td = Buffer.concat([Buffer.from(type), data]);
    const crc = Buffer.alloc(4); crc.writeUInt32BE(zlib.crc32(td) >>> 0);
    return Buffer.concat([len, td, crc]);
  };
  const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4); ihdr[8] = 8; ihdr[9] = 2;
  return Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk("IHDR", ihdr), chunk("IDAT", zlib.deflateSync(raw)), chunk("IEND", Buffer.alloc(0))]);
}
const FRAME = png(640, 480, (x, y) => { const d = Math.hypot(x - 320, y - 220); return d < 90 ? [200, 170, 140] : [30 + (y >> 4), 40 + (x >> 5), 55]; });
const MAP = png(264, 157, (x, y) => ((x < 6 || y < 6 || x > 257 || y > 150 || (x > 100 && x < 106 && y < 100) || (y > 80 && y < 86 && x > 150)) ? [20, 20, 20] : (x + y) % 40 < 1 ? [90, 90, 90] : [225, 225, 225]));

/* ── 假資料 ────────────────────────────────────────────────── */
const html = fs.readFileSync(path.join(DIST, "index.html"), "utf8");
function sigOf(h) {
  const names = new Set();
  for (const m of h.matchAll(/<(script|link)\b([^>]*)>/gi)) {
    const [, tag, a] = m;
    const attr = (n) => (a.match(new RegExp(`\\b${n}\\s*=\\s*["']([^"']+)["']`, "i")) || [])[1] || "";
    let u = "";
    if (tag.toLowerCase() === "script" && /type\s*=\s*["']module["']/i.test(a)) u = attr("src");
    else if (tag.toLowerCase() === "link" && /rel\s*=\s*["']stylesheet["']/i.test(a)) u = attr("href");
    u = u.split("?")[0].split("#")[0];
    if (u.includes("/assets/")) names.add(u.slice(u.lastIndexOf("/") + 1));
  }
  return [...names].sort().join("|");
}
let serverSig = sigOf(html);
const API = {
  "/api/session": () => ({ success: true, admin: !VISITOR }),
  "/api/maps": () => ({ success: true, maps: [{ map_id: "map_1dda0e955545", map_name: "測試20260929" }, { map_id: "map_127f250e543f", map_name: "實驗室舊圖" }, { map_id: "map_6aa732f181c6", map_name: "8月的圖" }] }),
  "/api/waypoints": () => ({ success: true, waypoints: [["起點", 3.6, -0.9], ["門口", -1.3, -0.8], ["櫃台", 0.97, -1.53], ["廁所", 4.15, -2.2], ["大廳", 3.89, -0.94], ["貴賓室", -1.25, -0.72]].map(([n, x, y], i) => ({ waypoint_id: "w" + i, waypoint_name: n, x, y, yaw: 0 })) }),
  "/api/maps/status": () => ({ success: true, mode: "localization", current_map: "map_1dda0e955545", mapping_job: null }),
  "/api/paths": () => ({ success: true, paths: [["門口→起點", 6.2, 120, 1], ["起點→櫃台", 5.1, 98, 0], ["起點→廁所", 7.4, 140, 2]].map(([n, l, p, c], i) => ({ path_id: "p" + i, name: n, length_m: l, num_points: p, num_cusps: c, source: "drive" })) }),
  "/api/users": () => ({ success: true, users: [["王小明", 1, "VIP", "貴賓客戶", 10], ["李大華", 0, "GUEST", "", 8], ["測試管理員", 2, "ADMIN", "行員", 10], ["黑名單A", 3, "BLACKLIST", "測試用", 6], ["陳先生", 1, "VIP", "", 10], ["林小姐", 0, "GUEST", "", 7]].map(([n, t, tn, d, s], i) => ({ user_uuid: "u" + i, user_name: n, user_type: t, user_type_name: tn, description: d, num_samples: s })) }),
  "/api/stats": () => ({ success: true, available: true, today: { count: 14, by_type: { VIP: 5, GUEST: 8, ADMIN: 1 } }, unique_people: 9, total: 120, by_hour: Array.from({ length: 24 }, (_, h) => (h > 8 && h < 17 ? 1 + ((h * 7) % 5) : 0)), by_day: Array.from({ length: 7 }, (_, d) => ({ date: `10/${16 + d}`, count: 5 + ((d * 5) % 11) })), recent: Array.from({ length: 14 }, (_, i) => ({ name: ["王小明", "李大華", "陳先生", "林小姐"][i % 4], type: ["VIP", "GUEST", "VIP", "GUEST"][i % 4], type_label: ["貴賓", "訪客", "貴賓", "訪客"][i % 4], time: `10/22 ${9 + (i >> 1)}:${(i % 2) * 30 || "00"}`, confidence: 0.7 + (i % 5) / 50 })) }),
  "/api/health": () => ({ success: true, host: "wheeltec-r680", camera: true, map: true, nodes: ["camera", "face_node", "user_auth_node", "llm_node", "hmi_server_node", "controller_server", "bt_navigator", "amcl", "map_server", "wheeltec_robot"].map((n, i) => ({ name: n, ok: i !== 3, state: i !== 3 ? "active" : "down", group: i < 3 ? "視覺" : "導航", device: i === 3 ? "down" : "pi" })), services: { register_face: true, list_users: true }, actions: { navigate_to_pose: true } }),
  "/api/hardware": () => ({ success: true, items: [["astra", "Astra S 相機＋麥克風", true], ["lidar", "N10 雷達", true], ["chassis", "底盤 STM32", true], ["imu", "IMU", true], ["speaker", "喇叭", false]].map(([k, l, p]) => ({ key: k, label: l, present: p, detail: p ? "/dev/xxx" : "", hint: p ? "" : "不採購" })) }),
  "/api/scenarios": () => ({ success: true, scenarios: [["greet_full", "完整迎賓故事線", "相機＋人臉＋語音＋LLM＋導航"], ["face_test", "測人臉辨識／註冊", "只開相機與人臉"], ["asr_test", "測語音辨識", "只開語音"], ["llm_test", "測 LLM 對話", "只開 LLM"], ["nav_test", "測導航", "底盤＋雷達＋導航"]].map(([k, l, w]) => ({ key: k, label: l, why: w, start: ["camera", "face"], stop: ["nav"], note: "" })) }),
  "/api/verify": () => ({ success: true, items: [["v0", "V0 開工前檢查", "42 項"], ["v1", "V1 相機", "相機幀率"]].map(([k, l, w]) => ({ key: k, label: l, why: w, runnable: true, script: k + ".sh" })) }),
  "/api/system/status": () => ({ success: true, units: [["sensors", "底盤＋雷達＋IMU", true], ["nav", "導航堆疊", true], ["camera", "相機", true], ["face", "人臉辨識", false], ["user_auth", "使用者認證", false], ["asr_chain", "語音輸入", false], ["llm", "LLM 對話", true], ["brain", "迎賓劇本", true], ["tts", "朗讀", false], ["explore", "自動探索", false], ["telegram", "Telegram 通報", false], ["hmi", "HMI", true]].map(([k, l, r]) => ({ key: k, label: l, hint: "說明文字範例", running: r, partial: false, blocked: "", variants: k === "nav" ? [{ key: "localization", label: "定位＋導航" }, { key: "mapping", label: "建圖" }] : [] })) }),
  "/api/version": () => ({ sig: serverSig }),
  "/api/guide/destinations": () => ({ success: true, can_navigate: VISITOR !== "BLACKLIST", tier: VISITOR === "VIP" || !VISITOR ? "vip" : "guest", message: "", busy: false,
     destinations: [["w2", "櫃台", false], ["w4", "大廳", false], ["w1", "門口", false], ...(VISITOR === "VIP" || !VISITOR ? [["w5", "貴賓室", true]] : [])].map(([id, n, v]) => ({ waypoint_id: id, name: n, vip_only: v })) }),
};
const server = http.createServer((req, res) => {
  const u = new URL(req.url, "http://x");
  if (u.pathname === "/" || u.pathname === "/index.html") { res.writeHead(200, { "Content-Type": "text/html; charset=utf-8", "Cache-Control": "no-store" }); return res.end(html); }
  if (u.pathname.startsWith("/assets/")) {
    const f = path.join(DIST, u.pathname);
    if (fs.existsSync(f)) { res.writeHead(200, { "Content-Type": f.endsWith(".css") ? "text/css" : "text/javascript" }); return res.end(fs.readFileSync(f)); }
  }
  if (u.pathname === "/api/map.png") { res.writeHead(200, { "Content-Type": "image/png" }); return res.end(MAP); }
  if (u.pathname === "/api/frame.jpg") { res.writeHead(200, { "Content-Type": "image/png" }); return res.end(FRAME); }
  if (u.pathname === "/__set_sig") { serverSig = u.searchParams.get("v") || sigOf(html); res.writeHead(200); return res.end(serverSig); }
  const h = API[u.pathname];
  res.writeHead(200, { "Content-Type": "application/json" });
  res.end(JSON.stringify(h ? h() : { success: true }));
});
await new Promise((r) => server.listen(0, "127.0.0.1", r));
const PORT = server.address().port;

/* ── Chrome + CDP ──────────────────────────────────────────── */
const profile = fs.mkdtempSync(path.join(os.tmpdir(), "hmi-shot-"));
const chrome = spawn(CHROME, ["--headless=new", "--remote-debugging-port=9333", `--user-data-dir=${profile}`, "--no-first-run", "--no-default-browser-check", "--disable-gpu", "--force-device-scale-factor=1", "about:blank"], { stdio: "ignore" });
let wsUrl = "";
for (let i = 0; i < 60 && !wsUrl; i++) {
  await new Promise((r) => setTimeout(r, 500));
  try {
    const list = await (await fetch("http://127.0.0.1:9333/json/list")).json();
    wsUrl = (list.find((t) => t.type === "page") || {}).webSocketDebuggerUrl || "";
  } catch { /* 還沒起來 */ }
}
if (!wsUrl) { console.error("Chrome 沒有起來"); chrome.kill(); process.exit(1); }
const ws = new WebSocket(wsUrl);
await new Promise((r) => (ws.onopen = r));
let nid = 0; const pend = new Map(); const logs = [];
ws.onmessage = (m) => {
  const d = JSON.parse(m.data);
  if (d.id && pend.has(d.id)) { pend.get(d.id)(d); pend.delete(d.id); return; }
  if (d.method === "Runtime.exceptionThrown") logs.push("EXC " + (d.params.exceptionDetails.exception?.description || d.params.exceptionDetails.text).slice(0, 300));
  if (d.method === "Runtime.consoleAPICalled" && d.params.type === "error") logs.push("ERR " + d.params.args.map((a) => a.value ?? a.description).join(" ").slice(0, 300));
};
const cdp = (method, params = {}) => new Promise((res) => { const id = ++nid; pend.set(id, res); ws.send(JSON.stringify({ id, method, params })); });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const ev = async (expr) => (await cdp("Runtime.evaluate", { expression: expr, returnByValue: true, awaitPromise: true })).result?.result?.value;
const shot = async (name) => {
  const r = await cdp("Page.captureScreenshot", { format: "png" });
  fs.writeFileSync(path.join(OUT, name + ".png"), Buffer.from(r.result.data, "base64"));
  console.log("  截圖", name);
};

await cdp("Page.enable"); await cdp("Runtime.enable");
const FAKE_WS = `
  ${VISITOR ? '' : "sessionStorage.setItem('hmiToken','x');"}
  window.__snap = () => { const n = Date.now()/1000; return {
    version: Math.floor(n), now: n,
    identity: { user_name:'${VISITOR === "BLACKLIST" ? "黑名單A" : VISITOR === "GUEST" ? "李大華" : "王小明"}', user_type:'${VISITOR || "VIP"}', recognized:true, similarity:0.78, description:'貴賓客戶', updated_at:n, bbox:[100,100,220,240] },
    messages: [
      {role:'robot', text:'王小明貴賓您好，歡迎蒞臨本行，請問今天需要什麼協助？', ts:n-60},
      {role:'user', text:'請問銀行幾點營業？', ts:n-50},
      {role:'robot', text:'本行週一到週五上午九點到下午三點半營業。', ts:n-48, stats:{total:0.74,think:0.2,generate:0.5,cps:30,chars:20}},
      {role:'user', text:'帶我去貴賓室', ts:n-30},
      {role:'robot', text:'好的，請跟我來。', ts:n-28},
      {role:'robot', text:'遇到可疑情況請找櫃檯行員或撥打 165 反詐騙專線。', ts:n-10},
    ],
    system: { camera_fps:5, voltage:24.49, speed:0, cpu_temp:59, cpu_load:1.2, map_id:'map_1dda0e955545', llm_model:'qwen2.5:3b' },
    map_meta: { version:1, width:264, height:157, resolution:0.05, origin_x:-6, origin_y:-4 },
    robot_pose: { x:1, y:-0.5, yaw:0.5 }, nav_path: null, jobs: [], registration: null }; };
  window.WebSocket = class FakeWS {
    static CONNECTING=0; static OPEN=1; static CLOSING=2; static CLOSED=3;
    constructor(url){ this.url=url; this.readyState=0; setTimeout(()=>{ this.readyState=1; this.onopen&&this.onopen({}); const send=()=>this.onmessage&&this.onmessage({data:JSON.stringify(window.__snap())}); send(); this._t=setInterval(send,1000); },50); }
    send(){} close(){ clearInterval(this._t); this.readyState=3; this.onclose&&this.onclose({}); }
  };`;
await cdp("Page.addScriptToEvaluateOnNewDocument", { source: FAKE_WS });

const TABS = [["greet", "迎賓"], ["stats", "到訪統計"], ["map", "地圖與導航"], ["teleop", "遙控建圖"], ["system", "系統開關"], ["users", "使用者管理"], ["health", "系統"], ["voice", "語音測試"]];
const clickTab = (label) => ev(`(() => { const b=[...document.querySelectorAll('button')].find(b=>[...b.querySelectorAll('span')].some(s=>s.textContent.trim()==='${label}')) ; if(!b) return 'NOTFOUND'; b.click(); return 'ok'; })()`);

async function run(w, h, prefix) {
  await cdp("Emulation.setDeviceMetricsOverride", { width: w, height: h, deviceScaleFactor: 1, mobile: true, screenOrientation: { type: w < h ? "portraitPrimary" : "landscapePrimary", angle: 0 } });
  await cdp("Page.navigate", { url: `http://127.0.0.1:${PORT}/` });
  await sleep(2500);
  for (const [id, label] of TABS) {
    if (ONLY.length && !ONLY.includes(id)) continue;
    const r = await clickTab(label);
    if (r !== "ok") console.log("  找不到分頁按鈕", label);
    await sleep(1600);
    await shot(`${prefix}_${id}`);
  }
}
if (!process.argv.includes("--banner-only")) {
  await run(820, 1180, "portrait");
  if (LANDSCAPE) await run(1180, 820, "landscape");
  if (WIDE) await run(1100, 1400, "wide");
}

if (BANNER) {
  console.log("── 版本檢查：回前景立刻重載、同版本不重複重載、一直開在前景時閒置自動重載 ──");
  const setSig = (v) => fetch(`http://127.0.0.1:${PORT}/__set_sig` + (v ? `?v=${encodeURIComponent(v)}` : ""));
  const resume = () => ev("document.dispatchEvent(new Event('visibilitychange')); 1");
  const banner = () => ev("!!document.querySelector('[role=status]')");
  const mark = () => ev("sessionStorage.getItem('versionReloadedFor')");
  await cdp("Emulation.setDeviceMetricsOverride", { width: 820, height: 1180, deviceScaleFactor: 1, mobile: true, screenOrientation: { type: "portraitPrimary", angle: 0 } });
  await cdp("Page.navigate", { url: `http://127.0.0.1:${PORT}/` });
  await sleep(2500);
  await ev("window.__marker = 'alive'; 1");
  console.log("  頁面載入時的檔案:", await ev("(() => [...document.querySelectorAll('script[type=module][src],link[rel=stylesheet][href]')].map(e=>(e.getAttribute('src')||e.getAttribute('href'))).join(' '))()"));
  console.log("  同版本時橫幅（應為 false）:", await banner());
  await resume(); await sleep(1500);
  console.log("  同版本時回前景，頁面沒有被重載（應為 true）:", (await ev("window.__marker")) === "alive");

  // A. 回前景：伺服器換版 → 立刻重載（沒有人在操作的時刻）
  await setSig("index-FAKE1.js|index-FAKE1.css");
  await resume(); await sleep(3500);
  console.log("  A1 換版後回前景 → 立刻重載（應為 true）:", (await ev("window.__marker")) !== "alive");
  console.log("  A2 重載後的記號（應為 FAKE1）:", await mark());
  console.log("  A3 重載後簽章仍不符 → 橫幅出現（應為 true）:", await banner());
  await shot("banner_shown");
  await ev("window.__marker = 'alive2'; 1");
  await resume(); await sleep(3500);
  console.log("  A4 同一版本再回前景 → 不再重載（應為 true，防迴圈）:", (await ev("window.__marker")) === "alive2");
  await setSig(""); await resume(); await sleep(1500);
  console.log("  A5 伺服器還原後橫幅消失（應為 false）:", await banner());

  // B. 一直開在前景：不碰畫面，等每分鐘的輪詢發現 → 橫幅 → 閒置 45 秒自動重載
  await ev("window.__marker = 'alive3'; 1");
  await setSig("index-FAKE2.js|index-FAKE2.css");
  const t0 = Date.now(); let appeared = 0, reloaded = 0;
  for (let i = 0; i < 70 && !reloaded; i++) {
    await sleep(2000);
    if (!appeared && (await banner())) appeared = Math.round((Date.now() - t0) / 1000);
    if ((await ev("window.__marker")) !== "alive3") reloaded = Math.round((Date.now() - t0) / 1000);
  }
  console.log(`  B1 輪詢發現換版、橫幅出現：${appeared ? appeared + " 秒後" : "沒有出現"}（預期 ≤ 60 秒）`);
  // 閒置是從「最後一次碰畫面」算起，不是從橫幅出現算起：畫面本來就閒置超過 45 秒時，一發現換版就在下一個 5 秒檢查點重載
  console.log(`  B2 閒置後自動重載：${reloaded ? reloaded + " 秒後" : "沒有重載"}（預期：橫幅出現後 ≤ 5 秒，因為這時畫面已閒置超過 45 秒）`);
  await sleep(2500);
  console.log("  B3 重載後的記號（應為 FAKE2）:", await mark());
  await setSig("");
}

console.log("── 頁面錯誤（空白＝沒有）──"); for (const l of [...new Set(logs)].slice(0, 12)) console.log("  " + l);
try { await cdp("Browser.close"); } catch { /* 已關 */ }
chrome.kill(); server.close();
setTimeout(() => process.exit(0), 500);
