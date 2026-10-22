/**
 * 送去朗讀之前的文字整理。★ 純函式、沒有副作用（不碰 window／localStorage），
 * 所以能在 Node 直接測：maprun/tools_1021/test_tts_forspeech.py。
 *
 * 為什麼要整理：瀏覽器的 speechSynthesis 把阿拉伯數字一律當「數量」唸，數字中間的「-」iOS 還會唸成「負」。
 *   10/20 實測「165 反詐騙專線」唸成「一百六十五」（應唸「一六五」）；
 *   10/22 實測「客服專線 0800-123456」唸成「0800 負 12萬3千4百5十6」。
 * 只改送去唸的字串，畫面上的字不動。
 *
 * 規則（依序）：
 *   0. 全形數字 → 半形
 *   1. 日期 2026-10-22 → 2026年10月22日；時間範圍 09:00-17:00 → 09:00到17:00
 *   2. 電話逐字唸，群組之間（連字號、空白）停頓。判斷是電話的依據：
 *        ① 關鍵字（撥打、專線、電話…）後面接號碼，中間最多隔「是／為／號碼／冒號」幾個字
 *        ② 號碼後面緊接「反詐騙／專線／熱線」
 *        ③ 手機 09 開頭 10 碼
 *   3. 其餘的「數字-數字」串：把**最大連續一串**（認得千分位 1,000、小數 0.5、百分號）一起判斷——
 *      全是純數字，而且「0 開頭」或「有 3 組以上」→ 當電話／帳號逐字唸，組與組之間停頓；
 *      否則當範圍唸「到」（10-20 分鐘 → 10到20 分鐘；1,000-12,000 元 → 1,000到12,000 元；1000-2000 元 → 1000到2000 元）
 * 數字總數不到 3 位的電話候選一律不動（「撥 3 次」「專線 2 條」）；沒有關鍵字也不像電話、
 * 也沒有「-」的數字（「下午 3 點半」「128 號」「1,000 元」）完全不動；開頭的負號（「-5 度」）也不動。
 *
 * ★ 不用 lookbehind（?<!…)：iPadOS 16.3 以前的 Safari 不認得，整包 JS 會在載入時就語法錯誤。
 * ★ 規則 1 與「-」唸成「負」的發現來自車上 agent 10/22（hmi_ipad.patch），電話關鍵字規則 2 是筆電端；兩邊併成這一份。
 * ★ 併入後補（筆電端 10/22 晚）：車上 agent 原本的規則 3 是「總共 ≥ 7 位數或 0 開頭就是電話」，
 *   會把「1,000-12,000 元」唸成「1,零零零，一二,000 元」、「1000-2000 元」唸成「一零零零，二零零零」，
 *   「帳號 012-345-678901」只轉前半、剩一個「-」；改成上面的「最大連續一串」之後都對。
 */

const DIGITS = "零一二三四五六七八九";

/** 全形數字 → 半形（引擎唸全形數字也是當數量，先統一再處理） */
const toHalfWidth = (s: string) =>
  s.replace(/[０-９]/g, (d) => String.fromCharCode(d.charCodeAt(0) - 0xfee0));

/** 連字號的各種寫法：半形、全形、U+2010、en dash、em dash */
const SEP = "\\-－‐–—";
/** 號碼本體：數字開頭、數字結尾，中間可夾連字號、空白、括號（區碼常寫成 (02)） */
const PHONE = `\\(?[0-9](?:[0-9()（） \\u3000${SEP}]*[0-9])?\\)?`;

/** 一個數字：可有千分位（1,000）、小數（0.5）、百分號 */
const NUM = "\\d+(?:,\\d{3})*(?:\\.\\d+)?%?";
/** 用連字號連起來的一串數字（最大連續一串） */
const CHAIN = new RegExp(`${NUM}(?:\\s*[${SEP}]\\s*${NUM})+`, "g");
const SEP_SPLIT = new RegExp(`\\s*[${SEP}]\\s*`);

/** 一串號碼逐字唸；群組之間（連字號、空白）換成逗號，讓引擎停頓一下 */
function readPhone(token: string): string {
  const digitCount = (token.match(/[0-9]/g) || []).length;
  if (digitCount < 3) return token; // 太短不像電話，原樣
  return token
    .replace(/[()（）]/g, "")
    .replace(new RegExp(`\\s*[${SEP}]\\s*|[ \\u3000]+`, "g"), "，")
    .replace(/[0-9]/g, (d) => DIGITS[Number(d)]);
}

export function forSpeech(text: string): string {
  let t = toHalfWidth(text);

  // 1. 日期、時間範圍（要在電話規則之前，免得被當成連字號號碼）
  t = t.replace(new RegExp(`(\\d{4})[${SEP}/](\\d{1,2})[${SEP}/](\\d{1,2})(?!\\d)`, "g"), "$1年$2月$3日");
  t = t.replace(new RegExp(`(\\d{1,2}:\\d{2})\\s*[${SEP}~～]\\s*(\\d{1,2}:\\d{2})`, "g"), "$1到$2");

  // 2-① 關鍵字 ＋（是／為／號碼／冒號）＋ 號碼
  t = t.replace(
    new RegExp(`(撥打|打給|撥|專線|電話|熱線|客服)([\\s\\u3000是為：:號碼]{0,4})(${PHONE})`, "g"),
    (_m, kw: string, filler: string, num: string) => kw + filler + readPhone(num),
  );

  // 2-② 號碼 ＋ 反詐騙／專線／熱線
  t = t.replace(
    new RegExp(`(${PHONE})(\\s*)(反詐騙|專線|熱線)`, "g"),
    (_m, num: string, sp: string, kw: string) => readPhone(num) + sp + kw,
  );

  // 2-③ 手機 09xxxxxxxx（前後沒有別的數字）
  t = t.replace(/(^|[^0-9])(09[0-9]{8})(?![0-9])/g, (_m, pre: string, num: string) => pre + readPhone(num));

  // 3. 其餘的「數字-數字」串：最大連續一串一起判斷，不會把千分位金額從中間切開
  t = t.replace(CHAIN, (m) => {
    const parts = m.split(SEP_SPLIT);
    const pureDigits = parts.every((x) => /^[0-9]+$/.test(x));
    return pureDigits && (parts.length >= 3 || parts[0].startsWith("0")) ? readPhone(parts.join("-")) : parts.join("到");
  });

  return t;
}
