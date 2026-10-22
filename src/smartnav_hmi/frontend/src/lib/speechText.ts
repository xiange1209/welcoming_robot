/**
 * 送去朗讀之前的文字整理。★ 純函式、沒有副作用（不碰 window／localStorage），
 * 所以能在 Node 直接測：maprun/tools_1021/test_tts_forspeech.py。
 *
 * 為什麼要整理：瀏覽器的 speechSynthesis 把阿拉伯數字一律當「數量」唸。
 * 10/20 實測「165 反詐騙專線」唸成「一百六十五」，電話號碼要逐字唸（「一六五」）。
 * 只改送去唸的字串，畫面上的字不動。
 *
 * 判斷「這串數字是電話」的依據，由嚴到寬：
 *   1. 關鍵字（撥打、專線、電話…）後面接號碼，中間最多隔「是／為／號碼／冒號」幾個字
 *   2. 號碼後面緊接「反詐騙／專線／熱線」
 *   3. 長得像台灣電話：09 開頭 10 碼；0 開頭加連字號的市話／0800
 * 數字總數不到 3 位一律不動（「撥 3 次」「專線 2 條」不會被誤改）；
 * 沒有關鍵字也不像電話的數字（「下午 3 點半」「128 號」）完全不動。
 *
 * ★ 不用 lookbehind（?<!…)：iPadOS 16.3 以前的 Safari 不認得，整包 JS 會在載入時就語法錯誤。
 */

const DIGITS = "零一二三四五六七八九";

/** 全形數字 → 半形（引擎唸全形數字也是當數量，先統一再處理） */
const toHalfWidth = (s: string) =>
  s.replace(/[０-９]/g, (d) => String.fromCharCode(d.charCodeAt(0) - 0xfee0));

const SEP = "\\-－–—";
/** 號碼本體：數字開頭、數字結尾，中間可夾連字號、空白、括號（區碼常寫成 (02)） */
const PHONE = `\\(?[0-9](?:[0-9()（） \\u3000${SEP}]*[0-9])?\\)?`;

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

  // 1. 關鍵字 ＋（是／為／號碼／冒號）＋ 號碼
  t = t.replace(
    new RegExp(`(撥打|打給|撥|專線|電話|熱線|客服)([\\s\\u3000是為：:號碼]{0,4})(${PHONE})`, "g"),
    (_m, kw: string, filler: string, num: string) => kw + filler + readPhone(num),
  );

  // 2. 號碼 ＋ 反詐騙／專線／熱線
  t = t.replace(
    new RegExp(`(${PHONE})(\\s*)(反詐騙|專線|熱線)`, "g"),
    (_m, num: string, sp: string, kw: string) => readPhone(num) + sp + kw,
  );

  // 3. 長得像台灣電話、前後沒有別的數字：手機 09xxxxxxxx、0 開頭加連字號的市話／0800
  t = t.replace(
    /(^|[^0-9])(09[0-9]{8}|0[0-9]{1,3}[-－–—][0-9]{3,4}(?:[-－–—][0-9]{3,4})?)(?![0-9])/g,
    (_m, pre: string, num: string) => pre + readPhone(num),
  );

  return t;
}
