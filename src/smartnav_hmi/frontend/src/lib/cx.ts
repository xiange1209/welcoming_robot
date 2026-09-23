/** 串接 class，順手濾掉 false/undefined。不引入 clsx 只為了這幾行 */
export function cx(...parts: (string | false | null | undefined)[]): string {
  return parts.filter(Boolean).join(" ");
}
