import { useStoreValue } from "../hooks/useStore";
import { versionStore } from "../lib/version";
import { Button } from "./ui";

/**
 * 「有新版本」橫幅。畫面上跑的 JS 比伺服器上部署的舊時出現，見 lib/version.ts。
 * 放在頂列下方、不蓋住任何操作；閒置一段時間會自動換新，也可以現在按。
 */
export function UpdateBanner() {
  const { stale } = useStoreValue(versionStore);
  if (!stale) return null;

  return (
    <div
      role="status"
      className="flex flex-none items-center gap-3 border-b border-outline-variant bg-blue/15 px-3 py-2"
    >
      <span className="min-w-0 flex-1 text-[14px] leading-snug">
        有新版本可用。閒置一會兒會自動更新，也可以現在更新。
      </span>
      <Button variant="primary" onClick={() => location.reload()}>
        立即更新
      </Button>
    </div>
  );
}
