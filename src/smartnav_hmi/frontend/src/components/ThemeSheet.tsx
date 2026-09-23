import { useStoreValue } from "../hooks/useStore";
import { SEED_COLORS, setMode, setSeed, themeStore, type ThemeMode } from "../lib/theme";
import { IconCheck } from "./icons";
import { Sheet } from "./Sheet";
import { Button, Segmented } from "./ui";

/**
 * 主題設定：淺／深色切換 + 7 色種子色。所有人皆可開啟修改，不用管理者權限
 * ——這是外觀偏好（比照現場光線調亮暗），不是系統設定，跟登入分開處理，
 * 所以開關狀態放在 TopBar 自己身上，不像 LoginSheet 要把 open 狀態抬到 App。
 */
export function ThemeSheet({ onClose }: { onClose: () => void }) {
  const { seed, mode } = useStoreValue(themeStore);

  return (
    <Sheet
      open
      onClose={onClose}
      title="主題設定"
      subtitle="種子色套用 Material 3 Tonal Spot 演算法產生全站配色"
      footer={
        <Button variant="primary" block onClick={onClose}>
          完成
        </Button>
      }
    >
      <div className="flex flex-col gap-4">
        <Segmented<ThemeMode>
          value={mode}
          onChange={setMode}
          options={[
            { value: "dark", label: "深色" },
            { value: "light", label: "淺色" },
          ]}
        />

        <div className="grid grid-cols-4 gap-x-2 gap-y-3.5">
          {SEED_COLORS.map((s) => {
            const active = s.id === seed;
            return (
              <button
                key={s.id}
                type="button"
                aria-pressed={active}
                onClick={() => setSeed(s.id)}
                className="tap pressable flex flex-col items-center gap-1.5"
              >
                <span
                  className="relative inline-flex size-11 items-center justify-center rounded-full"
                  style={{
                    background: s.hex,
                    boxShadow: active
                      ? "0 0 0 3px var(--color-ink), 0 0 0 5px var(--color-blue)"
                      : undefined,
                  }}
                >
                  {active && <IconCheck className="size-5 text-white" strokeWidth={2.4} />}
                </span>
                <span className="text-center text-[11.5px] leading-tight text-label-3">
                  {s.label}
                </span>
              </button>
            );
          })}
        </div>
      </div>
    </Sheet>
  );
}
