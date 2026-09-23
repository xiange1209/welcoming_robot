import { useEffect, useState } from "react";
import { LoginSheet } from "./components/LoginSheet";
import { Dialogs, Toasts } from "./components/Overlays";
import { TabBar } from "./components/TabBar";
import { TopBar } from "./components/TopBar";
import { useHeartbeat } from "./hooks/useVisibleInterval";
import { useStoreValue } from "./hooks/useStore";
import { refreshMaps, refreshWaypoints, startMapLayer } from "./lib/mapdata";
import { logout, restoreSession, sessionStore } from "./lib/session";
import { TABS, type TabId } from "./lib/tabs";
import { installTeleopGuards } from "./lib/teleop";
import { startTelemetry } from "./lib/telemetry";
import { toast } from "./lib/toast";
import { startChatTts, startTts } from "./lib/tts";
import { startRegistrationWatch } from "./lib/users";

import { GreetPage } from "./pages/GreetPage";
import { HealthPage } from "./pages/HealthPage";
import { MapPage } from "./pages/MapPage";
import { StatsPage } from "./pages/StatsPage";
import { SystemPage } from "./pages/SystemPage";
import { TeleopPage } from "./pages/TeleopPage";
import { UsersPage } from "./pages/UsersPage";
import { VoicePage } from "./pages/VoicePage";

export default function App() {
  const { admin } = useStoreValue(sessionStore);
  const [wantedTab, setTab] = useState<TabId>("greet");
  const [loginOpen, setLoginOpen] = useState(false);

  // 全域的長駐訂閱。每一支內部都有「只跑一次」的閘門，
  // 所以 StrictMode 把 effect 跑兩遍也不會變成兩條連線／兩個監聽。
  useEffect(() => {
    startTelemetry();
    startMapLayer();
    startChatTts();
    startTts();
    startRegistrationWatch();
    installTeleopGuards();

    // 地圖與地點都是管理端點，沒先確認權杖就打只會拿到一串 401
    void restoreSession().then((ok) => {
      if (!ok) return;
      void refreshMaps();
      void refreshWaypoints();
    });
  }, []);

  /* 管理分頁未登入時是「鎖上」而不是消失——見 TabBar 的說明。
     這一行是還原成舊行為（完全隱藏）的唯一開關：換成 visibleTabs(admin) 即可。 */
  const tabs = TABS;
  const isLocked = (t: (typeof TABS)[number]) => !!t.admin && !admin;

  /* 站在管理分頁時被登出（權杖過期也算），要把人帶回公開頁，否則會停在一個
     空白畫面上。用衍生值而不是 effect + setState：後者會先繪一次不存在的
     分頁再修正，中間那一幀就是那個空白畫面。 */
  const wanted = tabs.find((t) => t.id === wantedTab);
  const tab = wanted && !isLocked(wanted) ? wantedTab : "greet";

  /* 地圖分頁的「有人在看」心跳。後端的位姿來源與地圖 PNG 渲染都很貴，
     而且只有這兩頁看得到，所以做成按需開啟：切走就停，後端十幾秒後
     自動把訂閱收掉。不打這個心跳地圖就不會更新。 */
  useHeartbeat("/api/map/watch", 5000, tab === "map" || tab === "teleop");

  const page = (() => {
    switch (tab) {
      case "greet":
        return <GreetPage active />;
      case "stats":
        return <StatsPage />;
      case "map":
        return <MapPage active />;
      case "teleop":
        return <TeleopPage active />;
      case "system":
        return <SystemPage active />;
      case "users":
        return <UsersPage active />;
      case "health":
        return <HealthPage />;
      case "voice":
        return <VoicePage />;
    }
  })();

  return (
    <div className="flex h-full min-h-0 flex-col">
      <TopBar
        onLogin={() => setLoginOpen(true)}
        onLogout={async () => {
          try {
            await logout();
            toast.success("已登出");
          } catch (error) {
            toast.error("登出失敗，請稍後再試");
          }
        }}
      />
      {/*
        只掛目前這一頁，不是全部藏起來。
        原版是八頁全部留在 DOM 裡靠 display:none 切換，於是看不見的分頁照樣
        持有 <img> 串流、計時器與畫布。這裡改成切走即卸載，資源由各自的
        effect cleanup 收掉。

        key 讓 React 在換頁時重建子樹，順便觸發進場動畫。
        底部留白給懸浮導覽列，否則最後一列內容會被它蓋住。
      */}
      <main
        key={tab}
        className="anim-page flex min-h-0 flex-1 flex-col p-3 pb-[calc(env(safe-area-inset-bottom)+92px)]"
      >
        {page}
      </main>
      <TabBar
        tabs={tabs}
        active={tab}
        locked={isLocked}
        onSelect={setTab}
        // 鎖著的分頁點下去直接開登入框，不用自己去找右上角那顆按鈕
        onLocked={() => setLoginOpen(true)}
      />
      {/* 條件掛載：每次開啟都是全新的元件實例，帳密與錯誤訊息自然是空的。
          共用平板上「不要把上一個人的狀態留著」就靠這一點，不必寫重置邏輯。 */}
      {loginOpen && (
        <LoginSheet
          onClose={() => setLoginOpen(false)}
          onSuccess={() => {
            setLoginOpen(false);
            void refreshMaps();
            void refreshWaypoints();
          }}
        />
      )}
      <Toasts />
      <Dialogs />
    </div>
  );
}
