import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { initTheme } from "./lib/theme";
import App from "./App";
import "./styles/index.css";

// 模組載入時同步套用已存的主題選擇，要在 React 第一次繪製之前跑完，
// 否則畫面會先閃一下預設色再跳成使用者上次選的主題。
initTheme();

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
