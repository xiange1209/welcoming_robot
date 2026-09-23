import react from "@vitejs/plugin-react";
import tailwindcss from "@tailwindcss/vite";
import { defineConfig } from "vite";

// https://vite.dev/config/
export default defineConfig({
  plugins: [react(), tailwindcss()],
  build: {
    // Pi 4 上載入的是區網檔案，體積不是瓶頸；但 sourcemap 會讓
    // 單檔從數百 KB 變成數 MB，平板首次載入才是真正會被感覺到的成本。
    sourcemap: false,
    target: "es2020",
  },
  server: {
    host: true,
    // 開發時把 API／串流／WebSocket 全部轉給真的機器人，
    // 這樣 `npm run dev` 可以直接對著車上的服務改版面。
    proxy: {
      "/api": { target: "http://127.0.0.1:8080", changeOrigin: true },
      "/video": { target: "http://127.0.0.1:8080", changeOrigin: true },
      "/ws": { target: "ws://127.0.0.1:8080", ws: true },
    },
  },
});
