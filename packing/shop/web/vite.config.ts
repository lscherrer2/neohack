import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

const backend = process.env.VITE_API_TARGET ?? "http://127.0.0.1:8000";
const proxied = ["/api", "/robot", "/orders", "/health"];

export default defineConfig({
  base: "/dashboard/",
  plugins: [react()],
  build: {
    outDir: "../static/app",
    emptyOutDir: true,
    chunkSizeWarningLimit: 2000, // three.js is big; one chunk is fine for a local dashboard
  },
  server: {
    // SSE passes through http-proxy unbuffered; no special options needed.
    proxy: Object.fromEntries(proxied.map((p) => [p, { target: backend, changeOrigin: true }])),
  },
});
