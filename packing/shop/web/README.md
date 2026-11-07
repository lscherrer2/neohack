# Shop dashboard (React + TypeScript)

Source for the dashboard the shop server serves at `/dashboard/`: **Orders** (status plate, order chat, shelf,
parts-tray map, orders, live activity) and **Calibration** (live 3D arm + tray, tray calibration controls).
The API contract is in `.claude/plans/arm_viz.md`.

The built output in `../static/app/` is **committed**, so a Python-only setup runs the dashboard without Node.

```bash
cd packing/shop/web
npm install
npm run dev          # http://localhost:5173/dashboard/  (proxies /api /robot /orders /health -> :8000)
VITE_API_TARGET=http://127.0.0.1:8011 npm run dev   # use a shop server on another port
npm run typecheck
npm run build        # typecheck + build into ../static/app (commit the result)
```

Layout: `src/App.tsx` (tabs in the URL hash, shared polling of `/api/state` + `/api/tray`, activity SSE),
`src/orders/` (Orders tab), `src/calibration/` (Calibration tab; three.js loads only when this tab opens),
`src/api.ts` (fetch helper, SSE hook, formatting), `src/types.ts` (response shapes), `src/styles.css`.
Don't use CSS class names that equal event kinds (`tool/robot/order/chat/error/system`).
