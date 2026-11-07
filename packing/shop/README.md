# Shop (slim stack): MCP order server + robot agent + chat agent

Design: `.claude/plans/shop_slim.md`. Two terminals (`source .venv/bin/activate`, repo root):

```bash
python -m packing.shop start            # 1. server + dashboard + robot agent (add --robot for the real arm)
python -m packing.shop chat             # optional: same ordering agent in the terminal (the dashboard has it too)
```
Dashboard: http://localhost:8000/dashboard/. Only one server can use port 8000: stop any other
`serve`/`start` (Ctrl+C) before starting a new one.

Running the pieces separately (e.g. robot agent on another machine) still works:
`python -m packing.shop serve` + `python -m packing.shop robot [--server URL]` (the robot process then posts
its arm position to the server so the 3D view stays live). `serve --robot` makes the server own the real arm.

```
chat (Claude Haiku) ──MCP──▶ order server ◀──HTTP── robot agent ──USB──▶ arm
```

- **Real data only.** Orders: `packing/data/shop_orders.db`. Stock: the packing cell's inventory
  (`packing/data/inventory.db`); it starts empty: after filling the parts tray, mark the pockets full on the
  dashboard's tray map (click a pocket, or "Fill all"). Simulated packing still uses up real parts, so stock numbers stay true.
- **Packing starts by hand** (for now): new orders show as *received*; click **Action** to send one to the
  robot. **Delete** removes an order that isn't being packed or packed. `--auto` makes the robot take new
  orders on its own.
- **Server** (`server.py`): MCP tools `list_products`, `place_order(items, customer_name, phone?)`,
  `check_order(order_id)`, `cancel_order(order_id)`; tool replies are short sentences. Robot job API:
  `POST /robot/claim`, `POST /robot/orders/{id}/done|failed`. Debug: `GET /orders`, `GET /health`.
- **Robot agent** (`robot_agent.py`): claim -> pack -> report, one at a time. On a failure it reports and
  exits (check the arm, restart it). Restarting resumes an order left in progress.
- **Chat** (`chat.py`): only knows what the MCP server lists. `/orders`, `/new`, `/quit`.
  Prints "🔔 order N is packed" when the robot finishes.
- **Dashboard** (React app, source `web/`, see `web/README.md`; built output in `static/app/` is committed):
  http://localhost:8000/dashboard/.
  - **Orders** tab: big status plate (yellow = packing, steel = idle, red = robot agent offline or last order
    failed), ordering chat, stock + 2D tray map (click a pocket to mark it full/empty), the order list, and a live
    activity feed (MCP tool calls, robot pickups, packed/failed orders, tray changes). Events are kept in memory
    (last 500; cleared on server restart).
  - **Calibration** tab: live 3D view of the arm and the calibrated tray; **Preview pose** shows the arm in a
    generic pose without hardware. Calibrate the tray: Connect → Start calibration (hold the arm: motors turn
    off) → touch the gripper tip to each corner pocket and press its Capture button → Save when the fit is OK →
    Finish (motors back on, holding). The arm is owned by the server's `ArmHub` (`arm_hub.py`).
- Any MCP client can use the server, e.g. Claude Desktop / Claude Code (`http://localhost:8000/mcp`),
  or later a voice agent.

Tests: `python -m unittest packing.tests.test_shop packing.tests.test_tray packing.tests.test_arm_view`.
