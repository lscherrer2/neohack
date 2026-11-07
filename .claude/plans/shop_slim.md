# Plan: slim shop stack (MCP order server + robot agent + chat client)

Supersedes the Vapi-first approach for now (`vapi_order_service.md` / `packing/service/` stay as reference).

```
chat (Claude Haiku, MCP client) ──MCP /mcp──▶ shop server (order queue + tools)
                                              ▲            │
                         robot agent ──HTTP── /robot/claim, /robot/orders/{id}/done|failed
                         (owns the USB arm)
```

- **Shop server** (`python -m packing.shop serve`): FastAPI + MCP (Streamable HTTP, stateless JSON) at `/mcp`.
  Customer tools: `list_products`, `place_order(items, customer_name, phone?)`, `check_order(order_id)`,
  `cancel_order(order_id)` — results are short sentences for the caller. Robot endpoints: `POST /robot/claim`
  (returns the in-progress order first, so a restarted agent resumes it), `POST /robot/orders/{id}/done`,
  `POST /robot/orders/{id}/failed`. `GET /orders`, `GET /health` for humans.
- **Robot agent** (`python -m packing.shop robot [--robot]`): loop claim → pack → report. Simulated packing by
  default; `--robot` uses `RobotPacker` (Cell + RealArm). On a packing failure it reports and **exits** (a human
  checks the arm). Only process that opens the USB port.
- **Chat** (`python -m packing.shop chat`): Claude Haiku (`claude-haiku-5-5`) with the server's MCP tools
  (listed at start, called over MCP). Prints a notice when one of the orders changes to packed/failed.
- Reused: `service/orders.py` (OrderStore), `service/vapi.py` (Catalog, parse_items), `service/worker.py` (packers).
- Order states used: queued → packing → packed | failed; queued → cancelled.

Later: put Vapi (MCP tool type or webhook adapter) in front of the same server; auth tokens; cloud-host the server.
