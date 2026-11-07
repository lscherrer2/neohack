# Plan: Vapi voice ordering → robot packing → callback

## Flow
1. Customer calls the Vapi number. The Vapi assistant uses **tools** (function calls) that Vapi POSTs
   to our server: `list_products`, `place_order`, `check_order`, `cancel_order`.
2. `place_order` writes an order to a persistent queue (SQLite) and returns the order number + queue position.
3. A single **worker** thread takes queued orders one at a time (one arm), runs the packer
   (`Cell.package_box(skus)` on the real arm, or a timed simulation), marks the order packed.
4. Worker asks Vapi to **call the customer back** (`POST https://api.vapi.ai/call`, callback assistant,
   `assistantOverrides.variableValues` with order details). Result recorded on the order.

## Vapi contract (docs.vapi.ai, checked 2026-11-07)
- Inbound: `POST /vapi/webhook`, body `{"message": {"type": "tool-calls", "toolCallList": [{id, function: {name, arguments}}], "call": {...}}}`.
  Reply **always HTTP 200** with `{"results": [{"toolCallId": id, "result": "<string>"}]}` (or `"error"`).
  Other message types (status-update, end-of-call-report, ...) are acknowledged and logged.
- Auth: credential on the server URL; we accept `X-Vapi-Secret: <secret>` or `Authorization: Bearer <secret>`.
- Outbound: `POST https://api.vapi.ai/call`, `Authorization: Bearer <private key>`,
  `{assistantId, phoneNumberId, customer: {number}, assistantOverrides: {variableValues: {...}}}`.
- Caller number: taken from `message.call.customer.number` / `message.customer.number`, else tool arg `phone`.

## Design decisions
- Idempotent `place_order`: Vapi `toolCallId` stored UNIQUE, a retried tool call returns the same order.
- Order states: queued → packing → packed → calling → called; failed (pack error); call_failed; cancelled.
  Only `queued` orders can be cancelled.
- Order size ≤ number of box slots; SKUs validated against the catalog (cell.yaml `catalog`, falling back
  to part_slots SKUs); stock checked (available parts minus items reserved by queued orders).
- Dry-run notifier when Vapi keys are missing (logs the request it would send).
- Packer is pluggable: `SimPacker` (sleep) for development, `RobotPacker` (Cell + RealArm) for the real arm.

## Layout: `packing/service/`
`orders.py` store · `vapi.py` tool handlers + outbound client + tool definitions · `worker.py` ·
`app.py` FastAPI · `__main__.py` (`serve`, `tools`) · tests in `packing/tests/test_service.py`.

## Open items
- Public URL for Vapi (ngrok / cloudflared) + credential setup in Vapi dashboard.
- Vapi assistants: an inbound ordering assistant (tools attached) and a callback assistant (prompt uses
  `{{order_id}}`, `{{customer_name}}`, `{{items}}`).
- RobotPacker untested on hardware: needs all cell poses taught first.
