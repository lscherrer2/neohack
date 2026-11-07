# Order service (Vapi voice ordering → robot packing → callback)

Design: `.claude/plans/vapi_order_service.md`. **Handoff for the Vapi configurer: `VAPI_SETUP.md`.**

```
caller ──phone──> Vapi assistant ──tool call──> POST /vapi/webhook ──> order queue (SQLite)
                                                                          │ worker (one at a time)
                                                                          ▼
caller <──phone── Vapi callback assistant <── POST api.vapi.ai/call <── packed (Cell.package_box)
```

## Try it without Vapi: terminal phone agent
```bash
python -m packing.service chat                  # you "call" the shop from +15551234567
python -m packing.service chat --server http://localhost:8000   # tool calls over HTTP to a running `serve`
```
Claude Haiku (`claude-haiku-5-5`, key from `.env.local`) plays the Vapi ordering assistant. Tool calls are
wrapped in Vapi's exact `tool-calls` message and handled by the webhook code. When an order is packed,
the terminal rings; `/answer` talks to the callback assistant with the same `{{variables}}` Vapi gets.
Prompts are in `prompts/` (`ordering.md`, `callback.md`, `callback_first_message.txt`) — paste those into
Vapi when switching over. Commands: `/hangup`, `/answer`, `/orders`, `/quit`.

## Run
```bash
python -m packing.service serve                 # simulated packing (2 s/item), dry-run callbacks
python -m packing.service serve --robot         # pack on the real arm (all cell poses must be taught)
```
Vapi must reach the server over HTTPS: e.g. `cloudflared tunnel --url http://localhost:8000`
or `ngrok http 8000`, then use `https://<host>/vapi/webhook`.

## Vapi setup
1. `cp packing/service/.env.example packing/service/.env` and fill in the keys.
2. Tools: `python -m packing.service tools --url https://<host>/vapi/webhook` prints the 4 tool
   definitions (`list_products`, `place_order`, `check_order`, `cancel_order`); create them in Vapi
   (dashboard or API) and attach them to the **ordering** assistant.
3. Server credential: add a custom credential (Bearer, or header `X-Vapi-Secret`) with the same value as
   `VAPI_SERVER_SECRET`, and attach it to the tools' server URL.
4. **Callback** assistant (`VAPI_CALLBACK_ASSISTANT_ID`): its prompt/first message can use
   `{{customer_name}}`, `{{order_id}}`, `{{items}}`, e.g.
   "Hi {{customer_name}}, your order {{order_id}} ({{items}}) is packed and ready for pickup."
5. `VAPI_PHONE_NUMBER_ID` must be a number that can place outbound calls (not a free Vapi number).

## Behaviour
- Tool replies are spoken sentences; problems (unknown product, out of stock, too many items for one
  box) come back as normal results so the assistant can tell the caller. Always HTTP 200 to Vapi.
- `place_order` is idempotent on Vapi's `toolCallId`; queued orders reserve stock.
- Callback number = the number they called from unless the assistant passes `phone`.
- Order states: queued → packing → packed → calling → called (or failed / call_failed / cancelled).
- A packing failure **pauses the worker** (a human should look at the arm). Fix, then
  `POST /orders/{id}/retry` and `POST /worker/resume`. Failed callbacks: `POST /orders/{id}/retry`.
- On restart, orders stuck in `packing` are re-queued (the cell resumes from inventory).

## Admin API (Bearer `ADMIN_TOKEN`)
`GET /health` · `GET /orders?status=queued` · `POST /orders {"phone","items","customer_name"}` ·
`GET /orders/{id}` · `POST /orders/{id}/cancel` · `POST /orders/{id}/retry` · `POST /worker/pause|resume`
