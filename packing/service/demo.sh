#!/usr/bin/env bash
# One-command demo: order server + public Cloudflare quick tunnel + Vapi tool definitions.
#   packing/service/demo.sh            # simulated packing (real packing: python -m packing.shop start --robot)
# The public URL lasts as long as this script runs (Ctrl+C stops everything). A new run = a new URL.
set -euo pipefail
cd "$(dirname "$0")/../.."

PORT="${PORT:-8000}"
PY="${PY:-.venv/bin/python}"
LOGDIR="packing/data/demo_logs"
mkdir -p "$LOGDIR"
SERVER_LOG="$LOGDIR/server.log"
TUNNEL_LOG="$LOGDIR/tunnel.log"
TOOLS_OUT="packing/service/vapi_tools.json"

pids=()
cleanup() {
  echo; echo "Stopping..."
  for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

caffeinate -dims & pids+=($!)   # keep the Mac awake while the demo runs

"$PY" -m packing.service serve --port "$PORT" "$@" >"$SERVER_LOG" 2>&1 & pids+=($!)
for _ in $(seq 1 60); do curl -sf "http://localhost:$PORT/health" >/dev/null && break; sleep 0.5; done
curl -sf "http://localhost:$PORT/health" >/dev/null || { echo "server didn't start, see $SERVER_LOG"; exit 1; }

cloudflared tunnel --url "http://localhost:$PORT" >"$TUNNEL_LOG" 2>&1 & pids+=($!)
URL=""
for _ in $(seq 1 60); do
  URL=$(grep -Eo 'https://[a-z0-9-]+\.trycloudflare\.com' "$TUNNEL_LOG" | head -1 || true)
  [ -n "$URL" ] && break; sleep 0.5
done
[ -n "$URL" ] || { echo "tunnel didn't come up, see $TUNNEL_LOG"; exit 1; }

"$PY" -m packing.service tools --url "$URL/vapi/webhook" >"$TOOLS_OUT"

cat <<EOF

  Public URL:   $URL
  Vapi webhook: $URL/vapi/webhook
  Tool JSON:    $TOOLS_OUT   (give to the Vapi person with packing/service/VAPI_SETUP.md)
  Logs:         $SERVER_LOG, $TUNNEL_LOG

  Ctrl+C to stop. Waiting for the tunnel to answer publicly...
EOF
for _ in $(seq 1 30); do curl -sf "$URL/health" >/dev/null && { echo "  Public check OK: $(curl -s "$URL/health")"; break; }; sleep 2; done

tail -f "$SERVER_LOG"
