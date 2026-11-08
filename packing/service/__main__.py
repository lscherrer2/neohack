"""python -m packing.service serve [--robot] [--port 8000]
   python -m packing.service tools --url https://<public-host>/vapi/webhook

Env (or packing/service/.env): VAPI_API_KEY, VAPI_PHONE_NUMBER_ID, VAPI_CALLBACK_ASSISTANT_ID,
VAPI_SERVER_SECRET, ADMIN_TOKEN. Missing Vapi keys => callbacks are dry-run (logged only).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
from pathlib import Path

HERE = Path(__file__).parent
DEFAULT_CONFIG = HERE.parent / "cell.yaml"


def _load_dotenv(path: Path) -> None:
    if not path.exists():
        return
    for line in path.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


def build_core(args):
    """Order store, catalog and packer shared by `serve` and `chat`."""
    from ..config import load_config
    from ..cell.inventory import Inventory
    from .orders import OrderStore
    from .vapi import Catalog
    from .worker import RobotPacker, SimPacker

    cfg = load_config(args.config)
    products = cfg.catalog or {sku: f"product {sku}" for sku in dict.fromkeys(cfg.part_slots.values())}
    if args.robot:
        inv = Inventory(cfg.inventory_db)

        def available():
            out: dict[str, int] = {}
            for p in inv.parts("available"):
                out[p.sku] = out.get(p.sku, 0) + 1
            return out

        packer = RobotPacker(args.config)
    else:
        def available():
            return dict.fromkeys(products, 99)

        packer = SimPacker(args.sim_seconds)
    catalog = Catalog(products, max_items=cfg.max_items_per_order, available=available)
    store = OrderStore(args.db or cfg.inventory_db.with_name("orders.db"))
    return store, catalog, packer


def build(args):
    from .app import create_app
    from .vapi import VapiNotifier, VapiSettings
    from .worker import Worker

    store, catalog, packer = build_core(args)
    settings = VapiSettings.from_env()
    worker = Worker(store, packer, VapiNotifier(settings, catalog))
    return create_app(store, catalog, worker, settings, admin_token=os.getenv("ADMIN_TOKEN")), settings


def main() -> None:
    p = argparse.ArgumentParser(prog="python -m packing.service")
    p.add_argument("--config", default=str(DEFAULT_CONFIG))
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("serve")
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=8000)
    s.add_argument("--robot", action="store_true", help="pack on the real arm (default: simulated packing)")
    s.add_argument("--sim-seconds", type=float, default=2.0, help="simulated packing time per item")
    s.add_argument("--db", help="orders database (default packing/data/orders.db)")
    c = sub.add_parser("chat", help="terminal stand-in for the Vapi phone agent (Claude Haiku)")
    c.add_argument("--caller", default="+15551234567", help="the phone number you're 'calling from'")
    c.add_argument("--robot", action="store_true", help="pack on the real arm (default: simulated packing)")
    c.add_argument("--sim-seconds", type=float, default=3.0, help="simulated packing time per item")
    c.add_argument("--db", help="orders database (default packing/data/orders.db)")
    c.add_argument("--server", help="send tool calls to a running `serve` (e.g. http://localhost:8000) instead of in-process")
    t = sub.add_parser("tools", help="print Vapi tool definitions (JSON) for your public webhook URL")
    t.add_argument("--url", required=True)
    args = p.parse_args()
    _load_dotenv(HERE / ".env")

    if args.cmd == "chat":
        from .chat import run_chat

        logging.basicConfig(level=logging.WARNING)
        store, catalog, packer = build_core(args)
        run_chat(store, catalog, packer, args.caller, args.server)
        return
    if args.cmd == "tools":
        from .vapi import tool_definitions

        print(json.dumps(tool_definitions(args.url), indent=2))
        return
    import uvicorn

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    app, settings = build(args)
    if not settings.server_secret:
        logging.warning("VAPI_SERVER_SECRET not set: /vapi/webhook accepts unauthenticated requests")
    if not settings.can_call:
        logging.warning("Vapi call keys not set: callbacks are DRY RUN (logged only)")
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
