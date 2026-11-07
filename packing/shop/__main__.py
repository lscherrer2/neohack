"""python -m packing.shop start [--robot]  # everything in one: server + dashboard + robot agent
   python -m packing.shop serve            # order server: MCP tools at /mcp + robot job API
   python -m packing.shop robot [--robot]  # robot agent (simulated packing unless --robot)
   python -m packing.shop chat             # Claude Haiku ordering agent using the MCP tools

All data is real: orders in packing/data/shop_orders.db, stock from packing/data/inventory.db
(the packing cell's inventory; restock it from the dashboard after refilling the shelves).
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "cell.yaml"


def load(args):
    from ..config import load_config
    from ..service.orders import OrderStore
    from ..service.vapi import Catalog
    from .shelf import Shelf

    cfg = load_config(args.config)
    shelf = Shelf(cfg)
    products = cfg.catalog or {sku: f"product {sku}" for sku in dict.fromkeys(cfg.part_slots.values())}
    catalog = Catalog(products, max_items=len(cfg.box_slots), available=shelf.available)
    store = OrderStore(getattr(args, "db", None) or cfg.inventory_db.with_name("shop_orders.db"))
    return catalog, store, shelf


def make_packer(args, shelf):
    if args.robot:
        from ..service.worker import RobotPacker

        return RobotPacker(args.config)
    from .shelf import ShelfSimPacker

    return ShelfSimPacker(shelf, args.sim_seconds)


def main() -> None:
    p = argparse.ArgumentParser(prog="python -m packing.shop")
    p.add_argument("--config", default=str(DEFAULT_CONFIG))
    sub = p.add_subparsers(dest="cmd", required=True)
    st = sub.add_parser("start", help="server + dashboard + robot agent in one process")
    s = sub.add_parser("serve", help="order server only (MCP at /mcp + robot job API + dashboard)")
    r = sub.add_parser("robot", help="robot agent: claim -> pack -> report")
    for q in (st, s):
        q.add_argument("--host", default="127.0.0.1")
        q.add_argument("--port", type=int, default=8000)
        q.add_argument("--db", help="orders database (default packing/data/shop_orders.db)")
        q.add_argument("--auto", action="store_true",
                       help="robot packs new orders automatically (default: only after clicking Action)")
    for q in (st, r):
        q.add_argument("--robot", action="store_true", help="pack on the real arm (default: simulated packing)")
        q.add_argument("--sim-seconds", type=float, default=3.0, help="simulated packing time per item")
    r.add_argument("--server", default="http://localhost:8000")
    r.add_argument("--once", action="store_true", help="pack at most one order, then exit")
    c = sub.add_parser("chat", help="Claude Haiku ordering agent using the server's MCP tools")
    c.add_argument("--server", default="http://localhost:8000")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    if args.cmd in ("serve", "start"):
        import threading

        import uvicorn

        from .server import create_server

        catalog, store, shelf = load(args)
        if args.cmd == "start":  # robot agent as a background thread talking to this same server
            from .robot_agent import run

            url = f"http://127.0.0.1:{args.port}"
            threading.Thread(target=run, args=(url, make_packer(args, shelf)), name="robot-agent",
                             daemon=True).start()
        print(f"Dashboard: http://{args.host}:{args.port}/dashboard")
        uvicorn.run(create_server(store, catalog, auto_dispatch=args.auto, shelf=shelf),
                    host=args.host, port=args.port, timeout_graceful_shutdown=2)
    elif args.cmd == "robot":
        from .robot_agent import run

        _, _, shelf = load(args)
        sys.exit(run(args.server, make_packer(args, shelf), once=args.once))
    elif args.cmd == "chat":
        import asyncio

        from .chat import chat

        logging.getLogger().setLevel(logging.WARNING)
        asyncio.run(chat(args.server))


if __name__ == "__main__":
    main()
