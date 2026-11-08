"""python -m packing.shop start [--robot]  # everything in one: server + dashboard + robot agent
   python -m packing.shop serve [--robot]  # order server: MCP tools at /mcp + robot job API (+ arm with --robot)
   python -m packing.shop robot            # robot agent with simulated packing (real packing: `start --robot`)
   python -m packing.shop chat             # Claude Haiku ordering agent using the MCP tools
Remote agents (cloud agent via a tunnel): `start` / `serve` also serve a FastMCP server at /agent/mcp
(get_inventory, create_order, check_order, cancel_order) — see mcp_server.py; --mcp-token / --allow-host.

All data is real: orders in packing/data/shop_orders.db, stock from packing/data/inventory.db
(the packing cell's inventory; restock it from the dashboard after refilling the shelves).

The arm (live 3D view, calibration tab, packing) is owned by the server's ArmHub: the real arm with
--robot, a simulated one otherwise (sim runs save tray calibration to data/sim_tray_cal.json).
`serve` without --robot owns no arm. Real packing (TrayPacker) runs only inside `start --robot`: it needs the
server's ArmHub.
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
    products = cfg.catalog or {sku: f"product {sku}" for sku in dict.fromkeys(cfg.stock_slots.values())}
    catalog = Catalog(products, max_items=cfg.max_items_per_order, available=shelf.available, stock=shelf.stock)
    store = OrderStore(getattr(args, "db", None) or cfg.inventory_db.with_name("shop_orders.db"))
    return catalog, store, shelf


def make_hub(args, cfg):
    """ArmHub for the server: the real arm with --robot, else a simulated one."""
    from ..hardware.arm import FakeArm, RealArm
    from .arm_hub import ArmHub

    def model():  # re-reads the joint offsets file (ArmHub.reload_model after a fit is applied)
        return cfg.arm_model()

    if args.robot:
        from ..motion.kinematics import gravity_feedforward

        return ArmHub(lambda: RealArm(cfg.port, cfg.robot_id, cfg.motion.max_step), "real", model,
                      tune={"p_gain": cfg.motion.servo_p, "accel": cfg.motion.servo_accel},
                      gravity_factory=lambda: gravity_feedforward(cfg.robot_id, cfg.motion.gravity_gain))
    if cfg.tray:  # simulated arm: never overwrite the real tray calibration
        cfg.tray.calibration_file = cfg.tray.calibration_file.with_name("sim_" + cfg.tray.calibration_file.name)
    if cfg.desk_file:  # nor the real desk calibration
        cfg.desk_file = cfg.desk_file.with_name("sim_" + cfg.desk_file.name)
    if cfg.reach_comp_file:  # nor the real reach-compensation toggle
        cfg.reach_comp_file = cfg.reach_comp_file.with_name("sim_" + cfg.reach_comp_file.name)
    if cfg.joint_offsets_file:  # nor the real arm's joint offsets (those live in calibration/)
        cfg.joint_offsets_file = cfg.desk_file.with_name("sim_joint_offsets.json") if cfg.desk_file else None
    return ArmHub(lambda: FakeArm(closed=cfg.gripper.closed, opened=cfg.gripper.open), "sim", model)


def make_packer(args, shelf, hub=None, store=None, events=None):
    """--robot: TrayPacker on the server's ArmHub (in-process only: `start --robot`). Else simulated packing."""
    if args.robot:
        if hub is None or store is None:
            sys.exit("Packing on the real arm needs the server's arm: run `python -m packing.shop start --robot` "
                     "(a separate `robot --robot` process is no longer supported).")
        from ..boxes.packer import TrayPacker
        from ..boxes.store import BoxStore

        return TrayPacker(hub, shelf.cfg, BoxStore(store), shelf, events.record if events else lambda *a, **k: None)
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
                       help="start with Auto pack on (dashboard toggle): the robot packs the newest order with all "
                            "its stock, once a box is ready (default off: only after clicking Action)")
        q.add_argument("--mcp-token", help="remote-agent MCP (/agent/mcp): require 'Authorization: Bearer <token>' "
                                           "(default: env SHOP_MCP_TOKEN, else open)")
        q.add_argument("--allow-host", action="append", metavar="HOST",
                       help="remote-agent MCP: only accept these Host headers (+ localhost), e.g. "
                            "'*.trycloudflare.com' (repeatable; default: any host)")
    st.add_argument("--robot", action="store_true", help="pack on the real arm (default: simulated packing)")
    for q in (st, r):
        q.add_argument("--sim-seconds", type=float, default=3.0, help="simulated packing time per item")
    s.add_argument("--robot", action="store_true", help="this server owns the real arm (3D view + calibration)")
    r.add_argument("--robot", action="store_true", help=argparse.SUPPRESS)  # gone: exits with a hint
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

        from .events import EventLog

        catalog, store, shelf = load(args)
        events = EventLog()
        hub = make_hub(args, shelf.cfg)
        if args.cmd == "start":  # robot agent as a background thread talking to this same server
            from .robot_agent import run

            url = f"http://127.0.0.1:{args.port}"
            threading.Thread(target=run, args=(url, make_packer(args, shelf, hub, store, events)), name="robot-agent",
                             daemon=True).start()
        import os

        token = args.mcp_token or os.environ.get("SHOP_MCP_TOKEN") or None
        print(f"Dashboard: http://{args.host}:{args.port}/dashboard/")
        print(f"Remote-agent MCP: http://{args.host}:{args.port}/agent/mcp "
              f"(auth: {'bearer token' if token else 'NONE, open'}; hosts: {', '.join(args.allow_host or ['any'])})")
        try:
            uvicorn.run(create_server(store, catalog, events=events, auto_dispatch=args.auto, shelf=shelf, hub=hub,
                                      box_packing=bool(args.robot) and args.cmd == "start",
                                      agent_token=token, agent_hosts=args.allow_host),
                        host=args.host, port=args.port, timeout_graceful_shutdown=2)
        finally:
            if hub.connected and hub.mode == "idle":
                hub.close()  # torque stays on (holding)
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
