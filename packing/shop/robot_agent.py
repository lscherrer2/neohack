"""Robot agent: the only process that touches the arm. Loop: claim an order -> pack it -> report.

On a packing failure it reports the failure and exits: something physical went wrong and a human should
look at the arm before more orders run. Restarting resumes the in-progress order (the cell continues
from what the inventory says).
"""

from __future__ import annotations

import logging
import time

import httpx

from ..service.orders import Order

log = logging.getLogger("packing.robot_agent")


def run(server: str, packer, poll_s: float = 1.0, once: bool = False, http: httpx.Client | None = None) -> int:
    http = http or httpx.Client(base_url=server.rstrip("/"), timeout=10)
    log.info("robot agent polling %s", server)
    while True:
        try:
            data = http.post("/robot/claim").raise_for_status().json()
        except httpx.HTTPError as e:
            log.warning("server unreachable (%s); retrying", e)
            time.sleep(max(poll_s, 2.0))
            continue
        if not data.get("order"):
            if once:
                return 0
            time.sleep(poll_s)
            continue
        order = Order(**data["order"])
        log.info("packing order %s for %s: %s", order.id, order.customer_name, order.items)
        try:
            box = packer.pack(order)
        except KeyboardInterrupt:
            log.warning("interrupted while packing order %s; it stays in progress and resumes next start", order.id)
            raise
        except Exception as e:  # noqa: BLE001
            log.exception("order %s failed", order.id)
            http.post(f"/robot/orders/{order.id}/failed", json={"error": str(e)})
            log.error("stopping: check the arm, then restart the robot agent")
            return 1
        http.post(f"/robot/orders/{order.id}/done", json={"box_id": box}).raise_for_status()
        log.info("order %s packed (%s)", order.id, box)
        if once:
            return 0
