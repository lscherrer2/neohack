"""Robot agent: the only process that touches the arm. Loop: claim an order -> pack it -> report.

On a packing failure it reports the failure and exits: something physical went wrong and a human should
look at the arm before more orders run. If the packer refused before the arm moved (no free box, pack spot
taken, can't plan: an exception with `not_started`), the order goes back to "received" and the agent keeps going.
"""

from __future__ import annotations

import logging
import time

import httpx

from ..service.orders import Order

log = logging.getLogger("packing.robot_agent")


NOT_STARTED_BACKOFF_S = 30.0  # after a refused order (with --auto it would be claimed again right away)


def run(server: str, packer, poll_s: float = 1.0, once: bool = False, http: httpx.Client | None = None,
        sleep=time.sleep) -> int:
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
            sleep(poll_s)
            continue
        order = Order(**data["order"])
        log.info("packing order %s for %s: %s", order.id, order.customer_name, order.items)
        try:
            box = packer.pack(order)
        except KeyboardInterrupt:
            log.warning("interrupted while packing order %s; it stays in progress and resumes next start", order.id)
            raise
        except Exception as e:  # noqa: BLE001
            if getattr(e, "not_started", False):
                log.warning("order %s not started: %s", order.id, e)
                http.post(f"/robot/orders/{order.id}/requeue", json={"error": str(e)})
                if once:
                    return 0
                sleep(NOT_STARTED_BACKOFF_S)
                continue
            log.exception("order %s failed", order.id)
            http.post(f"/robot/orders/{order.id}/failed", json={"error": str(e)})
            log.error("stopping: check the arm, then restart the robot agent")
            return 1
        http.post(f"/robot/orders/{order.id}/done", json={"box_id": box}).raise_for_status()
        log.info("order %s packed (%s)", order.id, box)
        if once:
            return 0
