"""Robot agent: the only process that touches the arm. Loop: claim an order -> pack it -> report.

It never exits on its own (with `start` it's a thread nobody restarts; a dead agent shows as "offline"). On a
packing failure it reports it and keeps polling; the server turns auto pack off, so the next order runs only once
a person sends one with Action (after checking the arm). If the packer refused before the arm moved (no free box,
pack spot taken, can't plan: an exception with `not_started`), the order goes back to "received"; auto pack
leaves such orders alone until someone sends them with Action.
"""

from __future__ import annotations

import logging
import time

import httpx

from ..service.orders import Order

log = logging.getLogger("packing.robot_agent")


def run(server: str, packer, poll_s: float = 1.0, once: bool = False, http: httpx.Client | None = None,
        sleep=time.sleep) -> int:
    """Poll forever (`once`: until one claim was handled; returns 1 if that order failed). The agent never exits
    on its own: the dashboard shows "offline" when it stops checking in, and with `start` nothing restarts it."""
    http = http or httpx.Client(base_url=server.rstrip("/"), timeout=10)
    log.info("robot agent polling %s", server)
    while True:
        try:
            data = http.post("/robot/claim").raise_for_status().json()
        except httpx.HTTPError as e:
            log.warning("server unreachable (%s); retrying", e)
            sleep(max(poll_s, 2.0))
            continue
        if not data.get("order"):
            if once:
                return 0
            sleep(poll_s)
            continue
        rc = _handle(http, packer, Order(**data["order"]))
        if once:
            return rc
        sleep(poll_s)


def _report(http: httpx.Client, url: str, body: dict) -> None:
    try:
        http.post(url, json=body).raise_for_status()
    except httpx.HTTPError as e:  # e.g. the order was deleted meanwhile: log it, keep the agent alive
        log.error("couldn't report %s: %s", url, e)


def _handle(http: httpx.Client, packer, order: Order) -> int:
    log.info("packing order %s for %s: %s", order.id, order.customer_name, order.items)
    try:
        box = packer.pack(order)
    except KeyboardInterrupt:
        log.warning("interrupted while packing order %s; it stays in progress", order.id)
        raise
    except Exception as e:  # noqa: BLE001
        if getattr(e, "not_started", False):  # refused before the arm moved: back to received
            log.warning("order %s not started: %s", order.id, e)
            _report(http, f"/robot/orders/{order.id}/requeue", {"error": str(e)})
            return 0
        # something physical went wrong: the server marks it failed and turns auto pack off, so only an order a
        # person sends with Action runs next (that's the "checked the arm" OK)
        log.exception("order %s failed", order.id)
        _report(http, f"/robot/orders/{order.id}/failed", {"error": str(e)})
        return 1
    _report(http, f"/robot/orders/{order.id}/done", {"box_id": box})
    log.info("order %s packed (%s)", order.id, box)
    return 0
