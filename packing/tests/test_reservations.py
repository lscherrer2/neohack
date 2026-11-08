"""Per-unit stock reservations: each order item owns one specific pocket/slot (packing/service/orders.py),
wired through the shop (shelf stock, tray API, lock while packing, sim packing consumes exactly those units)."""

import tempfile
import threading
import unittest
from pathlib import Path

from fastapi.testclient import TestClient

from packing.config import load_config
from packing.service.orders import Order, OrderError, OrderStore
from packing.service.vapi import Catalog
from packing.shop.server import create_server
from packing.shop.shelf import Shelf, ShelfSimPacker
from packing.shop.tools import ShopTools
from packing.tests.test_tray import temp_config

STOCK = {"aa": ["aa_1", "aa_2", "aa_10", "aa_3"], "glue": ["glue_1"]}


def stock():
    return {k: sorted(v, key=lambda s: int(s.split("_")[1])) for k, v in STOCK.items()}


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.s = OrderStore(":memory:")

    def make(self, *items):
        return self.s.create("", list(items), "X", stock=stock)[0]

    def test_lowest_numbered_free_units_first(self):
        self.assertEqual(self.make("aa", "glue", "aa").slots, ["aa_1", "glue_1", "aa_2"])
        self.assertEqual(self.make("aa").slots, ["aa_3"])

    def test_all_or_nothing(self):
        self.make("aa", "aa", "aa")
        with self.assertRaises(OrderError) as e:
            self.make("aa", "aa")  # only aa_10 left
        self.assertIn("only 1 left", str(e.exception))
        self.assertEqual(len(self.s.list()), 1)
        self.assertEqual(self.make("aa").slots, ["aa_10"])

    def test_released_on_cancel_delete_packed_kept_on_failed(self):
        a, b, c, d = self.make("aa"), self.make("aa"), self.make("aa"), self.make("aa")
        self.s.cancel(a.id)
        self.s.delete(b.id)
        self.s.update(c.id, "packed")
        self.s.update(d.id, "failed")
        self.assertEqual(self.s.reserved_slots(), {"aa_10": d.id})

    def test_lock_only_while_packing(self):
        o = self.make("aa", "glue")
        self.assertEqual(self.s.locked_slots(), set())
        self.s.action(o.id)
        self.s.claim_next("ready")
        self.assertEqual(self.s.locked_slots(), {"aa_1", "glue_1"})

    def test_emptied_unit_moves_or_goes_short(self):
        o = self.make("glue")
        p = self.make("aa")
        self.assertEqual(self.s.release_slot("aa_1", {"aa": ["aa_2"]}), p.id)
        self.assertEqual(self.s.get(p.id).slots, ["aa_2"])
        self.s.release_slot("glue_1", {})
        self.assertEqual(self.s.get(o.id).slots, [None])  # short
        self.assertEqual(self.s.fill_short({"glue": ["glue_7"]}), [o.id])
        self.assertEqual(self.s.get(o.id).slots, ["glue_7"])

    def test_backfills_orders_from_before_reservations(self):
        old = self.s.create("", ["aa", "glue"], "Old")[0]  # no stock= -> no rows
        self.assertEqual(old.slots, [])
        self.assertEqual(self.s.fill_short(stock()), [old.id])
        self.assertEqual(self.s.get(old.id).slots, ["aa_1", "glue_1"])

    def test_concurrent_orders_never_share_a_unit(self):
        s = OrderStore(Path(tempfile.mkdtemp()) / "o.db")
        ok, refused = [], []

        def order():
            try:
                ok.append(s.create("", ["aa"], "T", stock=stock)[0].slots[0])
            except OrderError:
                refused.append(1)
        threads = [threading.Thread(target=order) for _ in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(sorted(ok), ["aa_1", "aa_10", "aa_2", "aa_3"])
        self.assertEqual(len(refused), 6)

    def test_order_round_trips_with_slots(self):  # robot agent: Order(**json)
        o = self.make("aa")
        self.assertEqual(Order(**o.to_dict()).slots, ["aa_1"])


class ShopTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        cfg = load_config(temp_config(Path(self._tmp.name)))
        self.shelf = Shelf(cfg)
        self.store = OrderStore(":memory:")
        self.catalog = Catalog(cfg.catalog, 10, available=self.shelf.available, stock=self.shelf.stock)
        self.http = TestClient(create_server(self.store, self.catalog, shelf=self.shelf))
        self.tools = ShopTools(self.store, self.catalog)
        for p in ("aa_1", "aa_2", "aa_3", "glue_4"):
            self.http.post(f"/api/tray/pockets/{p}", json={"full": True})

    def tearDown(self):
        self._tmp.cleanup()

    def pockets(self):
        return {p["id"]: p for p in self.http.get("/api/tray").json()["pockets"]}

    def test_tray_shows_reservations_and_stock_counts_them(self):
        o, _ = self.tools.create_order(["AA battery", "AA battery"], "Sam")
        self.assertEqual(o.slots, ["aa_1", "aa_2"])
        p = self.pockets()
        self.assertEqual((p["aa_1"]["reserved_by"], p["aa_3"]["reserved_by"]), (o.id, None))
        self.assertEqual(self.tools.in_stock()["aa"], 1)
        with self.assertRaises(OrderError):
            self.tools.create_order(["AA battery", "AA battery"], "Too many")

    def test_locked_while_packing(self):
        o, _ = self.tools.create_order(["AA battery"], "Sam")
        self.store.action(o.id)
        self.store.claim_next("ready")
        r = self.http.post("/api/tray/pockets/aa_1", json={"full": False})
        self.assertEqual(r.status_code, 409)
        self.assertTrue(self.pockets()["aa_1"]["locked"])
        r = self.http.post("/api/tray/fill", json={"full": False}).json()  # bulk empty skips it
        self.assertEqual(r["locked"], ["aa_1"])
        self.assertTrue(self.pockets()["aa_1"]["full"])

    def test_marking_reserved_pocket_empty_moves_the_reservation(self):
        o, _ = self.tools.create_order(["AA battery"], "Sam")
        self.http.post("/api/tray/pockets/aa_1", json={"full": False})
        self.assertEqual(self.store.get(o.id).slots, ["aa_2"])
        self.http.post("/api/tray/fill", json={"full": False, "sku": "aa"})
        self.assertEqual(self.store.get(o.id).slots, [None])  # short
        self.http.post("/api/tray/pockets/aa_3", json={"full": True})
        self.assertEqual(self.store.get(o.id).slots, ["aa_3"])

    def test_sim_packing_consumes_exactly_the_reserved_units(self):
        self.http.post("/api/tray/pockets/aa_5", json={"full": True})
        o, _ = self.tools.create_order(["AA battery", "glue stick"], "Sam")
        self.store.release_slot("aa_1", {"aa": ["aa_5"]})  # reserved unit is now aa_5, not the lowest
        o = self.store.get(o.id)
        ShelfSimPacker(self.shelf, 0).pack(o)
        full = {k for k, v in self.pockets().items() if v["full"]}
        self.assertEqual(full, {"aa_1", "aa_2", "aa_3"})  # aa_5 + glue_4 used


if __name__ == "__main__":
    unittest.main()
