import { useEffect, useState } from "react";
import { api, errText } from "../api";
import type { Order, ShopState } from "../types";

const WORDS: Record<string, string> = {
  queued: "received", ready: "ready", packing: "packing", packed: "packed", failed: "failed", cancelled: "cancelled",
};
const COUNT_ORDER = ["packing", "ready", "queued", "packed", "failed", "cancelled"];
const DELETABLE = ["queued", "ready", "failed", "cancelled"];

export default function OrdersTable({ state, refreshState }: { state: ShopState | null; refreshState: () => void }) {
  const [busy, setBusy] = useState<Set<number>>(new Set());
  const auto = state?.auto_dispatch ?? false;
  const [armDeleteAll, setArmDeleteAll] = useState(false); // second click within 3 s deletes
  const [deletingAll, setDeletingAll] = useState(false);

  useEffect(() => {
    if (!armDeleteAll) return;
    const t = setTimeout(() => setArmDeleteAll(false), 3000);
    return () => clearTimeout(t);
  }, [armDeleteAll]);

  async function deleteAll() {
    if (!armDeleteAll) { setArmDeleteAll(true); return; }
    setArmDeleteAll(false);
    setDeletingAll(true);
    try {
      await api("DELETE", "/api/orders");
    } catch (e) {
      alert(errText(e) || "Couldn't delete the orders.");
    }
    setDeletingAll(false);
    refreshState();
  }

  async function act(o: Order, what: "action" | "delete") {
    setBusy((s) => new Set(s).add(o.id));
    try {
      await api(what === "action" ? "POST" : "DELETE", `/api/orders/${o.id}${what === "action" ? "/action" : ""}`);
    } catch (e) {
      alert(errText(e) || `Couldn't ${what} order ${o.id}.`);
    }
    setBusy((s) => { const n = new Set(s); n.delete(o.id); return n; });
    refreshState();
  }

  const counts = state?.counts ?? {};
  const orders = state?.orders ?? [];
  const deletable = orders.filter((o) => DELETABLE.includes(o.status)).length;
  return (
    <section aria-labelledby="orders-h">
      <header>
        <h2 id="orders-h">Orders</h2>
        <span className="sub">{COUNT_ORDER.filter((s) => counts[s]).map((s) => `${counts[s]} ${WORDS[s]}`).join(", ")}</span>
        {deletable > 0 && (
          <button className="del small" disabled={deletingAll} onClick={deleteAll}
                  title="Deletes every order that isn't packing or packed">
            {armDeleteAll ? `Click again to delete ${deletable}` : "Delete all"}
          </button>
        )}
      </header>
      {!state ? null : !orders.length ? (
        <p className="empty">No orders yet. Place one at the order counter.</p>
      ) : (
        <div className="table-wrap">
          <table>
            <thead>
              <tr><th>#</th><th>Customer</th><th>Items</th><th>Status</th><th className="placed">Placed</th>
                <th><span className="sr-only">Actions</span></th></tr>
            </thead>
            <tbody>
              {orders.map((o) => (
                <tr key={o.id}>
                  <td className="num">{o.id}</td>
                  <td>{o.customer_name || "—"}</td>
                  <td>{o.items_text}</td>
                  <td><span className={`status ${o.status}`} title={o.error || ""}>{WORDS[o.status] || o.status}</span></td>
                  <td className="sub placed">{new Date(o.created_at).toLocaleTimeString([], { hour12: false })}</td>
                  <td className="btns">
                    {o.status === "ready" && <span className="sub">Robot picks it up next</span>}
                    {o.status === "queued" && !auto && (
                      <button className="act" disabled={busy.has(o.id)} onClick={() => act(o, "action")}>Action</button>
                    )}
                    {DELETABLE.includes(o.status) && (
                      <button className="del" disabled={busy.has(o.id)} onClick={() => act(o, "delete")}
                              aria-label={`Delete order ${o.id}`}>Delete</button>
                    )}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </section>
  );
}
