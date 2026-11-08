import { useState } from "react";
import { api, errText } from "../api";
import type { Order, ShopState } from "../types";

const WORDS: Record<string, string> = {
  queued: "received", ready: "ready", packing: "packing", packed: "packed", failed: "failed", cancelled: "cancelled",
};
const COUNT_ORDER = ["packing", "ready", "queued", "packed", "failed", "cancelled"];

export default function OrdersTable({ state, refreshState }: { state: ShopState | null; refreshState: () => void }) {
  const [busy, setBusy] = useState<Set<number>>(new Set());
  const auto = state?.auto_dispatch ?? false;

  async function act(o: Order, what: "action" | "delete") {
    if (what === "delete" && !confirm(`Delete order ${o.id}? This removes it for good.`)) return;
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
  return (
    <section aria-labelledby="orders-h">
      <header>
        <h2 id="orders-h">Orders</h2>
        <span className="sub">{COUNT_ORDER.filter((s) => counts[s]).map((s) => `${counts[s]} ${WORDS[s]}`).join(", ")}</span>
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
                    {["queued", "ready", "failed", "cancelled"].includes(o.status) && (
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
