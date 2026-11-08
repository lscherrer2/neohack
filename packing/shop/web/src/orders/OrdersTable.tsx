import { useEffect, useState } from "react";
import { api, errText } from "../api";
import { showFailed, showPlanned } from "../arm/plannedPath";
import type { Order, ShopState } from "../types";

const WORDS: Record<string, string> = {
  queued: "received", ready: "ready", packing: "packing", packed: "packed", failed: "failed", cancelled: "cancelled",
};
const COUNT_ORDER = ["packing", "ready", "queued", "packed", "failed", "cancelled"];
const DELETABLE = ["queued", "ready", "failed", "cancelled"];

export default function OrdersTable({ state, refreshState }: { state: ShopState | null; refreshState: () => void }) {
  const [busy, setBusy] = useState<Set<number>>(new Set());
  const autoPack = state?.auto_pack;
  const auto = autoPack?.enabled ?? state?.auto_dispatch ?? false;
  const [autoBusy, setAutoBusy] = useState(false);
  const [armDeleteAll, setArmDeleteAll] = useState(false); // second click within 3 s deletes
  const [deletingAll, setDeletingAll] = useState(false);
  const [checks, setChecks] = useState<Record<number, { ok: boolean; text: string }>>({});

  /** Dry run of packing this order on the arm (box in, picks + drops, box out): nothing moves. */
  async function check(o: Order) {
    setChecks((c) => ({ ...c, [o.id]: { ok: true, text: "Checking…" } }));
    try {
      const r = await api<{ plan: { box_id: number; items: { pocket: string; drop_tilt_deg: number }[] } }>(
        "POST", `/api/orders/${o.id}/pack-plan`);
      showPlanned(`pack order ${o.id}`, r);
      setChecks((c) => ({ ...c, [o.id]: { ok: true,
        text: `OK: box ${r.plan.box_id}, ${r.plan.items.map((i) => i.pocket).join(", ")} — the arm can pack it ` +
              "(path shown on the Controls tab)" } }));
    } catch (e) {
      showFailed(`pack order ${o.id}`, e);
      setChecks((c) => ({ ...c, [o.id]: { ok: false, text: errText(e) + " (see where on the Controls tab)" } }));
    }
  }

  useEffect(() => {
    if (!armDeleteAll) return;
    const t = setTimeout(() => setArmDeleteAll(false), 3000);
    return () => clearTimeout(t);
  }, [armDeleteAll]);

  /** Auto pack: the robot packs the newest received order with all its stock once a box is ready. */
  async function setAuto(enabled: boolean) {
    if (enabled === auto) return;
    setAutoBusy(true);
    try {
      await api("PUT", "/api/auto-pack", { enabled });
    } catch (e) {
      alert(errText(e) || "Couldn't change auto pack.");
    }
    setAutoBusy(false);
    refreshState();
  }

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
      <div className="auto-pack">
        <span className="label">Auto pack</span>
        <div className="seg" role="group" aria-label="Auto pack">
          <button type="button" className={!auto ? "on" : ""} aria-pressed={!auto} disabled={autoBusy || !state}
                  onClick={() => setAuto(false)}>Off</button>
          <button type="button" className={auto ? "on" : ""} aria-pressed={auto} disabled={autoBusy || !state}
                  onClick={() => setAuto(true)}>On</button>
        </div>
        <span className="sub">
          {!auto ? "Only orders sent with Action are packed."
            : autoPack?.next_order != null ? `Packs order ${autoPack.next_order} next (newest with all its stock).`
            : autoPack?.waiting ? `Waiting: ${autoPack.waiting}.`
            : "Packs the newest received order with all its stock once a box is ready."}
        </span>
      </div>
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
                  <td>{o.items_text}
                    {!!o.slots?.length && ["queued", "ready", "packing", "failed"].includes(o.status) && (
                      <div className={`slots sub${o.slots.includes(null) ? " short" : ""}`}>
                        {o.slots.includes(null)
                          ? `Short: ${o.slots.filter((x) => x === null).length} item(s) have no unit in stock`
                          : `Reserved: ${o.slots.join(", ")}`}
                      </div>
                    )}
                    {o.callback_status && (
                      <div className={`slots ${o.callback_status.startsWith("failed") ? "short" : "sub"}`}>
                        Callback: {o.callback_status}
                      </div>
                    )}
                    {checks[o.id] && (
                      <div className={`slots ${checks[o.id].ok ? "sub" : "short"}`}>{checks[o.id].text}</div>
                    )}
                  </td>
                  <td><span className={`status ${o.status}`} title={o.error || ""}>{WORDS[o.status] || o.status}</span></td>
                  <td className="sub placed">{new Date(o.created_at).toLocaleTimeString([], { hour12: false })}</td>
                  <td className="btns">
                    {o.status === "ready" && <span className="sub">Robot picks it up next</span>}
                    {(o.status === "queued" || o.status === "ready" || o.status === "failed") && (
                      <button className="linkish check-btn" title="Dry run: can the arm pack this order? Nothing moves."
                              onClick={() => check(o)}>Check</button>
                    )}
                    {o.status === "queued" && (
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
