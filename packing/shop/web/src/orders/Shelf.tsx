import { useState } from "react";
import { api, errText } from "../api";
import type { StockItem } from "../types";

export default function Shelf({ stock, refreshState }: { stock: StockItem[] | null; refreshState: () => void }) {
  const [busy, setBusy] = useState(false);
  const [note, setNote] = useState("");

  async function restock() {
    setBusy(true);
    try {
      const d = await api<{ parts_added: number; boxes_added: number }>("POST", "/api/shelf/restock");
      setNote(`Added ${d.parts_added} parts and ${d.boxes_added} boxes.`);
    } catch (e) {
      setNote(`Restock failed: ${errText(e)}`);
    }
    setBusy(false);
    refreshState();
  }

  const empty = !stock || stock.every((p) => p.on_shelf === 0);
  return (
    <section aria-labelledby="stock-h">
      <header>
        <h2 id="stock-h">Shelf</h2>
        <span>
          <span className="sub restock-note">{note}</span>
          <button type="button" className="act" disabled={busy} onClick={restock}
                  title="Click after refilling the part slots by hand">Restock</button>
        </span>
      </header>
      {!stock ? null : empty ? (
        <p className="empty">The shelf is empty. Fill the part slots, then click Restock.</p>
      ) : (
        <ul className="stock">
          {stock.map((p) => {
            const free = Math.max(0, p.on_shelf - p.reserved);
            return (
              <li key={p.sku}>
                <strong>{p.name}</strong>{" "}
                <span className="sub">
                  {free} free of {p.on_shelf} on the shelf{p.reserved ? `, ${p.reserved} reserved by orders` : ""}
                </span>
              </li>
            );
          })}
        </ul>
      )}
    </section>
  );
}
