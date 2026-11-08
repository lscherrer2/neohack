import { useMemo, useState } from "react";
import { api, errText, itemColor } from "../api";
import type { TrayLoad } from "../App";
import type { Pocket } from "../types";

const MIN_HIT_MM = 9; // thin pockets (breadboard slot, piezo pins) still get a clickable area

function PocketMark({ p, color, pending, onToggle }: {
  p: Pocket; color: string; pending: boolean; onToggle: () => void;
}) {
  const s = p.shape;
  const fill = p.full ? color : "var(--surface)";
  const res = p.reserved_by ?? null;
  const common = { fill, stroke: color, strokeWidth: 1.2 };
  let mark, hit;
  if (s.kind === "circle") {
    mark = <circle r={s.d / 2} {...common} />;
    hit = <circle r={Math.max(s.d / 2, MIN_HIT_MM / 2) + 1} />;
  } else if (s.kind === "rect") {
    mark = <rect x={-s.w / 2} y={-s.h / 2} width={s.w} height={s.h} rx={0.6} {...common} />;
    const hw = Math.max(s.w, MIN_HIT_MM) / 2 + 1, hh = Math.max(s.h, MIN_HIT_MM) / 2 + 1;
    hit = <rect x={-hw} y={-hh} width={2 * hw} height={2 * hh} />;
  } else {
    const h = s.spacing / 2;
    mark = (
      <>
        {/* full: the piezo disc sitting on its pins */}
        <circle r={6} fill={p.full ? color : "none"} stroke={color} strokeWidth={1} strokeDasharray={p.full ? undefined : "2 1.5"} />
        <circle cx={-h} r={Math.max(s.d / 2, 0.9)} fill={p.full ? "var(--surface)" : color} />
        <circle cx={h} r={Math.max(s.d / 2, 0.9)} fill={p.full ? "var(--surface)" : color} />
      </>
    );
    hit = <circle r={7} />;
  }
  const state = `${p.full ? "full" : "empty"}${res != null ? `, reserved for order ${res}` : ""}`;
  const tip = p.locked ? `${p.id}: ${state} — locked while order ${res} is being packed`
    : `${p.id}: ${state} (click to mark ${p.full ? "empty" : "full"})`;
  const act = () => { if (!p.locked) onToggle(); };
  const r = s.kind === "rect" ? Math.max(s.w, s.h) / 2 : s.kind === "circle" ? s.d / 2 : 6;
  return (
    <g className={`pocket${pending ? " pending" : ""}${res != null ? " reserved" : ""}${p.locked ? " locked" : ""}`}
       transform={`translate(${p.x} ${p.y})`}
       role="button" tabIndex={0} aria-pressed={p.full} aria-disabled={p.locked || undefined}
       aria-label={`Pocket ${p.id}, ${state}.${p.locked ? " Locked." : " Toggle."}`}
       onClick={act}
       onKeyDown={(e) => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); act(); } }}>
      <title>{tip}</title>
      <g transform={`rotate(${p.yaw_deg})`}>
        <g className="hit">{hit}</g>
        {mark}
        {res != null && s.kind !== "rect" && <circle className="res-ring" r={r + 1.6} />}
        {res != null && s.kind === "rect" && (
          <rect className="res-ring" x={-s.w / 2 - 1.6} y={-s.h / 2 - 1.6} width={s.w + 3.2} height={s.h + 3.2} rx={1.5} />
        )}
      </g>
      {res != null && (
        <text className="res-tag" transform={`translate(${s.kind === "rect" ? s.w / 2 + 3 : r + 2.5} ${r * 0.2}) scale(1 -1)`}>
          {p.locked ? "🔒" : ""}#{res}
        </text>
      )}
    </g>
  );
}

/** 2D parts tray, to scale (mm). DXF y is up; the y=0 row (nearest the arm) is drawn at the bottom. */
export default function TrayMap({ trayLoad }: { trayLoad: TrayLoad }) {
  const { tray, missing, reload } = trayLoad;
  const [pending, setPending] = useState<Record<string, boolean>>({}); // pocket id -> wanted state
  const [busySku, setBusySku] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const colors = useMemo(() => Object.fromEntries((tray?.types ?? []).map((t, i) => [t.sku, itemColor(i)])), [tray?.types]);

  if (missing) return null; // no tray configured: the shelf section covers stock
  if (!tray) {
    return (
      <section aria-labelledby="tray-h">
        <header><h2 id="tray-h">Parts tray</h2></header>
        <p className="empty">Loading the tray…</p>
      </section>
    );
  }

  async function toggle(p: Pocket) {
    const want = !(pending[p.id] ?? p.full);
    setPending((m) => ({ ...m, [p.id]: want }));
    setError(null);
    try {
      await api("POST", `/api/tray/pockets/${encodeURIComponent(p.id)}`, { full: want });
    } catch (e) {
      setError(`Couldn't mark ${p.id}: ${errText(e)}`);
    }
    await reload();
    setPending((m) => { const n = { ...m }; delete n[p.id]; return n; });
  }

  async function fill(full: boolean, sku?: string) {
    setBusySku(sku ?? "*");
    setError(null);
    try {
      await api("POST", "/api/tray/fill", sku ? { full, sku } : { full });
    } catch (e) {
      setError(errText(e));
    }
    await reload();
    setBusySku(null);
  }

  const o = tray.outline;
  const m = 4; // margin, mm
  const w = o.xmax - o.xmin, h = o.ymax - o.ymin;
  const shown = tray.pockets.map((p) => (p.id in pending ? { ...p, full: pending[p.id] } : p));
  const total = shown.filter((p) => p.full).length;
  const reservedN = shown.filter((p) => p.reserved_by != null).length;

  return (
    <section aria-labelledby="tray-h">
      <header>
        <h2 id="tray-h">Parts tray</h2>
        <span className="sub">{total} of {shown.length} pockets full{reservedN ? `, ${reservedN} reserved (#order)` : ""}.
          Click a pocket to toggle it.</span>
      </header>
      <div className="traymap">
        <svg viewBox={`${o.xmin - m} ${-(o.ymax + m)} ${w + 2 * m} ${h + 2 * m}`} role="group" aria-label="Parts tray map">
          <g transform="scale(1 -1)">
            <rect className="plate-outline" x={o.xmin} y={o.ymin} width={w} height={h} rx={o.corner_r} />
            {shown.map((p) => (
              <PocketMark key={p.id} p={p} color={colors[p.sku] ?? "var(--muted)"} pending={p.id in pending}
                          onToggle={() => toggle(p)} />
            ))}
          </g>
        </svg>
        <p className="arm-side sub">▼ arm side (row 1)</p>
      </div>
      {error && <p className="inline-err" role="alert">{error}</p>}
      <table className="tray-items">
        <tbody>
          {tray.types.map((t) => {
            const mine = shown.filter((p) => p.sku === t.sku);
            const n = mine.filter((p) => p.full).length;
            const resv = mine.filter((p) => p.reserved_by != null).length;
            return (
              <tr key={t.sku}>
                <td><span className="swatch" style={{ background: colors[t.sku] }} /> <strong>{t.name}</strong></td>
                <td className="sub">{n} of {mine.length} full{resv ? ` · ${resv} reserved · ${n - resv} free` : ""}</td>
                <td className="btns">
                  <button className="act" disabled={busySku !== null || n === mine.length} onClick={() => fill(true, t.sku)}>Fill all</button>
                  <button className="del" disabled={busySku !== null || n === 0} onClick={() => fill(false, t.sku)}>Empty all</button>
                </td>
              </tr>
            );
          })}
          <tr>
            <td><strong>Every pocket</strong></td>
            <td className="sub">{total} of {shown.length} full</td>
            <td className="btns">
              <button className="act" disabled={busySku !== null || total === shown.length} onClick={() => fill(true)}>Fill all</button>
              <button className="del" disabled={busySku !== null || total === 0}
                      onClick={() => fill(false)}>Empty all</button>
            </td>
          </tr>
        </tbody>
      </table>
      {tray.unmatched.length > 0 && (
        <details className="unmatched sub">
          <summary>{tray.unmatched.length} drawing feature(s) didn't match any item type</summary>
          <ul>{tray.unmatched.map((u) => <li key={u}>{u}</li>)}</ul>
        </details>
      )}
    </section>
  );
}
