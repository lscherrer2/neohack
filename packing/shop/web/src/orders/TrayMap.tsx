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
  return (
    <g className={`pocket${pending ? " pending" : ""}`} transform={`translate(${p.x} ${p.y}) rotate(${p.yaw_deg})`}
       role="button" tabIndex={0} aria-pressed={p.full}
       aria-label={`Pocket ${p.id}, ${p.full ? "full" : "empty"}. Toggle.`}
       onClick={onToggle}
       onKeyDown={(e) => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); onToggle(); } }}>
      <title>{`${p.id}: ${p.full ? "full" : "empty"} (click to mark ${p.full ? "empty" : "full"})`}</title>
      <g className="hit">{hit}</g>
      {mark}
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

  return (
    <section aria-labelledby="tray-h">
      <header>
        <h2 id="tray-h">Parts tray</h2>
        <span className="sub">{total} of {shown.length} pockets full. Click a pocket to toggle it.</span>
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
            return (
              <tr key={t.sku}>
                <td><span className="swatch" style={{ background: colors[t.sku] }} /> <strong>{t.name}</strong></td>
                <td className="sub">{n} of {mine.length} full</td>
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
                      onClick={() => { if (confirm("Mark every tray pocket empty?")) fill(false); }}>Empty all</button>
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
