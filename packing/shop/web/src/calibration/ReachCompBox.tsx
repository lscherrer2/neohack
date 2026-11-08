import { useCallback, useEffect, useState } from "react";
import { api, errText } from "../api";

interface ReachCompState {
  enabled: boolean;
  lift_mm: number;
  at_reach: number;
  power: number;
  max_reach_mm?: number;
  table?: { reach: number; r_mm: number; lift_mm: number }[];
}

/**
 * Reach compensation (optional): aim higher the further out the target is — extra sag correction.
 * lift = lift_mm · (r / (at_reach · max reach))^power. Applied when a move / order runs (dry runs show the true
 * aim). GET/PUT /api/reach-comp.
 */
export default function ReachCompBox() {
  const [rc, setRc] = useState<ReachCompState | null>(null);
  const [mm, setMm] = useState("");
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const load = useCallback(async () => {
    try {
      const r = await api<ReachCompState>("GET", "/api/reach-comp");
      setRc(r);
      setMm(String(r.lift_mm));
    } catch { /* older server */ }
  }, []);
  useEffect(() => { load(); }, [load]);

  async function put(body: { enabled?: boolean; lift_mm?: number }) {
    setBusy(true);
    setError(null);
    try {
      const r = await api<ReachCompState>("PUT", "/api/reach-comp", body);
      setRc(r);
      setMm(String(r.lift_mm));
    } catch (e) { setError(errText(e)); } finally { setBusy(false); }
  }

  if (!rc) return null;
  const pct = Math.round(rc.at_reach * 100);
  const typed = Number(mm);
  const mmChanged = mm !== "" && Number.isFinite(typed) && typed !== rc.lift_mm;
  return (
    <section aria-labelledby="reach-h">
      <header>
        <h2 id="reach-h">Reach compensation</h2>
        <span className="sub">{rc.enabled ? `on · +${rc.lift_mm} mm at ${pct}% reach` : "off"}</span>
      </header>
      <div className="cal-body">
        <p className="instr sub">
          Extra sag correction: the further out the arm aims, the higher it aims —
          +{rc.lift_mm} mm at {pct}% of its max reach{rc.max_reach_mm ? ` (${(rc.max_reach_mm * rc.at_reach / 10).toFixed(0)} cm
          of ${(rc.max_reach_mm / 10).toFixed(0)} cm)` : ""}, almost nothing close in. Applies to go-to, picks, box moves and
          orders from the next run; calibrations and dry-run paths are unchanged.
        </p>
        <div className="row-btns">
          <label>
            <input type="checkbox" checked={rc.enabled} disabled={busy}
                   onChange={(e) => put({ enabled: e.target.checked })} /> Aim higher further out
          </label>
          <label>
            mm at {pct}%:{" "}
            <input type="number" min={0} max={40} step={0.5} value={mm} disabled={busy} style={{ width: "5em" }}
                   onChange={(e) => setMm(e.target.value)}
                   onKeyDown={(e) => { if (e.key === "Enter" && mmChanged) put({ lift_mm: typed }); }} />
          </label>
          {mmChanged && (
            <button type="button" className="act" disabled={busy} onClick={() => put({ lift_mm: typed })}>Set</button>
          )}
        </div>
        {rc.table && (
          <table className="points">
            <thead><tr><th>Out from arm</th><th>Aims higher by</th></tr></thead>
            <tbody>
              {rc.table.map((t) => (
                <tr key={t.reach}>
                  <td>{(t.r_mm / 10).toFixed(0)} cm <span className="sub">({Math.round(t.reach * 100)}%)</span></td>
                  <td>{rc.enabled ? `+${t.lift_mm.toFixed(1)} mm` : <span className="sub">+{t.lift_mm.toFixed(1)} mm (off)</span>}</td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
        {error && <p className="inline-err">{error}</p>}
      </div>
    </section>
  );
}
