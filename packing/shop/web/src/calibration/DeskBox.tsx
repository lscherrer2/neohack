import { useCallback, useState } from "react";
import { api, errText, usePoll } from "../api";
import type { ArmState, DeskState } from "../types";
import ConfirmDialog from "./ConfirmDialog";

/**
 * Desk plane: motors off, touch the closed jaw tip to the desk at 3+ spots far apart (left, middle, right) → Save.
 * Box spots then take their height from the real desk (a tilted base / uneven desk). POST /api/desk/*.
 */
export default function DeskBox({ arm, onChange }: { arm: ArmState | null; onChange: (d: DeskState) => void }) {
  const [desk, setDesk] = useState<DeskState | null>(null);
  const [busy, setBusy] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [confirmRemove, setConfirmRemove] = useState(false);
  const apply = useCallback((d: DeskState) => { setDesk(d); onChange(d); }, [onChange]);
  const reload = useCallback(async () => {
    try { apply(await api<DeskState>("GET", "/api/desk")); } catch { /* older server */ }
  }, [apply]);
  usePoll(reload, 3000);

  async function act(name: string, method: string, url: string) {
    setBusy(name);
    setError(null);
    try { apply(await api<DeskState>(method, url)); } catch (e) { setError(errText(e)); } finally { setBusy(null); }
  }

  if (!desk) return null;
  const canCapture = !!arm?.connected && arm.torque === false;
  const s = desk.saved;
  return (
    <section aria-labelledby="desk-h">
      <header>
        <h2 id="desk-h">Desk</h2>
        <span className="sub">{s ? `calibrated · tilt ${s.tilt_deg.toFixed(2)}°` : "not calibrated (assumed flat)"}</span>
      </header>
      <div className="cal-body">
        <p className="instr sub">
          Box spots (the pack spot, the right-side spots) take their height from the desk. With the motors released,
          touch the <strong>closed jaw tip</strong> to the desk at 3 or more spots far apart — left, middle, right,
          near where boxes go — pressing <strong>Capture</strong> at each, then <strong>Save</strong>.
        </p>
        {s && <p className="saved">Saved {new Date(s.created).toLocaleString()}: {s.points.length} points, RMS {s.rms_mm.toFixed(1)} mm,
          tilt {s.tilt_deg.toFixed(2)}°</p>}
        <div className="row-btns">
          <button type="button" className="act" disabled={!canCapture || busy !== null}
                  title={canCapture ? "Add the point the jaw tip touches now" : "Connect and release the motors first"}
                  onClick={() => act("cap", "POST", "/api/desk/capture")}>
            {busy === "cap" ? "Capturing…" : `Capture point ${desk.points.length + 1}`}
          </button>
          <button type="button" className="act" disabled={!desk.fit || busy !== null}
                  onClick={() => act("save", "POST", "/api/desk/save")}>{busy === "save" ? "Saving…" : "Save desk"}</button>
          {desk.points.length > 0 && (
            <button type="button" className="del" disabled={busy !== null}
                    onClick={() => act("clear", "DELETE", "/api/desk/points")}>Clear points</button>
          )}
          {s && (
            <button type="button" className="del" disabled={busy !== null} onClick={() => setConfirmRemove(true)}>
              Remove calibration</button>
          )}
        </div>
        {arm?.connected && arm.torque && <p className="sub">Release the motors first (Controls → Motors → Released).</p>}
        {desk.points.length > 0 && (
          <table className="points">
            <thead><tr><th>#</th><th>Point (mm)</th><th /></tr></thead>
            <tbody>
              {desk.points.map((p, i) => (
                <tr key={i}>
                  <td>{i + 1}</td>
                  <td className="sub">{p.map((v) => (v * 1000).toFixed(0)).join(", ")}</td>
                  <td className="btns"><button className="del" disabled={busy !== null}
                    onClick={() => act(`del${i}`, "DELETE", `/api/desk/points/${i}`)}>×</button></td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
        {desk.fit && <p className="sub">Fit: RMS {desk.fit.rms_mm.toFixed(1)} mm, tilt {desk.fit.tilt_deg.toFixed(2)}° — Save to use it.</p>}
        {desk.fit_error && desk.points.length > 0 && <p className="sub">{desk.fit_error}</p>}
        {error && <p className="inline-err">{error}</p>}
      </div>
      {confirmRemove && (
        <ConfirmDialog title="Remove the desk calibration?" body="Box spots go back to assuming a flat desk at the calibrated box's height."
                       confirmLabel="Remove" onCancel={() => setConfirmRemove(false)}
                       onConfirm={() => { setConfirmRemove(false); act("rm", "DELETE", "/api/desk"); }} />
      )}
    </section>
  );
}
