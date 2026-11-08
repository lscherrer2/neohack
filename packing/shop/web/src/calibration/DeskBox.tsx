import { useCallback, useState } from "react";
import { api, errText, usePoll } from "../api";
import type { ArmState, DeskState, OffsetFit } from "../types";
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
  const [offsetFit, setOffsetFit] = useState<OffsetFit | null>(null);
  const [notes, setNotes] = useState<string[]>([]);
  const [confirmOffsets, setConfirmOffsets] = useState<"apply" | "remove" | null>(null);
  const apply = useCallback((d: DeskState) => { setDesk(d); onChange(d); }, [onChange]);
  const reload = useCallback(async () => {
    try { apply(await api<DeskState>("GET", "/api/desk")); } catch { /* older server */ }
  }, [apply]);
  usePoll(reload, 3000);

  async function act(name: string, method: string, url: string) {
    setBusy(name);
    setError(null);
    try {
      const d = await api<DeskState>(method, url);
      apply(d);
      if (url.startsWith("/api/joint-offsets")) {
        setOffsetFit(method === "POST" && url.endsWith("/fit") ? d.offset_fit ?? null : null);
        if (d.notes) setNotes(d.notes);
      } else if (url.startsWith("/api/desk/points") || url === "/api/desk/capture") {
        setOffsetFit(null); // the touches changed: fit again
      }
    } catch (e) { setError(errText(e)); } finally { setBusy(null); }
  }

  if (!desk) return null;
  const canCapture = !!arm?.connected && arm.torque === false;
  const s = desk.saved;
  const offsets = desk.joint_offsets?.offsets_deg ?? {};
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

        <h3>Joint angles</h3>
        <p className="instr sub">
          If the touches read right on the desk near the arm but higher (or lower) further out, the arm's
          lift/elbow zero is slightly off. <strong>Fit joint angles</strong> solves for the correction from
          6+ touches — near the arm <em>and</em> far out, left, middle and right (~25 touches let it fit the elbow
          too, not just the lift). Uses the touches above, else the saved desk calibration's.
        </p>
        <p className="saved">Current corrections: {Object.keys(offsets).length
          ? Object.entries(offsets).map(([j, v]) => `${j} ${v >= 0 ? "+" : ""}${v.toFixed(2)}°`).join(", ")
          : "none (plain calibration zero)"}</p>
        <div className="row-btns">
          <button type="button" className="act" disabled={busy !== null || (desk.points.length < 6 && !s?.has_poses)}
                  title="Dry run: nothing changes until Apply"
                  onClick={() => act("ofit", "POST", "/api/joint-offsets/fit")}>
            {busy === "ofit" ? "Fitting…" : "Fit joint angles"}</button>
          {offsetFit?.ok && (
            <button type="button" className="act" disabled={busy !== null} onClick={() => setConfirmOffsets("apply")}>
              Apply correction</button>
          )}
          {Object.keys(offsets).length > 0 && (
            <button type="button" className="del" disabled={busy !== null} onClick={() => setConfirmOffsets("remove")}>
              Remove corrections</button>
          )}
        </div>
        {offsetFit && (
          <>
            <p className="sub">
              {offsetFit.joints.map((j) => `${j} ${signed(offsetFit.delta_deg[j], 2)}° (±${offsetFit.std_deg[j].toFixed(2)})`).join(", ")}
              {" "}· touches {offsetFit.rms_before_mm.toFixed(1)} → {offsetFit.rms_after_mm.toFixed(1)} mm RMS off flat
              · desk tilt {offsetFit.tilt_before_deg.toFixed(2)}° → {offsetFit.tilt_after_deg.toFixed(2)}° · from {offsetFit.source}
            </p>
            {offsetFit.note && <p className="sub">{offsetFit.note}</p>}
            {offsetFit.problems.map((p) => <p key={p} className="inline-err">{p}</p>)}
            <table className="points">
              <thead><tr><th>From arm</th><th>Around</th><th>Off flat now</th><th>Corrected</th></tr></thead>
              <tbody>
                {offsetFit.points.map((p, i) => (
                  <tr key={i}>
                    <td>{(p.r_mm / 10).toFixed(0)} cm</td>
                    <td className="sub">{p.az_deg}°</td>
                    <td>{signed(p.before_mm, 1)} mm</td>
                    <td>{signed(p.after_mm, 1)} mm</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </>
        )}
        {notes.map((n) => <p key={n} className="sub"><strong>Note:</strong> {n}</p>)}
      </div>
      {confirmOffsets && (
        <ConfirmDialog
          title={confirmOffsets === "apply" ? "Apply the joint angle correction?" : "Remove the joint angle corrections?"}
          body={(confirmOffsets === "apply"
            ? "Every IK target and 3D position shifts slightly (taught poses in servo units don't change). "
            : "Back to the plain calibration zero. ")
            + "Desk points, the saved desk plane and the boxes are re-derived from their stored joints automatically; "
            + "the tray calibration must be redone. Nothing moves."}
          confirmLabel={confirmOffsets === "apply" ? "Apply" : "Remove"}
          onCancel={() => setConfirmOffsets(null)}
          onConfirm={() => {
            const which = confirmOffsets;
            setConfirmOffsets(null);
            act(which === "apply" ? "oapply" : "orm", which === "apply" ? "POST" : "DELETE",
                which === "apply" ? "/api/joint-offsets/apply" : "/api/joint-offsets");
          }} />
      )}
      {confirmRemove && (
        <ConfirmDialog title="Remove the desk calibration?" body="Box spots go back to assuming a flat desk at the calibrated box's height."
                       confirmLabel="Remove" onCancel={() => setConfirmRemove(false)}
                       onConfirm={() => { setConfirmRemove(false); act("rm", "DELETE", "/api/desk"); }} />
      )}
    </section>
  );
}

function signed(v: number, digits: number): string {
  return `${v >= 0 ? "+" : "−"}${Math.abs(v).toFixed(digits)}`;
}
