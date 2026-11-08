import { useState } from "react";
import { showFailed, showPlanned } from "../arm/plannedPath";
import { api, errText } from "../api";
import type { ArmState, CalSession, TrayState } from "../types";

/** Server reply of POST /api/arm/goto (see packing/shop/server.py). */
interface GotoReply {
  plan: { point: string; standoff_mm: number; approach_mm: number; tilt_deg: number; travel_units: number; line_mm: number };
  calibration: string; // "saved calibration" | "live fit"
  motors_will_turn_on: boolean;
  started: boolean;
}

const INCH_MM = 25.4;

/**
 * "Go to" preview: move the tip to a stand-off out along the tray normal from a corner / pocket, tool pointing
 * into the tray as closely as the arm allows. One click plans and moves (the server refuses an unplannable move
 * before anything moves).
 * While moving, a Stop button halts at the next control tick.
 */
export default function GotoBox({ arm, tray, session, setTarget }: {
  arm: ArmState | null; tray: TrayState | null; session: CalSession; setTarget: (p: string | null) => void;
}) {
  const [point, setPoint] = useState("corner_bl");
  const [standoffIn, setStandoffIn] = useState(0.5);
  const [busy, setBusy] = useState<"go" | "stop" | "release" | null>(null);
  const [error, setError] = useState<string | null>(null);

  const moving = arm?.mode === "moving";
  const canPlan = arm?.connected && !moving && arm.mode !== "packing";
  const label = (id: string) =>
    tray?.corners?.find((c) => c.id === id)?.label ?? id;

  /** `pathTitle`: a dry run — show its planned path (or where it fails) in the 3D view. */
  async function act<T>(what: typeof busy, url: string, body?: unknown, pathTitle?: string): Promise<T | null> {
    setBusy(what);
    setError(null);
    try {
      const r = await api<T>("POST", url, body);
      if (pathTitle) showPlanned(pathTitle, r);
      return r;
    } catch (e) {
      setError(errText(e));
      if (pathTitle) showFailed(pathTitle, e);
      return null;
    } finally {
      setBusy(null);
    }
  }

  async function go() {
    setTarget(point);
    await act<GotoReply>("go", "/api/arm/goto", { point, standoff_mm: standoffIn * INCH_MM, confirm_move: true },
                         `go to ${label(point)}`);
  }

  return (
    <section className="goto" aria-labelledby="goto-h">
      <h3 id="goto-h">Check: go to point</h3>
      <p className="sub">Moves the tip just off a corner or pocket, pointing into the tray — to check the calibration.
        Picking parts is on the Controls tab.</p>
      <div className="goto-row">
        <select aria-label="Point to go to" value={point} disabled={moving}
                onChange={(e) => { setPoint(e.target.value); setTarget(e.target.value); }}>
          <optgroup label="Plate corners">
            {(tray?.corners ?? []).map((c) => <option key={c.id} value={c.id}>{c.label}</option>)}
          </optgroup>
          <optgroup label="Pockets">
            {(tray?.pockets ?? []).map((pk) => <option key={pk.id} value={pk.id}>{pk.id}</option>)}
          </optgroup>
        </select>
        <label className="standoff">
          <input type="number" min={0.25} max={3} step={0.25} value={standoffIn} disabled={moving}
                 onChange={(e) => setStandoffIn(Math.min(3, Math.max(0.25, Number(e.target.value) || 0.5)))} />
          in off
        </label>
        {moving ? (
          <button type="button" className="big stop" disabled={busy === "stop"}
                  onClick={() => act("stop", "/api/arm/stop")}>{busy === "stop" ? "Stopping…" : "Stop"}</button>
        ) : (
          <button type="button" className="act" disabled={!canPlan || busy !== null} onClick={go}
                  title={canPlan ? "Plan and move (motors turn on first if off)" : "Connect the arm first"}>
            {busy === "go" ? "Starting…" : "Go to"}
          </button>
        )}
      </div>
      {moving && <p className="moving-note">Arm moving ({label(point)})…</p>}
      {session.active && arm?.torque && !moving && (
        <button type="button" className="linkish" title="The arm drops — hold it first"
                onClick={() => act("release", "/api/arm/release", { confirm_hold: true })}>
          Motors off again to keep capturing (hold the arm)
        </button>
      )}
      {error && <p className="inline-err">{error}</p>}

    </section>
  );
}
