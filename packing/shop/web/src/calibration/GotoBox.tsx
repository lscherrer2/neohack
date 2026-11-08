import { useState } from "react";
import { api, errText } from "../api";
import type { ArmState, CalSession, TrayState } from "../types";
import ConfirmDialog from "./ConfirmDialog";

/** Server reply of POST /api/arm/goto (see packing/shop/server.py). */
interface GotoReply {
  plan: { point: string; standoff_mm: number; approach_mm: number; tilt_deg: number; travel_units: number; line_mm: number };
  calibration: string; // "saved calibration" | "live fit"
  motors_will_turn_on: boolean;
  started: boolean;
}

/** Server reply of POST /api/arm/pick (packing/tray/pick.py summary). */
interface PickReply {
  plan: {
    pocket: string; sku: string; pitch_deg: number; tilt_deg: number; pull_mm: number; lift_mm: number;
    grasp_standoff_mm: number; travel_units: number; pull_twist_deg: number; jaw_off_deg: number;
    steps: { step: string; label: string }[];
  };
  calibration: string;
  motors_will_turn_on: boolean;
  started: boolean;
}

const INCH_MM = 25.4;

/**
 * "Go to" preview: move the tip to a stand-off out along the tray normal from a corner / pocket, tool pointing
 * into the tray as closely as the arm allows. Always: plan (dry run) -> confirm dialog with the plan -> move.
 * While moving, a Stop button halts at the next control tick.
 */
export default function GotoBox({ arm, tray, session, setTarget }: {
  arm: ArmState | null; tray: TrayState | null; session: CalSession; setTarget: (p: string | null) => void;
}) {
  const [point, setPoint] = useState("corner_bl");
  const [standoffIn, setStandoffIn] = useState(0.5);
  const [plan, setPlan] = useState<GotoReply | null>(null);
  const [pick, setPick] = useState<PickReply | null>(null);
  const [busy, setBusy] = useState<"plan" | "go" | "pick" | "stop" | "release" | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [confirmRelease, setConfirmRelease] = useState(false);

  const moving = arm?.mode === "moving";
  const canPlan = arm?.connected && !moving && arm.mode !== "packing";
  const label = (id: string) =>
    tray?.corners?.find((c) => c.id === id)?.label ?? id;

  async function act<T>(what: typeof busy, url: string, body?: unknown): Promise<T | null> {
    setBusy(what);
    setError(null);
    try {
      return await api<T>("POST", url, body);
    } catch (e) {
      setError(errText(e));
      return null;
    } finally {
      setBusy(null);
    }
  }

  async function planMove() {
    setTarget(point);
    const r = await act<GotoReply>("plan", "/api/arm/goto", { point, standoff_mm: standoffIn * INCH_MM });
    if (r) setPlan(r);
  }

  async function go() {
    setPlan(null);
    await act<GotoReply>("go", "/api/arm/goto", { point, standoff_mm: standoffIn * INCH_MM, confirm_move: true });
  }

  const isPocket = (tray?.pockets ?? []).some((pk) => pk.id === point);

  async function planPick() {
    setTarget(point);
    const r = await act<PickReply>("pick", "/api/arm/pick", { pocket: point });
    if (r) setPick(r);
  }

  async function goPick() {
    setPick(null);
    await act<PickReply>("go", "/api/arm/pick", { pocket: point, confirm_move: true });
  }

  const k = pick?.plan;
  const pickBody = k
    ? `Test pick of the ${k.sku} in ${k.pocket}: open the gripper, free move (largest joint change ` +
      `${k.travel_units.toFixed(0)} units) to the approach point, slowly straight in to ` +
      `${k.grasp_standoff_mm.toFixed(1)} mm off the tray face, grip, pull straight out ${k.pull_mm.toFixed(0)} mm ` +
      `along the normal, lift ${k.lift_mm.toFixed(0)} mm and hold the part there. ` +
      `Tool pitched ${k.pitch_deg.toFixed(0)}° down (${k.tilt_deg.toFixed(0)}° off the normal), ` +
      `turns ${k.pull_twist_deg.toFixed(0)}° during the pull, jaws ${k.jaw_off_deg.toFixed(0)}° off the grip direction. ` +
      `A missed grip opens and backs out. Uses the ${pick!.calibration}. Inventory is not changed.` +
      (pick!.motors_will_turn_on ? " The motors are off: they turn ON first and hold the arm where it is." : "") +
      " Keep the area clear; press Stop to halt."
    : "";

  const p = plan?.plan;
  const confirmBody = p
    ? `The arm will move to ${label(p.point)}: first a free move (largest joint change ${p.travel_units.toFixed(0)} units) ` +
      `to ${p.approach_mm.toFixed(0)} mm out, then slowly straight in to ${p.standoff_mm.toFixed(1)} mm ` +
      `(${(p.standoff_mm / INCH_MM).toFixed(2)} in) from it along the tray normal. ` +
      `The tool will point ${p.tilt_deg.toFixed(0)}° off the normal (closest this arm can do). ` +
      `Uses the ${plan!.calibration}.` +
      (plan!.motors_will_turn_on ? " The motors are off: they turn ON first and hold the arm where it is." : "") +
      " Keep the area clear; press Stop to halt."
    : "";

  return (
    <section className="goto" aria-labelledby="goto-h">
      <h3 id="goto-h">Go to point / pick test</h3>
      <p className="sub">Go to: tip just off a corner or pocket, pointing into the tray (checks the calibration).
        Pick test: grip the part in a pocket, pull it straight out, lift and hold.</p>
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
          <button type="button" className="act" disabled={!canPlan || busy !== null} onClick={planMove}
                  title={canPlan ? "Plan the move, then confirm" : "Connect the arm first"}>
            {busy === "plan" ? "Planning…" : "Go to…"}
          </button>
        )}
        {!moving && (
          <button type="button" className="act" disabled={!canPlan || !isPocket || busy !== null} onClick={planPick}
                  title={isPocket ? "Plan a test pick from this pocket, then confirm" : "Choose a pocket"}>
            {busy === "pick" ? "Planning…" : "Pick test…"}
          </button>
        )}
      </div>
      {moving && <p className="moving-note">Arm moving ({label(point)})…</p>}
      {session.active && arm?.torque && !moving && (
        <button type="button" className="linkish" onClick={() => setConfirmRelease(true)}>
          Motors off again to keep capturing (hold the arm)
        </button>
      )}
      {error && <p className="inline-err">{error}</p>}

      {plan && (
        <ConfirmDialog title={`Move the arm to ${label(plan.plan.point)}?`} body={confirmBody}
                       confirmLabel={plan.motors_will_turn_on ? "Turn motors on and move" : "Move the arm"}
                       onCancel={() => setPlan(null)} onConfirm={go} />
      )}
      {pick && (
        <ConfirmDialog title={`Test pick from ${pick.plan.pocket}?`} body={pickBody}
                       confirmLabel={pick.motors_will_turn_on ? "Turn motors on and pick" : "Pick"}
                       onCancel={() => setPick(null)} onConfirm={goPick} />
      )}
      {confirmRelease && (
        <ConfirmDialog title="Turn the motors off?" body="Hold the arm — motors will turn OFF and it will drop."
                       confirmLabel="I'm holding it, turn motors off" onCancel={() => setConfirmRelease(false)}
                       onConfirm={() => { setConfirmRelease(false); act("release", "/api/arm/release", { confirm_hold: true }); }} />
      )}
    </section>
  );
}
