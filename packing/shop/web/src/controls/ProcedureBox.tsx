import { useCallback, useEffect, useState } from "react";
import { showFailed, showPlanned } from "../arm/plannedPath";
import { api, errText, usePoll } from "../api";
import ConfirmDialog from "../calibration/ConfirmDialog";
import type { ArmState, TrayState } from "../types";

/** Server reply of POST /api/arm/pick (packing/tray/pick.py summary). */
interface PickReply {
  plan: {
    pocket: string; sku: string; pitch_deg: number; tilt_deg: number; pull_mm: number; lift_mm: number;
    grasp_standoff_mm: number; travel_units: number; pull_twist_deg: number; jaw_off_deg: number;
    grip_mm: number; open_value: number | null; side_offset_mm?: number;
    steps: { step: string; label: string }[];
  };
  calibration: string;
  motors_will_turn_on: boolean;
  started: boolean;
}

/** GET /api/arm/goto: the arm program runner (go-to previews and picks share it). */
interface JobState {
  running: boolean;
  last: { plan: { pocket?: string; point?: string }; status: "running" | "done" | "stopped" | "failed";
          error: string | null; step?: string; grip_width?: number } | null;
}

/**
 * Procedures: pick a part from a pocket. Plan (dry run) -> confirm dialog with the plan -> run; Stop while running.
 * The pick: open to the part width + margin, approach along the tray normal, grip, pull straight out, lift, hold.
 */
export default function ProcedureBox({ arm, tray, setHighlighted }: {
  arm: ArmState | null; tray: TrayState | null; setHighlighted: (p: string | null) => void;
}) {
  const pockets = tray?.pockets ?? [];
  const types = tray?.types ?? [];
  const [pocket, setPocket] = useState<string>("");
  useEffect(() => { // default: the first full pocket
    if (!pocket && pockets.length) setPocket((pockets.find((p) => p.full) ?? pockets[0]).id);
  }, [pockets, pocket]);
  useEffect(() => { setHighlighted(pocket || null); }, [pocket, setHighlighted]);

  const [plan, setPlan] = useState<PickReply | null>(null);
  const [busy, setBusy] = useState<"plan" | "go" | "stop" | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [job, setJob] = useState<JobState | null>(null);
  const refreshJob = useCallback(async () => {
    try { setJob(await api<JobState>("GET", "/api/arm/goto")); } catch { /* server without a runner yet */ }
  }, []);
  usePoll(refreshJob, 700);

  const running = !!job?.running || arm?.mode === "moving";
  const canPlan = !!arm?.connected && !running && arm.mode === "idle";
  const name = (sku: string) => types.find((t) => t.sku === sku)?.name ?? sku;
  const pretty = (id: string) => {
    const pk = pockets.find((p) => p.id === id);
    return pk ? `${name(pk.sku)} ${id.split("_").pop()}` : id;
  };

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
      refreshJob();
    }
  }

  const k = plan?.plan;
  const body = k
    ? `Pick the ${name(k.sku)} from ${k.pocket}: open the gripper to ${k.open_value ?? "fully open"} ` +
      `(part ${k.grip_mm} mm + margin), free move (largest joint change ${k.travel_units.toFixed(0)} units) to the ` +
      `approach point, slowly straight in to ${k.grasp_standoff_mm.toFixed(1)} mm off the tray face beside the part ` +
      `(fixed jaw ${(k.side_offset_mm ?? 0).toFixed(1)} mm to the side), grip, pull straight out ${k.pull_mm.toFixed(0)} mm ` +
      `along the normal, lift ${k.lift_mm.toFixed(0)} mm and hold it there. ` +
      `Tool pitched ${k.pitch_deg.toFixed(0)}° down (${k.tilt_deg.toFixed(0)}° off the normal); it turns ` +
      `${k.pull_twist_deg.toFixed(0)}° during the pull; jaws ${k.jaw_off_deg.toFixed(0)}° off the grip direction. ` +
      `A missed grip opens and backs out. Uses the ${plan!.calibration}. Inventory is not changed.` +
      (plan!.motors_will_turn_on ? " The motors are released: they lock first and hold the arm where it is." : "") +
      " Keep the area clear; press Stop to halt."
    : "";

  const last = job?.last;
  const lastName = last?.plan.pocket ? pretty(last.plan.pocket) : last?.plan.point ?? "";

  return (
    <section aria-labelledby="proc-h">
      <header>
        <h2 id="proc-h">Procedures</h2>
        <span className="sub">{running ? "running" : "pick a part from the tray"}</span>
      </header>
      <div className="kv-wrap">
        {!tray ? <p className="empty">No parts tray configured on the server.</p> : (
          <div className="goto-row proc-row">
            <select aria-label="Pocket to pick from" value={pocket} disabled={running}
                    onChange={(e) => setPocket(e.target.value)}>
              {types.map((t) => (
                <optgroup key={t.sku} label={t.name}>
                  {pockets.filter((p) => p.sku === t.sku).map((p) => (
                    <option key={p.id} value={p.id}>{pretty(p.id)} ({p.id}){p.full ? "" : " — empty"}</option>
                  ))}
                </optgroup>
              ))}
            </select>
            {running ? (
              <button type="button" className="big stop" disabled={busy === "stop"}
                      onClick={() => act("stop", "/api/arm/stop")}>{busy === "stop" ? "Stopping…" : "Stop"}</button>
            ) : (
              <button type="button" className="act" disabled={!canPlan || !pocket || busy !== null}
                      title={canPlan ? "Plan the pick, then confirm" : !arm?.connected ? "Connect the arm first" : `The arm is ${arm?.mode}`}
                      onClick={async () => {
                        const r = await act<PickReply>("plan", "/api/arm/pick", { pocket }, `pick ${pretty(pocket)}`);
                        if (r) setPlan(r);
                      }}>
                {busy === "plan" ? "Planning…" : "Pick…"}
              </button>
            )}
          </div>
        )}
        {pockets.find((p) => p.id === pocket && !p.full) && !running && (
          <p className="sub">This pocket is marked empty — the grip will likely miss (the arm then backs out).</p>
        )}
        {last && (
          <p className={`job job-${last.status}`}>
            {last.status === "running" ? `Running ${lastName}: ${last.step ?? "starting"}…`
              : last.status === "done" ? `Done: ${lastName}${last.grip_width != null ? ` (gripped at width ${last.grip_width})` : ""} — holding.`
              : last.status === "stopped" ? `Stopped: ${lastName} — holding where it is.`
              : `Failed: ${lastName} — ${last.error ?? "unknown error"}`}
          </p>
        )}
        {error && <p className="inline-err">{error}</p>}
      </div>
      {plan && (
        <ConfirmDialog title={`Pick ${pretty(plan.plan.pocket)}?`} body={body}
                       confirmLabel={plan.motors_will_turn_on ? "Lock motors and pick" : "Pick"}
                       onCancel={() => setPlan(null)}
                       onConfirm={async () => {
                         const p = plan;
                         setPlan(null);
                         await act<PickReply>("go", "/api/arm/pick", { pocket: p.plan.pocket, confirm_move: true });
                       }} />
      )}
    </section>
  );
}
