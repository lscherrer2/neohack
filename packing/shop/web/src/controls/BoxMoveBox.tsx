import { useState } from "react";
import { api, errText } from "../api";
import ConfirmDialog from "../calibration/ConfirmDialog";
import type { ArmState, BoxesState, BoxInfo } from "../types";

type Dest = BoxInfo["location"];
const DEST: Record<Dest, string> = { supply: "back to its left spot", pack: "to the pack spot (middle)", output: "to its right spot (done)" };

interface MoveReply {
  plan: { box_id: number; from: Dest; to: Dest; travel_units: number; carry_mm: number; land_tilt_deg: number;
          max_carry_tilt_deg?: number;
          steps: { step: string; label: string }[] };
  motors_will_turn_on: boolean;
  started: boolean;
}

/** Move a calibrated box by its tab: lift straight up, carry, set down at the exact height. POST /api/arm/box-move. */
export default function BoxMoveBox({ arm, boxes, onMoved }: {
  arm: ArmState | null; boxes: BoxesState | null; onMoved: () => void;
}) {
  const list = boxes?.boxes ?? [];
  const [boxId, setBoxId] = useState<number | null>(null);
  const box = list.find((b) => b.id === boxId) ?? list[0];
  const dests = (["pack", "output", "supply"] as Dest[]).filter((d) => d !== box?.location);
  const [dest, setDest] = useState<Dest>("pack");
  const to = dests.includes(dest) ? dest : dests[0];
  const [plan, setPlan] = useState<MoveReply | null>(null);
  const [busy, setBusy] = useState<"plan" | "go" | "stop" | null>(null);
  const [error, setError] = useState<string | null>(null);
  const moving = arm?.mode === "moving";
  const canPlan = !!arm?.connected && arm.mode === "idle" && !!box;

  async function post<T>(what: typeof busy, url: string, body?: unknown): Promise<T | null> {
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

  const k = plan?.plan;
  const body = k
    ? `Box ${k.box_id}: open the jaws, come in level along its tab, grip, lift straight up ${k.carry_mm} mm, carry it ` +
      `${DEST[k.to]} (largest joint change ${k.travel_units.toFixed(0)} units; the box tilts up to ` +
      `${(k.max_carry_tilt_deg ?? 0).toFixed(0)}° on the way), lower it straight down to exactly the height it was ` +
      `picked from (box ${k.land_tilt_deg.toFixed(1)}° off level), release and back out level along the tab. ` +
      (plan!.motors_will_turn_on ? "The motors are released: they lock first and hold the arm where it is. " : "") +
      "Keep the area clear; press Stop to halt."
    : "";

  return (
    <section aria-labelledby="boxmove-h">
      <header>
        <h2 id="boxmove-h">Move a box</h2>
        <span className="sub">{list.length ? `${list.length} calibrated` : "calibrate boxes on the Calibration tab"}</span>
      </header>
      <div className="kv-wrap">
        {list.length > 0 && (
          <div className="goto-row">
            <select aria-label="Box" value={box?.id ?? ""} disabled={moving} onChange={(e) => setBoxId(Number(e.target.value))}>
              {list.map((b) => <option key={b.id} value={b.id}>Box {b.id} ({b.location})</option>)}
            </select>
            <select aria-label="Move it to" value={to} disabled={moving} onChange={(e) => setDest(e.target.value as Dest)}>
              {dests.map((d) => <option key={d} value={d}>{DEST[d]}</option>)}
            </select>
            {moving ? (
              <button type="button" className="big stop" disabled={busy === "stop"}
                      onClick={() => post("stop", "/api/arm/stop")}>{busy === "stop" ? "Stopping…" : "Stop"}</button>
            ) : (
              <button type="button" className="act" disabled={!canPlan || busy !== null}
                      title={canPlan ? "Plan the move, then confirm" : !arm?.connected ? "Connect the arm first" : `The arm is ${arm?.mode}`}
                      onClick={async () => {
                        const r = await post<MoveReply>("plan", "/api/arm/box-move", { box_id: box!.id, to });
                        if (r) setPlan(r);
                      }}>{busy === "plan" ? "Planning…" : "Move…"}</button>
            )}
          </div>
        )}
        {error && <p className="inline-err">{error}</p>}
      </div>
      {plan && (
        <ConfirmDialog title={`Move box ${plan.plan.box_id}?`} body={body}
                       confirmLabel={plan.motors_will_turn_on ? "Lock motors and move" : "Move the box"}
                       onCancel={() => setPlan(null)}
                       onConfirm={async () => {
                         const p = plan;
                         setPlan(null);
                         await post("go", "/api/arm/box-move", { box_id: p.plan.box_id, to: p.plan.to, confirm_move: true });
                         onMoved();
                       }} />
      )}
    </section>
  );
}
