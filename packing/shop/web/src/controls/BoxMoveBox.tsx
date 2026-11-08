import { useState } from "react";
import { showFailed, showPlanned } from "../arm/plannedPath";
import { api, errText } from "../api";
import type { ArmState, BoxesState, BoxInfo } from "../types";

type Dest = BoxInfo["location"];
const DEST: Record<Dest, string> = { supply: "its left spot", pack: "to the pack spot (middle)", output: "to its right spot (done)" };
const ORDER: Dest[] = ["supply", "pack", "output"]; // boxes only move left → right

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
  const dests = box ? ORDER.slice(ORDER.indexOf(box.location) + 1) : [];
  const [dest, setDest] = useState<Dest>("pack");
  const to = dests.includes(dest) ? dest : dests[0];
  const [busy, setBusy] = useState<"go" | "stop" | null>(null);
  const [error, setError] = useState<string | null>(null);
  const moving = arm?.mode === "moving";
  const canPlan = !!arm?.connected && arm.mode === "idle" && !!box && !!to;

  /** `pathTitle`: a dry run — show its planned path (or where it fails) in the 3D view. */
  async function post<T>(what: typeof busy, url: string, body?: unknown, pathTitle?: string): Promise<T | null> {
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
                      title={canPlan ? "Plan and run the move (motors lock first if released)" : !arm?.connected ? "Connect the arm first" : `The arm is ${arm?.mode}`}
                      onClick={async () => {
                        const r = await post<MoveReply>("go", "/api/arm/box-move", { box_id: box!.id, to, confirm_move: true },
                                                        `move box ${box!.id} ${DEST[to]}`);
                        if (r) onMoved();
                      }}>{busy === "go" ? "Starting…" : "Move"}</button>
            )}
          </div>
        )}
        {error && <p className="inline-err">{error}</p>}
      </div>
    </section>
  );
}
