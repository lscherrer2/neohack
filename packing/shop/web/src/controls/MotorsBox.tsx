import { useState } from "react";
import ConfirmDialog from "../calibration/ConfirmDialog";
import type { ArmState } from "../types";

/**
 * Motors: Locked (torque on, holding the current pose so it doesn't slump) or Released (limp, movable by hand).
 * POST /api/arm/torque — only while idle; releasing needs a "hold the arm" confirmation (it drops).
 */
export default function MotorsBox({ arm, run, busy }: {
  arm: ArmState | null;
  run: (name: string, method: string, url: string, body?: unknown) => Promise<boolean>;
  busy: string | null;
}) {
  const [confirmRelease, setConfirmRelease] = useState(false);
  const connected = !!arm?.connected;
  const idle = arm?.mode === "idle";
  const locked = arm?.torque === true;
  const why = !connected ? "Connect the arm first"
    : arm?.mode === "calibrating" ? "Calibrating — the Calibration tab manages the motors"
    : !idle ? `The arm is ${arm?.mode}` : null;

  return (
    <section aria-labelledby="motors-h">
      <header>
        <h2 id="motors-h">Motors</h2>
        <span className="sub">{!connected ? "not connected" : locked ? "locked (holding)" : "released (limp)"}</span>
      </header>
      <div className="kv-wrap">
        <div className="seg" role="radiogroup" aria-label="Motors">
          <button type="button" role="radio" aria-checked={connected && locked} className={connected && locked ? "on" : ""}
                  disabled={!!why || busy !== null}
                  onClick={() => { if (!locked) run("lock", "POST", "/api/arm/torque", { on: true }); }}>
            {busy === "lock" ? "Locking…" : "Locked"}
          </button>
          <button type="button" role="radio" aria-checked={connected && !locked} className={connected && !locked ? "on" : ""}
                  disabled={!!why || busy !== null}
                  onClick={() => { if (locked) setConfirmRelease(true); }}>
            {busy === "release" ? "Releasing…" : "Released"}
          </button>
        </div>
        <p className="sub">
          {why ?? (locked
            ? "Holding the current pose (with gravity compensation). Holding heats the servos — release when idle."
            : "Limp: the arm can be moved by hand. Locking holds it wherever it is.")}
        </p>
      </div>
      {confirmRelease && (
        <ConfirmDialog title="Release the motors?" body="Hold the arm — the motors turn OFF and it will drop."
                       confirmLabel="I'm holding it, release"
                       onCancel={() => setConfirmRelease(false)}
                       onConfirm={() => {
                         setConfirmRelease(false);
                         run("release", "POST", "/api/arm/torque", { on: false, confirm_hold: true });
                       }} />
      )}
    </section>
  );
}
