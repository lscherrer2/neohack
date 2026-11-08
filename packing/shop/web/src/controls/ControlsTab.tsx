import { useCallback, useState } from "react";
import { api, ApiError, errText } from "../api";
import type { TrayLoad } from "../App";
import { liveStatus, useArmLive } from "../arm/useArmLive";
import { useBoxes } from "../arm/useBoxes";
import { ArmStatus } from "../calibration/Panel";
import Scene from "../calibration/Scene";
import type { ArmState } from "../types";
import BoxMoveBox from "./BoxMoveBox";
import MotorsBox from "./MotorsBox";
import ProcedureBox from "./ProcedureBox";

/**
 * Controls: run the arm. Live 3D view, connection + joint values, motors lock / release, and procedures
 * (pick a part from a pocket). Calibration lives on its own tab.
 */
export default function ControlsTab({ trayLoad }: { trayLoad: TrayLoad }) {
  const { arm, stale, stream, applyArm } = useArmLive();
  const { boxes, reload: reloadBoxes } = useBoxes();
  const [highlighted, setHighlighted] = useState<string | null>(null);
  const [busy, setBusy] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  /** One API action; an ArmState answer updates the view at once. */
  const run = useCallback(async (name: string, method: string, url: string, body?: unknown) => {
    setBusy(name);
    setError(null);
    try {
      const out = await api<unknown>(method, url, body);
      if (out && typeof out === "object" && "connected" in out && "mode" in out) applyArm(out as ArmState);
      return true;
    } catch (e) {
      setError(e instanceof ApiError && e.unavailable && e.status !== 0
        ? `Not available on this server (${e.status}) — restart it to update: ${e.detail}` : errText(e));
      return false;
    } finally {
      setBusy(null);
    }
  }, [applyArm]);

  const status = liveStatus(arm, stale, stream);
  return (
    <main className="cal-grid">
      <section className="scene-wrap" aria-label="3D view of the arm and the tray">
        <Scene arm={arm.data} stale={stale} tray={trayLoad.tray} session={null} highlighted={highlighted} boxes={boxes} />
        <div className="scene-status"><span className={`dot ${status.dot}`} />{status.text}</div>
      </section>
      <div className="panel stack">
        {error && (
          <p className="inline-err" role="alert">{error}{" "}
            <button type="button" className="linkish" onClick={() => setError(null)}>Dismiss</button></p>
        )}
        <ArmStatus arm={arm} stale={stale} run={run} busy={busy} />
        <MotorsBox arm={arm.data} run={run} busy={busy} />
        <ProcedureBox arm={arm.data} tray={trayLoad.tray} setHighlighted={setHighlighted} />
        <BoxMoveBox arm={arm.data} boxes={boxes} onMoved={reloadBoxes} />
      </div>
    </main>
  );
}
