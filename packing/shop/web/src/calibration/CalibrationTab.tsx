import { useCallback, useState } from "react";
import { api, ApiError, errText, usePoll } from "../api";
import type { TrayLoad } from "../App";
import { type Avail, liveStatus, useArmLive } from "../arm/useArmLive";
import { useBoxes } from "../arm/useBoxes";
import PathOverlay from "../arm/PathOverlay";
import { usePlannedPath } from "../arm/plannedPath";
import BoxesBox from "./BoxesBox";
import DeskBox from "./DeskBox";
import ReachCompBox from "./ReachCompBox";
import type { ArmState, CalSession, DeskPlaneInfo, DeskState, Vec3 } from "../types";
import Panel from "./Panel";
import Scene from "./Scene";

export type { Avail };

export default function CalibrationTab({ trayLoad }: { trayLoad: TrayLoad }) {
  const { arm, stale, stream, applyArm } = useArmLive();
  const planned = usePlannedPath();
  const [deskPts, setDeskPts] = useState<Vec3[]>([]);
  const [deskPlane, setDeskPlane] = useState<DeskPlaneInfo | null>(null);
  const onDesk = useCallback((d: DeskState) => {
    setDeskPts(d.points.length ? d.points : d.saved?.points ?? []);
    setDeskPlane(d.saved);
  }, []);
  const { boxes, apply: applyBoxes } = useBoxes();

  // ---- calibration session
  const [cal, setCal] = useState<Avail<CalSession>>({ data: null, unavailable: null });
  const refreshCal = useCallback(async () => {
    try {
      setCal({ data: await api<CalSession>("GET", "/api/calibration"), unavailable: null });
    } catch (e) {
      if (e instanceof ApiError && e.unavailable) setCal({ data: null, unavailable: e.detail });
    }
  }, []);
  usePoll(refreshCal, 2000);

  // ---- which pocket the user is about to capture (highlighted in 3D)
  const [target, setTarget] = useState<string | null>(null);
  const firstOpen = cal.data?.reference_points.find((r) => !r.captured)?.pocket ?? null;
  const highlighted = target ?? (cal.data?.active ? firstOpen : null);

  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState<string | null>(null);

  /** Run one API action; a CalSession / ArmState answer updates the view directly. */
  const run = useCallback(async (name: string, method: string, url: string, body?: unknown) => {
    setBusy(name);
    setError(null);
    try {
      const out = await api<unknown>(method, url, body);
      if (out && typeof out === "object") {
        if ("reference_points" in out) setCal({ data: out as CalSession, unavailable: null });
        if ("connected" in out && "mode" in out) applyArm(out as ArmState);
      }
      return true;
    } catch (e) {
      setError(e instanceof ApiError && e.unavailable && e.status !== 0
        ? `Not available yet on the server (${e.status}): ${e.detail}` : errText(e));
      return false;
    } finally {
      setBusy(null);
      refreshCal();
      trayLoad.reload(); // a saved calibration changes the tray pose / pocket robot_m
    }
  }, [refreshCal, trayLoad, applyArm]);
  const status = liveStatus(arm, stale, stream);

  return (
    <main className="cal-grid">
      <section className="scene-wrap" aria-label="3D view of the arm and the tray">
        <Scene arm={arm.data} stale={stale} tray={trayLoad.tray} session={cal.data} highlighted={highlighted}
               boxes={boxes} planned={planned} deskPoints={deskPts} deskPlane={deskPlane} />
        <div className="scene-status"><span className={`dot ${status.dot}`} />{status.text}</div>
        <PathOverlay />
      </section>
      <Panel arm={arm} stale={stale} cal={cal} tray={trayLoad.tray} trayMissing={trayLoad.missing}
             target={target} setTarget={setTarget} highlighted={highlighted}
             run={run} busy={busy} error={error} clearError={() => setError(null)}>
        <BoxesBox arm={arm.data} boxes={boxes} apply={applyBoxes} />
        <DeskBox arm={arm.data} onChange={onDesk} />
        <ReachCompBox />
      </Panel>
    </main>
  );
}
