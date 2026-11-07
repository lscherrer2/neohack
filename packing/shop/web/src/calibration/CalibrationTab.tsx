import { useCallback, useEffect, useRef, useState } from "react";
import { api, ApiError, errText, useEventStream, usePoll } from "../api";
import type { TrayLoad } from "../App";
import type { ArmState, CalSession } from "../types";
import Panel from "./Panel";
import Scene from "./Scene";

const STALE_MS = 2000;

export type Avail<T> = { data: T | null; unavailable: string | null };

export default function CalibrationTab({ trayLoad }: { trayLoad: TrayLoad }) {
  // ---- live arm state: SSE ~10 Hz, GET /api/arm once for availability / initial value
  const [arm, setArm] = useState<Avail<ArmState>>({ data: null, unavailable: null });
  const lastMsg = useRef(0);
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    const t = setInterval(() => setNow(Date.now()), 500);
    return () => clearInterval(t);
  }, []);

  const fetchArm = useCallback(async () => {
    try {
      const a = await api<ArmState>("GET", "/api/arm");
      lastMsg.current = Date.now();
      setArm({ data: a, unavailable: null });
    } catch (e) {
      if (e instanceof ApiError && e.unavailable) setArm((s) => ({ data: s.data, unavailable: e.detail }));
    }
  }, []);
  useEffect(() => { fetchArm(); }, [fetchArm]);

  const stream = useEventStream("/api/arm/stream", (raw) => {
    try {
      const a = JSON.parse(raw) as ArmState;
      lastMsg.current = Date.now();
      setArm({ data: a, unavailable: null });
    } catch { /* ignore a bad frame */ }
  }, { probeUrl: "/api/arm" });
  useEffect(() => { if (stream === "unavailable") fetchArm(); }, [stream, fetchArm]);

  const stale = stream !== "open" || now - lastMsg.current > STALE_MS;

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
        if ("connected" in out && "mode" in out) setArm({ data: out as ArmState, unavailable: null });
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
  }, [refreshCal, trayLoad]);

  return (
    <main className="cal-grid">
      <section className="scene-wrap" aria-label="3D view of the arm and the tray">
        <Scene arm={arm.data} stale={stale} tray={trayLoad.tray} session={cal.data} highlighted={highlighted} />
        <div className="scene-status">
          <span className={`dot ${arm.unavailable ? "dot-off" : stale ? "dot-stale" : "dot-live"}`} />
          {arm.unavailable ? "Arm view not available on the server yet"
            : stream === "connecting" ? "Connecting to the arm stream…"
            : stale ? (arm.data ? "Arm data stale — showing the last pose" : "No arm data yet")
            : `Live · ${arm.data?.source ?? "?"}`}
        </div>
      </section>
      <Panel arm={arm} stale={stale} cal={cal} tray={trayLoad.tray} trayMissing={trayLoad.missing}
             target={target} setTarget={setTarget} highlighted={highlighted}
             run={run} busy={busy} error={error} clearError={() => setError(null)} />
    </main>
  );
}
