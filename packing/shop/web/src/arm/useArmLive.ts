import { useCallback, useEffect, useRef, useState } from "react";
import { api, ApiError, useEventStream } from "../api";
import type { ArmState } from "../types";

const STALE_MS = 2000;

export type Avail<T> = { data: T | null; unavailable: string | null };

/** Live arm state for a tab: SSE /api/arm/stream (~10 Hz) + GET /api/arm for availability / the first value. */
export function useArmLive() {
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
  /** Apply an ArmState answer from an action right away (don't wait for the stream). */
  const applyArm = useCallback((a: ArmState) => setArm({ data: a, unavailable: null }), []);
  return { arm, stale, stream, applyArm };
}

/** Status line text + dot class for the 3D view overlay. */
export function liveStatus(arm: Avail<ArmState>, stale: boolean, stream: string): { dot: string; text: string } {
  if (arm.unavailable) return { dot: "dot-off", text: "Arm view not available on the server yet" };
  if (arm.data && !arm.data.connected) return { dot: "dot-off", text: "Arm not connected" };
  if (stream === "connecting") return { dot: "dot-stale", text: "Connecting to the arm stream…" };
  if (stale) return { dot: "dot-stale", text: arm.data ? "Arm data stale — showing the last pose" : "No arm data yet" };
  return { dot: "dot-live", text: `Live · ${arm.data?.source ?? "?"}` };
}
