import { useSyncExternalStore } from "react";
import { ApiError } from "../api";
import type { Vec3 } from "../types";

/** One piece of a planned tool-point path (server: packing/motion/program.py tip_path). Robot frame, meters. */
export interface PathSeg {
  kind: "free" | "line" | "grip" | "release";
  label: string;
  points: Vec3[];
}

/** The last dry run's path, drawn in the 3D views. `fail` = where planning failed (if it did). */
export interface PlannedPath {
  title: string;
  segments: PathSeg[];
  fail: Vec3 | null;
  failText: string | null;
}

let current: PlannedPath | null = null;
const listeners = new Set<() => void>();

export function setPlannedPath(p: PlannedPath | null) {
  current = p;
  listeners.forEach((l) => l());
}

export function usePlannedPath(): PlannedPath | null {
  return useSyncExternalStore((l) => { listeners.add(l); return () => listeners.delete(l); }, () => current);
}

/** From a successful dry-run reply ({path: [...]}). */
export function showPlanned(title: string, reply: unknown) {
  const path = (reply as { path?: PathSeg[] } | null)?.path;
  setPlannedPath(path ? { title, segments: path, fail: null, failText: null } : null);
}

/** From a failed dry run (422 with {message, path, fail_m}). Clears the view for other errors. */
export function showFailed(title: string, e: unknown) {
  const d = e instanceof ApiError ? (e.data as { message?: string; path?: PathSeg[]; fail_m?: Vec3 | null } | null) : null;
  if (!d || (!d.path?.length && !d.fail_m)) { setPlannedPath(null); return; }
  setPlannedPath({ title, segments: d.path ?? [], fail: d.fail_m ?? null, failText: d.message ?? null });
}
