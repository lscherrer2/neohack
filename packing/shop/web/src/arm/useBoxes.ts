import { useCallback, useState } from "react";
import { api, usePoll } from "../api";
import type { BoxesState } from "../types";

/** Calibrated packing boxes (GET /api/boxes), polled. `apply` takes a BoxesState answer from an action. */
export function useBoxes(ms = 2000) {
  const [boxes, setBoxes] = useState<BoxesState | null>(null);
  const reload = useCallback(async () => {
    try { setBoxes(await api<BoxesState>("GET", "/api/boxes")); } catch { /* older server: no boxes */ }
  }, []);
  usePoll(reload, ms);
  return { boxes, reload, apply: setBoxes };
}
