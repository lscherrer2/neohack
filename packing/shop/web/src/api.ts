import { useEffect, useRef, useState } from "react";

/** A failed request. `status` 0 = network error (server unreachable). */
export class ApiError extends Error {
  constructor(public status: number, public detail: string) {
    super(detail);
  }
  /** Endpoint missing or backend part not running yet: show "not available", not an error. */
  get unavailable(): boolean {
    return this.status === 0 || this.status === 404 || this.status === 503;
  }
}

function detailText(d: unknown, status: number): string {
  if (typeof d === "string") return d;
  if (Array.isArray(d)) // FastAPI validation errors
    return d.map((e) => (e && typeof e === "object" && "msg" in e ? String((e as { msg: unknown }).msg) : JSON.stringify(e))).join("; ");
  if (d != null) return JSON.stringify(d);
  return `The server answered ${status}.`;
}

export async function api<T>(method: string, url: string, body?: unknown): Promise<T> {
  let r: Response;
  try {
    r = await fetch(url, {
      method,
      headers: body === undefined ? undefined : { "Content-Type": "application/json" },
      body: body === undefined ? undefined : JSON.stringify(body),
    });
  } catch {
    throw new ApiError(0, "Couldn't reach the shop server.");
  }
  const data = await r.json().catch(() => null);
  if (!r.ok) throw new ApiError(r.status, detailText(data?.detail, r.status));
  return data as T;
}

export const errText = (e: unknown) => (e instanceof Error ? e.message : String(e));

/** Run `fn` now and every `ms`, plus whenever `deps` change. */
export function usePoll(fn: () => void, ms: number, deps: unknown[] = []) {
  const ref = useRef(fn);
  ref.current = fn;
  useEffect(() => {
    ref.current();
    const t = setInterval(() => ref.current(), ms);
    return () => clearInterval(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ms, ...deps]);
}

export type StreamStatus = "connecting" | "open" | "down" | "unavailable";

/**
 * EventSource with reconnect. The browser retries by itself on dropped connections, but gives up
 * (readyState CLOSED) when the endpoint answers non-200 (e.g. 404 while the backend isn't built yet):
 * then we probe `probeUrl` to tell "unavailable" from "down" and reopen after `retryMs`.
 */
export function useEventStream(url: string | null, onMessage: (data: string) => void,
                               opts: { probeUrl?: string; retryMs?: number } = {}): StreamStatus {
  const [status, setStatus] = useState<StreamStatus>("connecting");
  const cb = useRef(onMessage);
  cb.current = onMessage;
  const { probeUrl, retryMs = 5000 } = opts;

  useEffect(() => {
    if (!url) return;
    let es: EventSource | null = null;
    let timer: ReturnType<typeof setTimeout> | undefined;
    let dead = false;

    const open = () => {
      if (dead) return;
      setStatus((s) => (s === "open" ? s : "connecting"));
      es = new EventSource(url);
      es.onopen = () => setStatus("open");
      es.onmessage = (m) => cb.current(m.data);
      es.onerror = () => {
        if (!es || es.readyState !== EventSource.CLOSED) { setStatus("down"); return; }
        es = null;
        const retry = () => { timer = setTimeout(open, retryMs); };
        if (!probeUrl) { setStatus("down"); retry(); return; }
        fetch(probeUrl)
          .then((r) => setStatus(r.status === 404 || r.status === 503 ? "unavailable" : "down"))
          .catch(() => setStatus("down"))
          .finally(retry);
      };
    };
    open();
    return () => { dead = true; clearTimeout(timer); es?.close(); };
  }, [url, probeUrl, retryMs]);

  return status;
}

// ---------- formatting
export const clock = (ts: number) => new Date(ts * 1000).toLocaleTimeString([], { hour12: false });
export const ago = (s: number | null | undefined) =>
  s == null ? "never" : s < 2 ? "just now" : s < 120 ? `${Math.round(s)} seconds ago` : `${Math.round(s / 60)} minutes ago`;

// Item colors in the tray map / 3D view, by position in tray.types.
const PALETTE = ["#D9822B", "#3E7CB1", "#8E5BA8", "#2F8F6A", "#B5485D", "#6C7A2E", "#4A9BB0"];
export const itemColor = (i: number) => PALETTE[((i % PALETTE.length) + PALETTE.length) % PALETTE.length];
