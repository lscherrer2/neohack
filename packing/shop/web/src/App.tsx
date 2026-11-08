import { lazy, Suspense, useCallback, useEffect, useRef, useState } from "react";
import { api, useEventStream, usePoll } from "./api";
import type { ShopEvent, ShopState, TrayState } from "./types";
import { ApiError } from "./api";
import Plate from "./Plate";
import OrdersTab from "./orders/OrdersTab";

// three.js only loads when a tab with the 3D view is opened.
const CalibrationTab = lazy(() => import("./calibration/CalibrationTab"));
const ControlsTab = lazy(() => import("./controls/ControlsTab"));

type Tab = "orders" | "controls" | "calibration";
const TABS: { id: Tab; label: string }[] = [
  { id: "orders", label: "Orders" },
  { id: "controls", label: "Controls" },
  { id: "calibration", label: "Calibration" },
];
const tabFromHash = (): Tab => {
  const h = window.location.hash.slice(1);
  return h === "calibration" || h === "controls" ? h : "orders";
};

export type TrayLoad = { tray: TrayState | null; missing: string | null; reload: () => void };

export default function App() {
  const [tab, setTab] = useState<Tab>(tabFromHash);
  useEffect(() => {
    const on = () => setTab(tabFromHash());
    window.addEventListener("hashchange", on);
    return () => window.removeEventListener("hashchange", on);
  }, []);

  // ---- shop state (/api/state), polled + refreshed on every activity event
  const [state, setState] = useState<ShopState | null>(null);
  const [offline, setOffline] = useState(false);
  const refreshState = useCallback(async () => {
    try {
      setState(await api<ShopState>("GET", "/api/state"));
      setOffline(false);
    } catch {
      setOffline(true);
    }
  }, []);
  usePoll(refreshState, 1500);

  // ---- parts tray (/api/tray), shared by the 2D map and the 3D view
  const [tray, setTray] = useState<TrayState | null>(null);
  const [trayMissing, setTrayMissing] = useState<string | null>(null);
  const reloadTray = useCallback(async () => {
    try {
      setTray(await api<TrayState>("GET", "/api/tray"));
      setTrayMissing(null);
    } catch (e) {
      if (e instanceof ApiError && e.status === 404) { setTray(null); setTrayMissing(e.detail); }
    }
  }, []);
  usePoll(reloadTray, 5000);

  // ---- activity (SSE); kept here so it survives tab switches
  const [events, setEvents] = useState<{ ev: ShopEvent; fresh: boolean }[]>([]);
  const loaded = useRef(false);
  const streamStatus = useEventStream("/api/events/stream", (raw) => {
    let ev: ShopEvent;
    try { ev = JSON.parse(raw); } catch { return; }
    setEvents((list) => (list.some((x) => x.ev.seq === ev.seq) ? list
      : [{ ev, fresh: loaded.current }, ...list].slice(0, 300)));
    refreshState();
    if (ev.data && ("pocket" in ev.data || "sku" in ev.data)) reloadTray();
  });
  useEffect(() => {
    if (streamStatus !== "open") return;
    const t = setTimeout(() => { loaded.current = true; }, 800); // backlog first, then "fresh" ones
    return () => clearTimeout(t);
  }, [streamStatus]);

  const trayLoad: TrayLoad = { tray, missing: trayMissing, reload: reloadTray };

  return (
    <>
      <Plate state={state} />
      <div className={`conn${offline || streamStatus === "down" ? " show" : ""}`} role="alert">
        Lost connection to the shop server. Retrying every few seconds.
      </div>
      <nav className="tabs" aria-label="Dashboard sections">
        {TABS.map((t) => (
          <a key={t.id} href={`#${t.id}`} aria-current={tab === t.id ? "page" : undefined}>{t.label}</a>
        ))}
      </nav>
      {tab === "orders" ? (
        <OrdersTab state={state} refreshState={refreshState} events={events} trayLoad={trayLoad} />
      ) : (
        <Suspense fallback={<main className="loading-tab"><p className="empty">Loading the 3D view…</p></main>}>
          {tab === "controls" ? <ControlsTab trayLoad={trayLoad} /> : <CalibrationTab trayLoad={trayLoad} />}
        </Suspense>
      )}
    </>
  );
}
