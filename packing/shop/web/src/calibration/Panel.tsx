import { useState } from "react";
import type { ArmState, CalSession, TrayState } from "../types";
import type { Avail } from "./CalibrationTab";
import ConfirmDialog from "./ConfirmDialog";
import GotoBox from "./GotoBox";

interface Props {
  arm: Avail<ArmState>;
  stale: boolean;
  cal: Avail<CalSession>;
  tray: TrayState | null;
  trayMissing: string | null;
  target: string | null;
  setTarget: (p: string | null) => void;
  highlighted: string | null;
  run: (name: string, method: string, url: string, body?: unknown) => Promise<boolean>;
  busy: string | null;
  error: string | null;
  clearError: () => void;
}

const fmt = (v: number | null | undefined, digits = 1, unit = "") => (v == null ? "—" : `${v.toFixed(digits)}${unit}`);
const yesNo = (v: boolean | null | undefined, yes: string, no: string) => (v == null ? "—" : v ? yes : no);

/** Corner ids read as plain labels ("Bottom-left corner"); pockets keep their id ("Bottom-left (aa_1)"). */
const isCornerId = (id: string) => id.startsWith("corner_");
const captureName = (r: { pocket: string; label: string }) => (isCornerId(r.pocket) ? r.label : `${r.label} (${r.pocket})`);

type CalMode = CalSession["mode"];

/** Segmented control for the reference-point set. `mode` undefined = an older server without corner support. */
export function ModeSwitch({ mode, busy, onPick }: { mode: CalMode | undefined; busy: string | null; onPick: (m: CalMode) => void }) {
  const supported = mode !== undefined;
  return (
    <div className="mode-row">
      <span className="mode-label" id="mode-l">Reference points</span>
      <div className="seg" role="radiogroup" aria-labelledby="mode-l">
        {(["corners", "pockets"] as const).map((m) => (
          <button key={m} type="button" role="radio" aria-checked={mode === m}
                  className={mode === m ? "on" : ""} disabled={!supported || busy !== null}
                  onClick={() => { if (mode !== m) onPick(m); }}>
            {busy === `mode:${m}` ? "Switching…" : m === "corners" ? "Plate corners" : "Pocket centers"}
          </button>
        ))}
      </div>
      {!supported && (
        <p className="mode-note">This server doesn't support corner mode yet — restart it to update.</p>
      )}
    </div>
  );
}

/** Arm connection + status. `compact` (Calibration tab) leaves out the joint values; Controls shows them. */
export function ArmStatus({ arm, stale, run, busy, compact = false }: Pick<Props, "arm" | "stale" | "run" | "busy"> & { compact?: boolean }) {
  const a = arm.data;
  return (
    <section aria-labelledby="arm-h">
      <header>
        <h2 id="arm-h">Arm</h2>
        {arm.unavailable ? <span className="sub">not available</span>
          : <span className="sub">{a?.connected ? (stale ? "connected, data stale" : "connected") : "not connected"}</span>}
      </header>
      {arm.unavailable && !a ? (
        <p className="empty">The arm endpoints aren't available on this server yet ({arm.unavailable}).</p>
      ) : (
        <div className="kv-wrap">
          <dl className="kv">
            <dt>Source</dt><dd>{a?.source ?? "—"}</dd>
            <dt>Torque</dt><dd>{yesNo(a?.torque, "on (holding)", "off (limp)")}</dd>
            <dt>Mode</dt><dd>{a?.mode ?? "—"}</dd>
            <dt>Last data</dt><dd>{a?.stale_s == null ? "—" : `${a.stale_s.toFixed(1)} s ago`}</dd>
            <dt>Tip</dt><dd>{a?.tip ? a.tip.map((v) => (v * 1000).toFixed(0)).join(", ") + " mm" : "—"}</dd>
          </dl>
          {a?.error && <p className="inline-err">{a.error}</p>}
          {a?.connected && a.source === "sim" && (
            <p className="sim-note">Simulated arm — this server was started without <code>--robot</code>, so it never
              talks to the real arm. Run <code>python -m packing.shop start --robot</code> to use it.</p>
          )}
          {!compact && a?.joints && (
            <div className="joints">
              <p className="sub">Joint values (units; body −100..100, gripper 0..100)</p>
              <dl className="kv">
                {Object.entries(a.joints).map(([k, v]) => (<span key={k} className="kv-row"><dt>{k}</dt><dd>{v.toFixed(1)}</dd></span>))}
              </dl>
            </div>
          )}
          <div className="row-btns">
            {!a?.connected ? (
              <button className="act" disabled={busy !== null} onClick={() => run("connect", "POST", "/api/arm/connect")}>
                {busy === "connect" ? "Connecting…" : "Connect"}
              </button>
            ) : (
              <button className="del" disabled={busy !== null || a.mode !== "idle"}
                      title={a.mode !== "idle" ? `Can't disconnect while ${a.mode}` : "Close the arm (torque stays on)"}
                      onClick={() => run("disconnect", "POST", "/api/arm/disconnect")}>Disconnect</button>
            )}
          </div>
        </div>
      )}
    </section>
  );
}

export default function Panel(p: Props) {
  const { cal, tray, run, busy, error } = p;
  const [confirm, setConfirm] = useState<null | "start" | "finish">(null);
  const [extra, setExtra] = useState("");
  const s = cal.data;
  const names = Object.fromEntries((tray?.types ?? []).map((t) => [t.sku, t.name]));
  const pointFor = (pocket: string) => s?.points.find((pt) => pt.pocket === pocket);
  const refIds = new Set(s?.reference_points.map((r) => r.pocket));
  const extras = s?.points.filter((pt) => !refIds.has(pt.pocket)) ?? [];

  async function capture(pocket: string) {
    p.setTarget(pocket);
    await run(`capture:${pocket}`, "POST", "/api/calibration/capture", { pocket });
  }

  return (
    <div className="panel stack">
      {error && (
        <div className="inline-err banner" role="alert">
          {error} <button type="button" className="linkish" onClick={p.clearError}>Dismiss</button>
        </div>
      )}

      <ArmStatus arm={p.arm} stale={p.stale} run={run} busy={busy} compact />

      <section aria-labelledby="cal-h">
        <header>
          <h2 id="cal-h">Tray calibration</h2>
          <span className="sub">{s ? (s.active ? "capturing — motors off" : "not running") : ""}</span>
        </header>

        {cal.unavailable && !s ? (
          <p className="empty">Calibration isn't available on this server yet ({cal.unavailable}).</p>
        ) : p.trayMissing ? (
          <p className="empty">No parts tray configured ({p.trayMissing}).</p>
        ) : !s ? (
          <p className="empty">Loading…</p>
        ) : (
          <div className="cal-body">
            <SavedLine saved={s.saved} tray={tray} />

            {!s.active ? (
              <>
                <p className="sub">
                  Touch each reference point with the gripper tip while the motors are off. The arm goes limp: hold it first.
                </p>
                <div className="cal-actions single">
                  <button className="big" disabled={busy !== null} onClick={() => setConfirm("start")}>Start calibration</button>
                </div>
              </>
            ) : (
              <>
                <ModeSwitch mode={(s as Partial<CalSession>).mode} busy={busy}
                            onPick={(m) => { p.setTarget(null); run(`mode:${m}`, "POST", "/api/calibration/mode", { mode: m }); }} />
                <p className="sub instr">
                  {s.mode === "corners"
                    ? "Hold the tip (red dot) to the middle of each rounded plate corner, on the top edge, then press its button."
                    : "Guide the tip (red dot) onto the center of each pocket, at the tray face, then press its button."}
                </p>
                <div className="captures">
                  {s.reference_points.map((r) => {
                    const pt = pointFor(r.pocket);
                    return (
                      <div key={r.pocket} className={`capture-row${p.highlighted === r.pocket ? " targeted" : ""}`}>
                        <button className="big capture" disabled={busy !== null}
                                onMouseEnter={() => p.setTarget(r.pocket)} onFocus={() => p.setTarget(r.pocket)}
                                onClick={() => capture(r.pocket)}>
                          {busy === `capture:${r.pocket}` ? "Capturing…" : `${r.captured ? "Recapture" : "Capture"} ${captureName(r)}`}
                        </button>
                        <span className={`cap-state${r.captured ? " done" : ""}`}>
                          {r.captured ? <><span className="check" aria-label="captured">✓</span> {fmt(pt?.residual_mm, 1, " mm")}</> : "not yet"}
                        </span>
                        {pt ? (
                          <button className="del cap-del" disabled={busy !== null} aria-label={`Delete point ${r.pocket}`}
                                  title="Delete this point"
                                  onClick={() => run(`del:${r.pocket}`, "DELETE", `/api/calibration/points/${encodeURIComponent(r.pocket)}`)}>×</button>
                        ) : <span aria-hidden="true" />}
                      </div>
                    );
                  })}
                </div>

                <div className="extra">
                  <label htmlFor="extra-pocket">Extra point</label>
                  <select id="extra-pocket" value={extra}
                          onChange={(e) => { setExtra(e.target.value); p.setTarget(e.target.value || null); }}>
                    <option value="">Choose a point…</option>
                    <optgroup label="Plate corners">
                      {(tray?.corners ?? []).map((c) => (
                        <option key={c.id} value={c.id}>{c.label}{pointFor(c.id) ? " ✓" : ""}</option>
                      ))}
                    </optgroup>
                    <optgroup label="Pockets">
                      {(tray?.pockets ?? []).map((pk) => (
                        <option key={pk.id} value={pk.id}>{pk.id}{pointFor(pk.id) ? " ✓" : ""}</option>
                      ))}
                    </optgroup>
                  </select>
                  <button className="act" disabled={!extra || busy !== null} onClick={() => capture(extra)}>Capture</button>
                </div>

                {extras.length > 0 && (
                  <table className="points">
                    <thead><tr><th>Extra point</th><th>Residual</th><th /></tr></thead>
                    <tbody>
                      {extras.map((pt) => (
                        <tr key={pt.pocket}>
                          <td>{pt.label ? `${pt.label} (${pt.pocket})` : pt.pocket}</td>
                          <td>{fmt(pt.residual_mm, 1, " mm")}</td>
                          <td className="btns">
                            <button className="del" disabled={busy !== null}
                                    onClick={() => run(`del:${pt.pocket}`, "DELETE", `/api/calibration/points/${encodeURIComponent(pt.pocket)}`)}>Delete</button>
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                )}
              </>
            )}

            <FitBox session={s} names={names} />

            {(s.saved || s.fit?.ok) && (
              <GotoBox arm={p.arm.data} tray={tray} session={s} setTarget={p.setTarget} />
            )}

            {s.active && (
              <div className="cal-actions">
                <button className="big" disabled={busy !== null || !s.fit?.ok}
                        title={s.fit?.ok ? "Save this fit as the tray calibration" : "Needs a good fit first"}
                        onClick={() => run("save", "POST", "/api/calibration/save")}>
                  {busy === "save" ? "Saving…" : "Save calibration"}
                </button>
                <button className="big warn" disabled={busy !== null} onClick={() => setConfirm("finish")}>Finish</button>
              </div>
            )}
          </div>
        )}
      </section>

      {confirm === "start" && (
        <ConfirmDialog title="Start calibration?" body="Hold the arm — motors will turn OFF and it will drop."
                       confirmLabel="I'm holding it, turn motors off"
                       onCancel={() => setConfirm(null)}
                       onConfirm={() => { setConfirm(null); run("start", "POST", "/api/calibration/start", { confirm_hold: true }); }} />
      )}
      {confirm === "finish" && (
        <ConfirmDialog title="Finish calibration?" body="Hold the arm — motors turn back ON and it will hold its current pose."
                       confirmLabel="I'm holding it, turn motors on"
                       onCancel={() => setConfirm(null)}
                       onConfirm={() => {
                         setConfirm(null); p.setTarget(null); setExtra("");
                         run("finish", "POST", "/api/calibration/finish", { confirm_hold: true });
                       }} />
      )}
    </div>
  );
}

function SavedLine({ saved, tray }: { saved: CalSession["saved"]; tray: TrayState | null }) {
  const c = tray?.calibration;
  if (!saved && !c) return <p className="saved none">No saved calibration — the 3D tray is a guess.</p>;
  const created = saved?.created ?? c?.created ?? "";
  const rms = saved?.rms_mm ?? c?.rms_mm;
  return (
    <p className="saved">
      Saved calibration: <strong>{created ? new Date(created).toLocaleString() : "—"}</strong>, rms {fmt(rms, 2, " mm")}
      {c ? `, tilt ${fmt(c.tilt_deg, 1, "°")}` : ""}
    </p>
  );
}

function FitBox({ session, names }: { session: CalSession; names: Record<string, string> }) {
  const f = session.fit;
  if (!session.active && !f) return null;
  return (
    <div className={`fit${f?.ok ? " fit-ok" : f ? " fit-bad" : ""}`} aria-live="polite">
      <h3>Live fit</h3>
      {!f ? (
        <p className="sub">Capture at least three points to see a fit.</p>
      ) : (
        <>
          <dl className="kv">
            <dt>Result</dt><dd>{f.ok ? "good — can be saved" : "not usable yet"}</dd>
            <dt>RMS error</dt><dd>{fmt(f.rms_mm, 2, " mm")}</dd>
            <dt>Tray tilt</dt><dd>{fmt(f.tilt_deg, 1, "°")}</dd>
          </dl>
          {f.error && <p className="inline-err">{f.error}</p>}
          {f.warnings.length > 0 && <ul className="warnings">{f.warnings.map((w) => <li key={w}>{w}</li>)}</ul>}
          {Object.keys(f.grip_angles).length > 0 && (
            <>
              <h4>Grip angles</h4>
              <dl className="kv">
                {Object.entries(f.grip_angles).map(([sku, deg]) => (
                  <span key={sku} className="kv-row"><dt>{names[sku] ?? sku}</dt><dd>{deg.toFixed(1)}°</dd></span>
                ))}
              </dl>
            </>
          )}
        </>
      )}
    </div>
  );
}
