import { useState } from "react";
import { api, errText } from "../api";
import type { ArmState, BoxesState, BoxInfo } from "../types";
import ConfirmDialog from "./ConfirmDialog";

const WHERE: Record<BoxInfo["location"], string> = { supply: "left (supply)", pack: "pack spot", output: "right (done)" };
const STATE: Record<BoxInfo["state"], string> = { available: "empty", in_use: "in use", packed: "packed" };

/**
 * Packing boxes: calibrate by hand (motors off, jaws closed on the tab, gripper pointing down) → Add box.
 * Each box's tab faces the arm's swivel; supply boxes sit on the left. POST /api/boxes/capture.
 */
export default function BoxesBox({ arm, boxes, apply }: {
  arm: ArmState | null; boxes: BoxesState | null; apply: (b: BoxesState) => void;
}) {
  const [busy, setBusy] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [confirmRemove, setConfirmRemove] = useState<number | null>(null);
  const canCapture = !!arm?.connected && arm.torque === false;

  async function act(name: string, method: string, url: string) {
    setBusy(name);
    setError(null);
    try {
      apply(await api<BoxesState>(method, url));
    } catch (e) {
      setError(errText(e));
    } finally {
      setBusy(null);
    }
  }

  const list = boxes?.boxes ?? [];
  return (
    <section aria-labelledby="boxes-h">
      <header>
        <h2 id="boxes-h">Packing boxes</h2>
        <span className="sub">{list.length ? `${list.filter((b) => b.state === "available").length} empty of ${list.length}` : "none yet"}</span>
      </header>
      <div className="cal-body">
        <p className="instr sub">
          Put the empty boxes on the <strong>left</strong> of the arm, tabs pointing at the arm's swivel, box centers at
          least <strong>31 cm from the swivel</strong> (closer, the arm can't hold the gripper level at the tab) and
          ≥ 12 cm apart. For each box: release the motors (hold the arm), hold the gripper <strong>level</strong>,
          pointing along the tab toward the box, close the jaws up/down on the tab near the wall, then press
          <strong>Add box</strong>.
        </p>
        <button type="button" className="big" disabled={!canCapture || busy !== null}
                title={!arm?.connected ? "Connect the arm first" : arm.torque ? "Release the motors first (Controls → Motors, or Start calibration)" : "Add the box whose tab is in the jaws"}
                onClick={() => act("add", "POST", "/api/boxes/capture")}>
          {busy === "add" ? "Adding…" : "Add box (jaws closed on its tab)"}
        </button>
        {arm?.connected && arm.torque && (
          <p className="sub">Motors are on — release them first (Controls tab → Motors → Released, or Start calibration above).</p>
        )}
        {error && <p className="inline-err">{error}</p>}
        {list.length > 0 && (
          <table className="points">
            <thead><tr><th>Box</th><th>Where</th><th>State</th><th>Position</th><th /></tr></thead>
            <tbody>
              {list.map((b) => (
                <tr key={b.id}>
                  <td>{b.id}</td>
                  <td>{WHERE[b.location]}</td>
                  <td>{b.state === "in_use" ? `order ${b.order_id}` : STATE[b.state]}</td>
                  <td className="sub">{b.pose ? `${(b.pose[0][3] * 1000).toFixed(0)}, ${(b.pose[1][3] * 1000).toFixed(0)} mm` : "—"}
                    {b.report?.tilt_deg != null && ` · tilt ${b.report.tilt_deg}°`}</td>
                  <td className="btns">
                    {(b.location !== "supply" || b.state !== "available") && b.state !== "in_use" && (
                      <button className="act" disabled={busy !== null} title="Someone put it back, empty, at its left spot"
                              onClick={() => act(`reset:${b.id}`, "POST", `/api/boxes/${b.id}/reset`)}>Reset</button>
                    )}
                    <button className="del" disabled={busy !== null || b.state === "in_use"}
                            onClick={() => setConfirmRemove(b.id)}>Remove</button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </div>
      {confirmRemove != null && (
        <ConfirmDialog title={`Remove box ${confirmRemove}?`} body="Its calibration is deleted. Add it again to use it."
                       confirmLabel="Remove" onCancel={() => setConfirmRemove(null)}
                       onConfirm={() => { const id = confirmRemove; setConfirmRemove(null); act(`del:${id}`, "DELETE", `/api/boxes/${id}`); }} />
      )}
    </section>
  );
}
