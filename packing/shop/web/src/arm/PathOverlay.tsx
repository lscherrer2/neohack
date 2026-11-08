import { setPlannedPath, usePlannedPath } from "./plannedPath";

/** Over the 3D view: what the drawn path is (legend), why it failed, and a Clear button. */
export default function PathOverlay() {
  const p = usePlannedPath();
  if (!p) return null;
  return (
    <div className={`path-overlay${p.fail ? " failed" : ""}`}>
      <div className="path-title"><strong>{p.fail ? "Plan fails" : "Planned path"}:</strong> {p.title}
        <button type="button" className="linkish" onClick={() => setPlannedPath(null)}>Clear</button></div>
      {p.failText && <div className="path-why">{p.failText}</div>}
      <div className="path-legend">
        <span><i className="lg free" />free move</span><span><i className="lg line" />straight line</span>
        <span><i className="lg grip" />grip</span><span><i className="lg release" />release</span>
        {p.fail && <span><i className="lg fail" />fails here</span>}
      </div>
    </div>
  );
}
