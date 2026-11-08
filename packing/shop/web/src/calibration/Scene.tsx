import { Grid, Line, OrbitControls } from "@react-three/drei";
import { Canvas, useFrame } from "@react-three/fiber";
import { useMemo, useRef, type RefObject } from "react";
import * as THREE from "three";
import { itemColor } from "../api";
import type { PlannedPath } from "../arm/plannedPath";
import type { ArmState, BoxesState, BoxInfo, CalSession, Mat4, Pocket, TrayState, Vec3 } from "../types";

// Robot base frame: x forward, y left, z up, meters. three.js is y-up, so everything robot-side lives
// in <RobotFrame>, rotated -90° about x: robot (x, y, z) -> three (x, z, -y).
const ROBOT_TO_THREE: [number, number, number] = [-Math.PI / 2, 0, 0];

const JAW = "moving_jaw_so101_v1_link";

const C = { steel: "#3E5C76", ink: "#18222E", safety: "#F2B705", ok: "#2F7A4F", ghost: "#9AA8B6", plate: "#D6DCE2", hole: "#2A3440" };
const PLATE_MM = 6;

const toMatrix = (m: Mat4) => new THREE.Matrix4().set(...(m.flat() as Parameters<THREE.Matrix4["set"]>));
const origin = (m: Mat4) => new THREE.Vector3(m[0][3], m[1][3], m[2][3]);

/** Default tray pose when not calibrated: ~0.25 m in front, tilted 40° back toward the ceiling, tray x -> robot -y. */
function defaultTrayMatrix(o: TrayState["outline"]): Mat4 {
  const a = (40 * Math.PI) / 180, c = Math.cos(a), s = Math.sin(a);
  const cx = (o.xmin + o.xmax) / 2000, cy = (o.ymin + o.ymax) / 2000;
  const target = [0.25, 0, ((cy * 1000 - o.ymin) / 1000) * s + 0.01];
  const R = [[0, c, -s], [-1, 0, 0], [0, s, c]];
  const t = R.map((row, i) => target[i] - (row[0] * cx + row[1] * cy));
  return [[...R[0], t[0]], [...R[1], t[1]], [...R[2], t[2]], [0, 0, 0, 1]];
}

// ---------------------------------------------------------------- arm
// Stick figure through the real joint axes (from the URDF via the server's FK): one line per segment, an arc
// at each bend showing the joint angle, a short line for the moving jaw, and a bright red dot at the tool
// point (the point touched to pockets during calibration). Robot frame, meters.

const PAN_AXIS: Vec3 = [0.0388, 0, 0]; // pan (swivel) axis on the table, base_link frame
const PAN_TOP = 0.0624; // height of the pan joint
const CHAIN = ["shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "gripper_frame_link"];
const BENDS = ["upper_arm_link", "lower_arm_link", "wrist_link"]; // shoulder lift, elbow, wrist flex
// Moving jaw fingertip in the jaw's own frame (meets the fixed finger when closed), and its pivot.
const JAW_PIVOT: Vec3 = [0, 0, 0.019];
const JAW_TIP: Vec3 = [-0.0147, -0.0785, 0.019];
const ARC_R = 0.018;

const inFrame = (m: Mat4, p: Vec3) => new THREE.Vector3(...p).applyMatrix4(toMatrix(m));

/** Arc at joint j between the directions to its neighbours a and b. */
function arcPoints(a: THREE.Vector3, j: THREE.Vector3, b: THREE.Vector3, r = ARC_R, n = 16): THREE.Vector3[] {
  const u = a.clone().sub(j).normalize(), v = b.clone().sub(j).normalize();
  const angle = Math.acos(THREE.MathUtils.clamp(u.dot(v), -1, 1));
  if (angle < 1e-3) return [];
  const axis = new THREE.Vector3().crossVectors(u, v);
  if (axis.lengthSq() < 1e-9) axis.set(0, 0, 1); // straight: any perpendicular works for a half circle
  axis.normalize();
  return Array.from({ length: n + 1 }, (_, i) =>
    u.clone().applyAxisAngle(axis, (angle * i) / n).multiplyScalar(r).add(j));
}

function Base() {
  const [x, y] = PAN_AXIS, h = 0.045; // square footprint centered on the pan axis
  const sq: Vec3[] = [[x - h, y - h, 0], [x + h, y - h, 0], [x + h, y + h, 0], [x - h, y + h, 0], [x - h, y - h, 0]];
  return <Line points={sq} color={C.ink} lineWidth={2} />;
}

function Arm({ arm, stale }: { arm: ArmState; stale: boolean }) {
  const f = arm.frames!;
  const color = stale ? C.ghost : C.ink, accent = stale ? C.ghost : C.steel;
  // The arm moves in ONE vertical plane: through the pan axis, turned by the pan joint. The URDF frame origins
  // of the lift/elbow/wrist joints sit ~18 mm along their own (horizontal) axes, which doesn't change the motion
  // but would draw fake sideways jogs — so joints are projected into that plane. The tool point (red dot) is
  // the true FK position and is not projected.
  const pan = origin(f.shoulder_link ?? f.base_link);
  const lift = f.upper_arm_link;
  const n = lift ? new THREE.Vector3(lift[0][2], lift[1][2], lift[2][2]).normalize() : null; // shoulder-lift axis
  const inPlane = (p: THREE.Vector3) => (n ? p.clone().sub(n.clone().multiplyScalar(n.dot(p.clone().sub(pan)))) : p);
  const chain = [new THREE.Vector3(PAN_AXIS[0], PAN_AXIS[1], 0),
                 ...CHAIN.filter((l) => f[l]).map((l) => (l === "gripper_frame_link" ? origin(f[l]) : inPlane(origin(f[l]))))];
  const arcs = BENDS.map((l) => {
    const i = CHAIN.indexOf(l) + 1; // +1: chain[0] is the base point
    return f[l] && chain[i - 1] && chain[i + 1] ? arcPoints(chain[i - 1], chain[i], chain[i + 1]) : [];
  }).filter((pts) => pts.length > 1);
  // Moving jaw: from where its pivot sits along the gripper line (wrist roll -> tool point) out to its fingertip,
  // so it forms a V with the fixed finger that closes as the gripper closes.
  let jaw: THREE.Vector3[] | null = null;
  if (f[JAW] && f.gripper_link && f.gripper_frame_link) {
    const g0 = inPlane(origin(f.gripper_link)), g1 = origin(f.gripper_frame_link);
    const along = new THREE.Line3(g0, g1).closestPointToPoint(inFrame(f[JAW], JAW_PIVOT), true, new THREE.Vector3());
    jaw = [along, inFrame(f[JAW], JAW_TIP)];
  }
  return (
    <group>
      <Line points={chain} color={color} lineWidth={4} />
      {arcs.map((pts, i) => <Line key={i} points={pts} color={accent} lineWidth={2} />)}
      {/* pan joint: a small ring around the vertical axis */}
      <Line points={Array.from({ length: 25 }, (_, i) => [PAN_AXIS[0] + ARC_R * Math.cos((i / 24) * 2 * Math.PI),
                                                         PAN_AXIS[1] + ARC_R * Math.sin((i / 24) * 2 * Math.PI), PAN_TOP] as Vec3)}
            color={accent} lineWidth={2} />
      {jaw && <Line points={jaw} color={color} lineWidth={3} />}
      {arm.tip && (
        <group position={arm.tip}>
          <mesh>
            <sphereGeometry args={[0.0045, 16, 12]} />
            <meshBasicMaterial color="#ff1a1a" toneMapped={false} />
          </mesh>
          {!stale && (
            <mesh>
              <sphereGeometry args={[0.009, 16, 12]} />
              <meshBasicMaterial color="#ff3030" transparent opacity={0.3} toneMapped={false} depthWrite={false} />
            </mesh>
          )}
        </group>
      )}
    </group>
  );
}

function roundedRect(o: TrayState["outline"]): THREE.Shape {
  const r = Math.min(o.corner_r, (o.xmax - o.xmin) / 2, (o.ymax - o.ymin) / 2);
  const s = new THREE.Shape();
  s.moveTo(o.xmin + r, o.ymin);
  s.lineTo(o.xmax - r, o.ymin);
  s.quadraticCurveTo(o.xmax, o.ymin, o.xmax, o.ymin + r);
  s.lineTo(o.xmax, o.ymax - r);
  s.quadraticCurveTo(o.xmax, o.ymax, o.xmax - r, o.ymax);
  s.lineTo(o.xmin + r, o.ymax);
  s.quadraticCurveTo(o.xmin, o.ymax, o.xmin, o.ymax - r);
  s.lineTo(o.xmin, o.ymin + r);
  s.quadraticCurveTo(o.xmin, o.ymin, o.xmin + r, o.ymin);
  return s;
}

/** A calibration reference point on the tray (pocket or plate corner): position (mm) + highlight ring radius. */
function refPoint(tray: TrayState, id: string | null): { id: string; x: number; y: number; r: number } | undefined {
  if (!id) return undefined;
  const c = (tray.corners ?? []).find((k) => k.id === id);
  if (c) return { id, x: c.x, y: c.y, r: 3 };
  const p = tray.pockets.find((k) => k.id === id);
  return p && { id, x: p.x, y: p.y, r: pocketRadius(p) };
}

const pocketRadius = (p: Pocket) => (p.shape.kind === "circle" ? p.shape.d / 2 : p.shape.kind === "rect" ? Math.max(p.shape.w, p.shape.h) / 2 : 6);

/** One pocket on the tray face (tray frame, mm, z out of the face) + its item stub when full. */
function PocketMesh({ p, color, prot, ghost }: { p: Pocket; color: string; prot: number; ghost: boolean }) {
  const s = p.shape;
  const yaw = (p.yaw_deg * Math.PI) / 180;
  const op = ghost ? 0.35 : 1;
  const mat = <meshStandardMaterial color={color} transparent={ghost} opacity={op} />;
  const hole = <meshBasicMaterial color={C.hole} transparent={ghost} opacity={op} />;
  return (
    <group position={[p.x, p.y, 0]} rotation={[0, 0, yaw]}>
      {s.kind === "circle" && (
        <>
          <mesh position={[0, 0, 0.2]}><circleGeometry args={[s.d / 2, 28]} />{hole}</mesh>
          {p.full && (
            <mesh position={[0, 0, prot / 2]} rotation={[Math.PI / 2, 0, 0]}>
              <cylinderGeometry args={[s.d * 0.47, s.d * 0.47, prot, 24]} />{mat}
            </mesh>
          )}
        </>
      )}
      {s.kind === "rect" && (
        <>
          <mesh position={[0, 0, 0.2]}><planeGeometry args={[s.w, s.h]} />{hole}</mesh>
          {p.full && <mesh position={[0, 0, prot / 2]}><boxGeometry args={[s.w * 0.95, s.h * 0.95, prot]} />{mat}</mesh>}
        </>
      )}
      {s.kind === "pins" && (
        <>
          {[-1, 1].map((k) => (
            <mesh key={k} position={[(k * s.spacing) / 2, 0, 0.2]}><circleGeometry args={[Math.max(s.d / 2, 0.8), 12]} />{hole}</mesh>
          ))}
          {p.full && (
            <mesh position={[0, 0, prot / 2]} rotation={[Math.PI / 2, 0, 0]}>
              <cylinderGeometry args={[6, 6, prot, 24]} />{mat}
            </mesh>
          )}
        </>
      )}
    </group>
  );
}

function Tray({ tray, matrix, ghost, highlighted }: { tray: TrayState; matrix: Mat4; ghost: boolean; highlighted: string | null }) {
  const m = useMemo(() => toMatrix(matrix), [matrix]);
  const shape = useMemo(() => roundedRect(tray.outline), [tray.outline]);
  const colors = useMemo(() => Object.fromEntries(tray.types.map((t, i) => [t.sku, itemColor(i)])), [tray.types]);
  const hp = refPoint(tray, highlighted);
  return (
    <group matrix={m} matrixAutoUpdate={false}>
      <group scale={0.001}>
        <mesh position={[0, 0, -PLATE_MM]}>
          <extrudeGeometry args={[shape, { depth: PLATE_MM, bevelEnabled: false, curveSegments: 10 }]} />
          <meshStandardMaterial color={ghost ? C.ghost : C.plate} transparent={ghost} opacity={ghost ? 0.35 : 1} />
        </mesh>
        {tray.pockets.map((p) => (
          <PocketMesh key={p.id} p={p} color={colors[p.sku] ?? C.ghost} prot={tray.protrusion_mm} ghost={ghost} />
        ))}
        {(tray.corners ?? []).map((c) => (
          <Line key={c.id} points={[[c.x - 3, c.y - 3, 0.5], [c.x + 3, c.y + 3, 0.5], [c.x, c.y, 0.5], [c.x - 3, c.y + 3, 0.5], [c.x + 3, c.y - 3, 0.5]]}
                color={C.steel} lineWidth={1.5} />
        ))}
        {hp && (
          <group position={[hp.x, hp.y, 0.6]}>
            <mesh>
              <ringGeometry args={[hp.r + 2, hp.r + 5, 40]} />
              <meshBasicMaterial color={C.safety} side={THREE.DoubleSide} />
            </mesh>
          </group>
        )}
      </group>
    </group>
  );
}

/** Outline of the live fit (before saving), so a bad fit is visible against the saved / guessed tray. */
function FitOutline({ tray, matrix }: { tray: TrayState; matrix: Mat4 }) {
  const m = useMemo(() => toMatrix(matrix), [matrix]);
  const pts = useMemo(() => roundedRect(tray.outline).getPoints(12).map((v) => [v.x, v.y, 1] as Vec3), [tray.outline]);
  return (
    <group matrix={m} matrixAutoUpdate={false}>
      <group scale={0.001}>
        <Line points={[...pts, pts[0]]} color={C.ok} lineWidth={2} dashed dashSize={6} gapSize={4} />
      </group>
    </group>
  );
}

type Label = { key: string; text: string; cls: string; pos: THREE.Vector3 };

/** robot base frame (meters) -> three.js world (the -90° about x of the scene root). */
const robotToWorld = (p: number[]) => new THREE.Vector3(p[0], p[2], -p[1]);
// ---------------------------------------------------------------- packing boxes
const BOX_COLOR: Record<BoxInfo["state"], string> = { available: C.steel, in_use: C.safety, packed: C.ok };

/** Open-top box + its tab, in the box frame (origin bottom center, z up, tab along -y). */
function BoxMesh({ box, g }: { box: BoxInfo; g: BoxesState["geometry"] }) {
  const m = useMemo(() => toMatrix(box.pose!), [box.pose]);
  const color = BOX_COLOR[box.state] ?? C.ghost;
  const { w_m: w, h_m: h, tab_w_m: tw, tab_out_m: to } = g;
  const edges = useMemo(() => new THREE.EdgesGeometry(new THREE.BoxGeometry(w, w, h)), [w, h]);
  return (
    <group matrix={m} matrixAutoUpdate={false}>
      <group position={[0, 0, h / 2]}>
        <mesh>
          <boxGeometry args={[w, w, h]} />
          <meshStandardMaterial color={color} transparent opacity={0.22} depthWrite={false} />
        </mesh>
        <lineSegments geometry={edges}>
          <lineBasicMaterial color={color} />
        </lineSegments>
      </group>
      {/* tab: plate along the top edge + rib below it */}
      <mesh position={[0, -(w / 2 + to / 2), h - 0.001]}>
        <boxGeometry args={[tw, to, 0.002]} />
        <meshStandardMaterial color={color} />
      </mesh>
      <mesh position={[0, -(w / 2 + to * 0.4), h - 0.019]}>
        <boxGeometry args={[0.002, to * 0.65, 0.036]} />
        <meshStandardMaterial color={color} />
      </mesh>
    </group>
  );
}

/** Dashed outline on the table where orders are packed (tab facing the swivel). */
function PackSpot({ xy, g }: { xy: [number, number]; g: BoxesState["geometry"] }) {
  const [x, y] = [xy[0] / 1000, xy[1] / 1000];
  const a = Math.atan2(y - PAN_AXIS[1], x - PAN_AXIS[0]); // box +y = radially outward
  const h = g.w_m / 2;
  const ux = [Math.cos(a), Math.sin(a)], vx = [Math.sin(a), -Math.cos(a)]; // +y (out), +x
  const pt = (sx: number, sy: number): Vec3 => [x + vx[0] * sx + ux[0] * sy, y + vx[1] * sx + ux[1] * sy, 0.001];
  return <Line points={[pt(-h, -h), pt(h, -h), pt(h, h), pt(-h, h), pt(-h, -h)]} color={C.ink} lineWidth={1.5} dashed
               dashSize={0.008} gapSize={0.006} />;
}

// ---------------------------------------------------------------- planned path (dry runs)
const PATH_COLOR = { free: "#5B6875", line: "#1F6FB2", grip: C.ok, release: "#D9822B", fail: "#C8372D" };

/** The tool point's planned path: free moves dashed grey, straight lines solid blue, grips green, releases
 *  orange, start ring, and a red marker where planning failed. Robot frame. */
function PlannedPathView({ path }: { path: PlannedPath }) {
  const first = path.segments.find((s) => s.points.length)?.points[0];
  return (
    <group>
      {path.segments.map((s, i) => {
        if ((s.kind === "free" || s.kind === "line") && s.points.length > 1)
          return <Line key={i} points={s.points} color={PATH_COLOR[s.kind]} lineWidth={s.kind === "line" ? 3 : 2}
                       dashed={s.kind === "free"} dashSize={0.01} gapSize={0.007} />;
        if (s.kind === "grip" || s.kind === "release")
          return (
            <mesh key={i} position={s.points[0]}>
              <sphereGeometry args={[0.006, 14, 10]} />
              <meshStandardMaterial color={PATH_COLOR[s.kind]} />
            </mesh>
          );
        return null;
      })}
      {first && (
        <mesh position={first}>
          <torusGeometry args={[0.009, 0.0022, 8, 24]} />
          <meshStandardMaterial color={C.ink} />
        </mesh>
      )}
      {path.fail && (
        <mesh position={path.fail}>
          <sphereGeometry args={[0.011, 18, 14]} />
          <meshStandardMaterial color={PATH_COLOR.fail} emissive={PATH_COLOR.fail} emissiveIntensity={0.4} />
        </mesh>
      )}
    </group>
  );
}

/** tray frame point (mm) -> three.js world, through the tray -> robot matrix (meters). */
function trayToWorld(m: Mat4, xmm: number, ymm: number, zmm: number) {
  const v = [xmm / 1000, ymm / 1000, zmm / 1000, 1];
  return robotToWorld([0, 1, 2].map((i) => m[i].reduce((acc, mij, j) => acc + mij * v[j], 0)));
}

/** Moves the overlay label divs to their projected 3D positions every frame (no extra React roots). */
function LabelProjector({ labels, els }: { labels: Label[]; els: RefObject<Map<string, HTMLDivElement>> }) {
  const v = useMemo(() => new THREE.Vector3(), []);
  useFrame(({ camera, size }) => {
    for (const l of labels) {
      const el = els.current.get(l.key);
      if (!el) continue;
      v.copy(l.pos).project(camera);
      const hidden = v.z > 1 || v.z < -1;
      el.style.display = hidden ? "none" : "";
      el.style.transform = `translate(-50%, -50%) translate(${((v.x + 1) / 2) * size.width}px, ${((1 - v.y) / 2) * size.height}px)`;
    }
  });
  return null;
}

export default function Scene({ arm, stale, tray, session, highlighted, boxes = null, planned = null }: {
  arm: ArmState | null; stale: boolean; tray: TrayState | null; session: CalSession | null; highlighted: string | null;
  boxes?: BoxesState | null; planned?: PlannedPath | null;
}) {
  const fitMatrix = session?.active && session.fit?.matrix ? session.fit.matrix : null;
  const trayMatrix = tray ? tray.calibration?.matrix ?? defaultTrayMatrix(tray.outline) : null;
  const labelEls = useRef(new Map<string, HTMLDivElement>());

  const labels: Label[] = [];
  if (tray && trayMatrix) {
    const o = tray.outline;
    if (!tray.calibration)
      labels.push({ key: "ghost", text: "Tray not calibrated — position is a guess", cls: "ghost-label",
                    pos: trayToWorld(trayMatrix, (o.xmin + o.xmax) / 2, (o.ymin + o.ymax) / 2, 30) });
    const hp = refPoint(tray, highlighted);
    if (hp) labels.push({ key: "target", text: hp.id, cls: "target-label",
                          pos: trayToWorld(trayMatrix, hp.x, hp.y, tray.protrusion_mm + 10) });
  }
  for (const b of boxes?.boxes ?? []) {
    if (!b.pose) continue;
    const word = b.state === "in_use" ? `order ${b.order_id}` : b.state;
    labels.push({ key: `box:${b.id}`, text: `Box ${b.id} · ${word}`, cls: `box-label box-${b.state}`,
                  pos: robotToWorld([b.pose[0][3], b.pose[1][3], b.pose[2][3] + (boxes!.geometry.h_m + 0.02)]) });
  }
  if (boxes) labels.push({ key: "packspot", text: "pack spot", cls: "point-label",
                           pos: robotToWorld([boxes.pack_spot_mm[0] / 1000, boxes.pack_spot_mm[1] / 1000, 0.01]) });
  if (planned?.fail)
    labels.push({ key: "fail", text: "fails here", cls: "fail-label",
                  pos: robotToWorld([planned.fail[0], planned.fail[1], planned.fail[2] + 0.025]) });
  for (const pt of session?.points ?? [])
    labels.push({ key: `pt:${pt.pocket}`, text: pt.pocket, cls: "point-label",
                  pos: robotToWorld([pt.robot_m[0], pt.robot_m[1], pt.robot_m[2] + 0.014]) });

  return (
    <>
    <Canvas camera={{ position: [0.17, 0.5, 0.6], fov: 40, near: 0.01, far: 20 }} dpr={[1, 2]}>
      <color attach="background" args={["#E4E9EE"]} />
      <hemisphereLight args={["#ffffff", "#8A96A3", 0.9]} />
      <directionalLight position={[1, 2, 1.5]} intensity={1.4} />
      <Grid args={[2, 2]} cellSize={0.05} sectionSize={0.25} cellColor="#B7C2CD" sectionColor="#7F8E9C"
            fadeDistance={3} infiniteGrid position={[0, 0, 0]} />
      <OrbitControls target={[0.17, 0.1, 0]} makeDefault enableDamping />
      <group rotation={ROBOT_TO_THREE}>
        <Base />
        {arm?.frames && <Arm arm={arm} stale={stale} />}
        {tray && (
          <Tray tray={tray} highlighted={highlighted} matrix={trayMatrix!} ghost={!tray.calibration} />
        )}
        {tray && fitMatrix && <FitOutline tray={tray} matrix={fitMatrix} />}
        {boxes && <PackSpot xy={boxes.pack_spot_mm} g={boxes.geometry} />}
        {planned && <PlannedPathView path={planned} />}
        {boxes?.boxes.filter((b) => b.pose).map((b) => <BoxMesh key={b.id} box={b} g={boxes.geometry} />)}
        {session?.points.map((pt) => (
          <group key={pt.pocket} position={pt.robot_m}>
            <mesh>
              <sphereGeometry args={[pt.pocket === highlighted ? 0.007 : 0.005, 16, 12]} />
              <meshStandardMaterial color={pt.pocket === highlighted ? C.safety : C.ok} />
            </mesh>
          </group>
        ))}
      </group>
      <LabelProjector labels={labels} els={labelEls} />
    </Canvas>
    <div className="scene-labels" aria-hidden="true">
      {labels.map((l) => (
        <div key={l.key} className={`scene-label ${l.cls}`}
             ref={(el) => { if (el) labelEls.current.set(l.key, el); else labelEls.current.delete(l.key); }}>{l.text}</div>
      ))}
    </div>
    </>
  );
}
