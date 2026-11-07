import { Grid, Line, OrbitControls } from "@react-three/drei";
import { Canvas, useFrame } from "@react-three/fiber";
import { useMemo, useRef, type RefObject } from "react";
import * as THREE from "three";
import { itemColor } from "../api";
import type { ArmState, CalSession, Mat4, Pocket, TrayState, Vec3 } from "../types";

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
  const chain = [new THREE.Vector3(PAN_AXIS[0], PAN_AXIS[1], 0),
                 ...CHAIN.filter((l) => f[l]).map((l) => origin(f[l]))];
  const arcs = BENDS.map((l) => {
    const i = CHAIN.indexOf(l) + 1; // +1: chain[0] is the base point
    return f[l] && chain[i - 1] && chain[i + 1] ? arcPoints(chain[i - 1], chain[i], chain[i + 1]) : [];
  }).filter((pts) => pts.length > 1);
  // Moving jaw: from where its pivot sits along the gripper line (wrist roll -> tool point) out to its fingertip,
  // so it forms a V with the fixed finger that closes as the gripper closes.
  let jaw: THREE.Vector3[] | null = null;
  if (f[JAW] && f.gripper_link && f.gripper_frame_link) {
    const g0 = origin(f.gripper_link), g1 = origin(f.gripper_frame_link);
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
  const hp = highlighted ? tray.pockets.find((p) => p.id === highlighted) : undefined;
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
        {hp && (
          <group position={[hp.x, hp.y, 0.6]}>
            <mesh>
              <ringGeometry args={[pocketRadius(hp) + 2, pocketRadius(hp) + 5, 40]} />
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

export default function Scene({ arm, stale, tray, session, highlighted }: {
  arm: ArmState | null; stale: boolean; tray: TrayState | null; session: CalSession | null; highlighted: string | null;
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
    const hp = highlighted ? tray.pockets.find((p) => p.id === highlighted) : undefined;
    if (hp) labels.push({ key: "target", text: hp.id, cls: "target-label",
                          pos: trayToWorld(trayMatrix, hp.x, hp.y, tray.protrusion_mm + 10) });
  }
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
