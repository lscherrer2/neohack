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
// Every block is drawn in its own link frame. Link frames are square to the arm's geometry, so blocks
// stay aligned with each other and meet at the joints in every pose. A link block spans from its joint
// origin to the next joint origin (expressed in that frame), padded to a cross-section.
// Local coordinates below are meters, measured from the URDF (packing/urdf/so101_new_calib.urdf).

type Box = { lo: Vec3; hi: Vec3 };
const pad = (a: number[], b: number[], r: number): Box => ({
  lo: [0, 1, 2].map((i) => Math.min(a[i], b[i]) - r) as Vec3,
  hi: [0, 1, 2].map((i) => Math.max(a[i], b[i]) + r) as Vec3,
});

/** Chain links: [link, next link, half-thickness]. */
const LINKS: [string, string, number][] = [
  ["shoulder_link", "upper_arm_link", 0.017],   // pan housing up to the shoulder-lift servo
  ["upper_arm_link", "lower_arm_link", 0.016],  // upper arm
  ["lower_arm_link", "wrist_link", 0.015],      // forearm
  ["wrist_link", "gripper_link", 0.015],        // wrist-flex housing over to the roll axis
];
// Gripper, in gripper_link's frame: tool point is at (-0.0079, -0.0002, -0.0981), jaw pivot at (0.020, 0.019, -0.023).
const GRIPPER_BODY: Box = { lo: [-0.016, -0.018, -0.045], hi: [0.03, 0.03, 0.006] };
const FIXED_FINGER: Box = { lo: [-0.013, -0.009, -0.1], hi: [-0.003, 0.009, -0.04] };
// Moving jaw, in its own frame: from the pivot to its fingertip (where it meets the fixed finger when closed).
const JAW_A: Vec3 = [0, 0, 0.019];
const JAW_B: Vec3 = [-0.0147, -0.0785, 0.019];

const local = (m: Mat4, p: THREE.Vector3) => p.clone().applyMatrix4(toMatrix(m).invert());

function BoxIn({ m, box, color, opacity }: { m: Mat4; box: Box; color: string; opacity: number }) {
  const center = box.lo.map((v, i) => (v + box.hi[i]) / 2) as Vec3;
  const size = box.lo.map((v, i) => box.hi[i] - v) as Vec3;
  return (
    <group matrixAutoUpdate={false} matrix={toMatrix(m)}>
      <mesh position={center}>
        <boxGeometry args={size} />
        <meshStandardMaterial color={color} transparent={opacity < 1} opacity={opacity} />
      </mesh>
    </group>
  );
}

/** Box from local point a to b (long axis along a->b) in frame m, square cross-section `w` x `t`. */
function BarIn({ m, a, b, w, t, color, opacity }: { m: Mat4; a: Vec3; b: Vec3; w: number; t: number; color: string; opacity: number }) {
  const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b);
  const dir = B.clone().sub(A);
  const len = dir.length();
  const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
  return (
    <group matrixAutoUpdate={false} matrix={toMatrix(m)}>
      <mesh position={A.add(B).multiplyScalar(0.5)} quaternion={q}>
        <boxGeometry args={[w, len + w, t]} />
        <meshStandardMaterial color={color} transparent={opacity < 1} opacity={opacity} />
      </mesh>
    </group>
  );
}

function Arm({ arm, stale }: { arm: ArmState; stale: boolean }) {
  const f = arm.frames!;
  const op = stale ? 0.45 : 1;
  const light = stale ? C.ghost : C.steel, dark = stale ? C.ghost : C.ink;
  return (
    <group>
      {LINKS.filter(([l, n]) => f[l] && f[n]).map(([l, n, r], i) => (
        <BoxIn key={l} m={f[l]} box={pad([0, 0, 0], local(f[l], origin(f[n])).toArray(), r)}
               color={i % 2 ? dark : light} opacity={op} />
      ))}
      {f.gripper_link && <BoxIn m={f.gripper_link} box={GRIPPER_BODY} color={light} opacity={op} />}
      {f.gripper_link && <BoxIn m={f.gripper_link} box={FIXED_FINGER} color={dark} opacity={op} />}
      {f[JAW] && <BarIn m={f[JAW]} a={JAW_A} b={JAW_B} w={0.01} t={0.018} color={dark} opacity={op} />}
      {arm.tip && (
        <mesh position={arm.tip}>
          <boxGeometry args={[0.008, 0.008, 0.008]} />
          <meshStandardMaterial color={C.safety} emissive={C.safety} emissiveIntensity={stale ? 0 : 0.4} />
        </mesh>
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
    <Canvas camera={{ position: [0.05, 0.55, 0.62], fov: 40, near: 0.01, far: 20 }} dpr={[1, 2]}>
      <color attach="background" args={["#E4E9EE"]} />
      <hemisphereLight args={["#ffffff", "#8A96A3", 0.9]} />
      <directionalLight position={[1, 2, 1.5]} intensity={1.4} />
      <Grid args={[2, 2]} cellSize={0.05} sectionSize={0.25} cellColor="#B7C2CD" sectionColor="#7F8E9C"
            fadeDistance={3} infiniteGrid position={[0, 0, 0]} />
      <OrbitControls target={[0.18, 0.06, 0]} makeDefault enableDamping />
      <group rotation={ROBOT_TO_THREE}>
        {/* fixed base block under the pan housing (pan axis at x = 0.0388, top at z = 0.0624) */}
        <mesh position={[0.03, 0, 0.0312]}>
          <boxGeometry args={[0.1, 0.08, 0.0624]} />
          <meshStandardMaterial color={C.ink} />
        </mesh>
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
