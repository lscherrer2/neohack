// Shapes returned by the shop server (packing/shop/server.py, contract in .claude/plans/arm_viz.md).

export type Mat4 = number[][]; // 4x4 row-major
export type Vec3 = [number, number, number];

// ---------- orders / dashboard
export type OrderStatus = "queued" | "ready" | "packing" | "packed" | "failed" | "cancelled";

export interface Order {
  id: number;
  status: OrderStatus | string;
  customer_name?: string | null;
  items_text?: string;
  error?: string | null;
  created_at: string;
  [k: string]: unknown;
}

export type RobotInfo =
  | { state: "packing"; order: Order; items: string; last_seen_s: number | null }
  | { state: "idle"; last_seen_s: number | null; last_failure: Order | null }
  | { state: "offline"; last_seen_s: number | null };

export interface StockItem {
  sku: string;
  name: string;
  on_shelf: number;
  reserved: number;
}

export interface ShopState {
  robot: RobotInfo;
  counts: Partial<Record<string, number>>;
  auto_dispatch: boolean;
  stock: StockItem[] | null;
  orders: Order[];
}

export interface ShopEvent {
  seq: number;
  ts: number; // unix seconds
  kind: string; // tool | robot | order | chat | error | system
  text: string;
  data?: Record<string, unknown>;
}

export interface ToolCall {
  tool: string;
  args: unknown;
  result: string;
}

export interface ChatReply {
  reply?: string;
  error?: string;
  tool_calls?: ToolCall[];
}

// ---------- parts tray
export type PocketShape =
  | { kind: "circle"; d: number }
  | { kind: "rect"; w: number; h: number }
  | { kind: "pins"; d: number; spacing: number };

export interface Pocket {
  id: string;
  sku: string;
  x: number; // mm, tray frame (DXF: y up, y=0 row nearest the arm)
  y: number;
  yaw_deg: number;
  shape: PocketShape;
  full: boolean;
  robot_m?: Vec3;
}

export interface TrayOutline {
  xmin: number;
  ymin: number;
  xmax: number;
  ymax: number;
  corner_r: number;
}

export interface TrayType {
  sku: string;
  name: string;
  feature: string;
  grip_axis: string;
}

export interface TrayCalibration {
  created: string;
  rms_mm: number;
  tilt_deg: number;
  matrix: Mat4; // tray (meters) -> robot base
  reference_points: string[];
}

export interface TrayState {
  outline: TrayOutline;
  unmatched: string[];
  protrusion_mm: number;
  types: TrayType[];
  pockets: Pocket[];
  corners: TrayCorner[]; // plate corner reference points (middle of each rounded corner)
  calibration: TrayCalibration | null;
}

export interface TrayCorner {
  id: string; // corner_bl | corner_tl | corner_br | corner_tr
  label: string; // "Bottom-left corner" (bottom = nearest the arm)
  x: number; // mm, tray frame
  y: number;
}

// ---------- arm
export type JointName = "shoulder_pan" | "shoulder_lift" | "elbow_flex" | "wrist_flex" | "wrist_roll" | "gripper";

export interface ArmState {
  connected: boolean;
  source: "sim" | "real" | "remote" | null;
  torque: boolean | null;
  mode: "idle" | "calibrating" | "packing";
  stale_s: number | null;
  error: string | null;
  joints: Record<JointName, number> | null;
  frames: Record<string, Mat4> | null;
  tip: Vec3 | null;
}

// ---------- calibration session
export interface CalReference {
  pocket: string;
  label: string;
  captured: boolean;
}

export interface CalPoint {
  pocket: string;
  label: string | null;
  tray_mm: [number, number];
  robot_m: Vec3;
  residual_mm: number | null;
}

export interface CalFit {
  ok: boolean;
  error: string | null;
  rms_mm: number | null;
  tilt_deg: number | null;
  warnings: string[];
  grip_angles: Record<string, number>;
  matrix: Mat4 | null;
}

export interface CalSession {
  active: boolean;
  mode: "corners" | "pockets"; // which reference points the capture buttons use
  reference_points: CalReference[];
  points: CalPoint[];
  fit: CalFit | null;
  saved: { created: string; rms_mm: number } | null;
}
