"""Cartesian (straight-line) motion for the SO-101 using placo IK + the SO-101 URDF.

Joint-space move (Motion.move_to): every joint interpolates A->B; the gripper tip follows a CURVE.
Linear move (this module):        the tip position interpolates A->B in a STRAIGHT LINE (orientation
                                   slerps), IK converts each point on the line back into joint values.

Units: the arm runs in calibrated -100..100 units (0 = middle of each joint's RANGE); the URDF works in
degrees with 0 = the CALIBRATION POSE (upper arm up, forearm forward, wrist straight), which calibration
made read raw 2047 (HALF_TURN). The two zeros differ wherever the end stops aren't symmetric about the
calibration pose (elbow: 8.7 deg), so ArmModel converts raw-referenced: deg = (raw - 2047) * 360 / 4095.
(LeRobot's DEGREES mode uses the range middle instead; that cost us a ~15 deg gripper-pitch error, Nov 2026.)
Accuracy then depends on how well the calibration pose matched the URDF zero pose.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ..joints import BODY

URDF = Path(__file__).resolve().parent.parent / "urdf" / "so101_new_calib.urdf"
MAX_RES = 4095  # servo encoder steps per revolution, minus one
HALF_TURN = 2047  # raw reading at the calibration pose (= URDF zero), set by calibration


class PathError(RuntimeError):
    """Line can't be followed (unreachable, joint limit, or IK jump)."""


def load_ranges(robot_id: str) -> dict[str, tuple[int, int]]:
    """Calibrated (range_min, range_max) servo ticks of each body joint, from the robot's calibration file."""
    from ..hardware.so101 import load_calibration

    cal = load_calibration(robot_id)
    return {j: (cal[j].range_min, cal[j].range_max) for j in BODY}


def gravity_feedforward(robot_id: str, gain: float):
    """Callable pose -> per-joint command offset (units) that cancels gravity sag, or None if gain is 0.
    The servos are P-only, so a joint droops ~ torque / stiffness; measured Nov 2026 along the push path:
    sag = 4.17 * torque (shoulder_lift, rms residual 0.05 units) and 4.12 * torque (elbow_flex)."""
    if gain <= 0:
        return None
    model = ArmModel(load_ranges(robot_id))
    return lambda pose: {j: gain * t for j, t in model.gravity_torque(pose).items()}


class PlacoKinematics:
    """FK/IK on a URDF with placo, joint angles in degrees. `robot` is the placo RobotWrapper."""

    def __init__(self, urdf: str, tip_frame: str, joints: list[str]):
        import placo

        self.robot = placo.RobotWrapper(urdf)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)  # base is bolted down
        self.tip_frame, self.joints = tip_frame, list(joints)
        self.task = self.solver.add_frame_task(tip_frame, np.eye(4))

    def _set(self, q_deg: np.ndarray) -> None:
        for name, v in zip(self.joints, np.deg2rad(np.asarray(q_deg)[: len(self.joints)])):
            self.robot.set_joint(name, v)

    def forward_kinematics(self, q_deg: np.ndarray) -> np.ndarray:
        self._set(q_deg)
        self.robot.update_kinematics()
        return np.array(self.robot.get_T_world_frame(self.tip_frame))

    def inverse_kinematics(self, q_deg: np.ndarray, T: np.ndarray, position_weight: float = 1.0,
                           orientation_weight: float = 0.01) -> np.ndarray:
        """One soft IK step from seed `q_deg` toward target pose `T`; returns joint angles in degrees."""
        self._set(q_deg)
        self.task.T_world_frame = T
        self.task.configure(self.tip_frame, "soft", position_weight, orientation_weight)
        self.solver.solve(True)
        self.robot.update_kinematics()
        return np.rad2deg([self.robot.get_joint(name) for name in self.joints])


# ---------- rotation helpers (no scipy) ----------
def _mat_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> unit quaternion (w, x, y, z)."""
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def _quat_to_mat(q: np.ndarray) -> np.ndarray:
    """Unit quaternion (w, x, y, z) -> rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical interpolation between unit quaternions, taking the short way round."""
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(d)
    return (np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th)


class ArmModel:
    """SO-101 forward/inverse kinematics in calibrated joint units, plus straight-line path planning."""

    def __init__(self, ranges: dict[str, tuple[int, int]], urdf: Path = URDF):
        self.kin = PlacoKinematics(str(urdf), "gripper_frame_link", list(BODY))
        # degrees per calibrated unit, per joint
        self.scale = np.array([(ranges[j][1] - ranges[j][0]) / 2 * 360 / MAX_RES / 100 for j in BODY])
        # degrees at unit 0 (range middle), measured from the calibration pose
        self.zero_deg = np.array([((ranges[j][0] + ranges[j][1]) / 2 - HALF_TURN) * 360 / MAX_RES for j in BODY])

        # pan axis position (world xy), for azimuth of a point around the base
        self.kin.forward_kinematics(np.zeros(len(BODY)))
        self.pan_xy = np.array(self.kin.robot.get_T_world_frame("shoulder_link"))[:2, 3]

    # ---------- 5-DOF orientation handling ----------
    # A 5-joint arm can't hold an arbitrary tool yaw: yaw is tied to shoulder_pan. So orientation is
    # split into R = Rz(azimuth of the tip around the pan axis) @ L, and only the "local" part L
    # (tool pitch/roll relative to the arm's plane) is held/interpolated.
    def azimuth(self, p: np.ndarray) -> float:
        """Angle of point `p` around the pan axis, in the world xy plane."""
        d = np.asarray(p)[:2] - self.pan_xy
        return float(np.arctan2(d[1], d[0]))

    @staticmethod
    def _rz(a: float) -> np.ndarray:
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def local_rot(self, T: np.ndarray) -> np.ndarray:
        """Tool orientation of pose `T` relative to the arm's plane (azimuth removed)."""
        return self._rz(-self.azimuth(T[:3, 3])) @ T[:3, :3]

    def target_at(self, local_R: np.ndarray, p: np.ndarray) -> np.ndarray:
        """Full 4x4 target at point `p` whose local tool orientation is `local_R`."""
        T = np.eye(4)
        T[:3, 3] = p
        T[:3, :3] = self._rz(self.azimuth(p)) @ local_R
        return T

    def offset(self, pose: dict[str, float], offset_m: np.ndarray | tuple[float, float, float]) -> np.ndarray:
        """Target with the tip of `pose` shifted by offset_m, same tool tilt."""
        T = self.fk(pose)
        return self.target_at(self.local_rot(T), T[:3, 3] + np.asarray(offset_m, dtype=float))

    # ---------- unit conversion ----------
    def to_deg(self, pose: dict[str, float]) -> np.ndarray:
        """Body-joint pose in calibrated units -> URDF joint angles in degrees."""
        return np.array([pose[j] for j in BODY]) * self.scale + self.zero_deg

    def to_units(self, deg: np.ndarray) -> dict[str, float]:
        """URDF joint angles in degrees -> body-joint pose in calibrated units."""
        return {j: float(v) for j, v in zip(BODY, (np.asarray(deg)[: len(BODY)] - self.zero_deg) / self.scale)}

    # ---------- gravity ----------
    def gravity_torque(self, pose: dict[str, float]) -> dict[str, float]:
        """Static gravity torque (N*m) on each body joint at `pose`, from the URDF link masses."""
        self.kin._set(self.to_deg(pose))
        self.kin.robot.update_kinematics()
        g = self.kin.robot.static_gravity_compensation_torques_dict("base_link")
        return {j: float(g[j]) for j in BODY}

    # ---------- kinematics ----------
    def fk(self, pose: dict[str, float]) -> np.ndarray:
        """4x4 gripper-frame transform for a body-joint pose."""
        return np.array(self.kin.forward_kinematics(self.to_deg(pose)))

    def tip(self, pose: dict[str, float]) -> np.ndarray:
        """Gripper tip position in meters (base frame: x forward, y left, z up)."""
        return self.fk(pose)[:3, 3]

    def ik(self, seed: dict[str, float], T: np.ndarray, orientation_weight: float = 0.05,
           iters: int = 40, tol_m: float = 0.0005) -> tuple[dict[str, float], float]:
        """Joint pose reaching target `T`, solved iteratively from `seed`.
        Returns (pose, remaining tip position error in meters)."""
        q = self.to_deg(seed)
        for _ in range(iters):
            q = np.asarray(self.kin.inverse_kinematics(q, T, orientation_weight=orientation_weight))[: len(BODY)]
            err = float(np.linalg.norm(self.kin.forward_kinematics(q)[:3, 3] - T[:3, 3]))
            if err < tol_m:
                break
        return self.to_units(q), err

    # ---------- straight lines ----------
    def line(self, start: dict[str, float], end: dict[str, float] | np.ndarray, step_m: float = 0.003,
             keep_orientation: bool = False, max_err_m: float = 0.002, max_joint_step: float = 4.0,
             limit: float = 100.0, end_local_rot: np.ndarray | None = None, step_rad: float = 0.02,
             orientation_weight: float = 0.05) -> list[dict[str, float]]:
        """Joint poses whose tips lie on the straight line start-tip -> end.
        `end` is a joint pose (its tip + orientation are the target) or an xyz point in meters
        (orientation kept from start, or `end_local_rot` if given — e.g. a tilt in place).
        Steps every `step_m` of travel and every `step_rad` of rotation, whichever needs more.
        Raises PathError if any point is unreachable/unsafe."""
        T0 = self.fk(start)
        L0 = self.local_rot(T0)
        if isinstance(end, dict):
            T1 = self.fk(end)
            L1 = L0 if keep_orientation else self.local_rot(T1)
        else:
            L1 = L0 if end_local_rot is None else end_local_rot
            T1 = self.target_at(L1, np.asarray(end, dtype=float))
        p0, p1 = T0[:3, 3], T1[:3, 3]
        q0, q1 = _mat_to_quat(L0), _mat_to_quat(L1)
        turn = 2 * np.arccos(min(1.0, abs(float(np.dot(q0, q1)))))  # rotation angle, radians
        n = max(1, int(np.ceil(np.linalg.norm(p1 - p0) / step_m)), int(np.ceil(turn / step_rad)))
        path, seed = [dict(start)], dict(start)
        for i in range(1, n + 1):
            s = i / n
            T = self.target_at(_quat_to_mat(_slerp(q0, q1, s)), p0 + (p1 - p0) * s)
            pose, err = self.ik(seed, T, orientation_weight=orientation_weight)
            where = f"at {s:.0%} of the line ({np.round(T[:3, 3] * 1000, 1)} mm)"
            if err > max_err_m:
                raise PathError(f"unreachable {where}: IK off by {err * 1000:.1f} mm")
            out_of_range = {j: round(v, 1) for j, v in pose.items() if abs(v) > limit}
            if out_of_range:
                raise PathError(f"joint limit {where}: {out_of_range}")
            jump = max(abs(pose[j] - seed[j]) for j in BODY)
            if jump > max_joint_step:
                raise PathError(f"joint jump of {jump:.1f} units {where} (near a singularity?)")
            path.append(pose)
            seed = pose
        return path

    @staticmethod
    def path_length(points: list[np.ndarray]) -> float:
        """Total length of the polyline through `points`."""
        return float(sum(np.linalg.norm(b - a) for a, b in zip(points, points[1:])))
