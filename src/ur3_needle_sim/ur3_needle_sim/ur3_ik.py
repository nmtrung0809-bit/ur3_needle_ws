"""
UR3e FK/IK (Modified DH) — phong cách my_project / universal_robots motion.py.

Robot base trong world: translation (0, 0, 0.6).
IK downward: flange Z → −world Z; tip nằm cách flange TIP_LENGTH mét theo −Z flange.
"""
import math

try:
    import numpy as np
except ImportError:
    raise SystemExit('ur3_click_insert needs numpy')

# UR3e official-ish Modified DH (Craig)
_DH = [
    # (alpha_prev, a_prev, d, theta_offset)
    (0.0, 0.0, 0.15185, 0.0),
    (math.pi / 2, 0.0, 0.0, 0.0),
    (0.0, -0.24355, 0.0, 0.0),
    (0.0, -0.2132, 0.13105, 0.0),
    (math.pi / 2, 0.0, 0.08535, 0.0),
    (-math.pi / 2, 0.0, 0.0921, 0.0),
]

BASE_WORLD = np.array([0.0, 0.0, 0.6])  # UR3e translation in .wbt
# Kim ~18 cm; tip gần cuối tool khi flange Z chỉ xuống
TIP_LENGTH = 0.18
APPROACH_CLEARANCE = 0.04   # tip cao hơn mặt lỗ
INSERT_DEPTH = 0.045        # tip sâu vào lỗ (~4.5 cm)

LOWER_LIMIT = np.array([-6.28, -6.28, -3.14, -6.28, -6.28, -6.28])
UPPER_LIMIT = np.array([6.28, 6.28, 3.14, 6.28, 6.28, 6.28])

# Seed nhìn về phía +X (phantom), dáng giống PICK_ABOVE/PICK_DOWN nhưng pan≈π
SEED_APPROACH = [math.pi, -1.210, 1.370, -1.770, -1.590, 0.0]
SEED_INSERT = [math.pi, -1.010, 1.590, -2.170, -1.553, -0.020]


def _dh_matrix(alpha, a, d, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st, 0.0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def forward_kinematics(q):
    T = np.eye(4)
    for i, (alpha, a, d, offset) in enumerate(_DH):
        T = T @ _dh_matrix(alpha, a, d, q[i] + offset)
    return T


def world_to_base(p_world):
    return np.asarray(p_world, dtype=float) - BASE_WORLD


def base_to_world(p_base):
    return np.asarray(p_base, dtype=float) + BASE_WORLD


def clamp_q(q):
    return np.minimum(np.maximum(q, LOWER_LIMIT), UPPER_LIMIT)


def _normalize(v, eps=1e-9):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < eps:
        return v
    return v / n


def rotation_vector_from_matrix(R):
    cos_angle = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(float(cos_angle))
    vee = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if angle < 1e-6:
        return 0.5 * vee
    return (angle / (2.0 * math.sin(angle))) * vee


def orientation_error(R_current, R_target):
    return rotation_vector_from_matrix(R_target @ R_current.T)


def make_tool_down_rotation(seed_q):
    """Flange Z → −world Z (tool/needle tip hướng xuống phantom)."""
    R_seed = forward_kinematics(seed_q)[0:3, 0:3]
    z_axis = np.array([0.0, 0.0, -1.0])
    x_ref = R_seed[:, 0]
    x_axis = x_ref - np.dot(x_ref, z_axis) * z_axis
    if np.linalg.norm(x_axis) < 1e-6:
        x_ref = R_seed[:, 1]
        x_axis = x_ref - np.dot(x_ref, z_axis) * z_axis
    if np.linalg.norm(x_axis) < 1e-6:
        x_axis = np.array([1.0, 0.0, 0.0])
    x_axis = _normalize(x_axis)
    y_axis = _normalize(np.cross(z_axis, x_axis))
    x_axis = _normalize(np.cross(y_axis, z_axis))
    return np.column_stack((x_axis, y_axis, z_axis))


def numerical_jacobian_pose(q, eps=1e-4):
    q = np.asarray(q, dtype=float)
    T0 = forward_kinematics(q)
    p0 = T0[0:3, 3]
    R0 = T0[0:3, 0:3]
    J = np.zeros((6, 6))
    for i in range(6):
        q_eps = q.copy()
        q_eps[i] += eps
        T_eps = forward_kinematics(q_eps)
        J[0:3, i] = (T_eps[0:3, 3] - p0) / eps
        J[3:6, i] = rotation_vector_from_matrix(T_eps[0:3, 0:3] @ R0.T) / eps
    return J


def inverse_kinematics_pose(
    target_pos_base,
    target_R,
    seed_q,
    max_iters=800,
    pos_tolerance=0.006,
    ori_tolerance=0.12,
    damping=0.12,
    max_step=0.12,
    position_weight=1.0,
    orientation_weight=0.45,
    stay_near_seed=0.02,
):
    q = np.asarray(seed_q, dtype=float).copy()
    seed = q.copy()
    target_pos = np.asarray(target_pos_base, dtype=float)
    target_R = np.asarray(target_R, dtype=float)

    for _it in range(max_iters):
        T = forward_kinematics(q)
        pos_error = target_pos - T[0:3, 3]
        ori_error = orientation_error(T[0:3, 0:3], target_R)
        if np.linalg.norm(pos_error) < pos_tolerance and np.linalg.norm(ori_error) < ori_tolerance:
            return q.tolist()

        J = numerical_jacobian_pose(q)
        J[0:3, :] *= position_weight
        J[3:6, :] *= orientation_weight
        error = np.concatenate((position_weight * pos_error, orientation_weight * ori_error))
        A = J @ J.T + (damping ** 2) * np.eye(6)
        dq = J.T @ np.linalg.solve(A, error)
        dq += stay_near_seed * (seed - q)
        n = np.linalg.norm(dq)
        if n > max_step:
            dq = dq / n * max_step
        q = clamp_q(q + dq)
    return None


def flange_from_tip_world(tip_world, tip_length=TIP_LENGTH):
    """Tool Z xuống: flange ở trên tip một đoạn tip_length."""
    tip_base = world_to_base(tip_world)
    return tip_base + np.array([0.0, 0.0, tip_length])


def ik_tip_down(tip_world, seed_q=None, tip_length=TIP_LENGTH):
    """IK để kim/tip tới tip_world, hướng xuống."""
    if seed_q is None:
        seed_q = SEED_APPROACH
    R = make_tool_down_rotation(seed_q)
    flange = flange_from_tip_world(tip_world, tip_length=tip_length)
    return inverse_kinematics_pose(flange, R, seed_q)


def plan_hole_poses(holes, tip_length=TIP_LENGTH):
    """
    Precompute approach/insert joint poses cho từng lỗ.
    holes: list (x,y,z_surface_world)
    Returns (approach_list, insert_list) — phần tử None nếu IK fail.
    """
    approaches = []
    inserts = []
    last_ok = list(SEED_APPROACH)
    seed_bank = [
        list(SEED_APPROACH),
        list(SEED_INSERT),
        [2.6, -1.25, 1.45, -1.80, -1.57, 0.0],
        [2.9, -1.15, 1.50, -1.90, -1.57, 0.0],
        [2.4, -1.30, 1.40, -1.70, -1.57, 0.0],
    ]
    for i, (hx, hy, hz) in enumerate(holes):
        tip_app = [hx, hy, hz + APPROACH_CLEARANCE]
        tip_ins = [hx, hy, hz - INSERT_DEPTH]
        qa = None
        for seed in [last_ok] + seed_bank:
            qa = ik_tip_down(tip_app, seed_q=seed, tip_length=tip_length)
            if qa is not None:
                break
        if qa is None:
            print('[motion] IK FAIL approach hole', i, tip_app)
            approaches.append(None)
            inserts.append(None)
            continue
        qi = ik_tip_down(tip_ins, seed_q=qa, tip_length=tip_length)
        if qi is None:
            for seed in seed_bank:
                qi = ik_tip_down(tip_ins, seed_q=seed, tip_length=tip_length)
                if qi is not None:
                    break
        if qi is None:
            print('[motion] IK FAIL insert hole', i, tip_ins)
            approaches.append(qa)
            inserts.append(None)
            continue
        approaches.append(qa)
        inserts.append(qi)
        last_ok = qa
        Tw = base_to_world(forward_kinematics(qi)[0:3, 3])
        tip_est = Tw - np.array([0.0, 0.0, tip_length])
        err = float(np.linalg.norm(tip_est - np.array([hx, hy, hz - INSERT_DEPTH])))
        print(
            '[motion] hole', i,
            'tip_err_mm', round(err * 1000.0, 1),
            'est_tip', [round(float(v), 3) for v in tip_est],
            'pan', round(qi[0], 3),
        )
    return approaches, inserts
