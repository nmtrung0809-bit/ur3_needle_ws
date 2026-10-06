# motion.py — IK port từ universal_robots/controllers/clone_backup/motion.py
# DH / base transform chỉnh cho UR3e (ur3_needle.wbt: translation 0 0 0.6).
# KHÔNG dùng tip-Jacobian IK. KHÔNG dùng camera.
#
# Pipeline (clone_backup IK TEST, Supervisor mouth thay cube camera):
#   mouth_world → mouth_base → make_pick_targets_from_mouth_base
#   → inverse_kinematics_downward (seed UR3e SEED_DOWN + pan)
#   → DOWN seed = q_above

import math
import numpy as np

UR_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

_robot = None
_time_step = None
_ur_motors = []
_ur_sensors = []

# --- insert / pick offsets (clone_backup make_pick_targets_from_cube_base style) ---
HOVER_HEIGHT = 0.04          # kept for logs
INSERT_DEPTH = 0.02          # tip 2 cm below mouth (PICK_CLEARANCE = -0.02)
# Like FLANGE_TO_GRIPPER_TIP in clone_backup.py (tune via TCP calib)
FLANGE_TO_NEEDLE_TIP = 0.20
PICK_CLEARANCE = -INSERT_DEPTH   # negative = tip enters hole (clone used +0.01 above surface)
ABOVE_CLEARANCE = 0.16           # same as clone_backup ABOVE_CLEARANCE
GRIPPER_DOWN_AXIS = 'z'

# Seeds UR3e tip-down (không giữ pose ure). IK được phép lệch seed.
SEED_DOWN = [0.3246, -0.7505, 1.8532, -2.0386, -3.1544, 2.1924]
PICK_ABOVE = list(SEED_DOWN)
PICK_DOWN = [0.3246, -0.95, 1.95, -2.20, -3.1544, 2.1924]

TIP_LOCAL_NEEDLE = np.array([0.0, 0.0, -0.18])

# UR3e Modified DH (Craig) — NOT UR5e
_DH = [
    # (alpha_prev, a_prev,   d,       theta_offset)
    (0.0,          0.0,      0.15185, 0.0),
    (np.pi / 2,    0.0,      0.0,     0.0),
    (0.0,         -0.24355,  0.0,     0.0),
    (0.0,         -0.2132,   0.13105, 0.0),
    (np.pi / 2,    0.0,      0.08535, 0.0),
    (-np.pi / 2,   0.0,      0.0921,  0.0),
]

LOWER_LIMIT = np.array([-6.28, -6.28, -3.14, -6.28, -6.28, -6.28])
UPPER_LIMIT = np.array([6.28, 6.28, 3.14, 6.28, 6.28, 6.28])


def init(robot, time_step, motor_velocity=1.5):
    global _robot, _time_step, _ur_motors, _ur_sensors, _motor_velocity
    _robot = robot
    _time_step = time_step
    _motor_velocity = float(motor_velocity)
    _ur_motors = []
    _ur_sensors = []
    for name in UR_JOINT_NAMES:
        motor = robot.getDevice(name)
        sensor = robot.getDevice(name + '_sensor')
        motor.setVelocity(_motor_velocity)
        sensor.enable(time_step)
        _ur_motors.append(motor)
        _ur_sensors.append(sensor)
    _robot.step(_time_step)


_motor_velocity = 0.8


def move_ur_to(q):
    for motor, pos in zip(_ur_motors, q):
        if pos != pos:
            return
        motor.setPosition(float(pos))


def get_current_joint_positions():
    return [float(s.getValue()) for s in _ur_sensors]


def wait_steps(n):
    for _ in range(n):
        if _robot.step(_time_step) == -1:
            return False
    return True


def unwrap_angle(target, ref):
    """Shortest signed delta in (-π, π], then ref+delta (no while-loop flip)."""
    d = math.atan2(math.sin(float(target) - float(ref)),
                   math.cos(float(target) - float(ref)))
    return float(ref) + d


def unwrap_q_near(target_q, ref_q):
    """
    Map each target joint to the 2π-equivalent nearest ref_q.
    Avoids HOME→ABOVE taking an extra full turn on pan/wrist.
    """
    return [unwrap_angle(t, r) for t, r in zip(target_q, ref_q)]


def minimize_q_jump(target_q, ref_q):
    """Unwrap then report total |Δq| (for logging / picking among IK seeds)."""
    q = unwrap_q_near(target_q, ref_q)
    cost = sum(abs(a - b) for a, b in zip(q, ref_q))
    return q, cost


def max_joint_error(current_q, target_q):
    return max(abs(c - t) for c, t in zip(current_q, target_q))


def joint_errors(current_q, target_q):
    return [abs(c - t) for c, t in zip(current_q, target_q)]


def _steps_for_distance(dq_max, steps_hint, velocity=None):
    """Enough sim steps for the slowest joint at current motor velocity."""
    if velocity is None:
        velocity = max(_motor_velocity, 1e-6)
    dt = float(_time_step) / 1000.0
    # distance/vel /dt + margin; never fewer than caller hint
    need = int(math.ceil(float(dq_max) / (velocity * dt))) + 30
    return max(int(steps_hint), need)


def smooth_move_ur_to(target_q, steps=100):
    start_q = get_current_joint_positions()
    target_q = unwrap_q_near(target_q, start_q)
    dq = max_joint_error(start_q, target_q)
    steps = _steps_for_distance(dq, steps)
    for i in range(steps + 1):
        ratio = i / float(steps)
        q = [a + ratio * (b - a) for a, b in zip(start_q, target_q)]
        move_ur_to(q)
        if _robot.step(_time_step) == -1:
            return False
    return True


def wait_until_reached(target_q, tolerance=0.04, max_steps=None):
    start_q = get_current_joint_positions()
    target_q = unwrap_q_near(target_q, start_q)
    if max_steps is None:
        dq = max_joint_error(start_q, target_q)
        # Allow full catch-up after smooth lag (dt=8ms needs more steps than clone's 32)
        max_steps = _steps_for_distance(dq, 500)
        max_steps = max(max_steps, 1500)

    errors = joint_errors(start_q, target_q)
    for _ in range(max_steps):
        current_q = get_current_joint_positions()
        errors = joint_errors(current_q, target_q)
        if max(errors) < tolerance:
            return True
        move_ur_to(target_q)
        if _robot.step(_time_step) == -1:
            return False
    names = UR_JOINT_NAMES
    print(
        '[MOTION WARNING] wait_until_reached timeout. Max joint error =',
        round(max(errors), 5),
    )
    print(
        '  per-joint err',
        {names[i]: round(errors[i], 4) for i in range(len(errors))},
    )
    print(
        '  current', [round(v, 4) for v in get_current_joint_positions()],
        'target', [round(float(v), 4) for v in target_q],
    )
    return False


def goto_pose(target_q, steps=100, tolerance=0.04):
    start_q = get_current_joint_positions()
    target_q = unwrap_q_near(target_q, start_q)
    dq = max_joint_error(start_q, target_q)
    steps = _steps_for_distance(dq, steps)
    print(
        '[MOTION] goto dq_max=%.3f rad steps=%d dt=%dms v=%.2f'
        % (dq, steps, int(_time_step), _motor_velocity)
    )
    if not smooth_move_ur_to(target_q, steps=steps):
        return False
    return wait_until_reached(target_q, tolerance=tolerance)


def clamp_q(q):
    return np.minimum(np.maximum(np.asarray(q, dtype=float), LOWER_LIMIT), UPPER_LIMIT)


def _normalize(v, eps=1e-9):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < eps:
        return v
    return v / n


# ------------------------------------------------------------------
# World <-> Base (UR3e in ur3_needle.wbt: translation 0 0 0.6, no yaw)
#
# User reports: hole target mirrored across arm Y (X flips) and also
# across world X (Y flips). Combined = Rz(π): world(x,y) ↔ DH(−x,−y).
# ------------------------------------------------------------------

# +1 = same as world; -1 = flip that axis in DH base
# Both -1 ⇒ 180° about Z (proper rotation, not a reflection)
BASE_X_SIGN = -1.0
BASE_Y_SIGN = -1.0


def get_world_base_transform():
    """
    T_world_base: p_world = T @ p_dh_base

    R = diag(BASE_X_SIGN, BASE_Y_SIGN, 1), t = (0, 0, 0.6)
    With both signs -1 this is Rz(π).
    """
    T = np.eye(4)
    T[0, 0] = float(BASE_X_SIGN)
    T[1, 1] = float(BASE_Y_SIGN)
    T[2, 3] = 0.6
    return T


def world_point_to_base(p_world):
    T_base_world = np.linalg.inv(get_world_base_transform())
    p_h = np.array([p_world[0], p_world[1], p_world[2], 1.0], dtype=float)
    return (T_base_world @ p_h)[0:3]


def base_point_to_world(p_base):
    T = get_world_base_transform()
    p_h = np.array([p_base[0], p_base[1], p_base[2], 1.0], dtype=float)
    return (T @ p_h)[0:3]


# ------------------------------------------------------------------
# UR3e DH flange FK + clone_backup pose IK
# ------------------------------------------------------------------

def _dh_matrix(alpha, a, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
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


def forward_kinematics_world(q):
    return get_world_base_transform() @ forward_kinematics(q)


def fk_position(q):
    return forward_kinematics(q)[0:3, 3]


def fk_rotation(q):
    return forward_kinematics(q)[0:3, 0:3]


def rotation_vector_from_matrix(R):
    cos_angle = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    vee = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ])
    if angle < 1e-6:
        return 0.5 * vee
    return (angle / (2.0 * np.sin(angle))) * vee


def orientation_error(R_current, R_target):
    return rotation_vector_from_matrix(R_target @ R_current.T)


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
        dR = T_eps[0:3, 0:3] @ R0.T
        J[3:6, i] = rotation_vector_from_matrix(dR) / eps
    return J


def make_gripper_down_rotation(seed_q=None, flange_axis='z'):
    """clone_backup: force flange approach axis to world −Z."""
    if seed_q is None:
        seed_q = get_current_joint_positions()
    R_seed = fk_rotation(seed_q)
    down_base = np.array([0.0, 0.0, -1.0])
    if flange_axis == 'z':
        z_axis = down_base
    elif flange_axis == '-z':
        z_axis = -down_base
    else:
        raise ValueError("flange_axis supports 'z' or '-z'")
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


def inverse_kinematics_pose(
    target_pos_base,
    target_R_base_flange,
    seed_q=None,
    max_iters=200,
    pos_tolerance=0.003,
    ori_tolerance=0.04,
    damping=0.06,
    max_step=0.08,
    position_weight=1.0,
    orientation_weight=0.6,
    stay_near_seed=0.12,
):
    """clone_backup DLS pose IK (flange in Base)."""
    if seed_q is None:
        q = np.array(get_current_joint_positions(), dtype=float)
    else:
        q = np.array(seed_q, dtype=float)
    seed = q.copy()
    target_pos = np.asarray(target_pos_base, dtype=float)
    target_R = np.asarray(target_R_base_flange, dtype=float)
    best_q, best_score = list(q), 1e9

    for it in range(max_iters):
        T = forward_kinematics(q)
        pos_error = target_pos - T[0:3, 3]
        ori_error = orientation_error(T[0:3, 0:3], target_R)
        pos_norm = float(np.linalg.norm(pos_error))
        ori_norm = float(np.linalg.norm(ori_error))
        score = pos_norm + 0.05 * ori_norm
        if score < best_score:
            best_score = score
            best_q = list(q)

        if pos_norm < pos_tolerance and ori_norm < ori_tolerance:
            print(
                '[IK-POSE] Success iter', it,
                'pos_err', round(pos_norm, 5),
                'ori_deg', round(math.degrees(ori_norm), 2),
            )
            return unwrap_q_near(q.tolist(), seed.tolist())

        Jw = numerical_jacobian_pose(q)
        Jw[0:3, :] *= position_weight
        Jw[3:6, :] *= orientation_weight
        err = np.concatenate((
            position_weight * pos_error,
            orientation_weight * ori_error,
        ))
        A = Jw @ Jw.T + (damping ** 2) * np.eye(6)
        try:
            dq = Jw.T @ np.linalg.solve(A, err)
        except np.linalg.LinAlgError:
            break
        dq = dq + stay_near_seed * (seed - q)
        n = float(np.linalg.norm(dq))
        if n > max_step:
            dq = dq / n * max_step
        q = clamp_q(q + dq)

    T = forward_kinematics(best_q)
    final_pos = float(np.linalg.norm(target_pos - T[0:3, 3]))
    print('[IK-POSE] Failed. best pos_err =', round(final_pos, 5), 'm')
    # Accept near-miss so outer holes still move
    if final_pos < 0.05:
        print('[IK-POSE] Using BEST (pos_err < 5 cm)')
        return unwrap_q_near(list(best_q), seed.tolist())
    return None


def inverse_kinematics_downward(
    target_pos_base,
    seed_q=None,
    flange_axis=None,
    **kwargs
):
    """clone_backup: flange to target_pos_base, gripper down."""
    if seed_q is None:
        seed_q = get_current_joint_positions()
    if flange_axis is None:
        flange_axis = GRIPPER_DOWN_AXIS
    target_R = make_gripper_down_rotation(seed_q=seed_q, flange_axis=flange_axis)
    return inverse_kinematics_pose(
        target_pos_base=target_pos_base,
        target_R_base_flange=target_R,
        seed_q=seed_q,
        **kwargs
    )


def debug_check_gripper_down(q, flange_axis=None):
    if flange_axis is None:
        flange_axis = GRIPPER_DOWN_AXIS
    R = fk_rotation(q)
    approach = R[:, 2] if flange_axis == 'z' else -R[:, 2]
    desired = np.array([0.0, 0.0, -1.0])
    dot_val = float(np.clip(np.dot(_normalize(approach), desired), -1.0, 1.0))
    angle_deg = float(math.degrees(math.acos(dot_val)))
    print('[CHECK] approach', [round(float(v), 4) for v in approach],
          'angle_to_down_deg', round(angle_deg, 2))
    return angle_deg


def debug_check_fk(webots_position, webots_orientation=None):
    """clone_backup: compare DH FK world vs Webots TCP/flange position."""
    current_q = get_current_joint_positions()
    calc_pos = base_point_to_world(fk_position(current_q))
    print('\n=== [CHECK STEP 1] FORWARD KINEMATICS VERIFICATION ===')
    print('q', [round(a, 4) for a in current_q])
    print('Webots pos', [round(float(v), 6) for v in webots_position])
    print('FK world  ', [round(float(p), 6) for p in calc_pos])
    w = np.array(webots_position, dtype=float)
    error = float(np.linalg.norm(w - calc_pos))
    print('--> err', round(error, 6), 'm')
    print(
        '--> BASE_X_SIGN', BASE_X_SIGN, 'BASE_Y_SIGN', BASE_Y_SIGN,
        'dx(FK-TCP)', round(float(calc_pos[0] - w[0]), 4),
        'dy', round(float(calc_pos[1] - w[1]), 4),
    )
    if error < 0.015:
        print('[STATUS] FK OK')
        return True
    print('[WARNING] FK lệch Webots — chỉnh BASE_X_SIGN / BASE_Y_SIGN / Z')
    return False


# ------------------------------------------------------------------
# Tip measure (log / calib offset only)
# ------------------------------------------------------------------

def ori_to_R(ori9):
    v = list(ori9)
    return np.column_stack((v[0:3], v[3:6], v[6:9]))


def measure_tip_world(supervisor=None):
    robot = supervisor if supervisor is not None else _robot
    n = robot.getFromDef('NEEDLE')
    if n is None:
        raise RuntimeError('DEF NEEDLE not found')
    p = np.array(n.getPosition(), dtype=float)
    R = ori_to_R(n.getOrientation())
    tip = p + R @ TIP_LOCAL_NEEDLE
    direction = -R[:, 2]
    nrm = np.linalg.norm(direction)
    if nrm > 1e-9:
        direction = direction / nrm
    return tip, direction


def measure_tcp_world(supervisor=None):
    """Webots DEF TCP (toolSlot) — real flange/TCP, not DH FK."""
    robot = supervisor if supervisor is not None else _robot
    n = robot.getFromDef('TCP')
    if n is None:
        return None
    return np.array(n.getPosition(), dtype=float)


def calibrate_flange_to_tip(supervisor=None):
    """
    Estimate FLANGE_TO_NEEDLE_TIP from Webots TCP vs NEEDLE tip (not DH FK).
    Also pick GRIPPER_DOWN_AXIS from DH R at current q (orientation only).
    """
    global FLANGE_TO_NEEDLE_TIP, GRIPPER_DOWN_AXIS
    q = get_current_joint_positions()
    tip, tip_dir = measure_tip_world(supervisor)
    tcp = measure_tcp_world(supervisor)
    if tcp is not None:
        dz = float(tcp[2] - tip[2])
        print('[calib] Webots TCP', [round(float(v), 4) for v in tcp],
              'tip', [round(float(v), 4) for v in tip])
        if dz > 0.05:
            FLANGE_TO_NEEDLE_TIP = dz
            print('[calib] FLANGE_TO_NEEDLE_TIP <-', round(dz, 4), '(TCP−tip Z)')
        else:
            # tip may not be below TCP in this pose; use Euclidean as length
            L = float(np.linalg.norm(tcp - tip))
            if L > 0.05:
                FLANGE_TO_NEEDLE_TIP = L
                print('[calib] FLANGE_TO_NEEDLE_TIP <-', round(L, 4), '(||TCP−tip||)')
            else:
                print('[calib] keep FLANGE_TO_NEEDLE_TIP=', FLANGE_TO_NEEDLE_TIP)
    else:
        print('[calib] no DEF TCP — keep FLANGE_TO_NEEDLE_TIP=', FLANGE_TO_NEEDLE_TIP)

    R = fk_rotation(q)
    desired = np.array([0.0, 0.0, -1.0])
    best_axis, best_dot = 'z', -1e9
    for name, vec in (('z', R[:, 2]), ('-z', -R[:, 2])):
        d = float(np.dot(_normalize(vec), desired))
        if d > best_dot:
            best_dot, best_axis = d, name
    # Prefer tip direction: if tip aims down, keep axis that matches FK
    tip_down = float(np.dot(tip_dir, desired))
    if tip_down > 0.5:
        GRIPPER_DOWN_AXIS = best_axis
    else:
        GRIPPER_DOWN_AXIS = best_axis
    print('[calib] GRIPPER_DOWN_AXIS <-', GRIPPER_DOWN_AXIS,
          'fk_dot', round(best_dot, 3),
          'tip_down', round(tip_down, 3))
    return FLANGE_TO_NEEDLE_TIP, GRIPPER_DOWN_AXIS


def calibrate_needle_tip_offset(supervisor=None, settle_q=None, settle_steps=20):
    return calibrate_flange_to_tip(supervisor)


def calibrate_tip_in_flange(supervisor=None, settle_q=None, settle_steps=20):
    return calibrate_flange_to_tip(supervisor)[0]


# ------------------------------------------------------------------
# Mouth → flange targets + IK (seeds = PICK_ABOVE / PICK_DOWN)
# ------------------------------------------------------------------

def make_pick_targets_from_mouth_base(mouth_base, flange_to_tip=None):
    """
    Same as clone_backup.make_pick_targets_from_cube_base(cube_base):

      mouth_base = hole mouth in robot Base (like cube top in Base)
      p_down  = [x, y, mouth_z + FLANGE_TO_NEEDLE_TIP + PICK_CLEARANCE]
      p_above = [x, y, p_down_z + ABOVE_CLEARANCE]

    Returns flange targets in **Base** (IK frame). No tip IK.
    """
    if flange_to_tip is None:
        flange_to_tip = FLANGE_TO_NEEDLE_TIP
    mx, my, mz = [float(v) for v in mouth_base]
    # mouth already = top of hole (circle center), like cube_top_z
    p_down_base = np.array([
        mx, my, mz + flange_to_tip + PICK_CLEARANCE
    ], dtype=float)
    p_above_base = np.array([
        mx, my, p_down_base[2] + ABOVE_CLEARANCE
    ], dtype=float)
    return p_above_base, p_down_base


def ik_straight_z_path(
    p_from_base,
    p_to_base,
    seed_q,
    n_steps=16,
    flange_axis=None,
    stay_near_seed=0.004,
):
    """
    Cartesian line with fixed XY, only Z changes (straight insert/retract along Z).

    Returns list of joint vectors length n_steps+1 (includes start & end).
    """
    if flange_axis is None:
        flange_axis = GRIPPER_DOWN_AXIS
    x0, y0, z0 = [float(v) for v in p_from_base]
    _x1, _y1, z1 = [float(v) for v in p_to_base]
    # Force XY from start — true vertical in base (= world Z with our R)
    path = []
    q = list(seed_q)
    n_steps = max(1, int(n_steps))
    for i in range(n_steps + 1):
        t = i / float(n_steps)
        p = [x0, y0, z0 + t * (z1 - z0)]
        qi = inverse_kinematics_downward(
            p,
            seed_q=q,
            flange_axis=flange_axis,
            stay_near_seed=stay_near_seed,
            max_iters=200,
            pos_tolerance=0.006,
            ori_tolerance=0.05,
        )
        if qi is None:
            alt = '-z' if flange_axis == 'z' else 'z'
            qi = inverse_kinematics_downward(
                p,
                seed_q=q,
                flange_axis=alt,
                stay_near_seed=stay_near_seed,
                max_iters=200,
                pos_tolerance=0.006,
                ori_tolerance=0.05,
            )
            if qi is not None:
                flange_axis = alt
                globals()['GRIPPER_DOWN_AXIS'] = alt
        if qi is None:
            print('[Z-PATH] IK fail at step %d/%d p=%s' % (i, n_steps, [round(v, 4) for v in p]))
            return path if path else None
        qi = unwrap_q_near(qi, q)
        path.append([float(v) for v in qi])
        q = path[-1]
    print(
        '[Z-PATH] %d pts  XY=(%.4f, %.4f)  Z %.4f → %.4f'
        % (len(path), x0, y0, z0, z1)
    )
    return path


def follow_q_path(path, steps_per_segment=12, tolerance=0.05):
    """Follow joint waypoints; full settle only on last point."""
    if not path:
        return False
    for i, q in enumerate(path):
        if i == len(path) - 1:
            return goto_pose(q, steps=max(steps_per_segment, 40), tolerance=tolerance)
        if not smooth_move_ur_to(q, steps=steps_per_segment):
            return False
    return True


def goto_straight_z(p_from_base, p_to_base, seed_q, n_steps=16, steps_per_segment=12, tolerance=0.05):
    """Insert/retract along Z: IK path then follow."""
    path = ik_straight_z_path(p_from_base, p_to_base, seed_q, n_steps=n_steps)
    if not path:
        return False, None
    ok = follow_q_path(path, steps_per_segment=steps_per_segment, tolerance=tolerance)
    return ok, path


def make_pick_targets_from_mouth(mouth_world, flange_to_tip=None):
    """World mouth → Base mouth → flange ABOVE/DOWN in Base; also return world copies."""
    mouth_base = world_point_to_base(mouth_world)
    p_above_b, p_down_b = make_pick_targets_from_mouth_base(mouth_base, flange_to_tip)
    return (
        base_point_to_world(p_above_b),
        base_point_to_world(p_down_b),
        p_above_b,
        p_down_b,
        mouth_base,
    )


def debug_compare_mouth_world(supervisor, hole_id, mouth_used, cyl_height=0.08):
    """
    Like camera_utils.debug_check_camera_transform:
    compare used mouth vs Supervisor DEF HOLE_i real world position.
    """
    node = supervisor.getFromDef('HOLE_%d' % int(hole_id))
    if node is None:
        print('[CHECK] missing DEF HOLE_%d' % hole_id)
        return None
    center = np.array(node.getPosition(), dtype=float)
    actual_mouth = center + np.array([0.0, 0.0, cyl_height * 0.5])
    used = np.array(mouth_used, dtype=float)
    err = float(np.linalg.norm(actual_mouth - used))
    print('\n=== [CHECK] MOUTH WORLD vs SUPERVISOR ===')
    print('  used mouth     ', [round(float(v), 4) for v in used])
    print('  supervisor mouth', [round(float(v), 4) for v in actual_mouth])
    print('  --> err', round(err, 5), 'm')
    if err < 0.015:
        print('  [STATUS] mouth OK (< 1.5 cm)')
    else:
        print('  [WARNING] mouth mismatch — check phantom / HOLE Pose')
    return actual_mouth, err


def debug_compare_flange_world(q, target_flange_world, supervisor=None):
    """
    Compare FK flange world vs target vs Webots TCP (real).
    Like motion.debug_check_fk + camera world check.
    """
    fk_w = base_point_to_world(fk_position(q))
    tgt = np.array(target_flange_world, dtype=float)
    err_fk = float(np.linalg.norm(fk_w - tgt))
    print('\n=== [CHECK] FLANGE WORLD (FK vs target) ===')
    print('  target flange ', [round(float(v), 4) for v in tgt])
    print('  FK flange     ', [round(float(v), 4) for v in fk_w])
    print('  --> FK err', round(err_fk, 5), 'm')
    tcp = measure_tcp_world(supervisor)
    if tcp is not None:
        err_tcp = float(np.linalg.norm(tcp - tgt))
        err_fk_tcp = float(np.linalg.norm(tcp - fk_w))
        print('  Webots TCP    ', [round(float(v), 4) for v in tcp])
        print('  --> TCP vs target', round(err_tcp, 5), 'm')
        print('  --> TCP vs FK    ', round(err_fk_tcp, 5), 'm (DH vs Webots)')
    return err_fk


def _seed_with_pan(template, mouth_xy):
    """Seed tip-down + shoulder_pan ≈ atan2(y,x) in **DH base** (after world→base)."""
    q = [float(v) for v in template]
    mx, my = float(mouth_xy[0]), float(mouth_xy[1])
    if abs(mx) + abs(my) > 1e-9:
        q[0] = float(math.atan2(my, mx))
    return q


def make_insert_targets(mouth_world, seed_q=None, stay_near_seed=0.004, **kwargs):
    """
    clone_backup pick-target + flange IK; seed UR3e (không giữ pose ure).

      mouth_world → mouth_base → make_pick_targets_from_mouth_base
      → inverse_kinematics_downward (seed tip-down + pan)
    """
    kwargs.pop('flange_axis', None)
    axis = kwargs.pop('flange_axis_force', None) or GRIPPER_DOWN_AXIS

    mouth_world = [float(v) for v in mouth_world]
    (
        flange_above_w, flange_down_w,
        p_above_base, p_down_base, mouth_base,
    ) = make_pick_targets_from_mouth(mouth_world)

    base_seed = list(seed_q) if seed_q is not None else list(SEED_DOWN)
    # Pan from mouth_base (X may be flipped vs world)
    seed_above = _seed_with_pan(base_seed, mouth_base)
    # Thêm vài seed pan lệch cho lỗ ngoài
    seed_list = [seed_above]
    for dpan in (-0.4, 0.4, -0.8, 0.8):
        q = list(seed_above)
        q[0] = float(q[0] + dpan)
        seed_list.append(q)

    print('\n[IK] Mouth World:', [round(v, 4) for v in mouth_world])
    print('[IK] Mouth Base :', [round(float(v), 4) for v in mouth_base])
    print('[IK] Target above base:', [round(float(v), 4) for v in p_above_base])
    print('[IK] Target down  base:', [round(float(v), 4) for v in p_down_base])
    print('[IK] FLANGE_TO_NEEDLE_TIP={:.3f} CLEARANCE={:.3f} ABOVE_CLR={:.3f}'.format(
        FLANGE_TO_NEEDLE_TIP, PICK_CLEARANCE, ABOVE_CLEARANCE))
    print('[IK] seed ABOVE (UR3e+pan)', [round(v, 3) for v in seed_above])

    q_above = None
    for i, seed in enumerate(seed_list):
        print('[IK] Solving ABOVE seed#%d' % i)
        q_above = inverse_kinematics_downward(
            p_above_base,
            seed_q=seed,
            flange_axis=axis,
            stay_near_seed=stay_near_seed,
            max_iters=200,
            pos_tolerance=0.006,
            ori_tolerance=0.05,
        )
        if q_above is None:
            alt = '-z' if axis == 'z' else 'z'
            q_above = inverse_kinematics_downward(
                p_above_base,
                seed_q=seed,
                flange_axis=alt,
                stay_near_seed=stay_near_seed,
                max_iters=200,
                pos_tolerance=0.006,
            )
            if q_above is not None:
                globals()['GRIPPER_DOWN_AXIS'] = alt
                axis = alt
        if q_above is not None:
            break
    if q_above is None:
        print('[IK] q_above FAILED')
        return None, None

    print('[IK] Solving DOWN (seed=q_above)')
    q_down = inverse_kinematics_downward(
        p_down_base,
        seed_q=q_above,
        flange_axis=axis,
        stay_near_seed=stay_near_seed,
        max_iters=200,
        pos_tolerance=0.006,
        ori_tolerance=0.05,
    )
    if q_down is None:
        print('[IK] q_down FAILED')
        return q_above, None

    debug_check_gripper_down(q_above, flange_axis=axis)
    debug_check_gripper_down(q_down, flange_axis=axis)
    print('[IK] q_above', [round(float(v), 4) for v in q_above])
    print('[IK] q_down ', [round(float(v), 4) for v in q_down])
    return q_above, q_down


def pin_along_z(mouth_xyz, seed_q=None, **kwargs):
    return make_insert_targets(mouth_xyz, seed_q=seed_q, **kwargs)


def plan_approach_insert(hole_xyz, seed_q=None):
    return make_insert_targets(hole_xyz, seed_q=seed_q)
