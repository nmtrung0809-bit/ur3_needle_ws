"""
ur3_click_insert — clone_backup IK TEST port (no camera).

ure.wbt / clone_backup IK TEST:
  camera cube → cube_base → make_pick_targets_from_cube_base
  → inverse_kinematics_downward(seed=PICK_ABOVE) → seed=q_above for DOWN
  → goto ABOVE → DOWN → close → ABOVE

Here (ur3_needle.wbt):
  mouse click → Supervisor DEF HOLE_i mouth (world)
  → mouth_base → make_pick_targets_from_mouth_base
  → inverse_kinematics_downward(seed=SEED_DOWN+pan) → seed=q_above for DOWN
  → goto ABOVE → DOWN (insert) → ABOVE → HOME

No camera_utils / Recognition. Target comes from Supervisor only.
"""
from controller import Supervisor
import math
import numpy as np

import poses
import motion
import holes

MAX_CLICK_DIST = 0.03


def is_valid_vector(v, size=None):
    if v is None:
        return False
    arr = np.array(v, dtype=float)
    if size is not None and arr.size != size:
        return False
    return bool(np.all(np.isfinite(arr)))


def seed_with_pan(template, mouth_base, ref_q=None):
    """UR3e tip-down seed; shoulder_pan ≈ atan2(y,x) in DH base, nearest ref_q."""
    q = [float(v) for v in template]
    mx, my = float(mouth_base[0]), float(mouth_base[1])
    if abs(mx) + abs(my) > 1e-9:
        q[0] = float(math.atan2(my, mx))
    if ref_q is not None:
        q = motion.unwrap_q_near(q, ref_q)
    return q


def solve_above_down(mouth_world, stay_near_seed=0.004, ref_q=None):
    """
    Line-for-line clone_backup IK TEST solve, with Supervisor mouth instead of camera cube.

      mouth_world → mouth_base
      → make_pick_targets_from_mouth_base  (≡ make_pick_targets_from_cube_base)
      → inverse_kinematics_downward(seed=SEED_DOWN+pan)
      → inverse_kinematics_downward(seed=q_above)
    """
    axis = motion.GRIPPER_DOWN_AXIS
    mouth_world = [float(v) for v in mouth_world]
    mouth_base = motion.world_point_to_base(mouth_world)

    if not is_valid_vector(mouth_base, size=3):
        print('[IK TEST] mouth_base invalid:', mouth_base)
        return None, None, None, None

    # Same offsets as clone_backup.make_pick_targets_from_cube_base
    p_above_base, p_down_base = motion.make_pick_targets_from_mouth_base(mouth_base)

    print('\n[IK TEST] Mouth World:', [round(v, 4) for v in mouth_world])
    print(
        '[IK TEST] Mouth Base :', [round(float(v), 4) for v in mouth_base],
        '(BASE_X_SIGN=%g BASE_Y_SIGN=%g)' % (motion.BASE_X_SIGN, motion.BASE_Y_SIGN),
    )
    print('[IK TEST] Target above base:', [round(float(v), 4) for v in p_above_base])
    print('[IK TEST] Target down  base:', [round(float(v), 4) for v in p_down_base])
    print(
        '[IK TEST] FLANGE_TO_NEEDLE_TIP=%.3f PICK_CLEARANCE=%.3f ABOVE_CLEARANCE=%.3f axis=%s'
        % (
            motion.FLANGE_TO_NEEDLE_TIP,
            motion.PICK_CLEARANCE,
            motion.ABOVE_CLEARANCE,
            axis,
        )
    )

    if not is_valid_vector(p_above_base, size=3) or not is_valid_vector(p_down_base, size=3):
        print('[IK TEST] pick targets invalid')
        return None, None, None, None

    if ref_q is None:
        ref_q = motion.get_current_joint_positions()

    seed0 = seed_with_pan(poses.SEED_DOWN, mouth_base, ref_q=ref_q)
    seed_list = [seed0]
    for dpan in (-0.4, 0.4, -0.8, 0.8, -1.2, 1.2):
        q = list(seed0)
        q[0] = motion.unwrap_angle(float(q[0] + dpan), ref_q[0])
        seed_list.append(q)

    best_above = None
    best_cost = 1e9
    for i, seed in enumerate(seed_list):
        print('[IK TEST] Solving ABOVE seed#%d pan=%.3f' % (i, seed[0]))
        cand = motion.inverse_kinematics_downward(
            p_above_base,
            seed_q=seed,
            flange_axis=axis,
            stay_near_seed=stay_near_seed,
            max_iters=200,
            pos_tolerance=0.006,
            ori_tolerance=0.05,
        )
        if cand is None:
            alt = '-z' if axis == 'z' else 'z'
            cand = motion.inverse_kinematics_downward(
                p_above_base,
                seed_q=seed,
                flange_axis=alt,
                stay_near_seed=stay_near_seed,
                max_iters=200,
                pos_tolerance=0.006,
                ori_tolerance=0.05,
            )
            if cand is not None:
                motion.GRIPPER_DOWN_AXIS = alt
                axis = alt
        if not is_valid_vector(cand, size=6):
            continue
        cand, cost = motion.minimize_q_jump(cand, ref_q)
        print('[IK TEST]   cand pan=%.3f jump_cost=%.3f' % (cand[0], cost))
        if cost < best_cost:
            best_cost = cost
            best_above = cand

    q_above = best_above
    if not is_valid_vector(q_above, size=6):
        print('[IK TEST] Không tìm được q_above')
        return None, None, p_above_base, p_down_base

    q_down = motion.inverse_kinematics_downward(
        p_down_base,
        seed_q=q_above,
        flange_axis=axis,
        stay_near_seed=stay_near_seed,
        max_iters=200,
        pos_tolerance=0.006,
        ori_tolerance=0.05,
    )

    if not is_valid_vector(q_down, size=6):
        print('[IK TEST] Không tìm được q_down')
        return q_above, None, p_above_base, p_down_base

    q_down = motion.unwrap_q_near(q_down, q_above)
    print('[IK TEST] q_above:', [round(float(v), 4) for v in q_above],
          'pan_delta_from_ref=%.3f' % (q_above[0] - ref_q[0]))
    print('[IK TEST] q_down :', [round(float(v), 4) for v in q_down])

    if hasattr(motion, 'debug_check_gripper_down'):
        motion.debug_check_gripper_down(q_above, flange_axis=axis)
        motion.debug_check_gripper_down(q_down, flange_axis=axis)

    return q_above, q_down, p_above_base, p_down_base


def run_insert_ik_test(robot, mouth_world, hole_id):
    """One insert cycle — same motion sequence as clone_backup IK TEST."""
    motion.debug_compare_mouth_world(robot, hole_id, mouth_world)

    current_q = motion.get_current_joint_positions()
    if not is_valid_vector(current_q, size=6):
        print('[IK TEST] current_q invalid:', current_q)
        return False

    # FK / world-base (same checks as clone_backup IK TEST preamble)
    _T_base_flange = motion.forward_kinematics(current_q)
    _T_world_base = motion.get_world_base_transform()
    del _T_base_flange, _T_world_base

    q_now = motion.get_current_joint_positions()
    q_above, q_down, p_above_base, p_down_base = solve_above_down(
        mouth_world, ref_q=q_now,
    )
    if not is_valid_vector(q_above, size=6) or not is_valid_vector(q_down, size=6):
        print('[IK TEST] IK failed hole', hole_id)
        return False

    flange_above_w = motion.base_point_to_world(p_above_base)
    flange_down_w = motion.base_point_to_world(p_down_base)

    # One unwrap toward current — no SEED waypoint (SEED+ABOVE was ~2 half-turns)
    q_above = motion.unwrap_q_near(q_above, q_now)
    q_down = motion.unwrap_q_near(q_down, q_above)
    print('[IK TEST] q_above:', [round(float(v), 4) for v in q_above])
    print('[IK TEST] q_down :', [round(float(v), 4) for v in q_down])
    print(
        '[IK TEST] pan HOME→ABOVE Δ=%.3f rad (%.0f deg) — expect ~±180 max, not 360'
        % (q_above[0] - q_now[0], abs(q_above[0] - q_now[0]) * 180.0 / math.pi)
    )

    # Direct ABOVE (clone_backup IK TEST: no extra SEED spin)
    print('=== goto ABOVE ===')
    ok = motion.goto_pose(q_above, steps=100, tolerance=0.05)
    if not ok:
        print('[IK TEST] goto_pose(q_above) failed')
        return False
    print('[IK TEST] Đã tới q_above.')
    motion.wait_steps(20)
    motion.debug_compare_flange_world(q_above, flange_above_w, robot)
    tip, _ = motion.measure_tip_world(robot)
    print(
        '[ABOVE] tip', [round(float(v), 3) for v in tip],
        'xy_mm vs mouth',
        round(math.hypot(tip[0] - mouth_world[0], tip[1] - mouth_world[1]) * 1000, 1),
    )

    # 6. Insert straight down Z (fixed XY, IK waypoints — not joint-space lerp)
    print('=== goto DOWN (straight Z) ===')
    ok, z_path = motion.goto_straight_z(
        p_above_base, p_down_base, q_above,
        n_steps=16, steps_per_segment=10, tolerance=0.05,
    )
    if not ok:
        print('[IK TEST] straight-Z insert failed — fallback goto q_down')
        ok = motion.goto_pose(q_down, steps=60, tolerance=0.05)
        if not ok:
            print('[IK TEST] goto_pose(q_down) failed')
            return False
    else:
        q_down = z_path[-1]
    print('[IK TEST] Đã tới q_down (Z path).')
    motion.wait_steps(40)
    motion.debug_compare_flange_world(q_down, flange_down_w, robot)
    tip, _ = motion.measure_tip_world(robot)
    print(
        '[DOWN] tip', [round(float(v), 3) for v in tip],
        'xy_mm', round(math.hypot(tip[0] - mouth_world[0], tip[1] - mouth_world[1]) * 1000, 1),
        'depth_mm', round((mouth_world[2] - tip[2]) * 1000, 1),
    )

    # Retract straight up Z
    print('=== retract ABOVE (straight Z) ===')
    ok, _ = motion.goto_straight_z(
        p_down_base, p_above_base, q_down,
        n_steps=16, steps_per_segment=10, tolerance=0.05,
    )
    if not ok:
        motion.goto_pose(q_above, steps=50, tolerance=0.05)
    print('=== HOME ===')
    motion.goto_pose(poses.HOME, steps=90, tolerance=0.04)
    print('[IK TEST] Insert xong hole', hole_id)
    return True


def main():
    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())
    print('[ur3_click_insert] clone_backup IK TEST — Supervisor HOLE, no camera')

    # dt=8ms (world basicTimeStep): use higher v than clone's 0.8@32ms
    motion.init(robot, dt, motor_velocity=1.5)

    finger_motors = []
    for i in range(robot.getNumberOfDevices()):
        dev = robot.getDeviceByIndex(i)
        n = dev.getName()
        if 'finger joint' in n and 'sensor' not in n:
            try:
                dev.setVelocity(0.8)
                finger_motors.append(dev)
            except Exception:
                pass

    def close_fingers():
        for fm in finger_motors:
            fm.setPosition(poses.FINGER_CLOSED)

    mouse = robot.getMouse()
    mouse.enable(dt)
    mouse.enable3dPosition()

    # Real mouths from Supervisor (replaces camera cube detection)
    hole_list = holes.measure_hole_centers(robot)

    # Warm-up (clone_backup: step ×10, HOME, open gripper, PICK_ABOVE)
    for _ in range(10):
        if robot.step(dt) == -1:
            return

    close_fingers()
    print('=== SEED_DOWN warm-up + calib TCP→tip (UR3e) ===')
    motion.move_ur_to(poses.SEED_DOWN)
    motion.wait_steps(40)
    motion.goto_pose(poses.SEED_DOWN, steps=60, tolerance=0.04)
    motion.calibrate_flange_to_tip(robot)

    q_now = motion.get_current_joint_positions()
    tcp = motion.measure_tcp_world(robot)
    if tcp is not None:
        motion.debug_check_fk(tcp)
        fk_w = motion.base_point_to_world(motion.fk_position(q_now))
        print(
            '[CHECK] SEED_DOWN TCP', [round(float(v), 4) for v in tcp],
            'FK', [round(float(v), 4) for v in fk_w],
        )

    print('=== HOME ===')
    motion.goto_pose(poses.HOME, steps=80, tolerance=0.04)

    prev_left = False
    busy = False
    step_i = 0

    print('=== IK TEST READY — click hole (no camera) ===')
    print(
        '  tip_off=%.3f  above_clr=%.3f  axis=%s'
        % (motion.FLANGE_TO_NEEDLE_TIP, motion.ABOVE_CLEARANCE, motion.GRIPPER_DOWN_AXIS)
    )
    for h in hole_list:
        m = h['mouth']
        print('   id', h['id'], 'mouth world', [round(float(v), 4) for v in m])

    while robot.step(dt) != -1:
        step_i += 1
        close_fingers()
        if not busy:
            motion.move_ur_to(poses.HOME)

        mstate = mouse.getState()
        clicked = mstate.left and not prev_left
        prev_left = mstate.left

        if busy or not clicked:
            if step_i % 250 == 0:
                print('[ur3_click_insert] idle — click a hole')
            continue

        if mstate.x != mstate.x:
            print('[click] NaN — click trên mặt phantom')
            continue

        hid, dist = holes.nearest_hole_id(hole_list, mstate.x, mstate.y, MAX_CLICK_DIST)
        print(
            '[click] 3D=({:.3f},{:.3f},{:.3f}) dist={:.4f}'.format(
                mstate.x, mstate.y, mstate.z, dist
            )
        )
        if hid is None:
            print('[click] quá xa miệng lỗ')
            continue

        mouth = holes.mouth_xyz(hole_list, hid)
        print('=== Hole %d → IK TEST insert ===' % hid)

        busy = True
        ok = run_insert_ik_test(robot, mouth, hid)
        if not ok:
            motion.goto_pose(poses.HOME, steps=60, tolerance=0.05)
        busy = False


if __name__ == '__main__':
    main()
