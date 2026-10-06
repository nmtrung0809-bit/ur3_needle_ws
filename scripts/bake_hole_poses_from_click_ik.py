#!/usr/bin/env python3
"""
Offline bake: same IK as ur3_click_insert (clone_backup flange DH) → hole_poses.yaml for ROS.

Uses controllers/ur3_click_insert/motion.py + poses.py + holes.FALLBACK_MOUTH.
No Webots required (pure DH IK).
"""
from __future__ import annotations

import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
CTRL = os.path.join(ROOT, 'controllers', 'ur3_click_insert')
sys.path.insert(0, CTRL)

import holes  # noqa: E402
import motion  # noqa: E402
import poses  # noqa: E402


def is_valid_vector(v, size=None):
    if v is None:
        return False
    import numpy as np
    arr = np.array(v, dtype=float)
    if size is not None and arr.size != size:
        return False
    return bool(np.all(np.isfinite(arr)))


def seed_with_pan(template, mouth_base, ref_q):
    q = [float(v) for v in template]
    mx, my = float(mouth_base[0]), float(mouth_base[1])
    if abs(mx) + abs(my) > 1e-9:
        q[0] = float(math.atan2(my, mx))
    return motion.unwrap_q_near(q, ref_q)


def solve_above_down(mouth_world, ref_q, stay_near_seed=0.004):
    axis = motion.GRIPPER_DOWN_AXIS
    mouth_world = [float(v) for v in mouth_world]
    mouth_base = motion.world_point_to_base(mouth_world)
    p_above_base, p_down_base = motion.make_pick_targets_from_mouth_base(mouth_base)

    seed0 = seed_with_pan(poses.SEED_DOWN, mouth_base, ref_q)
    seed_list = [seed0]
    for dpan in (-0.4, 0.4, -0.8, 0.8, -1.2, 1.2):
        q = list(seed0)
        q[0] = motion.unwrap_angle(float(q[0] + dpan), ref_q[0])
        seed_list.append(q)

    best_above = None
    best_cost = 1e9
    for seed in seed_list:
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
        if cost < best_cost:
            best_cost = cost
            best_above = cand

    if not is_valid_vector(best_above, size=6):
        return None, None

    # Prefer branch near HOME so ROS FollowJointTrajectory does not take ±2π wrist flips
    home = list(poses.HOME)
    q_above = motion.unwrap_q_near(best_above, home)
    q_above = motion.unwrap_q_near(q_above, ref_q)
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
        return q_above, None
    q_down = motion.unwrap_q_near(q_down, q_above)
    q_down = motion.unwrap_q_near(q_down, home)
    return q_above, q_down


def fmt_list(vals):
    return '[' + ', '.join('%.6f' % float(v) for v in vals) + ']'


def prefer_near_refs(q, *refs):
    """For each joint pick ±2π image closest to any reference (HOME / SEED_DOWN)."""
    out = []
    for i, t in enumerate(q):
        t = float(t)
        best = t
        best_c = 1e9
        for ref in refs:
            r = float(ref[i])
            for k in (-2, -1, 0, 1, 2):
                cand = t + k * 2.0 * math.pi
                c = abs(cand - r)
                if c < best_c:
                    best_c = c
                    best = cand
        out.append(best)
    return out


def main():
    out_src = os.path.join(ROOT, 'src', 'ur3_needle_sim', 'config', 'hole_poses.yaml')
    out_ctrl = os.path.join(CTRL, 'hole_poses.yaml')
    ref_q = list(poses.HOME)
    home = list(poses.HOME)
    seed = list(poses.SEED_DOWN)

    print(
        'Bake with click IK: BASE_X=%g BASE_Y=%g INSERT_DEPTH=%g FLANGE_TO_TIP=%g axis=%s'
        % (
            motion.BASE_X_SIGN,
            motion.BASE_Y_SIGN,
            motion.INSERT_DEPTH,
            motion.FLANGE_TO_NEEDLE_TIP,
            motion.GRIPPER_DOWN_AXIS,
        )
    )

    rows = []
    ok = 0
    for hid, mouth in enumerate(holes.FALLBACK_MOUTH):
        q_above, q_down = solve_above_down(mouth, ref_q=ref_q)
        if q_above is None or q_down is None:
            print('FAIL hole', hid, mouth)
            continue
        q_above = prefer_near_refs(q_above, home, seed, ref_q)
        q_down = prefer_near_refs(q_down, q_above, home, seed)

        mouth_base = motion.world_point_to_base(mouth)
        p_above_b, p_down_b = motion.make_pick_targets_from_mouth_base(mouth_base)
        z_path = motion.ik_straight_z_path(
            p_above_b, p_down_b, q_above, n_steps=16,
        )
        if not z_path or len(z_path) < 2:
            print('FAIL hole', hid, 'z_path')
            continue
        z_path = [prefer_near_refs(q, q_above, home, seed) for q in z_path]
        q_down = z_path[-1]

        ok += 1
        ref_q = q_above
        print(
            'OK hole', hid, 'mouth', mouth,
            'pan_above', round(q_above[0], 3),
            'z_pts', len(z_path),
        )
        rows.append((hid, mouth, q_above, q_down, z_path))

    if ok < 9:
        print('ERROR: only %d/9 holes solved' % ok)
        sys.exit(1)

    lines = [
        '# Baked from click IK (ur3_click_insert/motion.py) — keep ROS motion = click.',
        '# BASE_X_SIGN=%g BASE_Y_SIGN=%g INSERT_DEPTH=%g FLANGE_TO_NEEDLE_TIP=%g'
        % (
            motion.BASE_X_SIGN,
            motion.BASE_Y_SIGN,
            motion.INSERT_DEPTH,
            motion.FLANGE_TO_NEEDLE_TIP,
        ),
        '# z_path = straight-Z insert waypoints (fixed XY)',
        'home: %s' % fmt_list(poses.HOME),
        'seed_down: %s' % fmt_list(poses.SEED_DOWN),
        'finger_closed: %.2f' % float(poses.FINGER_CLOSED),
        'holes:',
    ]
    for hid, mouth, q_above, q_down, z_path in rows:
        lines.append('  - id: %d' % hid)
        lines.append('    xyz: [%.3f, %.3f, %.3f]' % (mouth[0], mouth[1], mouth[2]))
        lines.append('    approach: %s' % fmt_list(q_above))
        lines.append('    insert: %s' % fmt_list(q_down))
        lines.append('    z_path:')
        for q in z_path:
            lines.append('      - %s' % fmt_list(q))
    text = '\n'.join(lines) + '\n'

    for path in (out_src, out_ctrl):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        print('Wrote', path)

    print('Done — rebuild package / restart run_ros_demo.sh')


if __name__ == '__main__':
    main()
