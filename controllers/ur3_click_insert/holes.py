"""
Tâm đường tròn 9 lỗ trụ (song song Z) trên phantom.

Mỗi DEF HOLE_i Pose gốc = tâm hình học cylinder.
Tâm đường tròn miệng lỗ (mặt trên) = center + (0,0, height/2) vì trục = Z.
"""
import math

CYL_HEIGHT = 0.08
CYL_RADIUS = 0.012
# Fallback nếu chưa đo được Supervisor (khớp layout .wbt)
FALLBACK_MOUTH = [
    (0.26, -0.10, 0.55),
    (0.36, -0.10, 0.55),
    (0.46, -0.10, 0.55),
    (0.26, 0.00, 0.55),
    (0.36, 0.00, 0.55),
    (0.46, 0.00, 0.55),
    (0.26, 0.10, 0.55),
    (0.36, 0.10, 0.55),
    (0.46, 0.10, 0.55),
]


def measure_hole_centers(supervisor):
    """
    Đo từ Webots: trả list dict
      id, center (xyz tâm cylinder), mouth (xyz tâm đường tròn miệng),
      axis (0,0,1), radius, height
    """
    holes = []
    for i in range(9):
        node = supervisor.getFromDef('HOLE_%d' % i)
        if node is None:
            print('[holes] missing DEF HOLE_%d — dùng fallback' % i)
            mx, my, mz = FALLBACK_MOUTH[i]
            holes.append({
                'id': i,
                'center': (mx, my, mz - CYL_HEIGHT * 0.5),
                'mouth': (mx, my, mz),
                'axis': (0.0, 0.0, 1.0),
                'radius': CYL_RADIUS,
                'height': CYL_HEIGHT,
            })
            continue
        c = node.getPosition()  # tâm hình học cylinder (Pose origin)
        # Trục Z world: miệng = tâm + height/2 theo +Z
        mouth = (float(c[0]), float(c[1]), float(c[2]) + CYL_HEIGHT * 0.5)
        center = (float(c[0]), float(c[1]), float(c[2]))
        holes.append({
            'id': i,
            'center': center,
            'mouth': mouth,
            'axis': (0.0, 0.0, 1.0),
            'radius': CYL_RADIUS,
            'height': CYL_HEIGHT,
        })
        expected = FALLBACK_MOUTH[i]
        err_xy = math.hypot(mouth[0] - expected[0], mouth[1] - expected[1])
        print(
            '[holes] id', i,
            'mouth', [round(v, 4) for v in mouth],
            'expected', [round(v, 4) for v in expected],
            'xy_err_mm', round(err_xy * 1000, 1),
        )
    return holes


def nearest_hole_id(holes, x, y, max_dist=0.03):
    best_i, best_d = None, 1e9
    for h in holes:
        mx, my, _mz = h['mouth']
        d = math.hypot(x - mx, y - my)
        if d < best_d:
            best_i, best_d = h['id'], d
    if best_d <= max_dist:
        return best_i, best_d
    return None, best_d


def mouth_xyz(holes, hole_id):
    return holes[hole_id]['mouth']
