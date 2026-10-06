# poses.py — UR3e needle (rad)
# [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
# Không bắt buộc khớp ure PICK_*; chỉ seed / HOME.

HOME = [0.000, -1.570, 1.570, -1.570, -1.570, 0.000]
# Tip-down warm-start cho UR3e (đã dùng ổn trước đây)
SEED_DOWN = [0.3246, -0.7505, 1.8532, -2.0386, -3.1544, 2.1924]
# Alias tên cũ — trỏ seed UR3e (không còn góc ure)
PICK_ABOVE = list(SEED_DOWN)
PICK_DOWN = [0.3246, -0.95, 1.95, -2.20, -3.1544, 2.1924]
FINGER_CLOSED = 0.75
