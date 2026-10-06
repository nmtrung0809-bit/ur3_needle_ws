# ur3_needle_sim — UR3e sâu kim (Webots + ROS 2 Jazzy)

Robot **UR3e** + gripper **Robotiq 2F-85** + kim ~18 cm + phantom hồng **9 lỗ**.

**Mục tiêu demo:** click chuột vào lỗ bất kỳ trong Webots → arm chạy  
`Home → Approach → Insert → Hold → Retract → Home` đúng lỗ đó.

---

## 1) Demo click trong Webots (cách chính)

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
webots worlds/ur3_needle.wbt
# hoặc: ./run_click_demo.sh
```

1. Bấm **Play**
2. **Click trái nhanh** gần một chấm đen (lỗ) trên khối hồng
3. Arm chạy Home → Approach → Insert → Hold → Retract → Home **đúng lỗ đó**
4. Khi `DONE` → click lỗ khác

Pose 9 lỗ đã **bake sẵn** trong `controllers/ur3_click_insert/hole_poses.yaml`  
(sai số tip bake ~**2–6 mm**). Click dùng cache → phản hồi nhanh.

Controller: `ur3_click_insert` (Mouse 3D + pose bake / live IK fallback).

| Lỗi thường gặp | Cách xử lý |
|----------------|------------|
| Click không nhận / `NaN` | Phải **Play**; click **trên mặt phantom**; click nhanh (không kéo xoay camera) |
| `too far from holes` | Click gần hơn chấm đen (ngưỡng 3 cm) |
| Tip lệch sau khi sửa world | Bake lại: đổi `.wbt` → `controller "ur3_bake_poses"`, Play, đợi `BAKE DONE` |

### Tune pose bằng bàn phím

Trong `.wbt` đổi tạm:

```text
controller "ur3_needle_native"
```

Phím: `1–6` chọn khớp, `Z/X` chỉnh, `H/A/I/R` home/approach/insert/retract, `P` in pose.  
Copy số mới vào `controllers/ur3_click_insert/ur3_click_insert.py` và `config/needle_params.yaml`.

---

## 2) Build ROS 2

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
source /opt/ros/jazzy/setup.bash
export WEBOTS_HOME=/usr/local/webots
colcon build --packages-select ur3_needle_sim
source install/setup.bash
```

---

## 3) Demo ROS (tuỳ chọn)

World ROS dùng `controller "<extern>"` (file trong package `worlds/ur3_needle.wbt`).

```bash
ros2 launch ur3_needle_sim demo.launch.py
```

Terminal khác:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/share/ros2-workspace/trung_test/ur3_needle_ws/install/setup.bash

# Chọn lỗ 0..8 rồi start (ví dụ lỗ góc id=8)
ros2 topic pub --once /needle/hole_id std_msgs/msg/Int32 "{data: 8}"
ros2 topic pub --once /needle/start std_msgs/msg/Bool "{data: true}"

ros2 topic echo /needle/phase
ros2 topic echo /needle/depth

# Dừng khẩn
ros2 topic pub --once /needle/abort std_msgs/msg/Bool "{data: true}"
```

Đổi độ sâu:

```bash
ros2 launch ur3_needle_sim demo.launch.py depth_m:=0.03
```

### Topics

| Topic | Kiểu | Nghĩa |
|-------|------|--------|
| `/needle/hole_id` | `Int32` | Lỗ 0..8 |
| `/needle/start` | `Bool` | Bắt đầu chu trình |
| `/needle/abort` | `Bool` | Hủy |
| `/needle/phase` | `String` | idle / move_home / … |
| `/needle/depth` | `Float32` | Độ sâu ước lượng (m) |

---

## 4) Bảng 9 lỗ (world)

Phantom tâm `(0.36, 0, 0.50)`, mặt trên `z≈0.55`.

| id | x | y |
|----|---|---|
| 0 | 0.26 | -0.10 |
| 1 | 0.36 | -0.10 |
| 2 | 0.46 | -0.10 |
| 3 | 0.26 | 0.00 |
| 4 | 0.36 | 0.00 | *(lỗ giữa — pose gốc)* |
| 5 | 0.46 | 0.00 |
| 6 | 0.26 | 0.10 |
| 7 | 0.36 | 0.10 |
| 8 | 0.46 | 0.10 |

Bước lưới **0.10 m** giữa tâm (local ±0.10 trên phantom); lỗ xa nhất ~**0.47 m** ngang — trong tầm UR3e.

File: `config/holes.yaml`.

Pose lỗ khác = pose lỗ giữa + `shoulder_pan = atan2(y, x)`.

---

## 5) Cấu trúc chính

```text
ur3_needle_ws/
  worlds/ur3_needle.wbt              # click demo
  controllers/ur3_click_insert/      # Mouse + FSM
  controllers/ur3_needle_native/     # bàn phím tune
  controllers/ur3_grasp_home/        # chỉ về home + khép ngón
  src/ur3_needle_sim/
    launch/sim.launch.py
    launch/demo.launch.py
    config/holes.yaml
    config/needle_params.yaml
    ur3_needle_sim/needle_insert_node.py
    ur3_needle_sim/depth_monitor.py
    resource/ur3e_needle.urdf
```

Hướng dẫn từng dòng 5 ngày: `../../KE_HOACH_UR3_SAU_KIM_5_NGAY.md`.
