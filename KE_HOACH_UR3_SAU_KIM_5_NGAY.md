# Tự học 5 ngày — Giải thích TỪNG DÒNG (UR3e sâu kim, ROS 2 + Webots)

> Bạn **tự gõ** lệnh và code.  
> Dưới mỗi khối lệnh/code có bảng **giải thích từng dòng**.  
> Làm xong checklist ngày đó rồi mới sang ngày sau.  
> Package dựng sẵn đã xóa — bạn tạo từ đầu.

> **Cập nhật mới nhất (đồng bộ code):**  
> - **Click:** `ur3_click_insert` = clone_backup IK TEST (flange DH, không camera / tip IK) + **straight-Z** insert. World: `worlds/ur3_needle.wbt`.  
> - **ROS 2:** oldest package `ur3_needle.wbt` (`<extern>`) + baked `z_path`. Xem mục **「Giải thích code dự án hiện tại」** và **「ROS 2 — cách chạy…」**.  
> - **Repo:** https://github.com/nmtrung0809-bit/ur3_needle_ws (branch `main`). File này nằm ở root repo + bản copy tại `trung_test/KE_HOACH_UR3_SAU_KIM_5_NGAY.md`.

---

## Workflow / How-to — làm dự án hiện tại (bắt đầu từ đây)

Root workspace:

```text
/workspace/share/ros2-workspace/trung_test/ur3_needle_ws
```

GitHub: `https://github.com/nmtrung0809-bit/ur3_needle_ws` · branch `main`

Có **hai chế độ** — **không** mở cùng lúc một instance Webots:

| Chế độ | World | Controller | Cách chạy |
|--------|-------|------------|-----------|
| **Click** (chính) | `ur3_needle_ws/worlds/ur3_needle.wbt` | `ur3_click_insert` | `./run_click_demo.sh` hoặc `webots worlds/ur3_needle.wbt` |
| **ROS 2** | package `src/.../worlds/ur3_needle.wbt` → install share | `<extern>` | `./run_ros_demo.sh` rồi pub `/needle/*` |

### 0) Clone / mở sẵn project

```bash
cd /workspace/share/ros2-workspace/trung_test
# nếu chưa có:
# git clone https://github.com/nmtrung0809-bit/ur3_needle_ws.git
cd ur3_needle_ws
```

Yêu cầu máy: ROS 2 **Jazzy**, Webots (`WEBOTS_HOME=/usr/local/webots`), Python hệ thống **3.12** (tránh conda 3.13 trước `ros2`).

### 1) Demo Click (Webots + chuột)

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
./run_click_demo.sh
# hoặc: webots worlds/ur3_needle.wbt
```

1. Bấm **Play** trong Webots.  
2. **Click trái nhanh** gần chấm đen (miệng lỗ) trên phantom hồng.  
3. Arm: `HOME → ABOVE → DOWN (thẳng Z) → Retract → HOME`.  
4. Khi xong → click lỗ khác.

**Pipeline code:** Mouse 3D → nearest `DEF HOLE_i` → `world_point_to_base` → `make_pick_targets_from_mouth_base` → `inverse_kinematics_downward` (seed `SEED_DOWN` + pan) → `goto_straight_z`.

| Lỗi | Xử lý |
|-----|--------|
| Click không nhận / NaN | Phải **Play**; click trên mặt phantom; click nhanh (không kéo xoay view) |
| `too far from holes` | Gần chấm đen hơn (ngưỡng ~3 cm) |
| Tip lệch / phantom lệch | Phantom phải `(0.36, 0, 0.5)` — Webots hay drift ~0.42; reload world, đừng save bản drift |
| IK fail | Kiểm `INSERT_DEPTH`, `FLANGE_TO_NEEDLE_TIP`, `BASE_*_SIGN` trong `motion.py` |

### 2) Build ROS 2 package

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
export PATH="/usr/bin:/bin:${PATH}"
hash -r
source /opt/ros/jazzy/setup.bash
export WEBOTS_HOME=/usr/local/webots
# nếu package lỗi lạ: rm -rf build/ur3_needle_sim install/ur3_needle_sim
colcon build --packages-select ur3_needle_sim
source install/setup.bash
```

**Không** dùng `colcon build --symlink-install` với package này (setuptools editable lỗi trên host này).

### 3) Demo ROS 2 (2 terminal)

**Đóng** Webots click trước. World ROS = bản `<extern>` trong package/install — **không** gắn `ur3_click_insert`.

**Terminal 1** (giữ mở):

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
./run_ros_demo.sh
```

Đợi controllers active / Webots mở.

**Terminal 2:**

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/share/ros2-workspace/trung_test/ur3_needle_ws/install/setup.bash

ros2 topic info /needle/start
# Subscription count: 1

ros2 control list_controllers -c /ur3/controller_manager
ros2 topic hz /ur3/joint_states

# chọn lỗ rồi start (ví dụ lỗ 8)
ros2 topic pub --once /needle/hole_id std_msgs/msg/Int32 "{data: 8}"
ros2 topic pub --once /needle/start std_msgs/msg/Bool "{data: true}"
ros2 topic echo /needle/phase
# idle → move_home → move_approach → insert → hold → retract → move_home → done

ros2 topic pub --once /needle/abort std_msgs/msg/Bool "{data: true}"   # hủy
```

Chi tiết topic / action: mục **「ROS 2 — cách chạy, đổi lỗ, điều khiển」** bên dưới.

### 4) Đổi độ sâu insert / bake lại ROS cho khớp click

1. Sửa `INSERT_DEPTH` (và nếu cần `FLANGE_TO_NEEDLE_TIP`, `BASE_X_SIGN`, `BASE_Y_SIGN`) trong:

```text
controllers/ur3_click_insert/motion.py
```

2. Mirror controller sang package:

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
cp controllers/ur3_click_insert/motion.py \
   src/ur3_needle_sim/controllers/ur3_click_insert/motion.py
cp controllers/ur3_click_insert/poses.py \
   src/ur3_needle_sim/controllers/ur3_click_insert/poses.py
cp controllers/ur3_click_insert/holes.py \
   src/ur3_needle_sim/controllers/ur3_click_insert/holes.py
cp controllers/ur3_click_insert/ur3_click_insert.py \
   src/ur3_needle_sim/controllers/ur3_click_insert/ur3_click_insert.py
```

3. Bake `hole_poses.yaml` (+ `z_path` thẳng Z):

```bash
/usr/bin/python3 scripts/bake_hole_poses_from_click_ik.py
# output: src/ur3_needle_sim/config/hole_poses.yaml
#         + controllers/ur3_click_insert/hole_poses.yaml
```

4. Build lại + chạy ROS:

```bash
colcon build --packages-select ur3_needle_sim
./run_ros_demo.sh
```

Click: reload `worlds/ur3_needle.wbt` sau khi sửa `controllers/`.

### 5) Đồng bộ world / phantom

| Bản | Path | Controller |
|-----|------|------------|
| Click runtime | `worlds/ur3_needle.wbt` | `ur3_click_insert` |
| Click mirror | `src/ur3_needle_sim/worlds/ur3_needle_click.wbt` | `ur3_click_insert` |
| ROS | `src/ur3_needle_sim/worlds/ur3_needle.wbt` (+ `ur3_needle_ros.wbt`) | `<extern>` |

Phantom Solid translation **luôn** `(0.36, 0, 0.5)` trên mọi `.wbt`. Lưới miệng: x∈{0.26,0.36,0.46}, y∈{−0.10,0,0.10}, z≈0.55.

### 6) Cấu trúc thư mục cần nhớ

```text
ur3_needle_ws/
  KE_HOACH_UR3_SAU_KIM_5_NGAY.md   # file này
  README.md
  run_click_demo.sh
  run_ros_demo.sh
  worlds/ur3_needle.wbt            # click
  controllers/ur3_click_insert/    # motion, poses, holes, FSM click
  scripts/bake_hole_poses_from_click_ik.py
  src/ur3_needle_sim/              # ROS package
    config/{holes,hole_poses,needle_params}.yaml
    launch/{sim,demo}.launch.py
    ur3_needle_sim/{needle_insert_node,depth_monitor,ur3_ik}.py
    worlds/{ur3_needle,ur3_needle_click,ur3_needle_ros}.wbt
  build/  install/  log/           # colcon artifacts (đã có trên GitHub; events.log/logger_all.log gitignore)
```

### 7) Push lên GitHub (`ur3_needle_ws`)

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
git status
git add -A
# KHÔNG add log/**/events.log hay logger_all.log (env đầy đủ → GitHub secret scanning chặn)
git commit -m "Update KE_HOACH workflow and how-to"
git push origin main
```

`.gitignore` đã loại `log/**/events.log` và `log/**/logger_all.log`. Remote sạch: `https://github.com/nmtrung0809-bit/ur3_needle_ws.git`.

Bản copy ngoài repo (cùng nội dung how-to):

```text
/workspace/share/ros2-workspace/trung_test/KE_HOACH_UR3_SAU_KIM_5_NGAY.md
```

Giữ hai file này **đồng bộ** khi sửa workflow.

### 8) Quy trình làm việc hàng ngày (tóm tắt)

```text
1. Sửa code click  →  controllers/ur3_click_insert/*
2. Mirror          →  src/ur3_needle_sim/controllers/...
3. Test click      →  ./run_click_demo.sh  (Play → click lỗ)
4. Nếu đổi IK/depth →  bake script → colcon build
5. Test ROS        →  đóng click Webots → ./run_ros_demo.sh → pub hole_id + start
6. Commit + push   →  git add/commit/push (tránh events.log)
```

Hằng số đang dùng: `INSERT_DEPTH=+0.02`, `ABOVE_CLEARANCE=0.16`, `FLANGE_TO_NEEDLE_TIP≈0.20`, `BASE_X_SIGN=BASE_Y_SIGN=-1`, `HOME=[0,-π/2,π/2,-π/2,-π/2,0]`.

Đọc sâu code: mục **「Giải thích code dự án hiện tại」**. Tutorial gõ từ đầu: **Ngày 1–5** phía dưới (một số đoạn cũ hơn code hiện tại).

---

## Bạn sẽ làm ra gì?

- Robot **UR3e** trong **Webots**
- Có **kim** + khối mô giả (**phantom**) với **9 lỗ** (giếng song song trục Z)
- **Click chuột** vào **một lỗ bất kỳ** → Supervisor đo **miệng lỗ** → IK flange downward → arm ghim kim **−Z** đúng lỗ
- Điều khiển chính bằng **Webots Supervisor + Mouse**; ROS 2 là nhánh riêng (`controller "<extern>"`)
- Chu trình mỗi lần click (clone_backup IK TEST):

```text
HOME → ABOVE (flange, tip trên miệng) → DOWN (flange, tip tại mouth_z − INSERT_DEPTH) → Retract ABOVE → HOME
```

`INSERT_DEPTH` hiện tại = **+0.02 m** → tip **xuống dưới miệng lỗ 2 cm** (`PICK_CLEARANCE = −0.02`). Đổi trong `motion.py`, rồi re-bake ROS poses.

---

## Mục tiêu cập nhật — Click lỗ rồi sâu kim

### Ý tưởng (1 câu)

Click trái gần chấm đen → `DEF HOLE_i` mouth → `world_point_to_base` → `make_pick_targets_from_mouth_base` → `inverse_kinematics_downward` (seed UR3e `SEED_DOWN` + pan) → `goto_pose` ABOVE/DOWN → HOME.

### Việc cần làm (checklist tổng — trạng thái hiện tại)

| # | Việc | Ngày gợi ý | Trạng thái |
|---|------|------------|------------|
| 1 | World UR3e + gripper + kim + phantom 9 lỗ | Ngày 1 | **Xong** `worlds/ur3_needle.wbt` |
| 2 | Bảng / đo tọa độ 9 lỗ (mouth z≈0.55) | Ngày 1–3 | **Xong** `holes.yaml` + `DEF HOLE_0..8` + `holes.py` |
| 3 | HOME + SEED_DOWN (tip hướng phantom) | Ngày 3 | **Xong** `poses.py` / warm-up calib TCP→tip |
| 4 | Approach / insert lỗ giữa | Ngày 3–4 | **Xong** clone_backup flange IK |
| 5 | Pose cho 8 lỗ còn lại | Ngày 3–4 | **Xong** — mỗi lỗ IK riêng; pan seed từ `mouth_base` |
| 6 | Controller Mouse + chọn lỗ gần nhất | Ngày 4 | **Xong** `ur3_click_insert` |
| 7 | Chu trình click → full cycle đúng `hole_id` | Ngày 4 | **Xong** HOME→ABOVE→DOWN→HOME (bỏ SEED waypoint để tránh xoay 1 vòng) |
| 8 | (Tuỳ chọn) ROS `/needle/hole_id` + `/needle/start` | Ngày 4–5 | **Xong + verified** — `./run_ros_demo.sh` trên oldest `ur3_needle.wbt` (`<extern>`); insert hole 4 → `done` |
| 9 | Demo + README + tóm tắt | Ngày 5 | **Xong** README + doc này |

### Cách chọn lỗ khi click (MVP Webots)

1. Bật chuột 3D: `Mouse.enable` + `Mouse.enable3dPosition()`.
2. Click trái → điểm `(x, y, z)` trên vật thể dưới con trỏ.
3. So `hypot(x - mx, y - my)` với **9 tâm miệng lỗ** (`holes.measure_hole_centers`).
4. Lỗ gần nhất < **0.03 m** → chọn `hole_id`; không thì bỏ qua.
5. Chỉ nhận click khi không đang bận cycle (tránh click giữa chừng).

> Lỗ có `DEF HOLE_i`. Mouth = Pose origin + `(0,0,height/2)`. Click gần chấm đen là đủ.  
> Phantom translation **phải** `(0.36, 0, 0.5)` — Webots save hay drift ~0.42 → lỗ lệch tầm với / fallback grid.

### Cách ra lệnh khớp cho từng lỗ (đang dùng — clone_backup IK TEST)

**Không** dùng tip-Jacobian / đo tip làm target IK. **Không** camera Recognition.

**Pipeline hiện tại** (`ure` / `clone_backup` IK TEST, Supervisor mouth thay cube camera):

1. Đo `mouth_world` từ `DEF HOLE_i` (Supervisor).
2. `mouth_base = world_point_to_base(mouth_world)` với  
   `BASE_X_SIGN = BASE_Y_SIGN = -1` (Rz π — DH base đối X/Y so với world).
3. `make_pick_targets_from_mouth_base` (cùng XY miệng; Z flange = mouth_z + `FLANGE_TO_NEEDLE_TIP` + `PICK_CLEARANCE`; ABOVE = down + `ABOVE_CLEARANCE`).
4. `inverse_kinematics_downward` — DH flange pose IK; seed = UR3e `SEED_DOWN` + pan `atan2(mouth_base_y, mouth_base_x)`; chọn cand `minimize_q_jump` gần pose hiện tại.
5. `goto_pose(q_above)` → `goto_pose(q_down)` → retract ABOVE → HOME.  
   Joint unwrap bằng `atan2` (đường ngắn ≤±180° mỗi khớp).

| Cách | Trạng thái |
|------|------------|
| A. Tune 9 cặp pose tay | Tuỳ chọn bake |
| B. Pose giữa + chỉ đổi pan `atan2` | **Bỏ** |
| C. Tip-Jacobian / tip đo Webots làm target IK | **Bỏ** (user: không tip IK) |
| **E. clone_backup IK TEST + Supervisor HOLE + flange DH** | **Đang dùng** |

### Tham số motion quan trọng (`controllers/ur3_click_insert/motion.py`)

| Tham số | Giá trị hiện tại | Ý nghĩa |
|---------|------------------|---------|
| `INSERT_DEPTH` | `+0.02` | Tip dưới miệng 2 cm (`PICK_CLEARANCE = −0.02`) |
| `ABOVE_CLEARANCE` | `0.16` | ABOVE cao hơn DOWN 16 cm |
| `FLANGE_TO_NEEDLE_TIP` | `0.20` (+ calib) | Offset flange→tip |
| `BASE_X_SIGN` / `BASE_Y_SIGN` | `-1` / `-1` | World↔DH = Rz(π) |
| Straight-Z insert | `ik_straight_z_path` / `z_path` | XY cố định, chỉ đổi Z (17 waypoints) |
| `SEED_DOWN` | UR3e tip-down | Warm-start / seed IK |
| Motor velocity | `1.5` rad/s | `basicTimeStep=8` ms |

Mirror controller sau mỗi sửa:  
`controllers/ur3_click_insert/` → `src/ur3_needle_sim/controllers/ur3_click_insert/`.

### File / thành phần chính

| Thành phần | Đường dẫn | Việc |
|------------|-----------|------|
| World **click** | `ur3_needle_ws/worlds/ur3_needle.wbt` | `controller "ur3_click_insert"` — **đang demo** |
| World **ROS** | `src/.../worlds/ur3_needle.wbt` (+ `ur3_needle_ros.wbt`) | `controller "<extern>"` — dùng với `ros2 launch` |
| Click | `controllers/ur3_click_insert/` | Mouse + IK TEST + `goto_pose` |
| Motion | `.../motion.py` | DH flange IK, base signs, unwrap |
| Poses | `.../poses.py` | `HOME`, `SEED_DOWN` |
| Tâm lỗ | `.../holes.py` | `measure_hole_centers` |
| ROS package | `src/ur3_needle_sim/` | `sim.launch.py`, `demo.launch.py`, nodes |
| Script ROS | `run_ros_demo.sh` | build + `demo.launch.py` |
| Tóm tắt | `TOM_TAT_DU_AN_UR3_SAU_KIM.md` | Bổ sung doc này |

### ROS 2 — cách chạy, đổi lỗ, điều khiển (verified)

**World ROS (bắt buộc):**  
`src/ur3_needle_sim/worlds/ur3_needle.wbt` → install `share/.../worlds/ur3_needle.wbt`  
- `controller "<extern>"`, phantom `(0.36, 0, 0.5)` (oldest package world)  
- **Không** dùng `ur3_needle_ws/worlds/ur3_needle.wbt` (click / `ur3_click_insert`) cùng lúc với ROS  

**Motion ROS = click IK bake:** `hole_poses.yaml` có `approach`, `insert`, và **`z_path`** (17 điểm thẳng trục Z).  
Insert/retract ROS gửi multi-point trajectory (XY cố định, chỉ Z đổi).

---

#### A) Chạy ROS demo (2 Terminal)

**Terminal 1 — giữ mở (Webots + nodes):**

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
./run_ros_demo.sh
```

Script tự: `PATH` → python3.12, `USER=abc`, xóa `/tmp/webots` IPC, `colcon build`, launch demo.  
Đợi Webots mở và log dạng `Configured and activated ... ur_joint_trajectory_controller`.

**Terminal 2 — điều khiển:**

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/share/ros2-workspace/trung_test/ur3_needle_ws/install/setup.bash
```

**Kiểm tra sẵn sàng (bắt buộc trước khi pub):**

```bash
ros2 topic info /needle/start
# Subscription count: 1   ← nếu = 0 sẽ mãi "Waiting for ... subscription(s)"

ros2 control list_controllers -c /ur3/controller_manager
# ur_joint_state_broadcaster      ... active
# ur_joint_trajectory_controller ... active

ros2 topic hz /ur3/joint_states
```

Nếu `Subscription count: 0` → Terminal 1 chưa chạy / demo đã tắt → chạy lại `./run_ros_demo.sh`.

---

#### B) Đổi lỗ (hole_id 0..8)

| id | world (x, y) | ghi chú |
|----|--------------|---------|
| 0 | 0.26, −0.10 | |
| 1 | 0.36, −0.10 | |
| 2 | 0.46, −0.10 | |
| 3 | 0.26, 0.00 | |
| 4 | 0.36, 0.00 | lỗ giữa (mặc định) |
| 5 | 0.46, 0.00 | |
| 6 | 0.26, 0.10 | |
| 7 | 0.36, 0.10 | |
| 8 | 0.46, 0.10 | |

```bash
# ví dụ chọn lỗ 8
ros2 topic pub --once /needle/hole_id std_msgs/msg/Int32 "{data: 8}"
```

Chỉ đổi `data:` thành `0`…`8`. Phải pub `hole_id` **trước** `start`.

---

#### C) Điều khiển chu trình insert

```bash
# bắt đầu: HOME → ABOVE → DOWN (straight Z) → hold → retract Z → HOME
ros2 topic pub --once /needle/start std_msgs/msg/Bool "{data: true}"

# xem trạng thái
ros2 topic echo /needle/phase
# idle → move_home → move_approach → insert → hold → retract → move_home → done

# hủy giữa chừng
ros2 topic pub --once /needle/abort std_msgs/msg/Bool "{data: true}"

# độ sâu ước lượng (monitor)
ros2 topic echo /needle/depth
```

Log insert đúng sẽ có dạng: `trajectory 17 pts over …s` (đường thẳng Z).

**Topic cheat sheet**

| Topic | Type | Việc |
|-------|------|------|
| `/needle/hole_id` | `Int32` | Chọn lỗ 0..8 |
| `/needle/start` | `Bool` | Bắt đầu chu trình |
| `/needle/abort` | `Bool` | Hủy |
| `/needle/phase` | `String` | Phase hiện tại |
| `/needle/depth` | `Float32` | Depth ước lượng |
| `/ur3/joint_states` | `JointState` | Góc khớp live |
| Action `/ur3/ur_joint_trajectory_controller/follow_joint_trajectory` | | Node gửi joint path |

**Điều khiển khớp trực tiếp (nâng cao — HOME):**

```bash
ros2 action send_goal /ur3/ur_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                    wrist_1_joint, wrist_2_joint, wrist_3_joint],
      points: [{
        positions: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        time_from_start: {sec: 3, nanosec: 0}
      }]
    }
  }"
```

---

#### D) Đổi depth / khớp lại ROS với click

1. Sửa `INSERT_DEPTH` trong `controllers/ur3_click_insert/motion.py`  
2. Re-bake (cùng IK click + `z_path`):

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
/usr/bin/python3 scripts/bake_hole_poses_from_click_ik.py
./run_ros_demo.sh
```

---

#### E) Kỹ thuật launch (đã fix)

| Hạng mục | Kết quả |
|----------|---------|
| ROS 2 Jazzy + package/deps | OK |
| World oldest `ur3_needle.wbt` (`<extern>`) | OK |
| Controllers active + `/ur3/joint_states` ~40 Hz | OK |
| Insert hole → `done` + straight-Z `z_path` | OK |

**`sim.launch.py`:** absolute world path; `mode='realtime'`; `ros2_supervisor=True`; một spawner + `--activate-as-group` + `--switch-timeout 90`.

**Lỗi đã fix:** conda python3.13 vs rclpy 3.12; `hole_poses` rỗng; `PathJoinSubstitution` bỏ `Ros2Supervisor`; hai spawner race lock; `USER`/IPC stale; `set -u` + `AMENT_TRACE_SETUP_FILES` trong `run_ros_demo.sh`.

### World xy của 9 lỗ (đã có trong `.wbt`)

Phantom tâm `(0.36, 0, 0.50)` — mặt trên `z ≈ 0.55`. **Giữ phantom tại 0.36** (tránh drift).

```text
 id | local (x,y)     | world (x,y)      | ghi chú
 0  | (-0.10, -0.10)  | (0.26, -0.10)    |
 1  | ( 0.00, -0.10)  | (0.36, -0.10)    |
 2  | ( 0.10, -0.10)  | (0.46, -0.10)    |
 3  | (-0.10,  0.00)  | (0.26,  0.00)    |
 4  | ( 0.00,  0.00)  | (0.36,  0.00)    | lỗ giữa
 5  | ( 0.10,  0.00)  | (0.46,  0.00)    |
 6  | (-0.10,  0.10)  | (0.26,  0.10)    |
 7  | ( 0.00,  0.10)  | (0.36,  0.10)    |
 8  | ( 0.10,  0.10)  | (0.46,  0.10)    |
```

Bước lưới **0.10 m**; lỗ xa nhất ngang ~**0.47 m** (< tầm UR3e ~0.50 m).

---

## Giải thích code dự án hiện tại (`ur3_needle_ws`)

> Đọc mục này để hiểu **code đang chạy hôm nay**. Phần Ngày 1–5 phía dưới vẫn là tutorial học từng bước (có chỗ cũ hơn).

### 1) Cây thư mục (file quan trọng)

```text
ur3_needle_ws/
├── worlds/ur3_needle.wbt              # CLICK demo (controller ur3_click_insert)
├── run_ros_demo.sh                    # ROS: PATH python3.12 + build + launch
├── run_click_demo.sh                  # (nếu có) mở click world
├── scripts/bake_hole_poses_from_click_ik.py
│                                      # Offline IK → hole_poses.yaml (+ z_path)
├── controllers/ur3_click_insert/      # Webots Python controller (CLICK)
│   ├── ur3_click_insert.py            # Mouse + chu trình insert
│   ├── motion.py                      # DH FK/IK, base Rz(π), straight-Z
│   ├── poses.py                       # HOME, SEED_DOWN
│   ├── holes.py                       # Đo DEF HOLE_0..8 / fallback grid
│   └── hole_poses.yaml                # Mirror bake (không dùng trực tiếp lúc click)
└── src/ur3_needle_sim/                # ROS 2 package
    ├── worlds/ur3_needle.wbt          # ROS world (<extern>) — oldest
    ├── worlds/ur3_needle_click.wbt    # Bản click trong package
    ├── worlds/ur3_needle_ros.wbt      # Bản ROS phụ
    ├── launch/sim.launch.py           # Webots + driver + spawner
    ├── launch/demo.launch.py          # sim + needle_insert_node + depth_monitor
    ├── config/hole_poses.yaml         # Baked approach/insert/z_path cho ROS
    ├── config/needle_params.yaml      # hold_sec, trajectory times, depth_m
    ├── resource/ur3e_needle.urdf      # robot_description + ros2_control
    ├── resource/ros2_control_config.yaml
    ├── controllers/ur3_click_insert/  # Mirror của controllers/ (giữ đồng bộ)
    └── ur3_needle_sim/
        ├── needle_insert_node.py      # State machine ROS
        ├── depth_monitor.py           # Pub /needle/depth
        └── hole_poses_loader.py       # Parse hole_poses.yaml
```

**Hai world, hai cách chạy**

| | Click | ROS 2 |
|--|-------|-------|
| World | `worlds/ur3_needle.wbt` | package `worlds/ur3_needle.wbt` |
| Controller | `ur3_click_insert` | `<extern>` + `webots_ros2_driver` |
| Target lỗ | Mouse + Supervisor live IK | Topic + baked joints/`z_path` |
| Lệnh | `webots worlds/ur3_needle.wbt` | `./run_ros_demo.sh` |

Sửa controller click → **copy** sang `src/ur3_needle_sim/controllers/ur3_click_insert/` (mirror).  
Sửa `INSERT_DEPTH` / IK → chạy lại `bake_hole_poses_from_click_ik.py` rồi restart ROS.

---

### 2) Click — luồng code (từng bước)

```text
Mouse click 3D
  → holes.nearest_hole_id / mouth_xyz          # chọn HOLE_i
  → solve_above_down(mouth_world)              # IK ABOVE + DOWN
       mouth_world
         → motion.world_point_to_base          # BASE_X/Y_SIGN=-1 (Rz π)
         → make_pick_targets_from_mouth_base   # flange ABOVE / DOWN (cùng XY)
         → inverse_kinematics_downward         # DH flange, tip-down
  → goto_pose(q_above)
  → goto_straight_z(ABOVE→DOWN)                # 17 điểm, XY cố định, chỉ Z
  → goto_straight_z(DOWN→ABOVE)                # retract thẳng Z
  → goto_pose(HOME)
```

#### `holes.py` — miệng lỗ

| Hàm / dữ liệu | Việc |
|---------------|------|
| `FALLBACK_MOUTH` | 9 điểm `(x,y,0.55)` nếu thiếu `DEF HOLE_i` |
| `measure_hole_centers(supervisor)` | Đọc Pose từng `HOLE_i`; mouth = center + `(0,0,height/2)` |
| `nearest_hole_id(...)` | Lỗ gần click nhất (ngưỡng ~0.03 m) |
| `mouth_xyz(...)` | Trả tọa độ miệng lỗ world |

#### `poses.py` — góc khớp sẵn

| Tên | Ý nghĩa |
|-----|---------|
| `HOME` | `[0, -π/2, π/2, -π/2, -π/2, 0]` — tư thế chờ |
| `SEED_DOWN` | Tip-down UR3e — seed IK / warm-up |
| `FINGER_CLOSED` | `0.75` — khép ngón giữ kim |

#### `motion.py` — toán + chuyển động (quan trọng nhất)

| Phần | Giải thích rõ |
|------|----------------|
| `INSERT_DEPTH = 0.02` | Tip **dưới** miệng 2 cm. `PICK_CLEARANCE = -INSERT_DEPTH` |
| `FLANGE_TO_NEEDLE_TIP` | Khoảng flange → tip (~0.20 m); calib TCP↔tip lúc warm-up |
| `ABOVE_CLEARANCE = 0.16` | ABOVE cao hơn điểm DOWN 16 cm |
| `BASE_X_SIGN=BASE_Y_SIGN=-1` | World `(x,y)` ↔ DH `(−x,−y)` (Rz π). Sửa lệch đối trục |
| `get_world_base_transform()` | `R=diag(sx,sy,1)`, `t=(0,0,0.6)` — base robot tại z=0.6 |
| `world_point_to_base` / `base_point_to_world` | Đổi frame miệng lỗ ↔ IK |
| `_DH` | Modified DH **UR3e** (không phải UR5e) |
| `forward_kinematics(q)` | 4×4 flange trong base DH |
| `inverse_kinematics_downward(p, seed_q, …)` | IK vị trí flange + hướng tip xuống (−Z) |
| `make_pick_targets_from_mouth_base` | Cùng XY miệng; Z flange = mouth_z + tip_off + clearance |
| `ik_straight_z_path` | Nhiều điểm: XY giữ nguyên, Z nội suy → IK từng điểm |
| `goto_straight_z` / `follow_q_path` | Đi theo path Z (insert/retract thẳng) |
| `unwrap_q_near` / `unwrap_angle` | Chọn nhánh ±2π gần pose hiện tại (tránh xoay 1 vòng) |
| `goto_pose` / `smooth_move_ur_to` | Nội suy joint + đợi tới tolerance |

**Công thức target flange (base):**

```text
p_down  = [mx, my, mz + FLANGE_TO_NEEDLE_TIP + PICK_CLEARANCE]
p_above = [mx, my, p_down_z + ABOVE_CLEARANCE]
```

Với `INSERT_DEPTH=+0.02` → tip thấp hơn miệng 2 cm khi ở DOWN.

#### `ur3_click_insert.py` — orchestration

| Hàm | Việc |
|-----|------|
| `seed_with_pan(template, mouth_base)` | `shoulder_pan ≈ atan2(my, mx)` trong **base DH** |
| `solve_above_down(mouth_world)` | Thử nhiều seed pan (±0.4…); chọn `minimize_q_jump` tốt nhất → `q_above`, `q_down` |
| `run_insert_ik_test(...)` | HOME (ngầm) → ABOVE → **straight-Z DOWN** → **straight-Z retract** → HOME |
| `main()` | `Supervisor`, Mouse 3D, đo holes, warm-up `SEED_DOWN` + calib tip, vòng click |

---

### 3) ROS 2 — luồng code (từng bước)

```text
./run_ros_demo.sh
  → source Jazzy (python3.12) + colcon build
  → demo.launch.py
       → sim.launch.py
            WebotsLauncher(world=absolute ur3_needle.wbt, ros2_supervisor=True)
            WebotsController UR3e (urdf + ros2_control)
            spawner: joint_state_broadcaster + joint_trajectory_controller
       → needle_insert_node   (load hole_poses.yaml)
       → depth_monitor

User Terminal 2:
  /needle/hole_id = 8
  /needle/start = true
       → MOVE_HOME (home joints)
       → MOVE_APPROACH (approach[q])
       → INSERT (z_path 17 pts — thẳng Z)
       → HOLD
       → RETRACT (reverse z_path)
       → MOVE_HOME → DONE
```

#### `run_ros_demo.sh` (giải thích dòng chính)

| Dòng / khối | Việc |
|-------------|------|
| `PATH=/usr/bin:/bin:...` | Ưu tiên **python3.12** (tránh conda 3.13 làm hỏng `rclpy`) |
| `USER=abc` | IPC Webots extern ổn định |
| `set +u` trước `source setup.bash` | Tránh lỗi `AMENT_TRACE_SETUP_FILES` unbound |
| `pkill -x webots-bin` | Tắt Webots cũ (không dùng `pkill -f` — dễ tự kill shell) |
| `rm -rf /tmp/webots/{default,abc}` | Xóa IPC cũ |
| `colcon build --packages-select ur3_needle_sim` | Build package |
| `ros2 launch ... demo.launch.py` | Mở sim + 2 node |

#### `demo.launch.py`

| Phần | Việc |
|------|------|
| `IncludeLaunchDescription(sim.launch.py)` | Webots + driver + controller spawner |
| `Node needle_insert_node` | Máy trạng thái insert |
| `Node depth_monitor` | Đọc phase → pub `/needle/depth` |
| Arg `depth_m` | Tham số monitor (joints thật lấy từ `z_path` bake) |

#### `sim.launch.py` (điểm dễ hỏng)

| Phần | Việc |
|------|------|
| `world_path = os.path.join(pkg, 'worlds', 'ur3_needle.wbt')` | **String tuyệt đối** — WebotsLauncher mới inject được `Ros2Supervisor` vào bản temp |
| `mode='realtime'` | Sim chạy ngay (không pause) |
| `ros2_supervisor=True` | Pub `/clock` cho `use_sim_time` |
| **Một** `spawner` + `--activate-as-group` | Tránh 2 spawner tranh lock CM |
| `--switch-timeout 90` | Đủ thời gian activate khi Webots mới lên |

#### `needle_insert_node.py`

| Phần | Việc |
|------|------|
| `Phase` enum | `idle → move_home → move_approach → insert → hold → retract → done` / `aborted` |
| Load `hole_poses.yaml` | `approach_qs`, `insert_qs`, **`z_paths`** (17 pts/lỗ) |
| Sub `/needle/hole_id` | Đổi lỗ 0..8 |
| Sub `/needle/start` | Bắt đầu chu trình nếu đang idle/done/aborted |
| Sub `/needle/abort` | Cancel goal + phase aborted |
| `send_joint_path(path, T)` | `FollowJointTrajectory` nhiều điểm, time chia đều trên `T` |
| INSERT | `send_joint_path(z_path)` — **thẳng Z** |
| RETRACT | `send_joint_path(reversed(z_path))` |
| Pub `/needle/phase` | String phase hiện tại |

#### `hole_poses_loader.py`

Đọc YAML đơn giản (không cần PyYAML): `home`, mỗi hole `xyz` / `approach` / `insert` / danh sách `z_path: - [q0..q5]`.

#### `bake_hole_poses_from_click_ik.py`

```text
Với mỗi FALLBACK_MOUTH:
  solve_above_down (cùng motion.py click)
  ik_straight_z_path(ABOVE→DOWN, n=16)
  ghi approach, insert=z_path[-1], z_path
→ src/.../config/hole_poses.yaml (+ mirror controllers/)
```

Chạy khi đổi `INSERT_DEPTH`, `BASE_*_SIGN`, hoặc `FLANGE_TO_NEEDLE_TIP`:

```bash
/usr/bin/python3 scripts/bake_hole_poses_from_click_ik.py
./run_ros_demo.sh
```

---

### 4) So sánh Click vs ROS (cùng mục tiêu, khác cơ chế)

| | Click live | ROS |
|--|------------|-----|
| Lấy miệng lỗ | Supervisor `DEF HOLE_i` | `xyz` trong `hole_poses.yaml` |
| Tính joint | IK mỗi lần click | Bake sẵn (cùng công thức IK) |
| Insert thẳng Z | `goto_straight_z` live | Trajectory `z_path` |
| Frame | `BASE_*_SIGN=-1` trong `motion.py` | Đã nằm trong joint bake |
| Depth | `INSERT_DEPTH` trong `motion.py` | Đổi depth → **phải bake lại** |

---

### 5) Checklist đọc code nhanh

1. Muốn đổi độ sâu → `motion.py` `INSERT_DEPTH` → bake → ROS restart  
2. Muốn hiểu lệch XY → `BASE_X_SIGN` / `BASE_Y_SIGN` + `world_point_to_base`  
3. Muốn hiểu insert thẳng → `ik_straight_z_path` / `z_path` trong yaml  
4. Muốn hiểu ROS topics → `needle_insert_node.py` `Phase` + `send_joint_path`  
5. ROS không nhận lệnh → `./run_ros_demo.sh` còn chạy? `ros2 topic info /needle/start` Subscription ≥ 1?

---

## Từ điển siêu ngắn

| Từ | Nghĩa |
|----|-------|
| Terminal | Cửa sổ gõ lệnh |
| Thư mục | Nơi chứa file |
| Package | Hộp project nhỏ trong ROS |
| Build | Đóng gói để chạy được |
| Topic | Kênh tin nhắn giữa các chương trình |
| Node | Một chương trình ROS |
| Launch | File mở nhiều chương trình cùng lúc |
| Joint | Khớp robot |
| Publisher | Bên gửi tin |
| Subscriber | Bên nhận tin |

---

## Quy tắc vàng (mỗi Terminal mới)

```bash
source /opt/ros/jazzy/setup.bash
export WEBOTS_HOME=/usr/local/webots
```

| Dòng | Giải thích |
|------|------------|
| `source /opt/ros/jazzy/setup.bash` | Bật ROS 2 Jazzy trong Terminal này. Không chạy dòng này thì lệnh `ros2` sẽ lỗi. |
| `export WEBOTS_HOME=/usr/local/webots` | Báo cho máy: Webots nằm ở `/usr/local/webots`. |

---

# NGÀY 1 — Tạo workspace, package, world Webots

### Mục tiêu
Tự tạo cấu trúc project + file thế giới có UR3e, kim, phantom.

### Checklist
- [x] Bước 1: tạo workspace + package
- [x] Bước 2: tạo thư mục con
- [x] Bước 3: hiểu/sửa `package.xml`
- [x] Bước 4: sửa `setup.py`
- [x] Bước 5: viết `ur3_needle.wbt`
- [x] Bước 6: build + mở Webots

---

## Ngày 1 — Bước 1: Tạo workspace và package

### Lệnh cần gõ

```bash
cd /workspace/share/ros2-workspace/trung_test
mkdir -p ur3_needle_ws/src
cd ur3_needle_ws/src
source /opt/ros/jazzy/setup.bash
ros2 pkg create --build-type ament_python ur3_needle_sim --dependencies rclpy std_msgs sensor_msgs trajectory_msgs control_msgs
ls ur3_needle_sim
```

### Giải thích từng dòng

| Dòng | Giải thích |
|------|------------|
| `cd /workspace/share/ros2-workspace/trung_test` | `cd` = change directory = đi vào thư mục làm việc của bạn. |
| `mkdir -p ur3_needle_ws/src` | `mkdir` = make directory = tạo thư mục. `-p` = tạo cả đường dẫn cha nếu chưa có, không báo lỗi nếu đã có. Tạo workspace tên `ur3_needle_ws`, bên trong có `src` (chỗ để source code). |
| `cd ur3_needle_ws/src` | Đi vào thư mục `src` vừa tạo. |
| `source /opt/ros/jazzy/setup.bash` | Bật ROS trước khi dùng lệnh `ros2`. |
| `ros2 pkg create ...` | Lệnh ROS tạo sẵn khung một package. |
| `  --build-type ament_python` | Package kiểu Python (không phải C++). |
| `  ur3_needle_sim` | Tên package bạn đặt. |
| `  --dependencies rclpy std_msgs ...` | Khai báo thư viện ROS mà package sẽ dùng. |
| `ls ur3_needle_sim` | `ls` = list = liệt kê file trong package vừa tạo để kiểm tra. |

### `ros2 pkg create` đã tạo giúp bạn những gì?

Thường có: `package.xml`, `setup.py`, `setup.cfg`, folder `ur3_needle_sim/`, file `resource/ur3_needle_sim`, …

---

## Ngày 1 — Bước 2: Tạo thư mục con

### Lệnh

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws/src/ur3_needle_sim
mkdir -p launch worlds config resource rviz controllers/ur3_needle_native
ls
```

### Giải thích từng dòng / từng tên thư mục

| Dòng / tên | Giải thích |
|------------|------------|
| `cd .../ur3_needle_sim` | Đi vào trong package. |
| `mkdir -p launch` | Chỗ để file `.launch.py` (mở sim). |
| `mkdir -p worlds` | Chỗ để file thế giới Webots `.wbt`. |
| `mkdir -p config` | Chỗ để file thông số `.yaml` (độ sâu, pose…). |
| `mkdir -p resource` | Chỗ để URDF + config controller. |
| `mkdir -p rviz` | Chỗ để file cấu hình RViz (tuỳ chọn). |
| `mkdir -p controllers/ur3_needle_native` | Chỗ để controller bàn phím của Webots. Webots bắt buộc: `controllers/TênController/TênController.py`. |
| `ls` | Xem lại đã có đủ folder chưa. |

---

## Ngày 1 — Bước 3: Đọc và bổ sung `package.xml`

Mở file:

```text
ur3_needle_ws/src/ur3_needle_sim/package.xml
```

### Các dòng quan trọng nghĩa là gì?

| Dòng XML (ví dụ) | Giải thích |
|------------------|------------|
| `<?xml version="1.0"?>` | Đây là file XML phiên bản 1.0. |
| `<package format="3">` | Bắt đầu package ROS, format 3 (chuẩn ROS 2). |
| `<name>ur3_needle_sim</name>` | Tên package. Phải khớp tên folder. |
| `<version>0.0.0</version>` | Số phiên bản (ban đầu 0.0.0 cũng được). |
| `<description>...</description>` | Mô tả ngắn project. |
| `<maintainer ...>` | Người giữ package (bạn). |
| `<license>...</license>` | Giấy phép (ví dụ Apache-2.0). |
| `<depend>rclpy</depend>` | Cần thư viện Python của ROS (`rclpy`). |
| `<depend>std_msgs</depend>` | Cần kiểu tin nhắn chuẩn (`Bool`, `String`, `Float32`…). |
| `<depend>sensor_msgs</depend>` | Cần tin nhắn cảm biến (ví dụ `JointState`). |
| `<depend>trajectory_msgs</depend>` | Cần tin nhắn quỹ đạo khớp. |
| `<depend>control_msgs</depend>` | Cần action điều khiển (FollowJointTrajectory). |
| `<depend>webots_ros2_driver</depend>` | Cần cầu nối Webots ↔ ROS. |
| `<depend>webots_ros2_control</depend>` | Cần plugin điều khiển khớp trong Webots. |
| `<export><build_type>ament_python</build_type></export>` | Báo đây là package Python. |
| `</package>` | Kết thúc file. |

Nếu thiếu dependency, thêm các dòng `<depend>...</depend>` **trước** `</package>`.

---

## Ngày 1 — Bước 4: Sửa `setup.py` (giải thích từng phần)

Mở `setup.py`.

### Phần import

```python
from setuptools import setup
from glob import glob
import os
```

| Dòng | Giải thích |
|------|------------|
| `from setuptools import setup` | Lấy hàm `setup` để khai báo cách cài package Python. |
| `from glob import glob` | `glob` = tìm file theo mẫu, ví dụ `launch/*.py`. |
| `import os` | Thư viện đường dẫn file/thư mục. |

### Phần tên package

```python
package_name = 'ur3_needle_sim'
```

| Dòng | Giải thích |
|------|------------|
| `package_name = 'ur3_needle_sim'` | Đặt biến tên package, dùng lại nhiều lần cho khỏi gõ sai. |

### Phần `data_files` (rất quan trọng)

Thêm các dòng kiểu:

```python
(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
(os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
(os.path.join('share', package_name, 'config'), glob('config/*')),
(os.path.join('share', package_name, 'resource'), glob('resource/*')),
(os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
```

| Mảnh code | Giải thích |
|-----------|------------|
| `os.path.join('share', package_name, 'launch')` | Khi install, copy vào `share/ur3_needle_sim/launch`. ROS tìm file launch ở đây. |
| `glob('launch/*.py')` | Lấy tất cả file `.py` trong folder `launch` hiện tại. |
| Tương tự `worlds` / `config` / `resource` / `rviz` | Copy đúng loại file vào đúng chỗ sau khi build. |

### Phần `entry_points` (khai báo chương trình chạy được)

```python
entry_points={
    'console_scripts': [
        'needle_insert_node = ur3_needle_sim.needle_insert_node:main',
        'depth_monitor = ur3_needle_sim.depth_monitor:main',
    ],
},
```

| Mảnh | Giải thích |
|------|------------|
| `console_scripts` | Đăng ký lệnh chạy từ Terminal. |
| `needle_insert_node = ...` | Tạo lệnh tên `needle_insert_node`. |
| `ur3_needle_sim.needle_insert_node` | Nghĩa là file `ur3_needle_sim/needle_insert_node.py`. |
| `:main` | Gọi hàm `main()` trong file đó. |

> Ngày 1 chưa cần có 2 file Python đó. Bạn khai báo trước; Ngày 4 mới viết.

---

## Ngày 1 — Bước 5: Viết `worlds/ur3_needle.wbt` (từng khối)

Tạo file mới `worlds/ur3_needle.wbt` và viết theo từng khối dưới đây.

### Khối 1 — dòng đầu + gọi mô hình có sẵn

```text
#VRML_SIM R2025a utf8
```

| Dòng | Giải thích |
|------|------------|
| `#VRML_SIM R2025a utf8` | Báo đây là world Webots phiên bản R2025a, mã hóa utf8. |

```text
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/Floor.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/appearances/protos/ThreadMetalPlate.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/solids/protos/SolidBox.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/appearances/protos/GalvanizedMetal.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/universal_robots/protos/UR3e.proto"
```

| Dòng | Giải thích |
|------|------------|
| `EXTERNPROTO "...TexturedBackground.proto"` | Tải mẫu nền trời/nhà máy. |
| `EXTERNPROTO "...Floor.proto"` | Tải mẫu sàn. |
| `EXTERNPROTO "...ThreadMetalPlate.proto"` | Tải vật liệu sàn kim loại. |
| `EXTERNPROTO "...SolidBox.proto"` | Tải khối hộp đặc. |
| `EXTERNPROTO "...GalvanizedMetal.proto"` | Tải vật liệu kim loại cho bệ đỡ. |
| `EXTERNPROTO "...UR3e.proto"` | Tải mô hình robot UR3e. |

### Khối 2 — thông tin thế giới + camera nhìn

```text
WorldInfo {
  title "UR3 Needle Insertion"
  basicTimeStep 8
}
Viewpoint {
  orientation -0.2 0.5 0.84 1.9
  position 1.6 -1.4 1.6
}
```

| Dòng | Giải thích |
|------|------------|
| `WorldInfo {` | Bắt đầu thông tin thế giới. |
| `title "..."` | Tên hiện trên Webots. |
| `basicTimeStep 8` | Mỗi bước mô phỏng = 8 ms. Số nhỏ = mượt hơn nhưng nặng hơn. |
| `}` | Kết thúc `WorldInfo`. |
| `Viewpoint {` | Góc nhìn camera. |
| `orientation ...` | Hướng nhìn (trục xoay + góc). |
| `position 1.6 -1.4 1.6` | Vị trí camera (x y z) mét. |

### Khối 3 — nền và sàn

```text
TexturedBackground {
  texture "factory"
}
Floor {
  size 8 8
  appearance ThreadMetalPlate {
  }
}
```

| Dòng | Giải thích |
|------|------------|
| `TexturedBackground {` | Tạo nền. |
| `texture "factory"` | Dùng texture nhà máy. |
| `Floor {` | Tạo sàn. |
| `size 8 8` | Sàn 8m × 8m. |
| `appearance ThreadMetalPlate { }` | Sơn sàn kiểu tấm kim loại. |

### Khối 4 — bệ đỡ robot

```text
SolidBox {
  translation 0 0 0.3
  size 0.35 0.35 0.6
  appearance GalvanizedMetal {
  }
}
```

| Dòng | Giải thích |
|------|------------|
| `SolidBox {` | Tạo hộp đặc. |
| `translation 0 0 0.3` | Đặt tâm hộp tại (0,0,0.3). Cao 0.3 m so với gốc. |
| `size 0.35 0.35 0.6` | Kích thước x y z (m). Cao 0.6 m. |
| `appearance GalvanizedMetal { }` | Nhìn như kim loại mạ kẽm. |

### Khối 5 — phantom 1×1×0.1 có 9 lỗ sâu kim (giải thích từng dòng)

> **Lưu ý:** `SolidBox` chỉ là hộp đặc, **không gắn thêm lỗ được**.  
> Muốn có lỗ: dùng `Solid` + `Box` (thân mô) + nhiều `Cylinder` màu đen trên mặt trên.

#### 5.1. Vì sao phải xếp lỗ trong tầm với của UR3e?

| Thông số | Giá trị |
|----------|---------|
| Tầm với UR3e (xấp xỉ) | ~ **0.50 m** từ tâm base |
| Base robot trong world | `(0, 0, 0.6)` |
| Vùng an toàn để đặt lỗ | phía trước arm, khoảng cách ngang ~ **0.26 → 0.47 m** |

Nếu lỗ ở xa > 0.5 m (ví dụ lưới ±0.3 m trên tấm 1×1 đặt lệch) → robot **không với tới**.

#### 5.2. Thân khối mô

```text
DEF PHANTOM Solid {
  translation 0.36 0.0 0.50
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.85 0.55 0.55
        roughness 0.7
        metalness 0
      }
      geometry Box {
        size 1 1 0.1
      }
    }
```

| Dòng | Giải thích |
|------|------------|
| `DEF PHANTOM Solid {` | Tạo vật rắn tên nội bộ `PHANTOM`. |
| `translation 0.36 0.0 0.50` | Đặt **tâm** khối tại (0.36, 0, 0.50) m — lệch về phía trước arm. |
| `geometry Box { size 1 1 0.1 }` | Ví dụ học; **world hiện tại** dùng `0.2 × 0.2 × 0.1` (gọn trong workspace). |

**Mặt trên (chỗ đâm):**  
`z = 0.50 + 0.05 = 0.55` m.

#### 5.3. Một lỗ (mẫu) — trong tầm với

```text
    DEF HOLE_0 Pose {
      translation -0.10 -0.10 0.01
      children [
        DEF HOLE_SHAPE Shape {
          appearance PBRAppearance {
            baseColor 0.05 0.05 0.05
            roughness 1
            metalness 0
          }
          geometry Cylinder {
            height 0.08
            radius 0.012
          }
        }
      ]
    }
```

| Dòng | Giải thích |
|------|------------|
| `DEF HOLE_0` | Để Supervisor đo tâm (`holes.measure_hole_centers`). Các lỗ 1..8: `DEF HOLE_i` + `USE HOLE_SHAPE`. |
| `translation -0.10 -0.10 0.01` | Tâm hình học cylinder trong phantom; miệng = tâm + `(0,0,height/2)`. |
| Cộng với tâm phantom `(0.36,0,0.50)` | → mouth world ≈ `(0.26, -0.10, 0.55)`. |
| **Không** `rotation` | Webots R2025a: Cylinder **mặc định trục Z**. Xoay `1 0 0 1.5708` sẽ thành giếng **ngang theo Y** — sai. |
| `height 0.08` / `radius 0.012` | Giếng sâu ~8 cm theo Z, miệng 1.2 cm. |

#### 5.4. Lưới 3×3 trong workspace (cách nhau 10 cm)

```text
# local (x,y) trên mặt phantom — toàn bộ map vào tầm UR3e
(-0.10, 0.10)  (0, 0.10)  (0.10, 0.10)
(-0.10, 0   )  (0, 0   )  (0.10, 0   )
(-0.10,-0.10)  (0,-0.10)  (0.10,-0.10)
```

Đổi sang **world xy** (cộng tâm 0.36, 0):

```text
 (0.26, 0.10)  (0.36, 0.10)  (0.46, 0.10)
 (0.26, 0   )  (0.36, 0   )  (0.46, 0   )
 (0.26,-0.10)  (0.36,-0.10)  (0.46,-0.10)
```

| Ô | Khoảng cách ngang tới base `(0,0)` |
|---|-------------------------------------|
| Gần nhất `(0.26, 0)` | **0.26 m** |
| Giữa `(0.36, 0)` | **0.36 m** |
| Xa nhất `(0.46, ±0.10)` | **~0.47 m** |

Tất cả **< 0.50 m** → nằm trong phạm vi hoạt động UR3e.

Các Pose còn lại chỉ đổi `translation x y`, dùng `USE HOLE_SHAPE`.

#### 5.4. Đóng Solid + va chạm

```text
  ]
  name "phantom"
  boundingObject Box {
    size 1 1 0.1
  }
}
```

| Dòng | Giải thích |
|------|------------|
| `]` | Hết danh sách `children`. |
| `name "phantom"` | Tên hiện trong cây scene Webots. |
| `boundingObject Box { size 1 1 0.1 }` | Khối dùng cho **va chạm vật lý** (cùng size thân mô). |
| `}` | Kết thúc Solid. |

> **Hiểu đúng về “lỗ” MVP:** miệng lỗ đen là **mốc nhìn để nhắm kim**.  
> `boundingObject` vẫn là hộp đặc — kim có thể “chạm” bề mặt theo physics hộp.  
> Nếu sau này cần lỗ thủng thật (kim đi xuyên không va chạm), phải làm mesh/CAD phức tạp hơn (giai đoạn sau).

#### 5.5. File world của bạn đã được cập nhật ở đâu?

| Đường dẫn | Ghi chú |
|-----------|---------|
| `ur3_needle_ws/worlds/ur3_needle.wbt` | World demo click (**đang dùng**, `ur3_click_insert`). |
| `ur3_needle_ws/src/ur3_needle_sim/worlds/ur3_needle.wbt` | Bản ROS (`<extern>`) — `ros2 launch` / `run_ros_demo.sh`. |
| `.../worlds/ur3_needle_click.wbt` | Bản click trong package (giữ `ur3_click_insert`). |
| `.../worlds/ur3_needle_ros.wbt` | Bản ROS phụ (`<extern>`). |

Mở lại world trong Webots để thấy 9 chấm đen trên mặt khối mô hồng.

### Khối 6 — robot UR3e + gripper 2 ngón nhỏ (Robotiq 2F-85) + kim dài gắp sẵn

Trước tiên, ở phần `EXTERNPROTO` đầu file phải có:

```text
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/devices/robotiq/protos/Robotiq2f85Gripper.proto"
```

| Dòng | Giải thích |
|------|------------|
| `...Robotiq2f85Gripper.proto` | Gripper **2 ngón**, hành trình 85 mm — nhỏ hơn 2F-140 và gọn hơn Robotiq 3F. |

#### 6.1. Robot + toolSlot

```text
UR3e {
  translation 0 0 0.6
  name "UR3e"
  controller "<extern>"
  supervisor TRUE
  selfCollision FALSE
  toolSlot [
    Robotiq2f85Gripper {
      name "gripper"
    }

    DEF NEEDLE Solid {
      translation 0 0 0.07
      children [
        Pose {
          translation 0 0 0.09
          rotation 1 0 0 1.5708
          children [
            Shape {
              appearance PBRAppearance {
                baseColor 0.75 0.75 0.85
                metalness 1
                roughness 0.3
              }
              geometry Cylinder {
                height 0.18
                radius 0.002
              }
            }
          ]
        }
      ]
      name "needle"
    }
  ]
}
```

| Dòng | Giải thích |
|------|------------|
| `Robotiq2f85Gripper { name "gripper" }` | Gripper 2 ngón nhỏ; trục chính ra khỏi flange là **+Z**. |
| `DEF NEEDLE Solid {` | Kim gắn cùng `toolSlot` → đi theo tay. |
| `translation 0 0 0.05` | Gốc kim trong vùng giữa 2 ngón (~5 cm theo +Z). |
| `height 0.18` | Kim dài **18 cm**; **đầu kim** = đầu cylinder (không dùng sphere đỏ). |
| `controller "ur3_grasp_home"` | Controller đặt **HOME** arm + **khép ngón** gắp kim khi mở world. |
| `hidden position_0_0 ... position_5_0` | Góc HOME arm: `[0, -1.57, 1.57, -1.57, -1.57, 0]`. |

> **Cấm** ghi `hidden position` cho khớp gripper (ví dụ `position_6_0`) — sẽ làm Webots **segfault**. Khép ngón chỉ bằng controller `ur3_grasp_home`.

**Mở world (để thấy gắp + home):**
```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
webots worlds/ur3_needle.wbt
```

**Khi dùng ROS 2:** đổi `controller` thành `"<extern>"` (ngón có thể mở lại trừ khi node ROS ra lệnh khép).

#### 6.2. So sánh gripper

| Gripper | Ngón | Ghi chú |
|---------|------|---------|
| Robotiq 3F | 3 | To hơn, mẫu cũ |
| **Robotiq 2F-85** | **2** | **Nhỏ, đang dùng** |
| Robotiq 2F-140 | 2 | Dài/hở rộng hơn 2F-85 |

#### 6.3. “Gắp sẵn” nghĩa là gì?

| Cách | Ý nghĩa |
|------|---------|
| Kim là object trong `toolSlot` | Dính theo flange mọi lúc |
| Không pick từ bàn | Đủ cho demo sâu kim 5 ngày |

> Muốn 2 ngón khép chặt thật: chỉnh motor ngón bằng controller. MVP chỉ cần kim fixed trong vùng gắp.

---

## Ngày 1 — Bước 6: Build và mở Webots

### Lệnh

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
source /opt/ros/jazzy/setup.bash
export WEBOTS_HOME=/usr/local/webots
colcon build --packages-select ur3_needle_sim
source install/setup.bash
webots /workspace/share/ros2-workspace/trung_test/ur3_needle_ws/src/ur3_needle_sim/worlds/ur3_needle.wbt
```

| Dòng | Giải thích |
|------|------------|
| `cd .../ur3_needle_ws` | Về gốc workspace (nơi sẽ có `build/`, `install/`, `log/`). |
| `source /opt/ros/jazzy/setup.bash` | Bật ROS. |
| `export WEBOTS_HOME=...` | Chỉ đường Webots. |
| `colcon build --packages-select ur3_needle_sim` | Chỉ build đúng 1 package của bạn. |
| `source install/setup.bash` | Bật package vừa build để ROS nhìn thấy. |
| `webots .../ur3_needle.wbt` | Mở đúng file world bạn vừa viết. |

**Xong Ngày 1 khi:** thấy robot + kim + khối hồng.

---

# NGÀY 2 — Nối Webots với ROS 2

### Mục tiêu
URDF + config controller + `sim.launch.py` → có `/ur3/joint_states`.

### Checklist
- [x] Viết URDF
- [x] Viết ros2_control yaml
- [x] Viết `sim.launch.py`
- [x] Chạy và echo joint states

---

## Ngày 2 — Bước 1: Viết `resource/ur3e_needle.urdf`

### Dòng đầu

```xml
<?xml version="1.0"?>
<robot name="UR3e">
```

| Dòng | Giải thích |
|------|------------|
| `<?xml version="1.0"?>` | File XML. |
| `<robot name="UR3e">` | Bắt đầu mô tả robot tên UR3e. |

### Plugin Webots

```xml
  <webots>
    <plugin type="webots_ros2_control::Ros2Control"/>
  </webots>
```

| Dòng | Giải thích |
|------|------------|
| `<webots>` | Khối cấu hình riêng cho Webots. |
| `<plugin type="webots_ros2_control::Ros2Control"/>` | Nạp plugin để ROS điều khiển khớp trong Webots. |

### Link (các “mắt xích”)

```xml
  <link name="base_link"/>
  <link name="shoulder_link"/>
  ...
  <link name="needle_tip"/>
```

| Dòng | Giải thích |
|------|------------|
| `<link name="..."/>` | Khai báo một mắt xích/khung. Dấu `/>` = tự đóng thẻ, chưa cần hình dạng chi tiết. |
| `needle_tip` | Điểm đầu kim (để sau này nói về độ sâu). |

### Một joint mẫu (các joint khác cùng ý tưởng)

```xml
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.15185" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="150" velocity="3.14"/>
  </joint>
```

| Dòng | Giải thích |
|------|------------|
| `<joint name="shoulder_pan_joint" type="revolute">` | Khớp quay tên `shoulder_pan_joint`. **Tên phải trùng** tên motor trong Webots UR3e. |
| `<parent link="base_link"/>` | Khớp nối từ link cha. |
| `<child link="shoulder_link"/>` | Tới link con. |
| `<origin xyz="..." rpy="..."/>` | Vị trí/hướng của khớp so với cha. `xyz` mét, `rpy` radian. |
| `<axis xyz="0 0 1"/>` | Quay quanh trục Z. |
| `<limit lower=... upper=...>` | Giới hạn góc, mô-men, tốc độ. |

### Fixed joint tới tip kim

```xml
  <joint name="tool0_to_needle_tip" type="fixed">
    <parent link="tool0"/>
    <child link="needle_tip"/>
    <origin xyz="0 0 0.12" rpy="0 0 0"/>
  </joint>
```

| Dòng | Giải thích |
|------|------------|
| `type="fixed"` | Không quay — gắn cứng. |
| `xyz="0 0 0.12"` | Tip cách tool0 đúng 12 cm (bằng chiều dài kim trong world). |

### Khối ros2_control (một joint mẫu)

```xml
  <ros2_control name="WebotsControl" type="system">
    <hardware>
      <plugin>webots_ros2_control::Ros2ControlSystem</plugin>
    </hardware>
    <joint name="shoulder_pan_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    ...
  </ros2_control>
</robot>
```

| Dòng | Giải thích |
|------|------------|
| `<ros2_control ... type="system">` | Khai báo hệ thống điều khiển. |
| `<plugin>webots_ros2_control::Ros2ControlSystem</plugin>` | Backend thật là Webots. |
| `<command_interface name="position"/>` | ROS được phép ra lệnh theo vị trí. |
| `<state_interface name="position"/>` | ROS đọc được vị trí hiện tại. |
| `<state_interface name="velocity"/>` | ROS đọc được vận tốc. |
| `</robot>` | Kết thúc URDF. |

Lặp khối `<joint ...>` trong `ros2_control` cho đủ 6 khớp.

---

## Ngày 2 — Bước 2: Viết `resource/ros2_control_config.yaml`

```yaml
ur3/controller_manager:
  ros__parameters:
    update_rate: 50
    ur_joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    ur_joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

ur3/ur_joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
    allow_partial_joints_goal: true
```

| Dòng | Giải thích |
|------|------------|
| `ur3/controller_manager:` | Namespace `ur3` + node quản lý controller. |
| `ros__parameters:` | Các tham số ROS 2. |
| `update_rate: 50` | Cập nhật 50 lần/giây. |
| `ur_joint_trajectory_controller:` | Đăng ký controller quỹ đạo. |
| `type: joint_trajectory_controller/...` | Dùng loại controller có sẵn của ROS. |
| `ur_joint_state_broadcaster:` | Đăng ký broadcaster đọc khớp. |
| `type: joint_state_broadcaster/...` | Loại broadcaster chuẩn. |
| `ur3/ur_joint_trajectory_controller:` | Cấu hình chi tiết cho controller quỹ đạo. |
| `joints:` | Danh sách khớp nó được phép điều khiển. |
| `- shoulder_pan_joint` | Một phần tử trong list YAML. |
| `command_interfaces: - position` | Ra lệnh theo position. |
| `state_interfaces: - position` | Đọc state position. |
| `allow_partial_joints_goal: true` | Cho phép goal không đủ mọi khớp (linh hoạt lúc học). |

---

## Ngày 2 — Bước 3: Viết `launch/sim.launch.py` (từng dòng)

```python
#!/usr/bin/env python3
```

| Dòng | Giải thích |
|------|------------|
| `#!/usr/bin/env python3` | Shebang: chạy file này bằng Python 3. |

```python
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
```

| Dòng | Giải thích |
|------|------------|
| `import os` | Xử lý đường dẫn. |
| `import launch` | Thư viện launch ROS (shutdown, event…). |
| `get_package_share_directory` | Tìm folder `share/ur3_needle_sim` sau khi build. |
| `LaunchDescription` | “Danh sách việc” launch sẽ chạy. |
| `DeclareLaunchArgument` | Khai báo tham số dòng lệnh (ví dụ `use_sim_time`). |
| `LaunchConfiguration` | Đọc giá trị tham số đó. |
| `PathJoinSubstitution` | Ghép đường dẫn trong launch. |
| `Node` | Chạy một node ROS. |
| `WebotsLauncher` | Mở Webots kèm world. |
| `WebotsController` | Driver nối ROS với robot trong Webots. |
| `WaitForControllerConnection` | Đợi driver sẵn sàng rồi mới spawn controller. |

```python
PACKAGE = 'ur3_needle_sim'
```

| Dòng | Giải thích |
|------|------------|
| `PACKAGE = 'ur3_needle_sim'` | Lưu tên package cho dễ dùng. |

```python
def generate_launch_description():
```

| Dòng | Giải thích |
|------|------------|
| `def generate_launch_description():` | Hàm bắt buộc của launch file Python. ROS gọi hàm này. |

```python
    pkg = get_package_share_directory(PACKAGE)
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(pkg, 'resource', 'ur3e_needle.urdf')
    controllers = os.path.join(pkg, 'resource', 'ros2_control_config.yaml')
```

| Dòng | Giải thích |
|------|------------|
| `pkg = get_package_share_directory(...)` | Lấy đường dẫn share của package. |
| `use_sim_time = LaunchConfiguration(...)` | Biến tham số: dùng đồng hồ mô phỏng. |
| `urdf = os.path.join(...)` | Đường dẫn file URDF. |
| `controllers = os.path.join(...)` | Đường dẫn yaml controller. |

```python
    webots = WebotsLauncher(
        world=PathJoinSubstitution([pkg, 'worlds', 'ur3_needle.wbt']),
        ros2_supervisor=True,
    )
```

| Dòng | Giải thích |
|------|------------|
| `WebotsLauncher(` | Tạo action mở Webots. |
| `world=PathJoinSubstitution([...])` | World cần mở = `share/.../worlds/ur3_needle.wbt`. |
| `ros2_supervisor=True` | Bật supervisor ROS cho Webots. |

```python
    driver = WebotsController(
        robot_name='UR3e',
        namespace='ur3',
        parameters=[
            {'robot_description': urdf},
            {'use_sim_time': use_sim_time},
            {'set_robot_state_publisher': True},
            controllers,
        ],
        respawn=True,
    )
```

| Dòng | Giải thích |
|------|------------|
| `robot_name='UR3e'` | Phải trùng `name "UR3e"` trong file `.wbt`. |
| `namespace='ur3'` | Mọi topic/controller nằm dưới `/ur3/...`. |
| `{'robot_description': urdf}` | Đưa URDF cho driver. |
| `{'use_sim_time': use_sim_time}` | Đồng bộ thời gian sim. |
| `{'set_robot_state_publisher': True}` | Cho phép cập nhật robot_state_publisher. |
| `controllers` | Nạp yaml controller. |
| `respawn=True` | Nếu driver chết, thử chạy lại. |

```python
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='ur3',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
            'use_sim_time': use_sim_time,
        }],
    )
```

| Dòng | Giải thích |
|------|------------|
| `package='robot_state_publisher'` | Dùng package có sẵn của ROS. |
| `executable='robot_state_publisher'` | Tên chương trình. |
| `namespace='ur3'` | Chạy trong namespace ur3. |
| `output='screen'` | In log ra Terminal. |
| `'robot_description': '<robot .../>'` | Placeholder rỗng; driver sẽ set lại. |

```python
    timeout = ['--controller-manager-timeout', '100']
    spawner_js = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['ur_joint_state_broadcaster', '-c', 'ur3/controller_manager'] + timeout,
    )
    spawner_traj = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['ur_joint_trajectory_controller', '-c', 'ur3/controller_manager'] + timeout,
    )
```

| Dòng | Giải thích |
|------|------------|
| `timeout = [...]` | Đợi tối đa 100 giây cho controller_manager. |
| `executable='spawner'` | Tool bật một controller theo tên. |
| `'ur_joint_state_broadcaster'` | Bật broadcaster khớp. |
| `'-c', 'ur3/controller_manager'` | Chỉ định manager nằm ở `/ur3/controller_manager`. |
| `'ur_joint_trajectory_controller'` | Bật controller quỹ đạo. |

```python
    waiting = WaitForControllerConnection(
        target_driver=driver,
        nodes_to_start=[spawner_js, spawner_traj],
    )
```

| Dòng | Giải thích |
|------|------------|
| `target_driver=driver` | Đợi driver này ready. |
| `nodes_to_start=[...]` | Chỉ khi ready mới spawn 2 controller. |

```python
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        webots,
        webots._supervisor,
        rsp,
        driver,
        waiting,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
```

| Dòng | Giải thích |
|------|------------|
| `return LaunchDescription([...])` | Trả về danh sách thứ tự/các thành phần launch. |
| `DeclareLaunchArgument('use_sim_time', ...)` | Cho phép `use_sim_time:=false` nếu cần. |
| `webots` | Mở Webots. |
| `webots._supervisor` | Chạy supervisor đi kèm. |
| `rsp` / `driver` / `waiting` | Các node đã giải thích. |
| `OnProcessExit(... Shutdown())` | Khi đóng Webots thì tắt hết launch. |

---

## Ngày 2 — Bước 4: Chạy và kiểm tra

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
source /opt/ros/jazzy/setup.bash
export WEBOTS_HOME=/usr/local/webots
colcon build --packages-select ur3_needle_sim
source install/setup.bash
ros2 launch ur3_needle_sim sim.launch.py
```

| Dòng | Giải thích |
|------|------------|
| `ros2 launch ur3_needle_sim sim.launch.py` | Chạy launch file `sim.launch.py` trong package `ur3_needle_sim`. |

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/share/ros2-workspace/trung_test/ur3_needle_ws/install/setup.bash
ros2 topic list
ros2 topic echo /ur3/joint_states --once
```

| Dòng | Giải thích |
|------|------------|
| `ros2 topic list` | Liệt kê mọi kênh tin đang có. |
| `ros2 topic echo /ur3/joint_states --once` | In **một lần** nội dung joint states rồi dừng. |

**Xong Ngày 2 khi:** echo ra được góc các khớp.

---

# NGÀY 3 — Controller bàn phím + pose lỗ giữa + bảng 9 lỗ

### Mục tiêu
Hiểu chuyển động bằng mắt; tune **HOME + APPROACH/INSERT lỗ giữa**; chuẩn bị bảng 9 lỗ cho click.

### Checklist Ngày 3
- [x] Viết / chạy `ur3_needle_native` (bàn phím)
- [x] Tune `HOME`, `SEED_DOWN` (tip xuống phantom)
- [x] Ghi 9 tọa độ lỗ vào `config/holes.yaml` + `DEF HOLE_0..8`
- [x] ~~`pan = atan2` only~~ → clone_backup flange IK + pan seed từ `mouth_base`
- [x] Đổi controller sang `ur3_click_insert` cho Ngày 4

---

## Ngày 3 — Bước 1: Viết `controllers/ur3_needle_native/ur3_needle_native.py`

```python
from controller import Robot, Keyboard
```

| Dòng | Giải thích |
|------|------------|
| `from controller import Robot, Keyboard` | API Webots: điều khiển robot + đọc bàn phím. |

```python
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
```

| Dòng | Giải thích |
|------|------------|
| `JOINT_NAMES = [...]` | List tên 6 motor — phải trùng tên trong PROTO UR3e. |

```python
HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
APPROACH = [0.0, -1.20, 1.40, -1.70, -1.57, 0.0]
INSERT = [0.0, -1.05, 1.55, -1.90, -1.57, 0.0]
```

| Dòng | Giải thích |
|------|------------|
| `HOME = [...]` | 6 góc (radian) tư thế nghỉ. |
| `APPROACH = [...]` | Tư thế gần bề mặt, chưa đâm. |
| `INSERT = [...]` | Tư thế đã đâm. |
| `-1.57` | Khoảng −π/2 radian ≈ −90°. |

```python
def main():
    robot = Robot()
    dt = int(robot.getBasicTimeStep())
```

| Dòng | Giải thích |
|------|------------|
| `def main():` | Hàm chính. |
| `robot = Robot()` | Tạo đối tượng robot Webots. |
| `dt = int(robot.getBasicTimeStep())` | Lấy bước thời gian mô phỏng (ms), ép kiểu int. |

```python
    kb = robot.getKeyboard()
    kb.enable(dt)
```

| Dòng | Giải thích |
|------|------------|
| `robot.getKeyboard()` | Lấy thiết bị bàn phím. |
| `kb.enable(dt)` | Bật đọc phím mỗi `dt` ms. |

```python
    motors = []
    for name in JOINT_NAMES:
        m = robot.getDevice(name)
        m.setVelocity(0.8)
        motors.append(m)
```

| Dòng | Giải thích |
|------|------------|
| `motors = []` | List rỗng chứa 6 motor. |
| `for name in JOINT_NAMES:` | Lặp từng tên khớp. |
| `m = robot.getDevice(name)` | Lấy motor theo tên. |
| `m.setVelocity(0.8)` | Giới hạn tốc độ khi đi tới vị trí (rad/s). |
| `motors.append(m)` | Thêm motor vào list. |

```python
    target = list(HOME)
    selected = 0

    def apply():
        for i, m in enumerate(motors):
            m.setPosition(target[i])
```

| Dòng | Giải thích |
|------|------------|
| `target = list(HOME)` | Copy list HOME (tránh sửa nhầm list gốc). |
| `selected = 0` | Đang chọn khớp số 1 (index 0). |
| `def apply():` | Hàm con: gửi `target` xuống motor. |
| `enumerate(motors)` | Cho vừa index `i` vừa motor `m`. |
| `m.setPosition(target[i])` | Ra lệnh khớp i tới góc `target[i]`. |

```python
    apply()
    print('Phim: 1-6 | Z/X | H A I R')
```

| Dòng | Giải thích |
|------|------------|
| `apply()` | Đưa robot về HOME lúc bắt đầu. |
| `print(...)` | In hướng dẫn ra console Webots. |

```python
    while robot.step(dt) != -1:
        key = kb.getKey()
        while key != -1:
```

| Dòng | Giải thích |
|------|------------|
| `while robot.step(dt) != -1:` | Vòng mô phỏng. `step` trả `-1` khi sim dừng. |
| `key = kb.getKey()` | Đọc 1 phím (hoặc -1 nếu không có). |
| `while key != -1:` | Xử lý hết phím đang chờ trong buffer. |

```python
            if ord('1') <= key <= ord('6'):
                selected = key - ord('1')
                print('Selected', selected + 1, JOINT_NAMES[selected])
```

| Dòng | Giải thích |
|------|------------|
| `ord('1')` | Mã số của ký tự `'1'`. |
| `selected = key - ord('1')` | Phím 1→0, phím 2→1, … |
| `print(...)` | Báo khớp đang chọn. |

```python
            elif key in (ord('z'), ord('Z')):
                target[selected] -= 0.02
                apply()
            elif key in (ord('x'), ord('X')):
                target[selected] += 0.02
                apply()
```

| Dòng | Giải thích |
|------|------------|
| `key in (ord('z'), ord('Z'))` | Nhận cả z thường và Z hoa. |
| `target[selected] -= 0.02` | Giảm góc 0.02 rad. |
| `target[selected] += 0.02` | Tăng góc 0.02 rad. |
| `apply()` | Gửi lệnh mới ngay. |

```python
            elif key in (ord('h'), ord('H')):
                target = list(HOME)
                apply()
            elif key in (ord('a'), ord('A')):
                target = list(APPROACH)
                apply()
            elif key in (ord('i'), ord('I')):
                target = list(INSERT)
                apply()
            elif key in (ord('r'), ord('R')):
                target = list(APPROACH)
                apply()
            key = kb.getKey()
```

| Dòng | Giải thích |
|------|------------|
| `H` | Về HOME. |
| `A` | Tới APPROACH. |
| `I` | Tới INSERT. |
| `R` | Retract = về lại APPROACH. |
| `key = kb.getKey()` | Đọc phím tiếp theo trong buffer. |

```python
if __name__ == '__main__':
    main()
```

| Dòng | Giải thích |
|------|------------|
| `if __name__ == '__main__':` | Chỉ chạy `main()` khi file được chạy trực tiếp. |

---

## Ngày 3 — Bước 2: Đổi controller trong `.wbt`

Trong `ur3_needle.wbt`:

```text
controller "ur3_needle_native"
```

| Dòng | Giải thích |
|------|------------|
| `controller "ur3_needle_native"` | Webots tìm folder `controllers/ur3_needle_native/` và chạy file `.py` cùng tên. |

Chạy:

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws/src/ur3_needle_sim
webots worlds/ur3_needle.wbt
```

| Dòng | Giải thích |
|------|------------|
| `cd .../ur3_needle_sim` | **Bắt buộc** đứng ở gốc project Webots (cha của `controllers/` và `worlds/`). |
| `webots worlds/ur3_needle.wbt` | Mở world tương đối. |

Sau khi tune xong, đổi lại:

```text
controller "<extern>"
```

---

## Ngày 3 — Bước 3: Viết `config/holes.yaml` (bảng 9 lỗ)

Tạo file `config/holes.yaml` (dùng cho cả Webots controller lẫn ROS sau này):

```yaml
phantom:
  center: [0.36, 0.0, 0.50]
  top_z: 0.55
holes:
  - {id: 0, x: 0.26, y: -0.10, z: 0.55}
  - {id: 1, x: 0.36, y: -0.10, z: 0.55}
  - {id: 2, x: 0.46, y: -0.10, z: 0.55}
  - {id: 3, x: 0.26, y:  0.00, z: 0.55}
  - {id: 4, x: 0.36, y:  0.00, z: 0.55}   # lỗ giữa — tune pose tại đây
  - {id: 5, x: 0.46, y:  0.00, z: 0.55}
  - {id: 6, x: 0.26, y:  0.10, z: 0.55}
  - {id: 7, x: 0.36, y:  0.10, z: 0.55}
  - {id: 8, x: 0.46, y:  0.10, z: 0.55}
click:
  max_distance_m: 0.025   # click xa hơn → bỏ qua
```

| Dòng | Giải thích |
|------|------------|
| `top_z: 0.55` | Mặt trên phantom (tâm z 0.50 + nửa dày 0.05). |
| `id: 4` | Lỗ giữa; mọi pose gốc tune quanh lỗ này. |
| `max_distance_m` | Bán kính chấp nhận click quanh tâm lỗ. |

## Ngày 3 — Bước 4: Viết `config/needle_params.yaml`

```yaml
/**:
  ros__parameters:
    insertion:
      depth_m: 0.025
      hold_sec: 1.5
    poses:
      home: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
      # Pose cho lỗ giữa (id=4). Các lỗ khác: đổi shoulder_pan ≈ atan2(y,x)
      approach_center: [0.0, -1.20, 1.40, -1.70, -1.57, 0.0]
      insert_center: [0.0, -1.05, 1.55, -1.90, -1.57, 0.0]
    trajectory:
      home_sec: 3.0
      approach_sec: 4.0
      insert_sec: 3.0
      retract_sec: 3.0
```

| Dòng | Giải thích |
|------|------------|
| `/**:` | Áp dụng param cho mọi node nạp file này. |
| `ros__parameters:` | Khối tham số ROS 2. |
| `insertion:` | Nhóm thông số sâu kim. |
| `depth_m: 0.025` | Độ sâu 0.025 m = 25 mm. |
| `hold_sec: 1.5` | Giữ 1.5 giây. |
| `poses:` | Nhóm tư thế. |
| `home: [...]` | 6 số HOME (thay bằng số bạn tune). |
| `approach_center: [...]` | APPROACH lỗ giữa — thay bằng số bạn tune. |
| `insert_center: [...]` | INSERT lỗ giữa — thay bằng số bạn tune. |
| `trajectory:` | Thời gian đi từng đoạn. |
| `home_sec: 3.0` | Đi về home trong 3 giây. |
| `approach_sec: 4.0` | Đi approach trong 4 giây. |
| `insert_sec: 3.0` | Đâm trong 3 giây. |
| `retract_sec: 3.0` | Rút trong 3 giây. |

## Ngày 3 — Bước 5: Công thức pose từng lỗ (copy vào controller)

```python
import math

def joints_for_hole(center_joints, hx, hy):
    """Giữ lift/elbow/wrist của lỗ giữa; chỉ đổi shoulder_pan hướng về lỗ."""
    q = list(center_joints)
    q[0] = math.atan2(hy, hx)   # shoulder_pan
    return q
```

| Dòng | Giải thích |
|------|------------|
| `list(center_joints)` | Copy 6 góc của pose giữa. |
| `atan2(hy, hx)` | Góc phương vị trong mặt phẳng XY từ base tới lỗ. |
| `q[0] = ...` | Khớp pan = khớp đầu tiên của UR. |

Thử nhanh: sau khi tune lỗ 4, gọi `joints_for_hole(APPROACH_CENTER, 0.42, 0.06)` (lỗ 8) và gửi xuống motor — tip phải lệch về góc lưới.

**Xong Ngày 3 khi:** có HOME + approach/insert lỗ giữa + `holes.yaml` + hiểu công thức pan.

---

# NGÀY 4 — Click chuột chọn lỗ (Webots Supervisor) + (tuỳ chọn) ROS

### Mục tiêu chính
**Click trái** gần 1/9 lỗ trong Webots → arm sâu kim đúng lỗ đó.

### Mục tiêu phụ (tuỳ chọn cùng ngày / Ngày 5)
Viết `needle_insert_node` + `depth_monitor` + `demo.launch.py` (ROS) nếu bạn muốn start bằng topic.

### Checklist Ngày 4
- [x] Tạo `controllers/ur3_click_insert/ur3_click_insert.py`
- [x] Đọc Mouse 3D + chọn `hole_id` gần nhất (`holes.py`)
- [x] Chu trình: HOME → ABOVE → DOWN (−Z) → Retract → HOME
- [x] Port **clone_backup IK TEST** (flange DH; **không** tip IK / camera)
- [x] `BASE_X_SIGN=BASE_Y_SIGN=-1`, unwrap shortest path (tránh xoay 1 vòng)
- [x] Đặt `controller "ur3_click_insert"` + `supervisor TRUE` trong world **click**
- [x] Thử click ≥ 3 lỗ khác nhau
- [x] ROS `./run_ros_demo.sh` trên oldest package `ur3_needle.wbt` (`<extern>`) — insert → `done`

---

## Ngày 4 — Bước 0 (quan trọng): Controller `ur3_click_insert`

### Cấu trúc thư mục

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
mkdir -p controllers/ur3_click_insert
# tạo file: controllers/ur3_click_insert/ur3_click_insert.py
```

Webots bắt buộc: tên folder = tên file `.py` = tên `controller` trong `.wbt`.

### Khung logic (bạn tự gõ; đây là bản đồ)

```python
from controller import Supervisor, Mouse
import math

# 1) danh sách 9 lỗ world (copy từ holes.yaml)
HOLES = [
    (0.26, -0.10, 0.55), (0.36, -0.10, 0.55), (0.46, -0.10, 0.55),
    (0.26,  0.00, 0.55), (0.36,  0.00, 0.55), (0.46,  0.00, 0.55),
    (0.26,  0.10, 0.55), (0.36,  0.10, 0.55), (0.46,  0.10, 0.55),
]
MAX_CLICK_DIST = 0.025

HOME = [...]              # số bạn tune Ngày 3
APPROACH_CENTER = [...]
INSERT_CENTER = [...]

def nearest_hole(x, y):
    best_i, best_d = None, 1e9
    for i, (hx, hy, hz) in enumerate(HOLES):
        d = math.hypot(x - hx, y - hy)
        if d < best_d:
            best_i, best_d = i, d
    if best_d <= MAX_CLICK_DIST:
        return best_i
    return None

def joints_for_hole(center_joints, hx, hy):
    q = list(center_joints)
    q[0] = math.atan2(hy, hx)
    return q
```

| Mảnh | Giải thích |
|------|------------|
| `Supervisor` | Controller đặc biệt: đọc chuột 3D, điều khiển robot. |
| `HOLES` | 9 tâm lỗ world — phải khớp `.wbt`. |
| `nearest_hole` | Click → id lỗ hoặc `None`. |
| `joints_for_hole` | Pose giữa + pan theo lỗ. |

### Vòng lặp Mouse + FSM (ý chính)

```python
robot = Supervisor()
dt = int(robot.getBasicTimeStep())
mouse = robot.getMouse()
mouse.enable(dt)
mouse.enable3dPosition()

phase = 'idle'          # idle|move_home|approach|insert|hold|retract|done
hole_id = None
# ... lấy 6 motor arm giống ur3_grasp_home ...

prev_left = False
while robot.step(dt) != -1:
    m = mouse.getState()
    # cạnh lên nút trái (tránh spam khi giữ chuột)
    clicked = m.left and not prev_left
    prev_left = m.left

    if phase in ('idle', 'done') and clicked:
        # u,v = toạ độ chuẩn hoá trên cửa sổ 3D; x,y,z = điểm pick thế giới
        if m.x == m.x:  # True khi không phải NaN
            hid = nearest_hole(m.x, m.y)
            if hid is not None:
                hole_id = hid
                phase = 'move_home'
                print('Click hole', hole_id, HOLES[hole_id], 'at', m.x, m.y, m.z)

    # mỗi phase: setPosition tới target; khi gần tới → sang phase sau
    # move_home → approach(hole) → insert(hole) → hold (đếm step) → retract → home → idle
```

| Mảnh | Giải thích |
|------|------------|
| `mouse.enable3dPosition()` | Bật picking 3D; không bật thì `m.x/y/z` = NaN. |
| `m.u`, `m.v` | Vị trí chuột trên cửa sổ (0→1). |
| `m.x`, `m.y`, `m.z` | Điểm 3D thế giới dưới con trỏ (mét). |
| `m.left` | Nút trái đang nhấn. |
| `clicked = m.left and not prev_left` | Chỉ lấy **lúc vừa nhấn**, không lấy lúc giữ. |
| `nearest_hole(m.x, m.y)` | Map điểm click → lỗ 0..8. |
| `phase = 'move_home'` | Bắt đầu chu trình sâu kim. |

> **Mẹo:** mỗi click in `print(m.x, m.y, m.z)`. Nếu toàn `nan` → quên `enable3dPosition()` hoặc click ra ngoài vật thể.  
> **Lưu ý UI Webots:** kéo chuột trái thường xoay camera — hãy **click nhanh** (nhấn-thả) khi phase idle; hoặc giữ phím tắt theo hướng dẫn Webots nếu camera bị xoay.

### Đổi `.wbt`

Trong khối `UR3e {`:

```text
controller "ur3_click_insert"
supervisor TRUE
```

Chạy từ gốc workspace (cha của `controllers/`):

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
webots worlds/ur3_needle.wbt
```

Bấm **Play** → click gần một chấm đen → xem console in `Click hole k` và arm chạy.

**Xong phần click khi:** click 3 lỗ khác nhau, mỗi lần kim hướng đúng vùng lỗ đó rồi rút về home.

---

## Ngày 4 — Phần ROS (đã verified)

> Hướng dẫn đầy đủ (chạy / đổi lỗ / điều khiển): mục **「ROS 2 — cách chạy, đổi lỗ, điều khiển」** ở đầu file.  
> ROS **không** chạy chung với `ur3_click_insert`. World oldest `ur3_needle.wbt` (`<extern>`).  
> Terminal 1: `./run_ros_demo.sh` — Terminal 2: `hole_id` → `start` → `echo /needle/phase`.  
> Insert dùng **`z_path` thẳng Z** (17 pts). Kiểm tra `ros2 topic info /needle/start` có Subscription count: 1.

---

## Ngày 4 — Bước 1: `ur3_needle_sim/depth_monitor.py` (từng dòng)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
```

| Dòng | Giải thích |
|------|------------|
| `import rclpy` | Thư viện ROS 2 Python. |
| `from rclpy.node import Node` | Class gốc để tạo node. |
| `from std_msgs.msg import Float32, String` | Kiểu tin: số thực 32-bit và chuỗi. |

```python
class DepthMonitor(Node):
    def __init__(self):
        super().__init__('depth_monitor')
```

| Dòng | Giải thích |
|------|------------|
| `class DepthMonitor(Node):` | Tạo class node kế thừa `Node`. |
| `def __init__(self):` | Hàm khởi tạo. |
| `super().__init__('depth_monitor')` | Đặt tên node là `depth_monitor`. |

```python
        self.declare_parameter('insertion.depth_m', 0.025)
        self.max_depth = float(self.get_parameter('insertion.depth_m').value)
```

| Dòng | Giải thích |
|------|------------|
| `declare_parameter(...)` | Khai báo param + giá trị mặc định. |
| `get_parameter(...).value` | Đọc giá trị param. |
| `float(...)` | Đảm bảo kiểu số thực. |

```python
        self.phase = 'idle'
        self.cmd = 0.0
```

| Dòng | Giải thích |
|------|------------|
| `self.phase = 'idle'` | Biến nhớ phase hiện tại. |
| `self.cmd = 0.0` | Biến nhớ độ sâu được lệnh. |

```python
        self.pub = self.create_publisher(Float32, '/needle/depth', 10)
        self.create_subscription(String, '/needle/phase', self.on_phase, 10)
        self.create_subscription(Float32, '/needle/depth_command', self.on_cmd, 10)
        self.create_timer(0.05, self.on_timer)
```

| Dòng | Giải thích |
|------|------------|
| `create_publisher(Float32, '/needle/depth', 10)` | Tạo kênh gửi `/needle/depth`, queue size 10. |
| `create_subscription(String, '/needle/phase', self.on_phase, 10)` | Mỗi khi có tin phase → gọi `on_phase`. |
| `create_subscription(..., self.on_cmd, ...)` | Nhận lệnh độ sâu. |
| `create_timer(0.05, self.on_timer)` | Mỗi 0.05 s gọi `on_timer` (20 Hz). |

```python
    def on_phase(self, msg: String):
        self.phase = msg.data

    def on_cmd(self, msg: Float32):
        self.cmd = float(msg.data)
```

| Dòng | Giải thích |
|------|------------|
| `msg.data` | Nội dung thật của tin nhắn (`String`/`Float32`). |

```python
    def on_timer(self):
        depth = 0.0
        if self.phase in ('insert', 'hold'):
            depth = max(0.0, min(self.cmd, self.max_depth))
        elif self.phase == 'retract':
            depth = max(0.0, self.cmd)
        out = Float32()
        out.data = float(depth)
        self.pub.publish(out)
```

| Dòng | Giải thích |
|------|------------|
| `depth = 0.0` | Mặc định chưa đâm. |
| `if self.phase in ('insert', 'hold'):` | Chỉ lúc đâm/giữ mới hiện độ sâu. |
| `min(self.cmd, self.max_depth)` | Không cho vượt setpoint. |
| `max(0.0, ...)` | Không cho âm. |
| `out = Float32()` | Tạo tin nhắn. |
| `out.data = ...` | Ghi số vào tin. |
| `self.pub.publish(out)` | Gửi ra `/needle/depth`. |

```python
def main():
    rclpy.init()
    node = DepthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
```

| Dòng | Giải thích |
|------|------------|
| `rclpy.init()` | Khởi động ROS client. |
| `node = DepthMonitor()` | Tạo node. |
| `rclpy.spin(node)` | Chạy mãi, chờ callback. |
| `except KeyboardInterrupt` | Bắt `Ctrl+C`. |
| `node.destroy_node()` | Hủy node sạch sẽ. |
| `rclpy.shutdown()` | Tắt ROS client. |

---

## Ngày 4 — Bước 2: `needle_insert_node.py` (từng phần, từng dòng)

### Import

```python
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool, Float32, String
```

| Dòng | Giải thích |
|------|------------|
| `Enum, auto` | Tạo liệt kê phase có tên. |
| `ActionClient` | Client gọi action (gửi quỹ đạo, chờ kết quả). |
| `Duration as RclDuration` | Duration của rclpy (tính thời gian hold). |
| `GoalStatus` | Biết goal thành công/thất bại. |
| `Duration` (builtin) | Duration gắn vào điểm quỹ đạo. |
| `FollowJointTrajectory` | Action chuẩn điều khiển khớp. |
| `JointTrajectoryPoint` | Một điểm trong quỹ đạo (vị trí + thời điểm). |
| `Bool, Float32, String` | Tin nhắn đơn giản. |

### Phase enum

```python
class Phase(Enum):
    IDLE = auto()
    MOVE_HOME = auto()
    MOVE_APPROACH = auto()
    INSERT = auto()
    HOLD = auto()
    RETRACT = auto()
    DONE = auto()
    ABORTED = auto()
```

| Dòng | Giải thích |
|------|------------|
| `class Phase(Enum):` | Tập hợp trạng thái có tên. |
| `IDLE = auto()` | Tự gán giá trị; bạn chỉ cần nhớ tên. |
| Các tên còn lại | Đúng kịch bản sâu kim. |

### Trong `__init__`: declare + đọc param

```python
        self.declare_parameter('insertion.depth_m', 0.025)
        self.depth_m = float(self.get_parameter('insertion.depth_m').value)
```

| Dòng | Giải thích |
|------|------------|
| `declare_parameter` | Cho phép yaml/launch ghi đè. |
| `self.depth_m = ...` | Lưu vào biến instance. |

(Tương tự với `hold_sec`, `poses.*`, `trajectory.*`)

### Pub/Sub + ActionClient

```python
        self.phase_pub = self.create_publisher(String, '/needle/phase', 10)
        self.depth_cmd_pub = self.create_publisher(Float32, '/needle/depth_command', 10)
        self.create_subscription(Bool, '/needle/start', self.on_start, 10)
        self.create_subscription(Bool, '/needle/abort', self.on_abort, 10)
        self.client = ActionClient(self, FollowJointTrajectory, self.action_name)
        self.create_timer(0.1, self.on_timer)
```

| Dòng | Giải thích |
|------|------------|
| publish `/needle/phase` | Báo đang ở bước nào. |
| publish `/needle/depth_command` | Ra lệnh độ sâu cho depth_monitor. |
| subscribe `/needle/start` | Nghe nút bắt đầu. |
| subscribe `/needle/abort` | Nghe nút dừng. |
| `ActionClient(...)` | Kết nối tới trajectory controller. |
| timer 0.1 | Mỗi 0.1 s kiểm tra HOLD + publish phase. |

### `set_phase` / `on_start` / `on_abort`

```python
    def set_phase(self, phase: Phase):
        self.phase = phase
        msg = String()
        msg.data = phase.name.lower()
        self.phase_pub.publish(msg)
```

| Dòng | Giải thích |
|------|------------|
| `self.phase = phase` | Đổi trạng thái nội bộ. |
| `phase.name.lower()` | `MOVE_HOME` → `"move_home"`. |
| `publish(msg)` | Gửi ra ngoài. |

```python
    def on_start(self, msg: Bool):
        if not msg.data:
            return
        if self.phase not in (Phase.IDLE, Phase.DONE, Phase.ABORTED):
            return
        self.returning_home = False
        self.set_phase(Phase.MOVE_HOME)
        self.send_joints(self.home, self.t_home)
```

| Dòng | Giải thích |
|------|------------|
| `if not msg.data: return` | Chỉ nhận `true`. |
| `if self.phase not in (...)` | Tránh start khi đang chạy dở. |
| `returning_home = False` | Đánh dấu đây là home đầu chu trình. |
| `send_joints(self.home, ...)` | Bắt đầu đi home. |

```python
    def on_abort(self, msg: Bool):
        if not msg.data:
            return
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.hold_until = None
        self.set_phase(Phase.ABORTED)
```

| Dòng | Giải thích |
|------|------------|
| `cancel_goal_async()` | Hủy quỹ đạo đang chạy. |
| `hold_until = None` | Hủy hẹn giờ HOLD. |
| `ABORTED` | Đánh dấu dừng. |

### `send_joints` + callback

```python
    def send_joints(self, positions, duration_sec: float):
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.set_phase(Phase.ABORTED)
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in positions]
        whole = int(duration_sec)
        nanos = int((duration_sec - whole) * 1e9)
        point.time_from_start = Duration(sec=whole, nanosec=nanos)
        goal.trajectory.points = [point]
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
```

| Dòng | Giải thích |
|------|------------|
| `wait_for_server(2.0)` | Đợi action server tối đa 2 giây. |
| `FollowJointTrajectory.Goal()` | Tạo goal trống. |
| `joint_names = ...` | Goal áp dụng cho 6 khớp này. |
| `point.positions = ...` | Góc đích. |
| `whole` / `nanos` | Tách giây và nano-giây. |
| `time_from_start` | Thời điểm điểm này trong quỹ đạo. |
| `points = [point]` | Quỹ đạo 1 điểm (MVP đơn giản). |
| `send_goal_async` | Gửi không chặn vòng spin. |
| `add_done_callback` | Khi server trả lời accept/reject → gọi hàm. |

```python
    def on_goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.set_phase(Phase.ABORTED)
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)
```

| Dòng | Giải thích |
|------|------------|
| `future.result()` | Lấy goal handle. |
| `accepted` | Server có nhận goal không. |
| `get_result_async()` | Đợi chạy xong quỹ đạo. |

```python
    def on_goal_result(self, future):
        self.goal_handle = None
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            if self.phase != Phase.ABORTED:
                self.set_phase(Phase.ABORTED)
            return
        self.advance()
```

| Dòng | Giải thích |
|------|------------|
| `STATUS_SUCCEEDED` | Đi tới pose thành công. |
| `self.advance()` | Chuyển sang bước kế của máy trạng thái. |

### Máy trạng thái `advance`

```python
    def advance(self):
        if self.phase == Phase.MOVE_HOME:
            if self.returning_home:
                self.returning_home = False
                self.depth_cmd_pub.publish(Float32(data=0.0))
                self.set_phase(Phase.DONE)
                return
            self.set_phase(Phase.MOVE_APPROACH)
            self.send_joints(self.approach, self.t_approach)
            return
```

| Dòng | Giải thích |
|------|------------|
| `if returning_home` | Đây là home **cuối** chu trình → DONE. |
| `else` | Home đầu → tiếp APPROACH. |

```python
        if self.phase == Phase.MOVE_APPROACH:
            self.set_phase(Phase.INSERT)
            self.insert_started_at = self.get_clock().now()
            self.depth_cmd_pub.publish(Float32(data=float(self.depth_m)))
            self.send_joints(self.insert, self.t_insert)
            return
```

| Dòng | Giải thích |
|------|------------|
| `insert_started_at` | Mốc giờ để ước lượng depth tăng dần. |
| publish depth_m | Báo depth_monitor setpoint. |
| `send_joints(insert)` | Đi tới tư thế đâm. |

```python
        if self.phase == Phase.INSERT:
            self.set_phase(Phase.HOLD)
            self.hold_until = self.get_clock().now() + RclDuration(seconds=self.hold_sec)
            return
```

| Dòng | Giải thích |
|------|------------|
| `hold_until = now + hold_sec` | Hẹn giờ giữ kim. |

```python
        if self.phase == Phase.RETRACT:
            self.returning_home = True
            self.set_phase(Phase.MOVE_HOME)
            self.send_joints(self.home, self.t_home)
            return
```

| Dòng | Giải thích |
|------|------------|
| `returning_home = True` | Lần MOVE_HOME tới sẽ là kết thúc. |

### `on_timer`

```python
        if self.phase == Phase.HOLD and self.hold_until is not None:
            if self.get_clock().now() >= self.hold_until:
                self.hold_until = None
                self.set_phase(Phase.RETRACT)
                self.depth_cmd_pub.publish(Float32(data=0.0))
                self.send_joints(self.approach, self.t_retract)
```

| Dòng | Giải thích |
|------|------------|
| `now >= hold_until` | Hết giờ giữ. |
| publish 0.0 | Độ sâu về 0 khi rút. |
| `send_joints(approach)` | Rút về approach. |

```python
        if self.phase == Phase.INSERT and self.insert_started_at is not None:
            elapsed = (self.get_clock().now() - self.insert_started_at).nanoseconds / 1e9
            ratio = max(0.0, min(1.0, elapsed / max(self.t_insert, 1e-3)))
            self.depth_cmd_pub.publish(Float32(data=float(ratio * self.depth_m)))
```

| Dòng | Giải thích |
|------|------------|
| `elapsed` | Số giây đã đâm. |
| `ratio` | 0→1 theo tiến độ. |
| `ratio * depth_m` | Độ sâu ước lượng MVP. |

---

## Ngày 4 — Bước 3: `launch/demo.launch.py`

```python
sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
    launch_arguments={'use_sim_time': use_sim_time}.items(),
)
```

| Dòng | Giải thích |
|------|------------|
| `IncludeLaunchDescription` | Gọi lại `sim.launch.py` (không viết trùng). |
| `PythonLaunchDescriptionSource(...)` | Chỉ đường tới file launch cần include. |
| `launch_arguments=...` | Truyền tham số xuống sim. |

```python
insert_node = Node(
    package=PACKAGE,
    executable='needle_insert_node',
    output='screen',
    parameters=[params, {
        'use_sim_time': use_sim_time,
        'insertion.depth_m': depth_m,
    }],
)
```

| Dòng | Giải thích |
|------|------------|
| `executable='needle_insert_node'` | Trùng tên trong `setup.py` entry_points. |
| `parameters=[params, {...}]` | Nạp yaml + ghi đè depth từ launch arg. |

```python
depth_node = Node(
    package=PACKAGE,
    executable='depth_monitor',
    ...
)
```

| Dòng | Giải thích |
|------|------------|
| `executable='depth_monitor'` | Chạy node đo/hiển thị depth. |

---

## Ngày 4 — Bước 4: Lệnh demo (từng dòng)

```bash
colcon build --packages-select ur3_needle_sim
source install/setup.bash
ros2 launch ur3_needle_sim demo.launch.py
```

| Dòng | Giải thích |
|------|------------|
| `colcon build ...` | Build lại vì vừa thêm Python/launch. |
| `source install/setup.bash` | Nạp bản mới. |
| `ros2 launch ... demo.launch.py` | Mở sim + 2 node. |

```bash
ros2 topic pub --once /needle/start std_msgs/msg/Bool "{data: true}"
```

| Mảnh | Giải thích |
|------|------------|
| `ros2 topic pub` | Gửi 1 tin vào topic. |
| `--once` | Gửi một lần rồi thoát. |
| `/needle/start` | Topic start. |
| `std_msgs/msg/Bool` | Kiểu tin. |
| `"{data: true}"` | Nội dung: true. |

```bash
ros2 topic echo /needle/phase
ros2 topic echo /needle/depth
ros2 topic pub --once /needle/abort std_msgs/msg/Bool "{data: true}"
```

| Dòng | Giải thích |
|------|------------|
| `echo /needle/phase` | In liên tục phase. `Ctrl+C` để dừng. |
| `echo /needle/depth` | In độ sâu. |
| `pub ... /needle/abort ... true` | Dừng khẩn cấp. |

**Xong Ngày 4 khi:** full cycle chạy từ code bạn tự viết.

---

# NGÀY 5 — README + video + báo cáo (kèm demo click lỗ)

## Ngày 5 — Bước 1: Tự viết README

Mỗi mục README nên có:

| Mục | Viết gì |
|-----|---------|
| Giới thiệu | UR3e sâu kim; **click chuột chọn 1/9 lỗ** |
| Mở Webots click | `webots worlds/ur3_needle.wbt` + Play + click lỗ |
| Build ROS (nếu dùng) | 4–5 lệnh build |
| Run ROS | `ros2 launch ...` |
| Start ROS | `/needle/hole_id` rồi `/needle/start` |
| Bảng 9 lỗ | Copy từ `holes.yaml` |
| Topics | Bảng topic (nếu có ROS) |
| Đổi depth | `depth_m` / hằng trong controller |
| Lỗi thường gặp | Click không nhận, pan lệch, OpenGL… |

Ví dụ Webots:

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
webots worlds/ur3_needle.wbt
```

| Mảnh | Giải thích |
|------|------------|
| Đứng ở gốc `ur3_needle_ws` | Webots tìm được `controllers/ur3_click_insert/`. |
| Play rồi click | Supervisor chỉ chạy khi sim đang Play. |

Ví dụ ROS (tuỳ chọn):

```bash
ros2 launch ur3_needle_sim demo.launch.py depth_m:=0.03
```

| Mảnh | Giải thích |
|------|------------|
| `depth_m:=0.03` | Truyền launch argument: độ sâu 30 mm. |

## Ngày 5 — Bước 2: Video

Quay đủ:

1. Webots Play  
2. Click **lỗ giữa** → full cycle  
3. Click **một lỗ góc** → full cycle  
4. Click **một lỗ khác** → full cycle  
5. (Tuỳ chọn) ROS start + echo phase/depth  

## Ngày 5 — Bước 3: Báo cáo

Điền: `hole_id` đã thử, setpoint depth, quan sát tip có vào đúng vùng lỗ không, thời gian 1 cycle, file tự viết (`ur3_click_insert.py`, yaml…).

---

# Definition of Done

- [x] Tự tạo workspace/package
- [x] Tự viết world (UR3e + kim + phantom **9 lỗ**)
- [x] Có bảng tọa độ 9 lỗ (`holes.yaml` + `DEF HOLE_0..8` + `holes.py`)
- [x] Tune HOME + approach/insert **lỗ giữa**
- [x] Controller **click chuột** chọn lỗ (`ur3_click_insert`)
- [x] Click lỗ → full cycle HOME→ABOVE→DOWN(−Z)→Hold→Retract→HOME
- [x] ROS launch + 2 node + `/needle/hole_id` + `/needle/start` — verified trên oldest `<extern>` world
- [x] README + `TOM_TAT_DU_AN_UR3_SAU_KIM.md`

---

# Cập nhật sau Definition of Done (đã làm)

> Đồng bộ với code `ur3_click_insert` / `motion.py` hiện tại. Chi tiết thêm: `TOM_TAT_DU_AN_UR3_SAU_KIM.md`.

| # | Việc đã làm | Kết quả / ghi chú |
|---|-------------|-------------------|
| 1 | Bỏ **chấm đỏ** (Sphere tip) trên kim | Tip = đầu cylinder |
| 2 | Lỗ **song song trục Z** | Cylinder mặc định Z; không xoay giếng ngang |
| 3 | Giếng `height 0.08`, `radius 0.012` | Mouth z≈0.55 |
| 4 | `DEF HOLE_0` … `HOLE_8` | Supervisor `holes.measure_hole_centers` |
| 5 | Port **clone_backup IK TEST** | Supervisor mouth → pick targets → flange DH IK → ABOVE/DOWN |
| 6 | **Không** tip-Jacobian IK / **không** camera | Target từ HOLE; calib TCP→tip chỉ cho offset Z |
| 7 | `BASE_X_SIGN=BASE_Y_SIGN=-1` | Sửa lệch đối X/Y (Rz π world↔DH) |
| 8 | Unwrap + bỏ SEED waypoint | Tránh xoay thừa ~360°; pan Δ ≤ ~180° |
| 9 | `INSERT_DEPTH` chỉnh dần | Hiện **+0.02 m** (tip dưới miệng 2 cm) |
| 10 | Mirror controller | `controllers/` ↔ `src/ur3_needle_sim/controllers/` |
| 11 | ROS 2 demo verified | Oldest `<extern>` world; run/change-hole/control docs; `z_path` straight-Z |
| 12 | Straight-Z insert/retract | Click `goto_straight_z`; ROS multi-point `z_path` (17 pts) |

### Hằng số motion hiện tại

```text
HOVER_HEIGHT        = 0.04 m   (log / legacy)
INSERT_DEPTH        = +0.02 m  → PICK_CLEARANCE = -0.02 (tip dưới miệng 2 cm)
ABOVE_CLEARANCE     = 0.16 m
BASE_X_SIGN         = -1
BASE_Y_SIGN         = -1
HOME                = [0, -1.57, 1.57, -1.57, -1.57, 0]
SEED_DOWN           = [0.3246, -0.7505, 1.8532, -2.0386, -3.1544, 2.1924]
FINGER_CLOSED       = 0.75
TIP_LOCAL_NEEDLE    = [0, 0, -0.18]
motor_velocity      = 1.5 rad/s
z_path steps        = 16 (+ endpoints → 17 pts)
```

### Việc còn mở (tuỳ chọn)

| Việc | Ghi chú |
|------|---------|
| Re-bake sau mỗi đổi `INSERT_DEPTH` / base signs | `scripts/bake_hole_poses_from_click_ik.py` rồi `./run_ros_demo.sh` |
| Phantom luôn `(0.36, 0, 0.5)` trên **mọi** bản `.wbt` | Tránh Webots save drift |
| Soft tissue / force / MoveIt | Ngoài phạm vi MVP |

---

# Phụ lục — Rebuild sạch (từng dòng)

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
rm -rf build install log
source /opt/ros/jazzy/setup.bash
export WEBOTS_HOME=/usr/local/webots
colcon build --packages-select ur3_needle_sim
source install/setup.bash
```

| Dòng | Giải thích |
|------|------------|
| `rm -rf build install log` | Xóa bản build cũ (`-r` đệ quy, `-f` không hỏi). |
| Các dòng còn lại | Build và nạp lại như Ngày 1. |

---

## Cách học với file này

1. Chỉ làm **1 bước** mỗi lần  
2. Đọc bảng “giải thích từng dòng” **trước khi gõ**  
3. Gõ xong chạy thử  
4. Đánh dấu checklist  

**Trạng thái:** checklist Ngày 1–5 và Definition of Done đã **xong** (gồm ROS 2 verified).  
Phần tutorial Ngày 1–5 phía dưới vẫn giữ để ôn / làm lại từ đầu (một số đoạn cũ hơn code hiện tại).  
**Làm / chạy dự án hôm nay:** đọc **「Workflow / How-to」** ở đầu file trước.  
**Code đang chạy:** **「Giải thích code dự án hiện tại」** + **「ROS 2 — cách chạy, đổi lỗ, điều khiển」**.

**Chạy demo click:**

```bash
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
webots worlds/ur3_needle.wbt
```

Play → click lỗ đen trên phantom (insert thẳng Z).

**Chạy + điều khiển ROS 2 (chi tiết: mục “ROS 2 — cách chạy, đổi lỗ, điều khiển”):**

```bash
# Terminal 1
cd /workspace/share/ros2-workspace/trung_test/ur3_needle_ws
./run_ros_demo.sh

# Terminal 2
source /opt/ros/jazzy/setup.bash
source /workspace/share/ros2-workspace/trung_test/ur3_needle_ws/install/setup.bash
ros2 topic info /needle/start          # Subscription count: 1
ros2 topic pub --once /needle/hole_id std_msgs/msg/Int32 "{data: 8}"
ros2 topic pub --once /needle/start std_msgs/msg/Bool "{data: true}"
ros2 topic echo /needle/phase
```

Nếu kẹt (`Waiting for subscription`): Terminal 1 chưa chạy / demo đã tắt. Gửi **lỗi đỏ đầy đủ**.
