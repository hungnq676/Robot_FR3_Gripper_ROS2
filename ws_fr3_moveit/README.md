# FAIRINO FR3 + custom gripper — ROS 2 Humble / MoveIt 2

Workspace này chuẩn hóa project cũ trong `Documents/lean_ROS`. Robot ở đây là FAIRINO FR3 sáu trục (`j1` đến `j6`), không phải Franka FR3.

Project cũ không bị sửa. `fairino_fr3_description` là nguồn mô tả robot duy nhất; Gazebo, `robot_state_publisher` và MoveIt đều đọc cùng Xacro này.

## Packages

- `fairino_fr3_description`: Xacro, visual/collision meshes và initial joint positions.
- `fairino_fr3_gazebo`: world, table, RGB-D camera, AprilTag, bridges và Gazebo controllers.
- `fairino_fr3_moveit_config`: SRDF, KDL, OMPL, MoveIt controllers và RViz.

## Build

```bash
cd /home/hung/ws_fr3_moveit
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
colcon build --symlink-install
source install/setup.bash
```

`GZ_VERSION=harmonic` là bắt buộc trên máy này vì ROS 2 Humble mặc định build
`gz_ros2_control` cho Gazebo Fortress, trong khi project dùng Gazebo Harmonic 8.
Workspace chứa source `gz_ros2_control` nhánh `humble` để tạo plugin đúng ABI.

Mỗi terminal mới đều cần:

```bash
source /opt/ros/humble/setup.bash
source /home/hung/ws_fr3_moveit/install/setup.bash
```

## Kiểm tra model độc lập

```bash
ros2 launch fairino_fr3_description display.launch.py
```

Lệnh này chỉ chạy `joint_state_publisher_gui`, `robot_state_publisher` và RViz; không cần Gazebo hoặc MoveIt.

## Mô phỏng — hai terminal

### Terminal 1 — toàn bộ Gazebo/ROS 2 control

```bash
ros2 launch fairino_fr3_gazebo simulation.launch.py
```

Launch này chạy Gazebo, `/clock`, robot description/TF, spawn robot, camera
bridge và ba controller. Không dùng delay cố định: robot spawner chờ service
của Gazebo, plugin chờ `robot_state_publisher`, và controller spawner chờ
`/controller_manager` tối đa 120 giây.

Trong mô phỏng, Xacro dùng `mount_to_world:=true`. Fixed joint
`world_to_fr3_mount` bắt `base_footprint` vào work cell tại
`xyz="1.20 0.72 1.00"`, `rpy="0 0 -1.57"`.

### Terminal 2 — MoveIt và RViz

Chờ terminal 1 báo cả ba controller `Configured and activated`, rồi chạy:

```bash
ros2 launch fairino_fr3_moveit_config moveit.launch.py
```

Tất cả controller, MoveIt và RViz dùng `use_sim_time=true`, lấy thời gian từ
cùng topic `/clock`.

### Kiểm tra hệ thống

Kiểm tra:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

Ba controller mong đợi ở trạng thái `active`:

```text
joint_state_broadcaster
arm_controller
gripper_controller
```

```bash
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/stereo/image_raw
ros2 topic echo /oak/rgb/camera_info --once
```

Camera mô phỏng là OAK-D Pro W (model `oak_d_pro_w`, mesh chính thức từ
`luxonis/depthai-ros`): RGB 1280×960, HFOV 95°/VFOV 72°, depth căn theo RGB,
clip 0.2–12 m, nhiễu 5 mm. Đặt 0.95 m trên mặt bàn, nghiêng 20° về phía robot.
Frame và topic giống driver `depthai-ros` (`oak-d-base-frame`,
`oak_rgb_camera_optical_frame`, `/oak/rgb/*`, `/oak/stereo/*`, `/oak/points`).
Driver thật publish depth dạng 16UC1 (mm), Gazebo dạng 32FC1 (m).

### Hiệu chuẩn vị trí camera bằng AprilTag

Mốc là tag36h11 id 0 (ô đen 0.12 m) đặt cố định ở vị trí đã biết; trong mô
phỏng tại world (0.45, 0.30), camera nhìn thấy và robot không với tới nên không
bị che. Node đo pose tag trong ảnh (PnP, tinh chỉnh độ nghiêng và khoảng cách
bằng mặt phẳng fit từ depth), suy ra `world -> oak-d-base-frame` và ghi vào
`~/.ros/fr3_oak_extrinsics.yaml`:

```bash
ros2 launch fairino_fr3_manipulation calibrate_camera.launch.py
```

Sau đó chạy lại `run_all.sh`: TF camera đọc file này (log báo "OAK extrinsics
from AprilTag calibration"). Chưa có file thì dùng pose danh định; muốn quay
về pose danh định thì xoá `~/.ros/fr3_oak_extrinsics.yaml`.
Dòng "vs current TF" so với TF đang publish — chỉ là sai số so với ground truth
khi TF đang là pose danh định (lần hiệu chuẩn đầu, hoặc xoá file trước).

Mô phỏng: lệch 5.9 mm / 0.39° so với pose thật; vị trí vật suy ra từ camera
lệch 1–4 mm trong vùng làm việc. Robot thật: đo pose tag so với robot (ví dụ
chạm TCP vào các góc tag), sửa `landmark_xyz/landmark_rpy` và `tag_size` (đo ô
đen bản in) trong `config/camera_calibration.yaml`.

Trong RViz:

1. Chọn planning group `arm`.
2. Chọn goal state hoặc kéo interactive marker.
3. Nhấn **Plan** trước.
4. Khi trajectory hợp lý mới nhấn **Execute**.

### Terminal 3 — demo vật tạm trong RViz

Sau khi hai terminal mô phỏng và MoveIt đã chạy ổn định:

```bash
ros2 launch fairino_fr3_manipulation attached_object_demo.launch.py
```

Demo thực hiện tuần tự: mở gripper, thêm khối `demo_box` tại `grasp_frame`,
attach khối vào TCP, đóng gripper, nâng lên 12 cm, detach và mở gripper. Mỗi
stage dừng 2 giây để quan sát trong RViz. Khối này chỉ là MoveIt collision
object; nó không được spawn trong Gazebo và không có vật lý tiếp xúc.

Kích thước, khoảng nâng và thời gian dừng được cấu hình tại:

```text
fairino_fr3_manipulation/config/attached_object_demo.yaml
```

### Manipulation Bước 1 — mô tả và hiển thị vật

Node Bước 1 chỉ publish mô tả và Marker của một box; chưa thêm box vào MoveIt
Planning Scene và không điều khiển robot:

```bash
ros2 launch fairino_fr3_manipulation object_visualization.launch.py
```

Kiểm tra dữ liệu nguồn của vật:

```bash
ros2 topic echo /fr3_demo/object_description --once
ros2 topic echo /fr3_demo/object_marker --once
```

Thông số nằm trong `fairino_fr3_manipulation/config/object.yaml`. Box mặc định
có kích thước `0.04 x 0.04 x 0.10 m`, pose trong `base_footprint` là
`[0.40, -0.15, 0.05]`; vì tâm cao 0.05 m nên đáy box nằm đúng trên Grid z=0.

### Manipulation Bước 2 — quản lý vật trong Planning Scene

Giữ node Bước 1 đang chạy, sau đó mở thêm terminal:

```bash
ros2 launch fairino_fr3_manipulation planning_scene_object.launch.py
```

Node Bước 2 chỉ lưu mô tả nhận từ `/fr3_demo/object_description`; nó không tự
thêm vật. Thêm và xoá collision object bằng service:

```bash
ros2 service call /fr3_demo/add_object std_srvs/srv/Trigger "{}"
ros2 service call /fr3_demo/remove_object std_srvs/srv/Trigger "{}"
```

Marker của Bước 1 vẫn hiển thị sau khi remove; điều này giúp phân biệt Marker
chỉ để nhìn với CollisionObject thực sự được MoveIt dùng để tránh va chạm.

Kiểm tra trực tiếp danh sách collision object trong MoveIt:

```bash
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
  "{components: {components: 8}}"
```

### Manipulation Bước 3 — sinh và kiểm tra ứng viên gắp

Giữ Bước 1, Bước 2 và MoveIt đang chạy, rồi mở thêm terminal:

```bash
ros2 launch fairino_fr3_manipulation grasp_candidates.launch.py
```

Node này sinh một phương án gắp từ trên xuống cho `grasp_frame`, cùng điểm
`pregrasp` cao hơn vật 0.20 m. Nó chỉ gọi `/compute_ik` để kiểm tra khả năng
vươn tới và publish marker; chưa lập quỹ đạo và tuyệt đối chưa làm robot chạy.

Kiểm tra kết quả:

```bash
ros2 topic echo /fr3_demo/grasp_pose --once
ros2 topic echo /fr3_demo/pregrasp_pose --once
ros2 topic echo /fr3_demo/grasp_ik_valid --once
ros2 topic echo /fr3_demo/pregrasp_ik_valid --once
ros2 topic echo /fr3_demo/grasp_markers --once
```

Màu xanh lá là điểm gắp, xanh dương là pregrasp, mũi tên vàng là hướng tiếp
cận; ba mũi tên đỏ/lục/lam biểu diễn các trục TCP. Các thông số nằm trong
`fairino_fr3_manipulation/config/grasp.yaml`. Sau khi thay đổi `object.yaml`,
khởi động lại node Bước 3 để tạo ứng viên mới.

### Manipulation Bước 4–7 — pick và mang vật

Giữ các node Bước 1–3, Gazebo và MoveIt đang chạy. Box phải được thêm vào
Planning Scene bằng `/fr3_demo/add_object`. Mở một terminal để chạy coordinator:

```bash
ros2 launch fairino_fr3_manipulation pick_and_carry.launch.py
```

Launch chỉ chuẩn bị node, robot chưa tự chuyển động. Mở terminal khác và gọi:

```bash
ros2 service call /fr3_demo/run_pick_and_carry std_srvs/srv/Trigger "{}"
```

Coordinator thực hiện tuần tự:

1. Mở gripper, lập kế hoạch và chạy tới `pregrasp`.
2. Cho phép riêng các link gripper tiếp xúc `demo_box`, rồi tiếp cận Cartesian.
3. Đóng gripper và attach box vào `grasp_frame`.
4. Nâng box 0.15 m và mang tới `place_prepose`.

Khi thành công, service trả `success=True`. Kết thúc Bước 7, box vẫn attached và
gripper vẫn đóng; không gọi lại service lần hai. Pose đích và các giới hạn nằm
trong `fairino_fr3_manipulation/config/pick_and_carry.yaml`. Bước 8 sau này sẽ
hạ box xuống, detach, đưa nó trở lại world collision object và mở gripper.

### Box vật lý Gazebo — bước 1–3

Luồng này thay thế `object_visualization.launch.py` và
`planning_scene_object.launch.py`: không chạy hai launch cũ cùng lúc để tránh
hai nguồn cùng publish `/fr3_demo/object_description`.

**Bước 1 — box động trong Gazebo:**

`simulation.launch.py` tự nạp `demo_box` trước khi spawn robot để plugin
DetachableJoint của gripper tìm thấy `demo_box::box_link`. Vì vậy không cần gọi
lệnh spawn trong phiên chạy bình thường. Chỉ dùng lệnh dưới đây nếu box đã bị
xóa thủ công khỏi Gazebo:

```bash
export GZ_VERSION=harmonic
ros2 launch fairino_fr3_gazebo spawn_demo_box.launch.py
```

Model `demo_box` có kích thước `0.04 x 0.04 x 0.10 m`, khối lượng 0.10 kg,
collision, inertia, ma sát và gravity. Tâm box được spawn tại world
`[1.05, 0.32, 1.05]`, tương ứng gần `[0.40, -0.15, 0.05]` trong
`base_footprint`; đáy box nằm trên mặt bàn world z=1.0 m.

**Bước 2 và 3 — đọc pose Gazebo, kiểm tra mặt bàn và đồng bộ MoveIt:**

```bash
export GZ_VERSION=harmonic
ros2 launch fairino_fr3_manipulation gazebo_object_sync.launch.py
```

Launch bridge `/model/demo_box/pose`, transform pose từ `world` sang
`base_footprint`, publish mô tả vật và tự áp dụng CollisionObject vào Planning
Scene. Kiểm tra:

```bash
ros2 topic echo /fr3_demo/gazebo_object_pose --once
ros2 topic echo /fr3_demo/object_description --once
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
  "{components: {components: 8}}"
```

Log `demo_box is on the table: world z=1.0500 m` xác nhận đáy box cao 0.05 m
nằm trên mặt bàn z=1.0 m. Khi box chưa attached, mọi thay đổi vị trí hoặc hướng
trong Gazebo được cập nhật sang Planning Scene. Khi MoveIt đã attach box, node
tạm dừng cập nhật world object để không phá trạng thái đang cầm.

Sau đó chạy `grasp_candidates.launch.py`; node này tiếp tục nhận đúng topic
`/fr3_demo/object_description` nhưng pose lúc này đến từ Gazebo thật.

`gazebo_object_sync.launch.py` cũng bridge ba topic attach vật lý:

```text
/fr3_demo/gazebo_attach
/fr3_demo/gazebo_detach
/fr3_demo/gazebo_attachment_state
```

Khi `pick_and_carry` đóng gripper, nó gửi attach sang Gazebo và phải nhận trạng
thái xác nhận trước khi attach CollisionObject trong MoveIt rồi nâng. Nếu bridge
hoặc DetachableJoint không hoạt động, chu trình dừng an toàn và không nâng vật.
Do plugin nằm trong model robot, phải khởi động lại Gazebo sau lần build đầu tiên.

### Chai nước và hộp carton trong world

`simulation.launch.py` cũng nạp hai model phục vụ bài toán gắp chai và đặt lên
hộp:

- `water_bottle`: model động, visual GLB có texture gốc cao 0.23 m, collision
  cylinder bán kính 0.035 m, khối lượng 0.25 kg; pose world
  `[1.25, 0.32, 1.115]`.
- `cardboard_box`: support tĩnh kích thước `0.24 x 0.18 x 0.16 m`; pose world
  `[0.75, 0.75, 1.08]`, yaw 1.0 rad để camera thấy rõ hai mặt, mặt trên tại
  world z=1.16 m. World dùng ambient và fill light sáng hơn nhưng không ghi đè
  material, nên visual vẫn giữ texture gốc trong GLB.

Mesh được đặt riêng trong `models/water_bottle/meshes/` và
`models/cardboard_box/meshes/` và texture PNG rời trong `materials/textures/`.
Collision dùng primitive thay vì mesh DAE chi tiết
để mô phỏng và planning ổn định. Hai model đã ở Gazebo world; bước perception
tiếp theo sẽ bridge pose chai và tạo các CollisionObject tương ứng trong MoveIt
Planning Scene.

### Nhận diện vật từ camera (lớp 1, hình học)

Node `tabletop_perception` (`fairino_fr3_manipulation`) chỉ biết trước mặt bàn;
mọi vật khác lấy từ camera:

1. depth (căn theo RGB, lọc median) → điểm 3D trong `world` qua TF đã hiệu chuẩn;
2. fit lại mặt phẳng bàn mỗi chu kỳ, giữ điểm cao 12 mm–40 cm trên mặt bàn;
3. bỏ điểm thuộc robot (mesh va chạm từ `/robot_description` đặt theo TF) và
   hình cầu quanh TCP (vật đang cầm);
4. chia cụm theo voxel 5 mm; mỗi cụm → hộp (hình chữ nhật đáy từ mặt trên) hoặc
   trụ (fit đường tròn), chiều cao, màu; `graspable` nếu bề rộng kẹp ≤ 62 mm;
5. vật mới phải thấy 3 lần liên tiếp; chỉ cập nhật khi robot đứng yên (lúc tay
   chạy depth và TF lệch nhau, mép tay bị nhận nhầm là vật).

Đầu ra: `/fr3_demo/perceived_objects` (`fairino_fr3_msgs/PerceivedObjectArray`),
marker `/fr3_demo/perceived_markers`, và vật cản `obj_N` trong planning scene.
Sai số đo trong mô phỏng: vị trí 1–5 mm, kích thước +1–5 mm.

`gazebo_object_sync` giờ chỉ thêm mặt bàn và giữ bridge attach Gazebo
(`sync_gazebo_object: true` để chép pose `demo_box` từ Gazebo như trước — chỉ để
debug; `pick_and_carry`/`grasp_candidates` cũ cần chế độ này).

Chọn vật để gắp (id hoặc màu + hình dạng), rồi gọi gắp:

```bash
ros2 topic pub -t 3 /fr3_demo/pick_target std_msgs/String "{data: 'green box'}"
ros2 service call /fr3_demo/run_mtc_pick std_srvs/srv/Trigger "{}"
```

MTC tự tính từ kích thước nhận diện: khe kẹp, độ cao TCP trên vật (lòng kẹp
không đè nóc, đầu ngón không chạm bàn, ngón ôm ≥ 15 mm), hướng kẹp (hộp kẹp
ngang cạnh ngắn). Trong mô phỏng `gripper.squeeze: -0.005` để ngón dừng cách vật
5 mm mỗi bên vì mối hàn Gazebo giữ vật; robot thật đặt số dương nhỏ. Mối hàn
Gazebo chỉ gắn cho model `demo_box`.

### Gripper một động cơ và cảm biến lực RFP602

- `joint_clamp_2` là mimic của `joint_clamp_1` (hệ số −1): một động cơ khép đều
  hai ngón. Chỉ `joint_clamp_1` nhận lệnh; `/joint_states` báo ngón kia là
  `joint_clamp_2_mimic` (tên do `gz_ros2_control` đặt), TF/MoveIt tự tính từ mimic.
- Collision ngón và thân gripper là hình hộp (mesh lõm làm bullet báo chạm sớm
  ~1 cm và giật).
- Mỗi ngón có contact sensor Gazebo; `fsr_emulator` giả lập RFP602 dưới tấm cứng
  phủ mặt ngón → `/fr3_demo/gripper/finger_forces` (`fairino_fr3_msgs/FingerForces`:
  lực N, ADC thô 12-bit, cờ chạm). Bo cảm biến thật publish cùng message này.
- MTC kẹp theo lực: khép 5 mm/s tới quá bề rộng đo 8 mm, dừng khi cả hai ngón
  ≥ 2 N; không đủ thì hủy, không nâng (`gripper.*` trong `mtc_pick_task.yaml`).
- `run_mtc_pick` đưa tay về tư thế `observe` (dựng đứng, không che camera), mở
  ngón đúng `open`, chờ 6 s cho camera cập nhật rồi mới plan. Robot khởi động
  ở tư thế này.
- Chọn vật: `ros2 topic pub -t 3 /fr3_demo/pick_target std_msgs/String "{data: 'white box'}"`.
  Màu có thể đổi theo ánh sáng (chai lúc "white" lúc "gray"); dùng id hoặc
  hình dạng nếu cần.

### Lệnh gắp bằng lời (Gemini Robotics-ER)

Node `gemini_target_selector` (tab 6 của `run_all.sh`). API key đặt trong
`~/.config/fr3_demo/gemini_api_key` (chmod 600) hoặc biến `GEMINI_API_KEY`;
không bao giờ được in ra log. Cần `pip install --user google-genai`.

```bash
ros2 topic pub --once /fr3_demo/pick_command std_msgs/String "{data: 'gắp cái chuột'}"
ros2 topic echo /fr3_demo/pick_status
```

Luồng: về tư thế `observe` → chiếu tâm từng vật `obj_N` lên ảnh RGB và đánh số
→ gửi Gemini ảnh + bảng mô tả hình học + câu lệnh → Gemini trả `{"id": N}` →
kiểm tra (số tồn tại, gắp được) → đặt `/fr3_demo/pick_target` và gọi
`/fr3_demo/run_mtc_pick`. Gemini chỉ chọn *vật nào*; vị trí luôn lấy từ depth.
Không có key, Gemini trả `null` (không rõ/mơ hồ) hoặc vật quá rộng → robot không
chạy. Ảnh đã đánh số: `/fr3_demo/gemini/annotated` và
`~/.ros/fr3_gemini/last_query.png`. `auto_pick: false` để chỉ chọn mà không gắp.
Ảnh camera được gửi lên máy chủ Google.

### MTC: plan toàn chuỗi rồi execute (luồng chính của `run_all.sh`)

Node `mtc_pick_task_planner` lập kế hoạch một lần cho cả chuỗi: mở kẹp, đi tới
pregrasp, tiếp cận thẳng xuống, khép kẹp, attach, nâng và mang vật. MTC thử các
góc kẹp (mỗi 90°, vì vật vuông) và chỉ trả lời giải khi **toàn bộ** chuỗi đều
không va chạm (bàn, chai nước, thùng carton). Robot không chuyển động khi plan.

Plan rồi execute trong một lần gọi (chỉ execute khi plan thành công):

```bash
ros2 service call /fr3_demo/run_mtc_pick std_srvs/srv/Trigger "{}"
```

Hoặc tách hai bước:

```bash
ros2 service call /fr3_demo/plan_pick_task std_srvs/srv/Trigger "{}"
ros2 service call /fr3_demo/execute_pick_task std_srvs/srv/Trigger "{}"
```

Execute chạy lần lượt từng đoạn của lời giải đã plan, chờ robot đứng yên giữa
các đoạn, và tại đoạn attach gửi `/fr3_demo/gazebo_attach` rồi chờ
`/fr3_demo/gazebo_attachment_state` = `attached`. Nếu Gazebo không xác nhận
trong `gazebo_attach_timeout` giây thì dừng, không nâng. Mỗi lời giải chỉ
execute một lần; muốn chạy lại phải plan lại.

Thông số trong `config/mtc_pick_task.yaml`: `grasp_tcp_clearance` (TCP cao hơn
tâm vật để lòng kẹp không đè nóc vật) và `gripper_grasp_positions` (khe kẹp =
0.070 − 2·|q| m, khép vừa bề rộng vật thay vì `closed`). Đổi vật thì chỉnh hai
giá trị này.

## MoveIt với fake hardware, không chạy Gazebo

Terminal 1:

```bash
ros2 launch fairino_fr3_moveit_config fake_hardware.launch.py
```

Terminal 2:

```bash
ros2 launch fairino_fr3_moveit_config move_group.launch.py use_sim_time:=false
```

Terminal 3:

```bash
ros2 launch fairino_fr3_moveit_config moveit_rviz.launch.py use_sim_time:=false
```

## Gripper semantic

- `custom_gripper`: MoveIt end effector.
- `grasp_frame`: TCP/IK frame.
- `open`: `joint_clamp_1=-0.001`, `joint_clamp_2=0.001`
  (khoảng hở xấp xỉ 68 mm và tránh sai số tại mechanical limit).
- `closed`: `joint_clamp_1=-0.03`, `joint_clamp_2=0.03`
  (khoảng hở xấp xỉ 10 mm).

Hai finger hiện dùng `FollowJointTrajectory` và được điều khiển độc lập để giữ nguyên hành vi mô phỏng cũ. Chỉ nên chuyển sang mimic joint hoặc `GripperCommand` sau khi xác nhận cơ cấu và driver phần cứng thật.

## Lưu ý

- Collision meshes hiện là bản sao có tổ chức của CAD meshes cũ. Cấu trúc đã tách `visual/` và `collision/`, nhưng nên tạo convex/decimated collision meshes trước khi tối ưu planning.
- World dùng bàn primitive vì model bàn cũ trỏ tới một STL không tồn tại trong `lean_ROS`.
- Workspace này mới nhắm đến fake hardware và Gazebo Sim. Driver cho robot FAIRINO thật chưa được giả định hoặc triển khai.
- Với robot thật, giữ `mount_to_world:=false`. Transform `world -> base_footprint`
  phải được publish từ kết quả calibration vị trí chân robot trong cell; fixed
  joint của Gazebo không đại diện cho kết nối phần cứng hoặc driver thật.
