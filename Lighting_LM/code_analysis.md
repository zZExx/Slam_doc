## 整体架构与主流程

### **入口程序 `run_loc_online`**

1. **启动与配置**
   - 初始化 `glog`、解析 `--config`。
   - `rclcpp::init` 启动 ROS2 通信。
   - 构造 `LocSystem`，调用 `loc.Init(config_yaml)`。

2. **设置初始位姿并进入主循环**
   - `loc.SetInitPose(SE3())`：给定位姿原点（也可从外部重定位）。
   - `loc.Spin()`：ROS2 `spin(node_)`，进入消息回调循环。
   - 退出时 `rclcpp::shutdown()` 收尾。

---

## LocSystem：ROS 接入与感知入口

### **初始化 `LocSystem::Init`**

1. **核心定位模块创建**
   - 创建 `loc::Localization`，设置为 `online_mode_ = true`。
   - 读取 YAML：
     - 地图路径：`system.map_path`
     - 话题：`common.imu_topic / lidar_topic / livox_lidar_topic`。

2. **ROS2 节点与订阅**
   - 创建 `node_ = rclcpp::Node("lightning_slam")`。
   - 订阅 IMU：
     - 将 `sensor_msgs::Imu` 转为内部 `IMU` 结构。
     - 调用 `LocSystem::ProcessIMU(imu)`。
   - 订阅激光：
     - `PointCloud2` 与 Livox `CustomMsg`。
     - 统一用 `Timer::Evaluate` 计时，调用 `ProcessLidar(...)`。

3. **TF 广播与 Localization 初始化**
   - 若 `options_.pub_tf_`：
     - 创建 `tf_broadcaster_`。
     - 调用 `loc_->SetTFCallback(...)`，将 PGO 输出绑定到 TF 发布。
   - 调用 `loc_->Init(yaml_path, map_path)` 完成内部模块初始化。

### **感知数据入口**

- **IMU 回调**
  - `LocSystem::ProcessIMU`：
    - 若 `loc_started_` 为真，调用 `loc_->ProcessIMUMsg(imu)`。
- **激光回调**
  - `ProcessLidar(PointCloud2)` → `loc_->ProcessLidarMsg(cloud)`。
  - `ProcessLidar(LivoxMsg)` → `loc_->ProcessLivoxLidarMsg(cloud)`。

### **外部设置初始位姿**

- `LocSystem::SetInitPose(SE3 pose)`：
  - 打印位姿日志。
  - `loc_->SetExternalPose(q, t)` → 传给 LidarLoc 作为初始 pose。
  - 设置 `loc_started_ = true`，之后感知数据才真正参与定位。

---

## Localization：感知融合与后端主控

### **Localization::Init**

1. **LIO（激光–IMU 里程计前端）**
   - 构造 `LaserMapping lio_`，`is_in_slam_mode_ = false`（定位模式）。
   - `lio_->Init(yaml)`：完成 IMU 标定、LIO 参数加载。

2. **LidarLoc（地图匹配定位）**
   - 读取 `lidar_loc.*` 配置：
     - 是否更新动态地图 `update_dynamic_cloud`。
     - 是否强制 2D 等。
   - 创建 `LidarLoc`，设置：
     - 地图选项 `map_option_.map_path_ = global_map_path`。
   - 若 `with_ui_`：
     - 创建 `PangolinWindow`，绑定到 LidarLoc 可视化。
   - `lidar_loc_->Init(yaml)`：完成地图索引加载、NDT 结构准备等。

3. **PGO（位姿图优化）**
   - 构造 `PGO`，关闭调试。
   - 设置跳帧策略：
     - `enable_lidar_loc_skip_ / lidar_loc_skip_num_`
     - `enable_lidar_odom_skip_ / lidar_odom_skip_num_`
   - 两个处理队列：
     - `lidar_odom_proc_cloud_`：LIO 前端线程入口。
     - `lidar_loc_proc_cloud_`：LidarLoc 匹配线程入口。
   - 若 `online_mode_`，启动两条处理线程。

4. **PGO 高频输出回调**
   - `pgo_->SetHighFrequencyGlobalOutputHandleFunction(...)`：
     - 统计定位 FPS。
     - 保存当前 `loc_result_`。
     - 若 `tf_callback_` 存在且结果有效：发布 TF。
     - 若 UI 存在：刷新导航状态与最近位姿。

5. **点云预处理器 PointCloudPreprocess**
   - `preprocess_ = new PointCloudPreprocess`。
   - 配置：
     - `blind`（盲区）
     - `time_scale`
     - `scan_line`
     - `point_filter_num`
     - `lidar_type`（AVIA/VELO32/OUST64）。
   - 后续所有点云先过 `preprocess_->Process(...)` 统一格式。

---

## 点云与 IMU 处理链

### **点云链路：Localization::ProcessLidarMsg / ProcessLivoxLidarMsg**

1. **预处理**
   - 加锁 `global_mutex_`。
   - 校验 `lidar_loc_ / lio_ / pgo_` 非空。
   - 构造 `CloudPtr laser_cloud`。
   - `preprocess_->Process(msg, laser_cloud)`：
     - 去畸变、滤波、格式转换。
   - 设置时间戳到 `laser_cloud->header.stamp`。

2. **分发到 LIO 前端线程**
   - 若 `online_mode_`：
     - `lidar_odom_proc_cloud_.AddMessage(laser_cloud)` → 交由线程 `Start()` 后的回调执行 `LidarOdomProcCloud`。
   - 否则离线模式直接调用 `LidarOdomProcCloud(laser_cloud)`。

### **IMU 链路：Localization::ProcessIMUMsg**

1. **输入与时间检查**
   - 加锁 `global_mutex_`。
   - 校验模块指针。
   - 检查 IMU 时间单调性（监控传感器异常）。

2. **送入 LIO**
   - `lio_->ProcessIMU(imu)` → 内部做 IMU 预积分、状态预测。
   - 从 LIO 中取出当前 DR（dead reckoning）状态 `dr_state = lio_->GetIMUState()`。
   - 若 `dr_state.pose_is_ok_`：
     - `lidar_loc_->ProcessDR(dr_state)`。
     - `pgo_->ProcessDR(dr_state)`。
   - 作用：
     - 即使没有新点云，也依靠 IMU 连续预测位姿，保证轨迹连续。

---

## LIO 前端：LidarOdomProcCloud

### **LidarOdomProcCloud(CloudPtr)**

1. **运行 LIO**
   - 若 `lio_` 为空直接返回。
   - `lio_->ProcessPointCloud2(cloud)`：推入 LIO。
   - `if (!lio_->Run()) return;`：LIO 一次迭代失败则不继续。

2. **里程计状态输出**
   - `auto lo_state = lio_->GetState()`：
     - **送入 LidarLoc**：`lidar_loc_->ProcessLO(lo_state)`。
     - **送入 PGO**：`pgo_->ProcessLidarOdom(lo_state)`。

3. **关键帧与去畸变点云**
   - `auto kf = lio_->GetKeyframe()`：
     - 若关键帧未更新，直接返回（可选 DR 处理留在注释里）。
   - 更新关键帧引用 `lio_kf_ = kf`。
   - `auto scan = lio_->GetScanUndist()`：得到去畸变点云。
   - 若 `online_mode_`：
     - `lidar_loc_proc_cloud_.AddMessage(scan)` → 交由 LidarLoc 线程 `LidarLocProcCloud`。
   - 否则直接调用 `LidarLocProcCloud(scan)`。

---

## LidarLoc：地图匹配与全局定位

### **地图与 NDT 结构**

1. **地图加载与策略**
   - `map_ = std::make_shared<TiledMap>(options_.map_option_)`。
   - 从磁盘加载地图索引，若有功能点则加载首个地图块并调用 `UpdateGlobalMap()`。
   - 根据配置 `maps.dyn_cloud_policy` 选择：
     - `SHORT / LONG / PERSISTENT` 等动态地图策略。
   - 读入动态云加载/保存策略。

2. **NDT / ICP 目标更新**
   - `UpdateGlobalMap()`：
     - 构建 NDT 对象 `pcl_ndt_`（细分辨率 1m，DIRECT7，4 iter）。
     - 若尚未初始化：
       - 构建粗 NDT `pcl_ndt_rough_`（5m 分辨率）用于大范围初始化。
     - 若启用 ICP 微调：
       - 对全局地图体素下采样后，设置为 ICP 目标。

3. **地图更新线程**
   - `update_map_thread_`：
     - 监控 `map_->MapUpdated() / DynamicMapUpdated()`。
     - 需要时调用 `UpdateGlobalMap()` 刷新 NDT/ICP 目标。
     - 更新 UI（静态/动态点云）。
     - 周期 sleep 10ms。

### **LidarLocProcCloud(CloudPtr)**

1. **处理去畸变扫描**
   - 调用 `lidar_loc_->ProcessCloud(scan_undist)` → 内部函数 `Align(...)`：
     - 利用 LO / DR / 自身历史外推构造多个初始位姿：
       - `guess_from_lo`、`guess_from_dr`、`guess_from_self`。
     - 基于这些初值在当前地图块上做 NDT 匹配（必要时再 ICP 微调）。
     - 处理停车状态 / 初始化 / 重定位等逻辑。
   - 得到 `LocalizationResult res = lidar_loc_->GetLocalizationResult()`。

2. **送入 PGO 与 UI / 状态**
   - `pgo_->ProcessLidarLoc(res)`：以本帧 LidarLoc 为观测，触发 PGO 图优化。
   - 若 UI 存在：
     - `ui_->UpdateScan(scan_undist, res.pose_)`。
   - 若注册了定位状态回调：
     - 发布 `loc_state`（`std_msgs::Int32`），代表定位状态码。

---

## PGO：多源融合、外推与平滑

### **多源输入**

- **DR（IMU / 车轮推算）**
  - 由 `Localization::ProcessIMUMsg` 调用 `pgo_->ProcessDR(dr_state)`。
  - 维护 `dr_pose_queue_`（高频）。

- **Lidar Odom（LIO）**
  - 由 `LidarOdomProcCloud` 调用 `pgo_->ProcessLidarOdom(lo_state)`。
  - 维护 `lidar_odom_pose_queue_`（中频）。

- **LidarLoc（地图匹配）**
  - 由 `LidarLocProcCloud` 调用 `pgo_->ProcessLidarLoc(res)`。
  - 每来一帧 LidarLoc：
    - 组装 `PGOFrame`，带上：
      - 时间戳、姿态、置信度、权重等。
    - `impl_->AddPGOFrame(frame)`，触发局部图优化。
    - 若时间条件满足，则 `PubResult()` 输出结果。

### **结果发布：PubResult + ExtrapolateLocResult + PoseSmoother**

1. **异常检测与标志位**
   - 判断 LidarLoc 置信度是否持续偏低：
     - 若连续 `> localization_unusual_thd_` 次，则置 `localization_unusual_tag_ = true`。
   - 跟踪最近一次 LidarLoc 时间戳。

2. **外推到当前时间**
   - 复制 `impl_->result_` 到局部 `result`。
   - 调用 `ExtrapolateLocResult(result)`：
     - 取当前最新时间 `latest_time = max(result.timestamp_, last DR, last LO)`。
     - 若 `latest_time` 与最后 DR 时间差 > `imu_interruption_time_thd_`：IMU 断流。
     - 若 DR 队列时间晚于当前结果：
       - 使用插值函数 `PoseInterp` 获取 DR 差分增量。
       - 把结果姿态外推到 DR 最新时间。
     - 若结果时间依然落后 `latest_time - 0.05`：
       - 用历史 LidarLoc 轨迹插值得到 `latest_time` 的姿态，避免长时间“卡帧”。

3. **轨迹平滑**
   - 将最新 DR 姿态推入 `PoseSmoother::PushDRPose`。
   - `smoother_->PushPose(result.pose_)`：
     - 用 DR 预测与当前姿态做插值平滑。
     - 根据平滑误差自适应调整 `smooth_factor_`。
     - 输出平滑后的 `result.pose_`。

4. **对外回调与缓存**
   - 将 `result` 通过 `high_freq_output_func_(result)` 回调给 Localization：
     - 后者再通过 TF 回调广播位姿，并刷新 UI。
   - 将输出姿态推入 `output_pose_queue_`，供后续插值使用。

---

## 对外输出与系统收尾

### **TF / UI / 状态输出**

- 在 `Localization::Init` 中：
  - 绑定 PGO 高频输出到：
    - TF 回调：供 `LocSystem` 内部 `tf_broadcaster_` 使用。
    - UI：更新导航状态与最近位姿。
- 在 `LidarLocProcCloud` 中：
  - 发布定位状态整形码，供监控或上层决策（如重定位触发）。

### **退出流程**

- `LocSystem` 析构：调用 `loc_->Finish()`：
  - `LidarLoc::Finish()`：保存动态地图等。
  - UI 线程退出。
  - 两个点云处理线程 `lidar_loc_proc_cloud_ / lidar_odom_proc_cloud_` 调用 `Quit()`。
- 上层 `main` 调用 `rclcpp::shutdown()`，干净退出。

---

## 总览

**Lightning-LM 在线定位/实时建图的完整链路**可以概括为：

> ROS2 采集 IMU/点云 → `LocSystem` 转内部格式 → `Localization` 中的点云预处理 → LIO 前端得到 LO/DR 和关键帧 → LidarLoc 在多源先验下做 NDT/ICP 地图匹配 → PGO 将 DR + LO + LidarLoc 融合优化并外推到当前时刻，再用 PoseSmoother 平滑 → 通过 TF/UI/状态话题对外高频发布位姿与地图。