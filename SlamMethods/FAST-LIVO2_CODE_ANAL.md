## 1. ESIKF顺序更新框架（解决LiDAR和视觉测量维度不匹配）

**位置：** `src/LIVMapper.cpp`

```262:274:src/LIVMapper.cpp
void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}
```

在 `sync_packages` 中，先处理LIO，再处理VIO，实现顺序更新：

```924:1026:src/LIVMapper.cpp
  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // ... LIO处理 ...
    case LIO:
    {
      // ... VIO处理 ...
      meas.lio_vio_flg = VIO;
```

## 2. 使用并细化LiDAR点的平面先验

**位置：** `src/voxel_map.cpp` 和 `src/vio.cpp`

在体素地图中使用平面法向量：

```713:755:src/voxel_map.cpp
void VoxelMapManager::build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess,
                                            double &prob, PointToPlane &single_ptpl)
{
  // ...
  if (current_octo->plane_ptr_->is_plane_)
  {
    VoxelPlane &plane = *current_octo->plane_ptr_;
    // ... 计算平面距离和法向量 ...
    pv.normal = plane.normal_;
    single_ptpl.normal_ = plane.normal_;
```

在VIO中细化平面先验：

```969:1034:src/vio.cpp
void VIOManager::updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map)
{
  // ...
  if (iter != plane_map.end())
  {
    VoxelOctoTree *current_octo;
    current_octo = iter->second->find_correspond(p_w);
    if (current_octo->plane_ptr_->is_plane_)
    {
      VoxelPlane &plane = *current_octo->plane_ptr_;
      // ... 使用平面法向量更新normal_ ...
      if (pt->previous_normal_.dot(plane.normal_) < 0) { pt->normal_ = -plane.normal_; }
      else { pt->normal_ = plane.normal_; }
```

## 3. 参考补丁更新策略（选择高质量、大视差、纹理丰富的参考补丁）

**位置：** `src/vio.cpp`

```1036:1120:src/vio.cpp
    float score_max = -1000.;
    for (auto it = pt->obs_.begin(), ite = pt->obs_.end(); it != ite; ++it)
    {
      Feature *ref_patch_temp = *it;
      // ... 计算NCC和视差 ...
      V3D pf = ref_patch_temp->T_f_w_ * pt->pos_;
      V3D norm_vec = ref_patch_temp->T_f_w_.rotation_matrix() * pt->normal_;
      pf.normalize();
      double cos_angle = pf.dot(norm_vec);
      
      // 计算NCC（归一化互相关）评估纹理质量
      // 计算视差和纹理细节
      // 选择score最高的参考补丁
```

该函数通过NCC、视差和纹理质量选择参考补丁。

## 4. 在线曝光时间估计（处理环境光照变化）

**位置：** `src/vio.cpp` 和 `src/IMU_Processing.cpp`

在状态更新中包含曝光时间：

```1628:1629:src/vio.cpp
          if (exposure_estimate_en) { H_sub.block<1, 7>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt, cur_value; }
          else { H_sub.block<1, 6>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt; }
```

在残差计算中考虑曝光时间：

```1621:1621:src/vio.cpp
          double res = state->inv_expo_time * cur_value - inv_ref_expo * P[patch_size_total * level + x * patch_size + y];
```

配置参数：

```64:65:src/LIVMapper.cpp
  nh.param<bool>("vio/exposure_estimate_en", exposure_estimate_en, true);
  nh.param<double>("vio/inv_expo_cov", inv_expo_cov, 0.2);
```

## 5. 按需体素光线投射（处理LiDAR近距离盲区）

**位置：** `src/vio.cpp`

```486:585:src/vio.cpp
  // RayCasting Module
  if (raycast_en)
  {
    for (int i = 0; i < length; i++)
    {
      if (grid_num[i] == TYPE_MAP || border_flag[i] == 1) continue;
      
      for (const auto &it : rays_with_sample_points[i])
      {
        V3D sample_point_w = new_frame_->f2w(it);
        // ... 在体素地图中查找平面 ...
        auto corre_feat_map = feat_map.find(sample_pos);
        if (corre_feat_map != feat_map.end())
        {
          // 找到体素中的点，用于补充盲区
        }
        else
        {
          // 从LiDAR体素地图中获取平面信息
          auto iter = plane_map.find(sample_pos);
          if (iter != plane_map.end())
          {
            VoxelOctoTree *current_octo;
            current_octo = iter->second->find_correspond(sample_point_w);
            if (current_octo->plane_ptr_->is_plane_)
            {
              // 添加平面中心点到视觉子地图
              visual_submap->add_from_voxel_map.push_back(plane_center);
```

配置参数：

```63:63:src/LIVMapper.cpp
  nh.param<bool>("vio/raycast_en", raycast_en, false);
```

---

**总结：**
- 优化1：`LIVMapper.cpp` 中的顺序更新逻辑（LIO → VIO）
- 优化2：`voxel_map.cpp` 和 `vio.cpp` 中的平面先验使用与细化
- 优化3：`vio.cpp` 的 `updateReferencePatch` 函数
- 优化4：`vio.cpp` 和 `IMU_Processing.cpp` 中的曝光时间估计
- 优化5：`vio.cpp` 中的 `raycast_en` 光线投射模块