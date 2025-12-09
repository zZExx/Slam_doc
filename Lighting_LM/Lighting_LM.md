# Lighting-LM 项目说明
## 项目地址
https://github.com/gaoxiang12/lightning-lm?tab=readme-ov-file
## 相关说明文档
原理说明：https://zhuanlan.zhihu.com/p/1969360325880030621

系统说明：https://mp.weixin.qq.com/s/XD78OP6Ng_bpTKFiMwE8ag?scene=1

# 使用说明
## 环境
ubuntu-22.04.5-desktop-amd64.iso

https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/22.04.5/
## 依赖
- ros2 humble 及以上
  使用一键安装ros2指令

  wget http://fishros.com/install -O fishros && . fishros

- Pangolin（用于可视化，见thirdparty，需要先解压编译）
    ``` bash
    cd thirdparty/Pangolin
    mkdir build && cd build
    cmake .. && make -j
    sudo make install
    ```
- OpenCV
- PCL
- yaml-cpp
- glog
- gflags
- pcl_conversions

在Ubuntu 22.04上，执行：```bash ./scripts/install_dep.sh即可```。

## 编译
```colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release```本包即可。

然后```source install/setup.bash```即可使用。（每次打开新终端都需source）

## 运行
### 数据集准备
BaiduYun: https://pan.baidu.com/s/1NEGQcYoMtd57oqog09pB6w?pwd=5v8h 提取码: 5v8h

使用NCLT2013010.db3数据集（已转换至ros2bag类型）
```bash 
# 数据集信息检查
ros2 bag info /home/cache/20130110.db3
```
将ros1数据转换至ros2
   安装:
   ```pip install -i https://pypi.tuna.tsinghua.edu.cn/simple rosbags```

   转换:
    ```rosbags-convert --src [你的ROS1_bag文件.bag] --dst [输出ROS2bag目录]```
### 离线建图
```bash
cd ~/ros2_lighting_lm/lightning-lm
source install/setup.bash

ros2 run lightning run_slam_offline \
  --input_bag /home/cache/20130110.db3 \
  --config ./config/default_nclt.yaml
```
![](2025-11-07-14-50-45.png)

建图结果将输出在  map_path: ./data/new_map/ 下

通过 ```pcl_viewer global.pcd``` 可视化结果
