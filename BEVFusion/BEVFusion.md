# Lidar AI Solution 项目
项目地址：https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution
## 项目配置
- Ubuntu 22.04.5
- pyhon3.9
## UV 配置
- PyTorch：
``` bash uv pip install torch==2.1.0+cu118 torchvision==0.16.0+cu118 --extra-index-url https://download.pytorch.org/whl/cu118 ```

- 安装 OpenMMLab 基础依赖：
``` bash uv pip install "mmcv>=2.0.0,<2.1.0" --extra-index-url https://pypi.org/simple   --index-url https://pypi.tuna.tsinghua.edu.cn/simple```

- 安装 MMDetection 3.3.0：
```bash uv pip install "mmdet>=3.0.0,<3.1.0"   --index-url https://pypi.tuna.tsinghua.edu.cn/simple```

- 安装 MMDetection3D 1.1.0（最新稳定版，支持 BEVFusion 类模型）
```bash uv pip install "mmdet3d==1.1.0"   --index-url https://pypi.tuna.tsinghua.edu.cn/simple```

- 安装 NumPy：
``` bash uv pip install "numpy==1.23.5"   --index-url https://pypi.tuna.tsinghua.edu.cn/simple```

- 安装其他常用依赖：
```bash uv pip install opencv-python pyyaml tqdm   --index-url https://pypi.tuna.tsinghua.edu.cn/simple```

- （可选但推荐）安装 tensorboard 用于可视化：
``` bash uv pip install tensorboard   --index-url https://pypi.tuna.tsinghua.edu.cn/simple```

- 使用 NuScenes 或 Waymo 数据集：
``` bash uv pip install nuscenes-devkit waymo-open-dataset-tf-2-11-0 --index-url https://pypi.tuna.tsinghua.edu.cn/simple```


## 验证数据及模型下载


