# README.md

## 一. 项目作用
用于作为一个简单的仿真器平台
- 加载环境
  - 加载指定的几个数据集的某几个室内环境
  - 加载室内环境带有可能随机出生的物体
---

- ROS2消息发送/接收/录制ROS bag:
  - 消息发送:
    - 机器人于房间的位姿Pose
    - 机器人此时的RGBD信息
    - 机器人此时的RGB图像对应的所有物体的检测框和实例分割图像
  - 消息接收:
    - 接收ROS2的控制指令，视角进行移动
  - 录制ROS bag
    - 录制对应的ROS2 bag用于建图等方面的测试
---



## 二. 环境安装

> 此设置已在 **Ubuntu 22.04** 和 **Python 3.10** 上通过测试。

#### 2.1 克隆带有子模块的仓库

```bash

# 强烈建议将项目放置在Documents路径下
cd ~/Documents
git clone --recurse-submodules git@github.com:Tipriest/vln_simulator.git
cd vln_simulator
```

#### 2.2 创建 Conda 环境

```bash
conda env create -f environment.yml
conda activate vln_simulator
```

#### 3. 编译并安装 Habitat Sim & Lab

> 此步骤需要一些时间，因为它会从源码编译 Habitat-Sim。
> Habitat 无法通过 conda 在 Python 3.10 中直接安装，因此必须手动编译。

```bash
# 这里编译的时候有可能会出现一个什么包装不了的问题，需要删掉3rdparty路径下的habitat-sim文件夹重新运行下面的命令:
bash scripts/install_habitat.sh
```

> 在编译 habitat-sim 过程中，如果遇到 OpenGL 错误（如 `Could NOT find OpenGL`）或编译 `zlib_external` 时出错，请安装以下依赖库：
```bash
 sudo apt install libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev zlib1g-dev
 sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp 
```

## 📦 数据集设置

在运行工具之前，请按照 [数据集设置指南](documents/dataset/dataset_netdisk.md) 准备所需的数据集。


## ⚙️ 配置指南

有关配置选项和结构的详细说明，请参阅 [配置参考](documents/config_reference/config_reference_zh.md)。正确设置配置对于运行此工具至关重要。


## 运行采集器

从根目录运行主仿真程序：

```bash
python -m habitat_data_collector.main
```

默认情况下，它使用位于 `config/habitat_data_collector.yaml` 的配置文件。有关配置详情，请参阅 [配置参考](documents/config_reference/config_reference.md)。

### ROS2 集成（可选）

如果您希望接收和发送 ROS2 话题输出或录制 ROS2 bag：

1. 按照 [官方指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 安装 **ROS2 Humble**。
2. 在运行采集器之前 source ROS2 环境：

```bash
source /opt/ros/humble/setup.bash  # 或者 setup.zsh
```

source 之后，仿真器将向 ROS2 话题发布数据。您可以通过在 `config/habitat_data_collector.yaml` 中启用 ROS 录制配置来录制这些数据。有关话题配置和 ROS2-to-ROS1 桥接设置，请参阅 [ROS 集成文档](documents/ros.md)。


## 📘 用户指南

**仿真器成功启动后，请参阅 [使用指南](documents/usage/usage.md) 了解如何**：

- 移动相机并探索场景
- 添加、放置、抓取和删除物体
- 开始和停止录制（原始数据 + ROS2 bag）
- 保存并重新加载场景配置

该指南包含视觉预览和终端输出示例，以便更好地理解。


## 📁 项目结构


```
habitat-data-collector/
├── habitat_data_collector/   # 主应用程序代码
│   ├── main.py
│   └── utils/
├── config/                   # YAML 配置文件
├── 3rdparty/                 # Git 子模块: habitat-sim & habitat-lab
├── documents/               # Markdown 文档和媒体文件
├── scripts/                 # 辅助脚本 (例如 build, setup)
├── environment.yml          # Conda 环境规范
└── README.md
```

## ⚠️ 注意事项
使用 ROS 功能前必须安装并 `source ROS2 Humble`。
配置通过 `OmegaConf` 和 `Hydra` 处理。
所有路径、话题和行为均在 `habitat_data_collector.yaml` 中配置。


## 🔗 引用
如果您觉得我们的工作有帮助，请考虑给这个仓库点个星 🌟 并引用：
```
@article{jiang2025dualmap,
  title={DualMap: Online Open-Vocabulary Semantic Mapping for Natural Language Navigation in Dynamic Changing Scenes},
  author={Jiang, Jiajun and Zhu, Yiming and Wu, Zirui and Song, Jie},
  journal={arXiv preprint arXiv:2506.01950},
  year={2025}
}
```

## 🙏 致谢
本项目建立在以下杰出工作的基础之上：
- Habitat-Sim
- Habitat-Lab
感谢这些项目的作者和贡献者将其开源并积极维护。

本项目还受到 VLMaps 数据采集流程的启发，我们感谢 HOVSG 和 VLMaps 的作者所做的贡献。

特别感谢 @TOM-Huang 和 @aclegg3 在开发过程中提供的宝贵建议和支持。

