# 配置参考：`habitat_data_collector.yaml`

本文档提供了 Habitat 数据收集器配置选项的简单概述。配置文件路径：`config/habitat_data_collector.yaml`

---

## 📁 保存的ROSBAG与RGBD数据集输出路径设置 (Dataset Result Output Settings)


```yaml
dataset_name: hm3d
output_path: /path/to/output
scene_name: 00829-QaLdnwvtxbs
```

- `dataset_name`: 数据集标识符（例如 `hm3d`, `mp3d`, `replica`）。
- `output_path`: 保存输出数据的目录。
- `scene_name`: 用于组织输出的场景标识符。

收集的数据将保存在以下路径下：
```
${output_path}/${dataset_name}/${scene_name}_X
```
其中 X 表示运行索引（例如，第一次运行为 *.../00829-QaLdnwvtxbs_0*）。这确保了多次运行的输出被存储而不会相互覆盖。

---

## 🗺️ 使用场景配置 (Scene Configuration)

```yaml
load_from_config: false
scene_config: /path/to/scene_config.json

scene_path: /path/to/scene.glb
scene_dataset_config: /path/to/scene_dataset_config.json
```

- `load_from_config`: 如果为 `true`，场景将使用提供的 `scene_config` 加载。
- `scene_config`: 动态场景配置文件的路径。该文件包含所有手动添加的对象，并支持复现预先安排的环境。
- `scene_path`: 原始场景资产文件的直接路径（例如 `.glb`, `.ply`）。
- `scene_dataset_config`: 数据集特定配置文件的路径。

> YAML 中包含了 **HM3D**、**MP3D** 和 **Replica** 的注释路径以供参考。

加载场景有两种模式：

1. **原始场景加载 (Raw scene loading)**：直接加载 HM3D 或 Replica 等数据集提供的场景资产。这将渲染原始环境，没有任何手动放置的对象。每个数据集的详细用法和目录布局在 [数据集设置指南](../dataset/dataset.md) 中有描述。

2. **预安排场景加载 (Pre-arranged scene loading)**：从保存的 `scene_config` 文件加载完整的场景设置。这包括基础场景以及用户定义的对象放置。这是可复现实验或合成场景生成的推荐方法。设置和保存过程在 `usage.md` 中有说明。

> 注意：由于技术限制，目前 **Replica** 不支持场景配置功能（[参见 issue](https://github.com/facebookresearch/habitat-sim/issues/2484#issuecomment-2461778776)）。

---

## 🎯 对象配置 (Object Configuration)

```yaml
objects_path: /path/to/objects
```

- `objects_path`: 包含在模拟过程中插入的对象资产（GLB 模型）的目录。目前仅支持 [YCB 对象](https://www.ycbbenchmarks.com/)。有关更多详细信息，请参阅 [数据集设置指南](../dataset/dataset.md)。

一旦正确配置了 `objects_path`，您应该会在终端中看到类似于以下的注册日志：

```bash
Registered 003_cracker_box with semantic ID: 29
Registered 005_tomato_soup_can with semantic ID: 12
Registered 011_banana with semantic ID: 87
Registered 019_pitcher_base with semantic ID: 59
Registered 024_bowl with semantic ID: 30
Registered 025_mug with semantic ID: 74
Registered 029_plate with semantic ID: 87
Registered 037_scissors with semantic ID: 14
```
---

## 🎥 传感器和相机设置 (Sensor and Camera Settings)

```yaml
data_cfg:
  seed: 12
  rgb: true
  depth: true
  semantic: true
  resolution:
    w: 1200
    h: 680
  camera_height: 1.5
```

- `seed`: 用于可复现性和初始起点的随机种子。
- `rgb`, `depth`, `semantic`: 启用/禁用特定的传感器输出。
- `resolution`: 图像尺寸（像素）。
- `camera_height`: 相机离地高度（米）。设置为机器人的高度。

---

## 🕹️ 代理移动 (Agent Movement)

```yaml
movement_cfg:
  move_forward: 0.4
  move_backward: 0.4
  turn_left: 3
  turn_right: 3
  look_up: 3
  look_down: 3
```

- 配置代理控制速度（米/秒或度/帧，取决于动作）。

---


## 🧱 可放置类别 (Placable Categories)

```yaml
placable_categories:
  - table
  - counter
  - desk
  ...
show_placable_categories: false
```

- `placable_categories`: 可放置对象的类别列表。只有此列表中的对象才能与其他对象一起进行添加和删除操作。
- `show_placable_categories`: 可视化边界框以进行调试或房间布置。

<div align="center">
  <img src="BBOX.png" 
       alt="Bounding box example" 
       width="80%"/>
  <p>
    <em>示例：为可放置对象类别渲染的边界框。</em>
  </p>
</div>


---

## 📡 ROS2 集成 (ROS2 Integration)

```yaml
use_ros: true
record_rosbag: true
```

- `use_ros`: 启用 ROS2 发布者。
- `record_rosbag`: 在录制期间选择性地录制 ROS bag。详见 `usage.md`。

---

## 🧩 其他 (Miscellaneous)

```yaml
frame_rate: 30.0
id_handle_dict: {}
```

- `frame_rate`: 模拟频率（帧每秒）。
- `id_handle_dict`: 用于将语义 ID 映射到对象句柄的占位符（保留供内部使用）。

---

## 📝 注意事项 (Notes)

- 用户应更新所有路径以反映其本地或共享数据集结构。
- 默认配置中的绝对路径仅在开发者的本地环境中有效。
