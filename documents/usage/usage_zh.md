# 使用指南

## 移动控制

| 按键             | 动作                       | 预览    |
|------------------|----------------------------|---------|
| `W` / `S`        | 向前 / 向后移动             | <img src="gif/1-1.gif" alt="Move forward/backward" width="300"/> |
| `A` / `D` 或 `←` / `→` | 向左 / 向右旋转视角       | <img src="gif/1-2.gif" alt="Rotate view" width="300"/> |
| `↑` / `↓`         | 向上 / 向下看               | <img src="gif/1-3.gif" alt="Look up/down" width="300"/> |

有关设置移动比例的详细信息，请参阅 [代理移动配置](../config_reference/config_reference_zh.md#-代理移动-agent-movement)。


## 功能控制

| 按键     | 动作                                                                   | 预览                 |
|----------|------------------------------------------------------------------------|----------------------|
| `q`      | 退出工具并保存录制内容                                                  |                      |
| `m`      | 切换俯视地图视图                                                        | <img src="gif/2-1.gif" alt="Top-down view" width="300"/> |
| `n`      | 在地图上选择一个点并开始导航                                            | <img src="gif/2-2.gif" alt="Start navigation" width="300"/> |
| `e`      | 保存所有放置的对象并生成 `scene_config.json`                           |                      |
| `space`  | 开始或停止录制（切换）                                                  | <img src="gif/2-4.gif" alt="Start/stop recording" width="300"/> |


### 场景配置保存 -> `e`

场景配置 (`scene_config.json`) 存储用户放置的对象位置，以实现未来的可复现性。在场景中排列好对象后，按 `e` 保存它们的位置。此文件稍后可以重新加载以恢复相同的排列。

保存后，您将在终端中看到此消息：

```bash
Configuration saved to ${output_path}/${dataset_name}/${scene_name}_X/scene_config.json
```

要重新加载，请参阅 [场景配置](../config_reference/config_reference_zh.md#-场景配置-scene-configuration) 中的指南。


### 录制 -> `space`

录制系统根据配置文件捕获原始数据和可选的 ROS2 bag。

> **注意：** 要启用 ROS bag 录制，请在 `habitat_data_collector.yaml` 中设置 `record_rosbag: true`。
> 
> **注意：** 请确保在 [场景输出设置](../config_reference/config_reference_zh.md#-场景输出设置-scene-output-settings) 中正确配置输出路径。

当您按 `space` 时，录制开始。右下角会出现闪烁的红色 `REC` 指示器。您还会看到如下消息：

```bash
Recording started
Start ROS bag recording: ${output_path}/${dataset_name}/${scene_name}_X/rosbag2
[INFO] [rosbag2_recorder]: Press SPACE for pausing/resuming
[INFO] [rosbag2_storage]: Opened database '.../rosbag2_0.db3' for READ_WRITE.
...
[INFO] [rosbag2_recorder]: Recording...
[INFO] [rosbag2_recorder]: Subscribed to topic '/camera/depth/image_raw'
[INFO] [rosbag2_recorder]: Subscribed to topic '/camera/rgb/image_raw'
[INFO] [rosbag2_recorder]: Subscribed to topic '/camera/pose'
[INFO] [rosbag2_recorder]: Subscribed to topic '/camera_info'
```

再次按 `space` 停止录制：

```bash
Recording stopped
[INFO] [rosbag2_cpp]: Writing remaining messages from cache to the bag.
[INFO] [rosbag2_recorder]: Event publisher thread: Exiting
[INFO] [rosbag2_recorder]: Recording stopped
```

您可以继续与模拟进行交互。要完成并保存录制，请按 `q`。您将看到：

```bash
Replay the recording and saving to disk...
Replaying and saving: 100%|██████████████████████████| 54/54 [00:05<00:00, 10.69it/s]
Replay and saving obs and pose completed in ${output_path}/${dataset_name}/${scene_name}_X
```

您的输出目录中将出现以下结构：

```
${output_path}/${dataset_name}/${scene_name}_X
├── camera_intrinsics.json
├── class_bbox.json
├── class_num.json
├── depth/
│   ├── 1744721112.0172191.png
│   └── ...
├── rgb/
│   ├── 1744721112.0172191.png
│   └── ...
├── pose.txt
└── rosbag2/
    ├── metadata.yaml
    └── rosbag2_0.db3
```

- `class_bbox.json`: 包含每个类别的边界框。
- `class_num.json`: 跟踪存在的每个类别的数量。
- `depth/`, `rgb/`: 保存带有时间戳的图像帧。
- `pose.txt`: 将相机位姿存储为 4x4 变换矩阵。

<div align="center">
  <img src="gif/recording.gif" alt="Recording Example" width="60%"/>
  <p><em>示例：录制过程可视化</em></p>
</div>

## 对象交互

| 按键     | 动作                                                                   | 预览    |
|----------|------------------------------------------------------------------------|---------|
| `+`      | 在可放置表面上随机添加一个对象                                          | <img src="gif/3-1.gif" alt="Add object" width="300"/> |
| `-`      | 从场景中随机删除一个对象                                                | <img src="gif/3-2.gif" alt="Delete object" width="300"/> |
| `p`      | 将对象添加到当前查看的可放置表面                                        | <img src="gif/3-3.gif" alt="Place object" width="300"/> |
| `g`      | 抓取最近的对象                                                          | <img src="gif/3-4.gif" alt="Grab object" width="300"/> |
| `r`      | 将当前抓取的对象放置在最近的可放置表面上                                | <img src="gif/3-5.gif" alt="Place grabbed object" width="300"/> |


### 添加/删除对象 (`+` / `-`)

所有候选对象都在对象目录中定义，例如来自 YCB 数据集的对象。有关设置，请参阅 [对象配置](../config_reference/config_reference_zh.md#-对象配置-object-configuration)。

对象仅放置在可放置类别上。这些在 [可放置类别](../config_reference/config_reference_zh.md#-可放置类别-placable-categories) 中定义，例如 `sofa`（沙发）、`table`（桌子）、`desk`（书桌）。您可以通过在 `habitat_data_collector.yaml` 中设置 `show_placable_categories: true` 来可视化它们。

按 `+` 或 `-` 时，会随机添加或删除对象。它们的位置可以在俯视地图上查看（按 `m`）。添加的对象显示为绿点。



<div align="center">
  <img src="gif/objects.gif" alt="Place grabbed object" width="300"/>
  <p><em>示例：对象添加/删除可视化</em></p>
</div>


示例终端日志：

**添加 (Add):**
```bash
Adding object to scene...
Object placed successfully. ID: 1, Semantic ID: 77
Object added and recorded.
Adding object to scene...
Object placed successfully. ID: 4, Semantic ID: 25
Object added and recorded.
```

**删除 (Delete):**
```bash
Removed object ID 7 from scene, remaining 4 objects.
Removed object ID 5 from scene, remaining 3 objects.
```

> **注意：** 如果在录制期间进行对象操作，更改也将被保存。


### 在视图中放置对象 (`p`)

要快速装饰场景，请在面向可放置对象（例如床）时按 `p`。这会将对象直接放置在该表面上。

终端输出：
```bash
Object placed successfully. ID: 3, Semantic ID: 19
Object placed within camera view.
```


### 抓取 / 释放对象 (`g` / `r`)

要启用场景中对象的动态重新排列，请使用抓取和释放控件：

- 站在放置的对象附近
- 按 `g` 抓取最近的对象
- 按 `r` 将其放置在最近的可放置表面上

终端输出：

**抓取 (Grab):**
```bash
Object ID: 3, 011_banana object is being grabbed, press 'r' to release to nearest bbox
```

**释放 (Release):**
```bash
Object placed successfully. ID: 3, Semantic ID: 19
Object 011_banana released.
```

> **注意：** 请确保在按 `r` 之前抓取对象。否则，您将收到：`No grabbed object, Please grab object first.`（没有抓取的对象，请先抓取对象。）
