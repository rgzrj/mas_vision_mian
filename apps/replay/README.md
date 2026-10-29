# 成对录制与回放

回放是独立的C++应用：直接读取视频和逐帧四元数，调用原检测、位姿、track及重投影，显示模型并输出CSV。原实机入口、相机、串口、track、target和四板算法文件不修改。

代码、启动器、录制和检查程序全部收拢在`apps/replay`，配置只放`config`，录像和回放结果放`data`。运行不依赖tools目录，不保留两套启动入口。

## 统一入口

在工程根目录执行：

```bash
python3 apps/replay/run.py record
python3 apps/replay/run.py replay --config config/replay.yaml
python3 apps/replay/run.py check --config config/replay.yaml
```

`record`录制，`replay`启动C++回放，`check`只检查素材。Python负责配置、构建缓存及启动，实际回放不经过Python逐帧传图。子命令均支持`--config`，默认配置为`config/record.yaml`或`config/replay.yaml`。

需要Python的`PyYAML`；录像完成后的自动校验和`check`还需要`opencv-python`。检查素材无需C++构建，也可在Windows执行。先安装依赖：

```bash
python3 -m pip install PyYAML opencv-python
```

首次或源码变化时在工程外`~/.cache/mas_vision_tools/replay_app_工程路径哈希/`构建C++回放。源码、头文件或构建脚本变化才增量编译；改YAML不触发编译，失败不更新缓存标记。同一工程的构建通过锁串行执行。

## 直接构建和运行C++

在现有Linux视觉开发环境中，从工程根目录执行：

```bash
cmake -S apps/replay -B build-replay -DCMAKE_BUILD_TYPE=Release
cmake --build build-replay --target replay -j2
./build-replay/replay --config config/replay.yaml
```

以后只改YAML，直接重新执行最后一行；修改C++后才需重新构建。独立构建需要C++17、OpenCV、Eigen3、yaml-cpp、spdlog、nlohmann-json、SDL2/image/ttf，以及构建时的Python3解释器；不需要相机SDK，也不启动串口或相机线程。

原工程根CMake会自动发现此应用，因此已有完整工程环境也可`cmake --build build --target replay -j2`，执行`./build/replay`。根CMake文件未修改；replay显式指定目标才构建，普通base构建不额外编译回放。

可追加`--headless`关闭显示，`--check`仅验证全部解码帧和CSV配对，不执行算法。`--help`查看入口用法。

## 文件职责

| 文件 | 职责 |
|---|---|
| `run.py` | 唯一Python入口，record/replay/check分发及回放构建缓存 |
| `record.py` | 录制副本准备、构建、停止及配对格式校验；不导入启动器 |
| `recorder.hpp` | 录制后台写入器及60FPS抽帧，插入位置只作用于副本 |
| `player.hpp / player.cpp` | 配对数据、CSV检查、视频读取、录制时间还原 |
| `main.cpp` | 读取配置、调用原算法、播放按键、重投影显示、CSV及运行结果保存 |
| `prepare_target.py` | 仅在构建目录生成回放专用Target及审计patch，原Target文件不写入 |
| `CMakeLists.txt` | 登记正式replay目标；可独立构建，也可由原根CMake发现 |
| `tests/check_replay.py` | 格式、源码隔离、抽帧、缓存、命令入口及参数分支回归检查 |

本应用不导入在线绘图脚本。其他已有工程工具保持原状，与录制回放独立。

## 录像怎么操作

先停止正在运行的原base，避免同时访问相机和串口，再执行`python3 apps/replay/run.py record`。保持有效四元数和红/蓝自瞄模式。录够后按一次Ctrl+C，等待缓存排空、文件关闭和`Paired recording verified`。不要直接断电。

录制仍在原实机程序的隔离副本中运行，**会保留当前shoot、云台控制及串口发送流程**，并非只读监听；原工程线程文件不改。回放只运行到track，不运行shoot和电控发送。

录制缓存沿用`~/.cache/mas_vision_tools/record_工程路径哈希/`，不每次新建工程。只改录制YAML不会触发编译；源码变化增量构建，未变化文件保留修改时间，旧副本文件会被清理。`--prepare`只准备副本，不编译或访问设备：

```bash
python3 apps/replay/run.py record --prepare
```

默认`config/record.yaml`保存60FPS成对录像，MJPG编码、32帧队列。录像和四元数一起按曝光时间选帧；每帧保存在线实际使用的`serial.q(frame.timestamp)`，不改变在线track处理频率。`fps`只影响视频元数据，`max_record_fps`才控制实际保存频率；设为0时保存全部处理帧。

默认输出为`data/recordings/session_时间/`，包含视频、逐帧CSV、录制汇总、配置快照、分类模型和可直接使用的回放YAML。结束时会打印具体回放命令，例如：

```bash
python3 apps/replay/run.py replay --config data/recordings/session_实际目录/replay.yaml
```

默认录制目录内生成的回放配置将结果放到`data/replay`。自定义外置录制目录时，生成的配置保留session内`replay_output`，便于整份拷贝。每次回放独立创建run，不覆盖历史结果。

先录10秒，确认配对校验通过且`recorded_input_fps`约60，再录长数据。发生队列满时保留失败标记，现有文件只代表之前保存的前缀，不能当作完整录像。

## 对照RMCS的边界

本轮依据本地`rmcs_auto_aim_v2-main`以下源码核对职责：

- `src/io/recorder/recorder.cpp`：保存同一CameraFrame的原始图像、曝光/接收/同步时间和IMU快照。
- `src/io/recorder/player.cpp`：读取成对记录，恢复帧和姿态，输出到同一camera_frame接口。
- `src/component.cpp`：核心自瞄消费该帧，采集与回放提供相同类型的输入。

在本项目，Player生成`PairedFrame`，复用原CameraFrame及原检测/PnP/track。录制器保存同一份图像与匹配姿态；独立副本的录制接入保留原线程执行顺序。

两项差异有明确原因：RMCS使用raw字节与CSV，本项目继续兼容已经稳定录下的MJPG视频与CSV；RMCS播放器超时会跳帧，核心组件也采用最新帧消费。本应用用于EKF参数对照，必须逐帧处理，暂停不更新、不重发同一帧，到结尾停止，录制时间差不受播放速度影响。不能声称照搬了RMCS的调度行为。

## 配置

全局示例放在`config/replay.yaml`，录制参数放在`config/record.yaml`。全局回放示例中的session是占位目录，需填写实际素材或直接使用session内生成的YAML。所有配置里的相对路径以该YAML所在目录为起点。

```yaml
replay:
  video: video.avi
  frames_csv: frames.csv
  auto_aim_yaml: auto_aim.yaml
  camera_yaml: hikcamera.yaml
  output_dir: replay_output_v1_10
  speed: 1.0
  display: true
  ekf:
    v1: 10.0
```

- `speed: 1`原速，`0`尽快处理，`0.5`半速。始终使用录制时间差更新，计算慢时逐帧继续。
- 空格暂停/继续，`n`暂停时前进一帧，`q`或Esc提前结束；Ctrl+C也会保存未完成标记。
- 初版不提供倒退和任意跳转，保证滤波历史连续。
- 删除整个`ekf`块则使用原代码默认值。可选`v1/v2/q_h`及`r_scale: [1,1,1,1]`；r_scale依次对应位置方位角、位置仰角、距离、板yaw的观测噪声缩放。仅覆盖四板目标，其他分支保留原值。

为了尽量不动原算法，本版保留一项构建隔离：CMake在构建目录生成`replay_target.cpp`，只将原参数取值接到应用的运行时配置，并插入已有的只读逐板观察调用。公式、默认参数、关联、更新顺序和约束保留。构建目录的`replay_instrumentation.patch`可以核查全部差异。**这不是每次运行复制整个工程，也不需要每组参数重新编译。**

## 录像格式和存放

`video.avi`每个解码帧严格对应`frames.csv`一行：

```csv
frame_index,timestamp_ns,t_recv_ns,q_w,q_x,q_y,q_z,mode
0,1000000000,1012960000,1,0,0,0,0
1,1016666667,1029626667,1,0,0,0,0
```

这是格式示例，不能配到任意视频冒充实录。frame_index从0连续，曝光时间严格递增，接收时间不早于曝光时间，四元数为在线实际配到该帧的wxyz。mode=0红、1蓝，2～5按旧回放跳过自瞄。

程序发现缺帧、额外帧、时间倒序或非法四元数会报错，不静默补帧。视频FPS元数据不参与EKF计时。播放器将全部录制时间以同一个偏移映射到当前steady_clock时间轴，保留原始帧间隔和曝光至接收间隔；CSV输出仍使用原录制的整数纳秒时间。

素材统一放`data/recordings/session_xxx/`，输出统一放`data/replay/`。素材输入路径仍可指定到其他位置；支持原格式不代表继续维护旧程序入口。

MJPG解码的像素来自已有有损录像，不能宣称等同于相机无损原图；60FPS抽样素材不能等同于完整165FPS在线滤波响应。

## 结果

每次在所选output_dir中新建`run_时间_纳秒/`，包含：

- `replay.yaml`、有效`auto_aim.yaml`、原标定`hikcamera.yaml`、原`frames.csv`。
- `inputs.json`：实际输入及显示、速度设置。
- `parameter_overrides.json`：本次运行的EKF覆盖值，空对象表示不覆盖。
- `source_manifest.json`：构建时的源码哈希；`replay_instrumentation.patch`：构建专用Target的完整差异。
- `backend.log`、`summary.json`：算法警告、错误、输入/处理数量、是否完成、处理FPS。
- `csv/plot_s*.csv`：按字段集合分流，保留`frame_index`、`timestamp_ns`、`t_recv_ns`、`t_recorded`。

四板逐次更新字段继续使用69的`replay_update.*`：实际slot_id、帧内order、检测四角、PnP、更新前后11维状态、新息和重投影误差。更新后与整帧最终贴合误差分开记录；`trial_update_applied`与`frame_measurement_accepted`保留原含义。初始化没有调用普通更新时不伪造观察行。

CSV文件名由旧的`plot_日期_sN.csv`简化为`plot_sN.csv`，列名保持；仍需读取整个csv目录，不能假定第几号文件一定是tracker。

旧版的`native_workspace.txt`、`parameter_changes.patch`和`diagnostic_instrumentation.patch`由上述构建来源、运行时覆盖文件和完整patch替代。旧分析结果不会被覆盖。

## 验证与回退

执行`python3 apps/replay/tests/check_replay.py`。其中g++可编译实际C++元数据读取函数和生成参数表达式；其余检查包括配对异常、隔离副本、未知源码结构拒绝、60FPS抽帧、缓存复用/失败重试及统一入口。没有g++时相关检查会明确跳过；这些检查不等于完整OpenCV/EKF应用验收。

当前Windows缺完整原生依赖，尚未在此环境构建运行整个应用。Linux更新后先执行`check`，再用69对应的原录像和`v1=10`完整回放；应核对原始观测、关联ID、11维状态及逐板记录与69一致，允许明确说明的浮点格式差异。首帧tracker的启动dt与程序初始化耗时有关，不作为确定性比较项。

本次收拢前快照：`C:/Users/34855/.codex/snapshots/replay_consolidation_20261016_140005`。此前C++迁移前快照仍在`replay_app_20261016_130249`。原track、target、pose、配置和根CMake继续按已有哈希核对，保留用户的未提交改动。

## 旧录像与车端文件迁移

本地旧素材目录只有占位，没有找到真实session，因此没有移动或删除录像。69等桌面分析目录也不属于本次整理范围。

车端若还有`tools/replay_native/session_*`或`tools/doc/replay/recordings/session_*`，先把每个session完整复制到`data/recordings`，保持视频、CSV、model和配置一起移动，再执行：

```bash
python3 apps/replay/run.py check --config data/recordings/session_实际目录/replay.yaml
python3 apps/replay/run.py replay --config data/recordings/session_实际目录/replay_v1_10.yaml
```

第二条仅用于已有该参数文件的session，否则使用其`replay.yaml`。核对文件哈希、配对和回放后，再清理旧位置。不要整目录删除仍含录像的旧目录。历史输出/来源文件保持原样；确有绝对输入路径失效时，另存运行配置修正，不覆盖历史记录。

将整个`apps/replay`、`config/record.yaml`和`config/replay.yaml`同步到Linux。保留车端已有实际回放配置，勿用占位示例覆盖已填好的路径。旧的六个tools程序/配置和旧回放文档已在本地删除，车端同步软件若不自动删除，可在迁好录像后清理对应旧文件；不影响其他工程工具。

## 调试中踩过的问题：后续必须保留的约束

| 既有问题 | 当前处理与检查方式 |
|---|---|
| 找不到CMakeLists.txt.in、文件改名后仍报错 | 正式app直接使用CMakeLists.txt，不再运行旧模板工程 |
| 每次录像/改参数都全量编译 | 两种缓存固定在工程外；运行YAML不参与构建指纹；失败后不能把旧程序标记成新版本 |
| FFV1/高帧率编码跟不上、队列满 | 默认MJPG、60FPS实际选帧；先验证吞吐，不仅加队列；满队列保留失败汇总 |
| fps改60但实际上仍写165帧 | fps只是元数据，max_record_fps按曝光时间在clone/编码前选帧；原始输入和实际保存帧率分开统计 |
| MJPG后端速度差异 | 优先OpenCV MJPEG，失败才回退；以RECORDING VIDEO的实际backend和写入耗时为准 |
| 视频与四元数错配、停止后文件未完成 | 相同帧一起入队，wxyz原配对；Ctrl+C后排空；核对解码帧数、整数时间及failed标记 |
| session在另一目录、YAML相对路径出错 | 使用明确的session配置；配置内部相对路径以文件所在目录为起点；单入口可从其他工作目录调用 |
| 回放卡顿、被误认为单核 | 无逐帧PNG文件或跨进程BGR传输；OpenCV使用默认线程；帧间保持顺序，处理FPS排除主动等待 |
| 暂停/变速后EKF改变、漏帧被掩盖 | dt只取录制时间差；暂停不重复更新，不为赶进度跳帧；少帧/多帧都报错 |
| 67/68/69中心更稳却感觉框落后 | v1降低不保证观测贴合更好；保留全部逐板日志，区分中心、相位、形状误差，不能先加统一预测时间 |
| 个别靶板倾角可能并非15° | 名义角不是逐板实测；先核对实物倾角/尺寸，不在目录整理中改PnP或四板模型 |

录制的图像、姿态和时序可以重放输入，不能模拟“新控制指令会怎样改变云台”。四元数本身也不能替代原始陀螺仪数据来判定INS误差。机械摩擦轮问题已另行处理，本次不把中心摆动重新归因到摩擦轮。
