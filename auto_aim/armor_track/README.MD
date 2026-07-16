# 装甲板跟踪

## 装甲板姿态解算

### 坐标系定义

| 坐标系 | 原点 | X轴 | Y轴 | Z轴 | 说明 |
|--------|------|-----|-----|-----|------|
| **装甲板坐标系** ($\{A\}$) | 装甲板中心 | 水平向右 | 竖直向下 | 垂直装甲板向里 | 随装甲板旋转而旋转 |
| **相机坐标系** ($\{C\}$) | 相机光心 | 指向图像右侧 | 指向图像下方 | 沿光轴向前 | 右手坐标系，Z轴指向视野前方 |
| **云台坐标系** ($\{G\}$) | 云台转轴中心 | 水平向前 | 水平向左 | 竖直向上 | 随云台转动而转动 |
| **IMU体坐标系** ($\{I\}$) | IMU中心 | - | - | - | 与IMU固连，随IMU运动 |
| **世界坐标系** ($\{W\}$) | 与云台中心重合 | 水平向前 | 水平向左 | 竖直向上 | 惯性坐标系，Z轴始终竖直向上 |

#### 坐标系之间的转换关系

整体转换链：

$$P_{armor} \xrightarrow{\text{PnP}} P_{camera} \xrightarrow{\text{Hand-Eye}} P_{gimbal} \xrightarrow{\text{Similarity}} P_{world}$$

---

#### 具体转换步骤

##### 1. PnP 解算：装甲板坐标系 → 相机坐标系

利用 `cv::solvePnP` 求解装甲板相对于相机的位姿：

- **输入**：装甲板 3D 模型点（`BIG_ARMOR_POINTS` 或 `SMALL_ARMOR_POINTS`）和检测到的 2D 图像角点
- **输出**：旋转向量 $rvec$ 和平移向量 $tvec$

**数学推导：**
- 旋转向量转旋转矩阵：$R_{armor2camera} = \text{Rodrigues}(rvec)$
- 装甲板在相机坐标系下的位置：$P_{camera} = tvec$

**代码实现：**
```cpp
cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
cv::cv2eigen(tvec, xyz_in_camera);
```

---

##### 2. 手眼标定：相机坐标系 → 云台坐标系

利用手眼标定得到的外参，将相机坐标系下的点转换到云台坐标系。

**外参定义：**
- $R_{camera2gimbal}$：相机到云台的旋转矩阵
- $t_{camera2gimbal}$：相机原点在云台坐标系下的坐标（平移向量）

**数学推导：**

位置转换：
$$P_{gimbal} = R_{camera2gimbal} \cdot P_{camera} + t_{camera2gimbal}$$

姿态转换：
$$R_{armor2gimbal} = R_{camera2gimbal} \cdot R_{armor2camera}$$

**代码实现：**
```cpp
armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
```

---

##### 3. 相似变换：云台坐标系 → 世界坐标系

> **注意**：世界坐标系原点与云台中心重合（相对坐标系），因此**没有平移向量**，只有旋转变换。

**问题 如何用imu求这个旋转变换：**
- IMU 直接测量的是 IMU 体坐标系相对于世界坐标系的姿态
- 但云台和 IMU 之间存在固定的安装偏差
- 需要将 IMU 的姿态转换到云台坐标系下

**已知条件：**
- $q$：IMU 姿态四元数，表示 $R_{imubody2world}$（IMU体坐标系到世界坐标系的旋转）
- $R_{gimbal2imubody}$：云台到 IMU 的固定安装偏差（手动填入）

**数学推导：**

相似变换公式：
$$R_{gimbal2world} = R_{gimbal2imubody}^T \cdot R_{imubody2world} \cdot R_{gimbal2imubody}$$

**公式解释：**
- 这是一个**相似变换**（Similarity Transform），形式为 $A^T R A$
- 作用：将 $R_{imubody2world}$ 投影到云台的几何轴系上
- 目的：消除云台和 IMU 之间的静态安装误差，使得世界坐标系始终保持"稳定"（Z轴竖直向上）

**变换步骤：**
1. $R_{gimbal2imubody}$：将云台坐标转换到 IMU 坐标系
2. $R_{imubody2world}$：在 IMU 坐标系下进行世界旋转
3. $R_{gimbal2imubody}^T$：转回云台坐标系

**位置与姿态转换：**
- 位置：$P_{world} = R_{gimbal2world} \cdot P_{gimbal}$
- 姿态：$R_{armor2world} = R_{gimbal2world} \cdot R_{armor2gimbal}$

**代码实现：**
```cpp
void set_R_gimbal2world(const Eigen::Quaterniond &q) {
    Eigen::Matrix3d R_imubody2world = q.toRotationMatrix();
    R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2world * R_gimbal2imubody_;
}

armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
```

---

##### 4. 提取欧拉角：$R_{armor2world} \rightarrow YPR$

描述装甲板的旋转姿态，将旋转矩阵参数化为 yaw-pitch-roll 角度。

**提取顺序：** Z-Y-X (2-1-0)
- 先绕 Z 轴旋转 → **Yaw**（偏航角）
- 再绕 Y 轴旋转 → **Pitch**（俯仰角）
- 最后绕 X 轴旋转 → **Roll**（滚转角）

**代码实现：**
```cpp
armor.ypr_in_world = rm_utils::eulers(R_armor2world, 2, 1, 0);  // Z-Y-X 顺序
```

**欧拉角含义：**
- **Yaw**：装甲板水平方向的朝向
- **Pitch**：装甲板俯仰角度（大装甲板约为 ±15°）
- **Roll**：装甲板滚转角度（通常接近 0）

---

##### 5. 转换为极坐标：$XYZ \rightarrow YPD$

将笛卡尔坐标 $(x, y, z)$ 映射到球坐标，描述目标相对于云台转轴中心的位置信息。

**数学公式：**

- **Yaw**（方位角）：$\phi = \text{atan2}(y, x)$
  - 目标在水平面的方位角

- **Pitch**（仰角）：$\theta = \text{atan2}(z, \sqrt{x^2 + y^2})$
  - 目标相对于水平面的仰角
  - **注意**：不是与 Z 轴的夹角！

- **Distance**（距离）：$d = \sqrt{x^2 + y^2 + z^2}$
  - 装甲板到云台转轴中心的直线距离

**代码实现：**
```cpp
armor.ypd_in_world = rm_utils::xyz2ypd(armor.xyz_in_world);
```

    

 

