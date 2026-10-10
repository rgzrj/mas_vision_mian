# 装甲板跟踪

## 装甲板姿态解算

### 坐标系定义

| 坐标系 | 原点 | X轴 | Y轴 | Z轴 | 说明 |
|--------|------|-----|-----|-----|------|
| **装甲板坐标系** ($\{A\}$) | 装甲板中心 | 垂直板面向里 | 正视装甲板时向左 | 正视装甲板时向上 | 模型点位于 YZ 平面，随装甲板旋转 |
| **相机坐标系** ($\{C\}$) | 相机光心 | 指向图像右侧 | 指向图像下方 | 沿光轴向前 | 右手坐标系，Z轴指向视野前方 |
| **云台坐标系** ($\{G\}$) | 云台转轴中心 | 水平向前 | 水平向左 | 竖直向上 | 随云台转动而转动 |
| **IMU体坐标系** ($\{I\}$) | IMU中心 | - | - | - | 与IMU固连，随IMU运动 |
| **世界坐标系** ($\{W\}$) | 与云台中心重合 | 参考朝向的前方 | 参考朝向的左方 | 参考竖直向上 | 方向由 IMU 参考系转换得到，不随云台当前朝向旋转；原点随云台中心平移 |

装甲板模型点按**左上、右上、右下、左下**排列，对应检测器的 `armor.points`：

$$
P_{左上}=\begin{bmatrix}0\\+W/2\\+L/2\end{bmatrix},\quad
P_{右上}=\begin{bmatrix}0\\-W/2\\+L/2\end{bmatrix},\quad
P_{右下}=\begin{bmatrix}0\\-W/2\\-L/2\end{bmatrix},\quad
P_{左下}=\begin{bmatrix}0\\+W/2\\-L/2\end{bmatrix}
$$

这里的左右、上下是装甲板自身的正视约定，不表示旋转后仍沿图像水平、竖直方向。`W` 是两根灯条的中心距，`L` 是模型灯条长度；尺寸在 `armor_types.hpp` 中定义，单位为米。

#### 坐标系之间的转换关系

整体转换链：

$$
P_{armor} \xrightarrow{\text{PnP}} P_{camera}
\xrightarrow{\text{Hand-Eye}} P_{gimbal}
\xrightarrow{\text{Rotation / Change of Basis}} P_{world}
$$

本文按列向量约定：$R_{A2B}$ 将 A 系向量转换到 B 系。位置单位为米，内部角度为弧度；调试输出若转为度，应以具体字段的代码为准。

---

#### 具体转换步骤

##### 1. PnP 解算：装甲板坐标系 → 相机坐标系

利用 `cv::solvePnP` 求解装甲板相对于相机的位姿：

- **输入**：装甲板 3D 模型点（`BIG_ARMOR_POINTS` 或 `SMALL_ARMOR_POINTS`）和检测到的 2D 图像角点
- **输出**：旋转向量 $rvec$ 和平移向量 $tvec$

**数学推导：**

旋转向量转旋转矩阵：

$$
R_{armor2camera}=\operatorname{Rodrigues}(rvec)
$$

任意装甲板模型点在相机坐标系下的位置：

$$
P_{camera}=R_{armor2camera}\cdot P_{armor}+tvec
$$

装甲板坐标系原点位于板中心，代入 $P_{armor}=0$，得到：

$$
P_{camera,center}=tvec
$$

因此原版的“$P_{camera}=tvec$”适用于**板中心**，不能直接用于四个角点。

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

$$
P_{gimbal}=R_{camera2gimbal}\cdot P_{camera}+t_{camera2gimbal}
$$

姿态转换：

$$
R_{armor2gimbal}=R_{camera2gimbal}\cdot R_{armor2camera}
$$

反向转换：

$$
P_{camera}=R_{camera2gimbal}^{T}\cdot
\left(P_{gimbal}-t_{camera2gimbal}\right)
$$

这里的 $t_{camera2gimbal}$ 是相机光心在云台系中的坐标；PnP 输出的 `tvec` 则是装甲板中心在相机系中的坐标，两者不同。

**代码实现：**
```cpp
armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
```

---

##### 3. 相似变换（旋转换基）：云台坐标系 → 世界坐标系

> **注意**：当前世界坐标系原点与云台中心重合，因此此步只有旋转，没有额外平移。代码没有在此补偿自身底盘的世界位移；自身移动时，不能将它当作原点固定的惯性坐标系。

**问题：如何用 IMU 求这个旋转变换？**

- 按当前实现的前提，IMU 四元数提供体坐标系相对于 IMU 参考坐标系的旋转
- 云台体轴与 IMU 体轴之间用固定旋转连接
- IMU 参考轴还需转换到项目使用的世界参考轴，不能与当前云台体坐标系混称

**已知条件：**

- $q$：IMU 姿态四元数，转成矩阵记为 $R_{imubody2world}$，与源码命名一致
- $A=R_{gimbal2imubody}$：云台体轴到 IMU 体轴的固定旋转（配置提供）

这里有一个命名区别：$R_{imubody2world}$ 中的 world 指 **IMU 使用的世界参考轴**；$R_{gimbal2world}$ 中的 world 指 **项目使用的世界参考轴**。当前代码用左侧的 $A^T$ 转换这两套参考轴，不需要增加额外的坐标系名称。

**数学推导：**

按当前代码的参考轴约定，保留原版的三项乘法：

$$
R_{gimbal2world}=R_{gimbal2imubody}^{T}\cdot
R_{imubody2world}\cdot R_{gimbal2imubody}
$$

令 $Q=R_{imubody2world}$，简写为：

$$
R_{gimbal2world}=A^TQA=A^{-1}QA
$$

**公式解释：**

- $A$ 为旋转矩阵，因此 $A^T=A^{-1}$；$A^{-1}QA$ 在矩阵意义上是相似变换，在这里用于旋转换基，不是投影，也没有尺度缩放
- 当前代码按上述约定使用四元数，实际参考轴与四元数方向应与电控约定一致；这个公式不能自动消除任意安装误差，也不能单凭公式保证实际重力轴正确

**变换步骤：**

1. $A$：云台体坐标系 → IMU 体坐标系
2. $Q$：IMU 体坐标系 → 世界坐标系
3. $A^T$：IMU 参考坐标系 → 项目世界参考坐标系（按当前约定），**不是转回当前云台体坐标系**

**位置与姿态转换：**

位置：

$$
P_{world}=R_{gimbal2world}\cdot P_{gimbal}
$$

姿态：

$$
R_{armor2world}=R_{gimbal2world}\cdot R_{armor2gimbal}
$$

把相机到云台的转换代入，得到完整的位置转换与其逆变换：

$$
P_{world}=R_{gimbal2world}\cdot
\left(R_{camera2gimbal}\cdot P_{camera}+t_{camera2gimbal}\right)
$$

$$
P_{camera}=R_{camera2gimbal}^{T}\cdot
\left(R_{gimbal2world}^{T}\cdot P_{world}-t_{camera2gimbal}\right)
$$

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

**提取顺序：** 内禀 Z-Y-X (2-1-0)，`eulers` 默认 `extrinsic=false`。

- 先绕 Z 轴旋转 → **Yaw**（偏航角）
- 再绕更新后的 Y 轴旋转 → **Pitch**（俯仰角）
- 最后绕更新后的 X 轴旋转 → **Roll**（滚转角）

**数学推导：**

令 $\psi=yaw$、$\theta=pitch$、$\varphi=roll$，旋转矩阵为：

$$
R_{armor2world}=R_z(\psi)R_y(\theta)R_x(\varphi)
$$

各轴旋转矩阵分别为：

$$
R_z(\psi)=\begin{bmatrix}
\cos\psi&-\sin\psi&0\\
\sin\psi&\cos\psi&0\\
0&0&1
\end{bmatrix}
$$

$$
R_y(\theta)=\begin{bmatrix}
\cos\theta&0&\sin\theta\\
0&1&0\\
-\sin\theta&0&\cos\theta
\end{bmatrix}
$$

$$
R_x(\varphi)=\begin{bmatrix}
1&0&0\\
0&\cos\varphi&-\sin\varphi\\
0&\sin\varphi&\cos\varphi
\end{bmatrix}
$$

内禀旋转依次绕更新后的 Z、Y、X 轴；等价的固定轴旋转顺序是 X、Y、Z。不能把两者都解释为“依次绕固定 Z、Y、X 轴”。

**代码实现：**
```cpp
armor.ypr_in_world = rm_utils::eulers(R_armor2world, 2, 1, 0);  // Z-Y-X 顺序
```

**欧拉角含义：**

- **Yaw**：装甲板水平方向的朝向
- **Pitch**：装甲板绕局部 Y 轴的姿态角，不等同于目标位置的仰角
- **Roll**：装甲板滚转角度（通常接近 0）

---

##### 5. 转换为极坐标：$XYZ \rightarrow YPD$

将笛卡尔坐标 $(x, y, z)$ 映射到球坐标，描述目标相对于云台转轴中心的位置信息。

**数学公式：**

**Yaw**（方位角）：

$$
\phi=\operatorname{atan2}(y,x)
$$

表示目标在水平面的方位角。

**Pitch**（仰角）：

$$
\theta=\operatorname{atan2}\left(z,\sqrt{x^2+y^2}\right)
$$

表示目标相对于水平面的仰角，正值表示目标在上方；**不是与 Z 轴的夹角**。

**Distance**（距离）：

$$
d=\sqrt{x^2+y^2+z^2}
$$

表示装甲板中心到云台转轴中心的直线距离。YPD 的单位依次为弧度、弧度、米。

**代码实现：**
```cpp
armor.ypd_in_world = rm_utils::xyz2ypd(armor.xyz_in_world);
```

**YPR 与 YPD 不同**：YPR 描述板自身姿态，YPD 描述板中心相对于坐标原点的方向和距离。YPD 的正 pitch 表示目标在水平面上方；YPR 的 pitch 按 $R_y$ 右手旋转定义，不能直接当作相同符号的瞄准仰角。

---
