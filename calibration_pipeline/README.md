# 机器人URDF标定流程 (Semi_Taks_T1 & Taks_T1)

## 流程概览

```
Step 1: 仿真可视化 & 候选姿态规划
    ↓ candidate_poses.csv
Step 2: 真机数据采集 (或仿真模拟)
    ↓ measurements.csv
Step 3: URDF几何标定优化
    ↓ calib_results.yaml + *_calibrated.urdf
Step 4: 一键同步参数到 MuJoCo XML
    ↓ 更新后的 *.xml
```

## 快速开始

### 仿真模式（无真机，全流程验证）

```bash
cd /home/xhz/figaroh-plus/calibration_pipeline

# Semi_Taks_T1 全流程（仿真）
python run_pipeline.py --robot Semi_Taks_T1 --sim --skip_viz

# Taks_T1 全流程（仿真）
python run_pipeline.py --robot Taks_T1 --sim --skip_viz
```

### 真机模式（分步执行）

```bash
# Step 1: 仿真可视化，查看姿态（打开浏览器）
python step1_visualize.py --robot Taks_T1 --mode calibration

# Step 2: 真机数据采集（需先实现 RobotInterface 接口）
python step2_collect_data.py --robot Taks_T1 --n_poses 30

# Step 3: 标定优化
python step3_calibrate_urdf.py --robot Taks_T1 --plot

# Step 4: 一键同步到 MuJoCo XML
python step4_urdf_to_mujoco.py --robot Taks_T1
```

---

## 各步骤详细说明

### Step 1: 仿真可视化 (`step1_visualize.py`)

在 Meshcat 浏览器中预览机器人并生成候选标定姿态。

```bash
# 静态展示随机姿态
python step1_visualize.py --robot Semi_Taks_T1 --mode static --n_poses 30

# 播放插值轨迹（激励轨迹预览）
python step1_visualize.py --robot Taks_T1 --mode trajectory

# 循环展示标定候选姿态
python step1_visualize.py --robot Taks_T1 --mode calibration
```

**参数说明：**
| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--robot` | 机器人名称 | 必填 |
| `--mode` | `static` / `trajectory` / `calibration` | `static` |
| `--n_poses` | 随机姿态数量 | `30` |
| `--delay` | 姿态切换间隔(秒) | `1.0` |
| `--no_collision_check` | 跳过碰撞检测 | False |

**输出：** `data/<robot>/candidate_poses.csv`

---

### Step 2: 数据采集 (`step2_collect_data.py`)

**仿真模式（无真机）：**
```bash
python step2_collect_data.py --robot Taks_T1 --sim --n_poses 30
```

**真机模式（需适配SDK）：**

打开 `step2_collect_data.py`，修改 `RobotInterface` 类中的三个方法：

```python
def send_joints(self, q_joints: np.ndarray, timeout: float) -> bool:
    # 向真机发送关节角度指令
    # 示例 (Unitree SDK):
    # self.low_cmd.motorCmd[i].q = q_joints[i]
    # self.safe.PositionLimit(self.low_cmd)
    # self.udp.SetSend(self.low_cmd)
    # self.udp.Send()
    pass

def read_joints(self) -> np.ndarray:
    # 读取真机当前关节角度
    # 示例 (ROS2):
    # return np.array(self.joint_state_msg.position)
    pass

def read_pee(self, q_joints: np.ndarray) -> np.ndarray:
    # 读取末端标记点位置
    # 方式1: mocap系统 (OptiTrack/Vicon)
    # 方式2: 激光跟踪仪 (Leica AT960)
    # 方式3: 视觉定位 (AprilTag)
    pass
```

**输出：** `data/<robot>/measurements.csv`

---

### Step 3: URDF 标定优化 (`step3_calibrate_urdf.py`)

使用 Levenberg-Marquardt 优化各关节的 RPY 偏置，最小化末端位置误差。

```bash
# 基本用法
python step3_calibrate_urdf.py --robot Taks_T1

# 带结果可视化
python step3_calibrate_urdf.py --robot Taks_T1 --plot

# 调整优化参数
python step3_calibrate_urdf.py --robot Taks_T1 --method lm --max_iter 5 --outlier_thresh 2.5
```

**参数说明：**
| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--method` | 优化方法 `lm`/`trf`/`dogbox` | `lm` |
| `--max_iter` | 最大异常值剔除轮次 | `3` |
| `--outlier_thresh` | 异常值阈值（标准差倍数） | `3.0` |
| `--plot` | 绘制标定前后残差对比图 | False |

**输出：**
- `data/<robot>/calib_results.yaml` — 标定结果报告（RMSE、各关节偏置）
- `<robot_dir>/<robot>_calibrated.urdf` — 优化后的 URDF

**结果示例（calib_results.yaml）：**
```yaml
robot: Taks_T1
rmse_mm: 1.234
mae_mm: 0.987
n_samples_used: 28
n_outliers: 2
calibrated_joints:
  left_hip_pitch_joint:
    offset_rad: 0.003142
    offset_deg: 0.18
    xyz_delta: [0.0, 0.0, 0.0]
    rpy_delta: [0.0, 0.003142, 0.0]
```

---

### Step 4: 同步到 MuJoCo XML (`step4_urdf_to_mujoco.py`)

将优化后 URDF 的所有参数精确写入 MuJoCo XML。

```bash
# 自动使用最新标定URDF（*_calibrated.urdf）
python step4_urdf_to_mujoco.py --robot Taks_T1

# 指定自定义URDF
python step4_urdf_to_mujoco.py --robot Taks_T1 --urdf /path/to/custom.urdf

# 只同步关节位置（跳过惯量参数）
python step4_urdf_to_mujoco.py --robot Taks_T1 --joint_only

# 只同步惯量参数（跳过关节位置）
python step4_urdf_to_mujoco.py --robot Taks_T1 --inertial_only

# 输出到指定路径（不覆盖原文件）
python step4_urdf_to_mujoco.py --robot Taks_T1 --output /tmp/test.xml
```

**同步内容：**
| URDF 字段 | MuJoCo XML 字段 |
|-----------|----------------|
| `joint/origin xyz` | `body pos` |
| `joint/origin rpy` | `body quat`（自动转换） |
| `link/inertial/mass` | `body/inertial mass` |
| `link/inertial/origin xyz` | `body/inertial pos` |
| `link/inertial/inertia ixx/iyy/izz/ixy/ixz/iyz` | `body/inertial diaginertia + quat`（主轴分解） |

**原始 XML 自动备份为：** `<robot>.xml.bak.<timestamp>`

---

## 目录结构

```
calibration_pipeline/
├── run_pipeline.py           # 一键运行全流程
├── step1_visualize.py        # 仿真可视化
├── step2_collect_data.py     # 数据采集
├── step3_calibrate_urdf.py   # URDF标定优化
├── step4_urdf_to_mujoco.py   # 同步到MuJoCo XML
├── common/
│   ├── urdf_utils.py         # URDF解析/更新工具
│   └── mujoco_utils.py       # MuJoCo XML工具
├── config/
│   ├── Semi_Taks_T1.yaml     # Semi_Taks_T1配置
│   └── Taks_T1.yaml          # Taks_T1配置
└── data/                     # 运行时生成
    ├── Semi_Taks_T1/
    │   ├── candidate_poses.csv
    │   ├── measurements.csv
    │   ├── calib_results.yaml
    │   └── calib_plot.png
    └── Taks_T1/
        └── ...
```

## 依赖

```bash
pip install pinocchio scipy numpy pyyaml meshcat matplotlib
```

---

## 标定原理

**几何标定**（`step3`）识别各关节的安装偏差：

- **标定参数**: 每个关节沿旋转轴方向的角度偏置 δθ
- **测量模型**: PEE_measured = FK(q + δθ, URDF)
- **优化目标**: min Σ ||PEE_measured_i - FK(q_i + δθ)||²
- **算法**: Levenberg-Marquardt + 迭代异常值剔除（3σ准则）

**惯量参数同步**（`step4`）：

URDF 中的 6 分量惯量矩阵经主轴分解（特征值分解）转换为 MuJoCo 所需的 `diaginertia + quat` 格式，精度保留到 10 位有效数字。
