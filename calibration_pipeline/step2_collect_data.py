"""
Step 2: 真机数据采集脚本模板
用法:
  python step2_collect_data.py --robot Semi_Taks_T1 --n_poses 30
  python step2_collect_data.py --robot Taks_T1 --n_poses 30 --sim   # 仿真模式(无真机)

功能:
  1. 读取 Step1 生成的候选姿态 candidate_poses.csv
  2. 逐个发送关节角度指令到真机 (通过 ROS topic 或 SDK 接口)
  3. 每个姿态稳定后, 采集:
       - 关节角度 q (编码器)
       - 末端/标记点位姿 PEE (外部测量: mocap/激光跟踪仪/视觉)
  4. 数据存储为 data/<robot>/measurements.csv
     格式: q0...qN, pee_x, pee_y, pee_z [, pee_rx, pee_ry, pee_rz]

接口适配:
  修改 RobotInterface 类中的 send_joints() / read_joints() / read_pee()
  以对接实际机器人SDK (unitree SDK / ROS2 / 串口 等)
"""

import argparse
import time
import numpy as np
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from common.urdf_utils import urdf_to_pinocchio

# ── 配置 ──────────────────────────────────────────────────────────────
ROBOT_CONFIGS = {
    "Semi_Taks_T1": {
        "urdf": Path(__file__).parent.parent / "Semi_Taks_T1" / "Semi_Taks_T1.urdf",
        "pkg_dir": Path(__file__).parent.parent / "Semi_Taks_T1",
        "floating_base": True,
        "n_joints": 20,             # 实际驱动关节数 (不含浮动基座)
        "settle_time": 2.0,         # 运动到位后等待稳定时间(秒)
        "move_timeout": 5.0,        # 单次运动超时(秒)
        "joint_names": [            # 与真机关节顺序一致
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint",
            "neck_yaw_joint", "neck_roll_joint", "neck_pitch_joint",
        ],
        # 末端帧名(用于前向运动学计算参考值)
        "end_frame": "left_wrist_pitch_link",
        "start_frame": "base_link",
        # 测量自由度: [x, y, z, rx, ry, rz] True=测量
        "measurability": [True, True, True, False, False, False],
    },
    "Taks_T1": {
        "urdf": Path(__file__).parent.parent / "Taks_T1" / "Taks_T1.urdf",
        "pkg_dir": Path(__file__).parent.parent / "Taks_T1",
        "floating_base": True,
        "n_joints": 32,
        "settle_time": 2.0,
        "move_timeout": 5.0,
        "joint_names": [
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint",
            "neck_yaw_joint", "neck_roll_joint", "neck_pitch_joint",
        ],
        "end_frame": "left_ankle_roll_link",
        "start_frame": "pelvis",
        "measurability": [True, True, True, False, False, False],
    },
}


# ── 真机接口 (需根据实际SDK修改) ──────────────────────────────────────
class RobotInterface:
    """
    真机/仿真接口抽象层
    真机模式: 实现 send_joints / read_joints / read_pee
    仿真模式: 用Pinocchio前向运动学计算末端位置作为"测量值" + 添加噪声
    """

    def __init__(self, robot_name: str, cfg: dict, sim_mode: bool = False):
        self.cfg = cfg
        self.sim_mode = sim_mode
        self.robot_name = robot_name

        # 加载Pinocchio (仿真模式 或 真机模式均用于参考)
        self.robot = urdf_to_pinocchio(
            str(cfg["urdf"]), str(cfg["pkg_dir"]), cfg["floating_base"]
        )
        self.model = self.robot.model
        self.data = self.robot.data

        if sim_mode:
            print("  [仿真模式] 使用Pinocchio前向运动学模拟测量值 + 高斯噪声")
        else:
            print("  [真机模式] 请确保机器人已连接并处于安全状态!")
            self._init_hardware()

    def _init_hardware(self):
        """初始化真机连接 —— 按实际SDK修改"""
        # 示例: ROS2
        # import rclpy
        # from sensor_msgs.msg import JointState
        # ...
        # 示例: Unitree SDK
        # from unitree_sdk2py.core.channel import ChannelSubscriberFactory
        # ...
        print("  [TODO] 请在 RobotInterface._init_hardware() 中实现真机连接")
        print("  支持方式: ROS2 topic / Unitree SDK / 串口 / 自定义TCP")

    def send_joints(self, q_joints: np.ndarray, timeout: float = 5.0) -> bool:
        """
        发送关节角度指令到真机
        q_joints: shape (n_joints,), 单位 rad, 顺序与 cfg['joint_names'] 一致
        返回: True=到位, False=超时/失败
        """
        if self.sim_mode:
            return True   # 仿真模式直接成功
        # ── 真机实现 ──
        # 示例: ROS2
        # msg = JointState()
        # msg.name = self.cfg['joint_names']
        # msg.position = q_joints.tolist()
        # self.pub.publish(msg)
        # ... 等待到位
        raise NotImplementedError("请实现 send_joints() 真机接口")

    def read_joints(self) -> np.ndarray:
        """
        读取真机当前关节角度
        返回: shape (n_joints,), 单位 rad
        """
        if self.sim_mode:
            # 仿真模式: 返回上次设置的值 (由外部维护)
            return getattr(self, "_last_q_joints", np.zeros(len(self.cfg["joint_names"])))
        # ── 真机实现 ──
        raise NotImplementedError("请实现 read_joints() 真机接口")

    def read_pee(self, q_joints: np.ndarray) -> np.ndarray:
        """
        读取末端/标记点位姿
        真机模式: 从外部测量设备读取 (mocap/激光跟踪仪)
        仿真模式: Pinocchio前向运动学 + 噪声

        返回: shape (n_measured_dofs,), 按 measurability 过滤后的 [x,y,z,rx,ry,rz] 子集
        """
        if self.sim_mode:
            return self._sim_read_pee(q_joints)
        # ── 真机实现 ──
        # 示例: 从mocap系统读取标记点位置
        # marker_pos = self.mocap_client.get_marker_position("end_effector")
        # return np.array(marker_pos)
        raise NotImplementedError("请实现 read_pee() 真机接口")

    def _sim_read_pee(self, q_joints: np.ndarray) -> np.ndarray:
        """仿真模式: Pinocchio FK + 噪声模拟测量"""
        import pinocchio as pin

        # 构造完整q向量
        q_full = pin.neutral(self.model)
        if self.model.joints[1].shortname() == "JointModelFreeFlyer":
            q_full[2] = 0.8
            q_full[6] = 1.0   # quat w=1

        # 填入关节角
        for i, jname in enumerate(self.cfg["joint_names"]):
            jid = self.model.getJointId(jname)
            idx = self.model.joints[jid].idx_q
            q_full[idx] = q_joints[i]

        # 前向运动学
        pin.framesForwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        end_id = self.model.getFrameId(self.cfg["end_frame"])
        oMee = self.data.oMf[end_id]
        trans = oMee.translation.tolist()
        orient = pin.rpy.matrixToRpy(oMee.rotation).tolist()
        loc = trans + orient

        # 添加高斯噪声 (模拟测量误差)
        noise_pos = np.random.normal(0, 5e-4, 3)    # 0.5mm
        noise_ori = np.random.normal(0, 1e-3, 3)    # 0.001 rad
        noise = np.concatenate([noise_pos, noise_ori])

        measurability = self.cfg["measurability"]
        result = []
        for i, mea in enumerate(measurability):
            if mea:
                result.append(loc[i] + noise[i])
        return np.array(result)


# ── 数据采集主流程 ────────────────────────────────────────────────────
def load_candidate_poses(robot_name: str) -> np.ndarray:
    """读取Step1生成的候选姿态"""
    csv_path = Path(__file__).parent / "data" / robot_name / "candidate_poses.csv"
    if not csv_path.exists():
        raise FileNotFoundError(
            f"候选姿态文件不存在: {csv_path}\n请先运行: python step1_visualize.py --robot {robot_name}"
        )
    poses = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            poses.append([float(v) for v in row.values()])
    return np.array(poses)


def extract_joint_angles(q_full: np.ndarray, model, joint_names: list) -> np.ndarray:
    """从完整q向量中提取指定关节角度"""
    import pinocchio as pin
    vals = []
    for jname in joint_names:
        jid = model.getJointId(jname)
        idx = model.joints[jid].idx_q
        vals.append(q_full[idx])
    return np.array(vals)


def collect_data(robot_name: str, n_poses: int, sim_mode: bool,
                  output_path: Path) -> None:
    """主采集循环"""
    cfg = ROBOT_CONFIGS[robot_name]
    np.random.seed(0)

    print(f"\n[Step 2] 数据采集 — 机器人: {robot_name}")
    iface = RobotInterface(robot_name, cfg, sim_mode=sim_mode)

    # 加载候选姿态
    all_poses = load_candidate_poses(robot_name)
    n_available = len(all_poses)
    if n_poses > n_available:
        print(f"  [警告] 请求 {n_poses} 个姿态, 但只有 {n_available} 个可用, 使用全部")
        n_poses = n_available
    selected = all_poses[:n_poses]

    measurability = cfg["measurability"]
    n_pee_dofs = sum(measurability)
    joint_names = cfg["joint_names"]
    n_joints = len(joint_names)

    # 表头
    q_cols = [f"q_{jname}" for jname in joint_names]
    dof_labels = ["pee_x", "pee_y", "pee_z", "pee_rx", "pee_ry", "pee_rz"]
    pee_cols = [dof_labels[i] for i, m in enumerate(measurability) if m]
    header = q_cols + pee_cols

    output_path.parent.mkdir(parents=True, exist_ok=True)
    collected = []
    skipped = 0

    print(f"\n  开始采集 {n_poses} 个姿态...")
    print(f"  {'姿态':>6}  {'状态':<12}  末端位置(xyz)")
    print(f"  {'-'*55}")

    for i, q_full in enumerate(selected):
        q_joints = extract_joint_angles(q_full, iface.model, joint_names)

        # 发送指令
        success = iface.send_joints(q_joints, timeout=cfg["move_timeout"])
        if not success:
            print(f"  {i+1:>6}  {'超时/失败':<12}  跳过")
            skipped += 1
            continue

        # 存储当前关节值供仿真模式使用
        iface._last_q_joints = q_joints

        # 等待稳定
        time.sleep(cfg["settle_time"])

        # 读取当前关节角 (真机可能有偏差)
        q_actual = iface.read_joints() if not sim_mode else q_joints

        # 读取末端测量
        pee = iface.read_pee(q_actual)

        row = list(q_actual) + list(pee)
        collected.append(row)

        pos_str = f"[{pee[0]:.3f}, {pee[1]:.3f}, {pee[2]:.3f}]"
        print(f"  {i+1:>6}  {'OK':<12}  {pos_str}")

    # 保存CSV
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(collected)

    print(f"\n  采集完成: {len(collected)} 条有效 / {skipped} 条跳过")
    print(f"  数据保存: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Step2: 真机数据采集")
    parser.add_argument("--robot", choices=list(ROBOT_CONFIGS.keys()), required=True)
    parser.add_argument("--n_poses", type=int, default=30, help="采集姿态数")
    parser.add_argument("--sim", action="store_true",
                        help="仿真模式(无需真机, 用FK+噪声模拟测量)")
    args = parser.parse_args()

    out_path = Path(__file__).parent / "data" / args.robot / "measurements.csv"
    collect_data(args.robot, args.n_poses, args.sim, out_path)

    print(f"\n[Step 2 完成]")
    print(f"  下一步: python step3_calibrate_urdf.py --robot {args.robot}")


if __name__ == "__main__":
    main()
