"""
Step 1: 仿真可视化 & 标定姿态预规划
用法:
  python step1_visualize.py --robot Semi_Taks_T1
  python step1_visualize.py --robot Taks_T1
  python step1_visualize.py --robot Semi_Taks_T1 --mode trajectory   # 播放随机激励轨迹
  python step1_visualize.py --robot Taks_T1 --mode calibration  # 显示标定候选姿态

功能:
  1. 从URDF加载模型, 在Meshcat浏览器中展示初始姿态
  2. 检查关节范围, 自动采样随机姿态集
  3. 播放预规划激励轨迹并检查碰撞/可达性
  4. 将通过检查的姿态保存到 data/<robot>/candidate_poses.csv
"""

import argparse
import time
import numpy as np
import pinocchio as pin
from pathlib import Path
import csv
import sys

sys.path.insert(0, str(Path(__file__).parent))
from common.urdf_utils import urdf_to_pinocchio, get_revolute_joints, load_urdf_tree

# ── 配置 ──────────────────────────────────────────────────────────────
ROBOT_CONFIGS = {
    "Semi_Taks_T1": {
        "urdf": Path(__file__).parent.parent / "Semi_Taks_T1" / "Semi_Taks_T1.urdf",
        "pkg_dir": Path(__file__).parent.parent / "Semi_Taks_T1",
        "root_link": "base_link",
        "floating_base": True,
        "active_joints": [
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint",
            "neck_yaw_joint", "neck_roll_joint", "neck_pitch_joint",
        ],
        "calib_joints": [   # 标定时关注的关节子集 (运动学链)
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint",
        ],
    },
    "Taks_T1": {
        "urdf": Path(__file__).parent.parent / "Taks_T1" / "Taks_T1.urdf",
        "pkg_dir": Path(__file__).parent.parent / "Taks_T1",
        "root_link": "pelvis",
        "floating_base": True,
        "active_joints": [
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
        "calib_joints": [
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        ],
    },
}


def get_joint_limits(model, joint_names: list) -> tuple:
    """从Pinocchio模型获取关节上下限"""
    lowers, uppers = [], []
    for jname in joint_names:
        jid = model.getJointId(jname)
        idx = model.joints[jid].idx_q
        lowers.append(float(model.lowerPositionLimit[idx]))
        uppers.append(float(model.upperPositionLimit[idx]))
    return np.array(lowers), np.array(uppers)


def get_joint_q_indices(model, joint_names: list) -> list:
    """获取关节在q向量中的下标"""
    indices = []
    for jname in joint_names:
        jid = model.getJointId(jname)
        idx = model.joints[jid].idx_q
        indices.append(idx)
    return indices


def sample_random_poses(model, data, joint_names: list, n_poses: int = 50,
                         soft_ratio: float = 0.7, seed: int = 42) -> np.ndarray:
    """在关节范围内随机采样姿态, 软限制 soft_ratio 避免极端角度"""
    np.random.seed(seed)
    lowers, uppers = get_joint_limits(model, joint_names)
    q_indices = get_joint_q_indices(model, joint_names)
    nq = model.nq
    poses = []
    q_base = pin.neutral(model)

    for _ in range(n_poses):
        q = q_base.copy()
        center = (lowers + uppers) / 2.0
        half_range = (uppers - lowers) / 2.0 * soft_ratio
        vals = center + np.random.uniform(-1, 1, len(joint_names)) * half_range
        vals = np.clip(vals, lowers, uppers)
        for i, idx in enumerate(q_indices):
            q[idx] = vals[i]
        # 对于有freejoint的机器人，保持基座在合理高度
        if model.joints[1].shortname() == "JointModelFreeFlyer":
            q[2] = 0.8   # 离地高度 (Semi: 上半身固定台; Taks_T1: 全身)
            q[3:7] = [0, 0, 0, 1]   # 单位四元数 [x,y,z,w]
        poses.append(q.copy())
    return np.array(poses)


def check_self_collision(robot, q: np.ndarray) -> bool:
    """检查给定姿态是否自碰撞 (需要几何模型)"""
    try:
        pin.computeCollisions(robot.model, robot.data,
                              robot.collision_model, robot.collision_data, q, False)
        for cr in robot.collision_data.collisionResults:
            if cr.isCollision():
                return True
        return False
    except Exception:
        return False


def visualize_with_meshcat(robot, poses: np.ndarray, joint_names: list,
                            mode: str = "static", delay: float = 1.0):
    """在Meshcat中可视化姿态序列"""
    try:
        from pinocchio.visualize import MeshcatVisualizer
        viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
        viz.initViewer(loadModel=True)
        print(f"\n  Meshcat 浏览器地址: {viz.viewer.url()}")
        print("  请在浏览器中打开上面的地址查看可视化")

        if mode == "static":
            print(f"\n  播放 {len(poses)} 个姿态, 间隔 {delay}s ...")
            for i, q in enumerate(poses):
                viz.display(q)
                print(f"    姿态 {i+1}/{len(poses)}", end="\r")
                time.sleep(delay)

        elif mode == "trajectory":
            # 三角波插值: 在姿态之间平滑运动
            print(f"\n  播放插值轨迹 ({len(poses)} 个关键帧) ...")
            n_interp = 30
            for i in range(len(poses) - 1):
                q0, q1 = poses[i], poses[i + 1]
                for t in np.linspace(0, 1, n_interp):
                    q_t = q0 + t * (q1 - q0)
                    # 归一化四元数 (freejoint部分)
                    if robot.model.joints[1].shortname() == "JointModelFreeFlyer":
                        quat = q_t[3:7]
                        norm = np.linalg.norm(quat)
                        if norm > 1e-9:
                            q_t[3:7] = quat / norm
                    viz.display(q_t)
                    time.sleep(0.033)

        elif mode == "calibration":
            print(f"\n  显示标定候选姿态 (共 {len(poses)} 个) ...")
            print("  按 Ctrl+C 退出")
            idx = 0
            while True:
                q = poses[idx % len(poses)]
                viz.display(q)
                print(f"    第 {idx % len(poses) + 1}/{len(poses)} 个标定姿态", end="\r")
                time.sleep(2.0)
                idx += 1

        return viz
    except ImportError:
        print("[错误] 未找到 MeshcatVisualizer, 请安装 meshcat: pip install meshcat")
        return None


def save_poses_to_csv(poses: np.ndarray, output_path: Path):
    """保存姿态到CSV (每行一个q向量)"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([f"q{i}" for i in range(poses.shape[1])])
        for q in poses:
            writer.writerow([f"{v:.8f}" for v in q])
    print(f"  已保存 {len(poses)} 个姿态 -> {output_path}")


def print_robot_info(robot, joint_names: list):
    """打印机器人基本信息"""
    model = robot.model
    print(f"\n{'='*50}")
    print(f"  模型: {model.name}")
    print(f"  关节数: {model.njoints-1}  nq={model.nq}  nv={model.nv}")
    print(f"  标定关节数: {len(joint_names)}")
    lowers, uppers = get_joint_limits(model, joint_names)
    print(f"\n  关节名                           下限(rad)  上限(rad)")
    print(f"  {'-'*50}")
    for i, name in enumerate(joint_names):
        print(f"  {name:<35} {lowers[i]:>8.3f}  {uppers[i]:>8.3f}")
    print(f"{'='*50}")


def main():
    parser = argparse.ArgumentParser(description="Step1: 仿真可视化与标定姿态规划")
    parser.add_argument("--robot", choices=list(ROBOT_CONFIGS.keys()), required=True)
    parser.add_argument("--mode", choices=["static", "trajectory", "calibration"],
                        default="static", help="可视化模式")
    parser.add_argument("--n_poses", type=int, default=30, help="随机姿态数量")
    parser.add_argument("--delay", type=float, default=1.0, help="姿态切换间隔(秒)")
    parser.add_argument("--no_collision_check", action="store_true",
                        help="跳过碰撞检测(加速)")
    args = parser.parse_args()

    cfg = ROBOT_CONFIGS[args.robot]
    urdf_path = str(cfg["urdf"])
    pkg_dir = str(cfg["pkg_dir"])

    print(f"\n[Step 1] 加载机器人: {args.robot}")
    print(f"  URDF: {urdf_path}")

    # 加载Pinocchio模型
    robot = urdf_to_pinocchio(urdf_path, pkg_dir, floating_base=cfg["floating_base"])
    print_robot_info(robot, cfg["calib_joints"])

    # 采样随机姿态
    print(f"\n[采样] 生成 {args.n_poses} 个随机姿态...")
    all_poses = sample_random_poses(robot.model, robot.data,
                                     cfg["calib_joints"], n_poses=args.n_poses)

    # 碰撞过滤
    if not args.no_collision_check:
        print("[碰撞检测] 过滤自碰撞姿态...")
        valid_poses = []
        for i, q in enumerate(all_poses):
            if not check_self_collision(robot, q):
                valid_poses.append(q)
        valid_poses = np.array(valid_poses)
        print(f"  有效姿态: {len(valid_poses)}/{len(all_poses)}")
    else:
        valid_poses = all_poses
        print(f"  (跳过碰撞检测) 使用全部 {len(valid_poses)} 个姿态")

    if len(valid_poses) == 0:
        print("[错误] 无有效姿态，请检查关节限位或调整 soft_ratio")
        return

    # 保存候选姿态
    out_dir = Path(__file__).parent / "data" / args.robot
    save_poses_to_csv(valid_poses, out_dir / "candidate_poses.csv")

    # 可视化
    print(f"\n[可视化] 模式: {args.mode}")
    viz = visualize_with_meshcat(robot, valid_poses, cfg["calib_joints"],
                                  mode=args.mode, delay=args.delay)

    if viz is not None:
        input("\n  按回车退出可视化...")

    print(f"\n[Step 1 完成] 候选姿态已保存到: {out_dir}/candidate_poses.csv")
    print(f"  下一步: python step2_collect_data.py --robot {args.robot}")


if __name__ == "__main__":
    main()
