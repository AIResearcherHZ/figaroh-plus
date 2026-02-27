"""
一键运行完整标定流程
用法:
  python run_pipeline.py --robot Semi_Taks_T1 --sim          # 仿真模式 (无真机)
  python run_pipeline.py --robot Taks_T1 --sim
  python run_pipeline.py --robot Taks_T1 --steps 1,3,4       # 只运行指定步骤
  python run_pipeline.py --robot Taks_T1 --skip_viz          # 跳过可视化

步骤说明:
  1 - 仿真可视化, 生成候选姿态
  2 - 数据采集 (真机/仿真)
  3 - URDF 几何标定优化
  4 - 同步优化参数到 MuJoCo XML
"""

import argparse
import subprocess
import sys
from pathlib import Path

STEPS = {
    1: ("step1_visualize.py",      "仿真可视化 & 候选姿态生成"),
    2: ("step2_collect_data.py",   "真机数据采集"),
    3: ("step3_calibrate_urdf.py", "URDF几何标定优化"),
    4: ("step4_urdf_to_mujoco.py", "同步参数到MuJoCo XML"),
}

SCRIPT_DIR = Path(__file__).parent


def run_step(step_num: int, robot: str, extra_args: list) -> bool:
    script, desc = STEPS[step_num]
    print(f"\n{'#'*60}")
    print(f"# Step {step_num}: {desc}")
    print(f"{'#'*60}")

    cmd = [sys.executable, str(SCRIPT_DIR / script), "--robot", robot] + extra_args
    print(f"  命令: {' '.join(cmd)}")

    result = subprocess.run(cmd, cwd=str(SCRIPT_DIR))
    if result.returncode != 0:
        print(f"\n[错误] Step {step_num} 失败 (returncode={result.returncode})")
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description="一键运行完整标定流程")
    parser.add_argument("--robot", required=True,
                        choices=["Semi_Taks_T1", "Taks_T1"])
    parser.add_argument("--sim", action="store_true",
                        help="使用仿真模式 (无需真机)")
    parser.add_argument("--steps", default="1,2,3,4",
                        help="运行哪些步骤, 逗号分隔, 默认全部")
    parser.add_argument("--skip_viz", action="store_true",
                        help="Step1中跳过交互式可视化等待")
    parser.add_argument("--n_poses", type=int, default=30,
                        help="采集/生成的姿态数量")
    parser.add_argument("--no_backup", action="store_true",
                        help="Step4: 不备份原XML")
    args = parser.parse_args()

    steps_to_run = [int(s.strip()) for s in args.steps.split(",")]
    robot = args.robot

    print(f"\n{'='*60}")
    print(f"  标定流程 — {robot}")
    print(f"  模式: {'仿真' if args.sim else '真机'}")
    print(f"  步骤: {steps_to_run}")
    print(f"{'='*60}")

    for step_num in steps_to_run:
        if step_num not in STEPS:
            print(f"[警告] 未知步骤 {step_num}, 跳过")
            continue

        # 组装各步骤的特定参数
        extra = []
        if step_num == 1:
            extra += ["--n_poses", str(args.n_poses)]
            if args.skip_viz:
                extra += ["--mode", "static", "--delay", "0.1"]
        elif step_num == 2:
            extra += ["--n_poses", str(args.n_poses)]
            if args.sim:
                extra.append("--sim")
        elif step_num == 3:
            pass   # 使用默认参数
        elif step_num == 4:
            if args.no_backup:
                extra.append("--no_backup")

        ok = run_step(step_num, robot, extra)
        if not ok:
            print(f"\n[中止] Step {step_num} 失败，停止后续步骤")
            sys.exit(1)

    print(f"\n{'='*60}")
    print(f"  [全部完成] 流程正常结束")
    data_dir = SCRIPT_DIR / "data" / robot
    urdf_dir = SCRIPT_DIR.parent / robot
    print(f"\n  产出文件:")
    print(f"  - 候选姿态  : {data_dir}/candidate_poses.csv")
    print(f"  - 测量数据  : {data_dir}/measurements.csv")
    print(f"  - 标定结果  : {data_dir}/calib_results.yaml")
    print(f"  - 标定URDF  : {urdf_dir}/{robot}_calibrated.urdf")
    print(f"  - 更新XML   : {urdf_dir}/{robot}.xml")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
