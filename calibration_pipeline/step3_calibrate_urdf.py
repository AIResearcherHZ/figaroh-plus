"""
Step 3 (v2): 基于 FIGAROH 框架的 URDF 几何标定
用法:
  python step3_calibrate_urdf_v2.py --robot Semi_Taks_T1
  python step3_calibrate_urdf_v2.py --robot Taks_T1
  python step3_calibrate_urdf_v2.py --robot Taks_T1 --model full_params  # 完整6参数/关节
  python step3_calibrate_urdf_v2.py --robot Taks_T1 --model joint_offset # 仅关节偏置

改进点 (相比v1):
  1. 支持 full_params 模式: 每关节6参数 (d_x,d_y,d_z,phi_x,phi_y,phi_z)
  2. 使用 QR 分解自动识别可观测参数 (需要FIGAROH)
  3. 带正则化的代价函数，提高数值稳定性
  4. 统一配置文件格式，与官方示例一致
"""

import argparse
import sys
import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from common.robot_calibration import (
    create_calibrator, load_unified_config, SimpleCalibration
)
from common.urdf_utils import (
    load_urdf_tree, apply_calibration_results, urdf_to_pinocchio
)

# ── 配置 ──────────────────────────────────────────────────────────────
ROBOT_CONFIGS = {
    "Semi_Taks_T1": {
        "urdf": Path(__file__).parent.parent / "Semi_Taks_T1" / "Semi_Taks_T1.urdf",
        "pkg_dir": Path(__file__).parent.parent / "Semi_Taks_T1",
        "config": Path(__file__).parent / "config" / "Semi_Taks_T1_unified.yaml",
        "floating_base": True,
    },
    "Taks_T1": {
        "urdf": Path(__file__).parent.parent / "Taks_T1" / "Taks_T1.urdf",
        "pkg_dir": Path(__file__).parent.parent / "Taks_T1",
        "config": Path(__file__).parent / "config" / "Taks_T1_unified.yaml",
        "floating_base": True,
    },
}


def params_to_urdf_delta(calibrator, calib_model: str) -> dict:
    """
    将标定参数转换为 URDF 更新格式
    返回: {joint_name: {'xyz_delta': [...], 'rpy_delta': [...]}}
    """
    params = calibrator.get_calibrated_params()
    chain_joints = calibrator.calib_config.get("chain_joints", [])
    
    result = {}
    
    if calib_model == "full_params":
        # 每关节6参数
        for jname in chain_joints:
            d_x = params.get(f"{jname}_d_x", 0.0)
            d_y = params.get(f"{jname}_d_y", 0.0)
            d_z = params.get(f"{jname}_d_z", 0.0)
            phi_x = params.get(f"{jname}_phi_x", 0.0)
            phi_y = params.get(f"{jname}_phi_y", 0.0)
            phi_z = params.get(f"{jname}_phi_z", 0.0)
            
            result[jname] = {
                "xyz_delta": [d_x, d_y, d_z],
                "rpy_delta": [phi_x, phi_y, phi_z],
            }
    else:
        # joint_offset: 偏置沿关节轴方向
        _, urdf_root = load_urdf_tree(str(calibrator._config_dir.parent / 
                                          calibrator.calib_config["robot_name"] /
                                          f"{calibrator.calib_config['robot_name']}.urdf"))
        
        for jname in chain_joints:
            offset = params.get(f"{jname}_offset", 0.0)
            
            # 找关节轴方向
            ax = [0, 0, 1]  # 默认Z轴
            for jel in urdf_root.findall("joint"):
                if jel.get("name") == jname:
                    axis_el = jel.find("axis")
                    if axis_el is not None:
                        ax = list(map(float, axis_el.get("xyz", "0 0 1").split()))
                    break
                    
            # 偏置转为rpy增量
            delta_rpy = [ax[k] * offset for k in range(3)]
            result[jname] = {
                "xyz_delta": [0.0, 0.0, 0.0],
                "rpy_delta": delta_rpy,
            }
            
    return result


def main():
    parser = argparse.ArgumentParser(description="Step3 v2: FIGAROH几何标定")
    parser.add_argument("--robot", choices=list(ROBOT_CONFIGS.keys()), required=True)
    parser.add_argument("--model", choices=["full_params", "joint_offset"],
                        default=None, help="标定模型 (默认从配置读取)")
    parser.add_argument("--method", default="lm", choices=["lm", "trf", "dogbox"])
    parser.add_argument("--max_iter", type=int, default=3)
    parser.add_argument("--outlier_thresh", type=float, default=3.0)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    cfg = ROBOT_CONFIGS[args.robot]
    config_file = str(cfg["config"])
    
    print(f"\n[Step 3 v2] FIGAROH 几何标定")
    print(f"  机器人: {args.robot}")
    print(f"  配置  : {config_file}")

    # 加载配置
    unified_config = load_unified_config(config_file)
    
    # 覆盖标定模型 (如果命令行指定)
    if args.model:
        unified_config["tasks"]["calibration"]["parameters"]["calibration_level"] = args.model
        
    calib_model = unified_config["tasks"]["calibration"]["parameters"]["calibration_level"]
    print(f"  模型  : {calib_model}")

    # 加载机器人
    robot = urdf_to_pinocchio(
        str(cfg["urdf"]), str(cfg["pkg_dir"]), cfg["floating_base"]
    )
    
    # 创建标定器
    print(f"\n[初始化]")
    calibrator = create_calibrator(robot, config_file)
    
    # 初始化
    calibrator.initialize()
    
    # 求解
    print(f"\n[优化]")
    result = calibrator.solve(
        method=args.method,
        max_iterations=args.max_iter,
        outlier_threshold=args.outlier_thresh,
        enable_logging=True,
        plotting=args.plot
    )
    
    # 打印参数
    params = calibrator.get_calibrated_params()
    print(f"\n  标定参数 ({len(params)} 个):")
    for name, val in params.items():
        if abs(val) > 1e-8:
            if "phi" in name or "offset" in name:
                print(f"    {name:<40} {val:>+12.6f} rad ({np.degrees(val):>+8.4f} deg)")
            else:
                print(f"    {name:<40} {val:>+12.6f} m   ({val*1000:>+8.4f} mm)")

    # 保存结果
    out_dir = Path(__file__).parent / "data" / args.robot
    out_dir.mkdir(parents=True, exist_ok=True)
    
    results_yaml = out_dir / "calib_results_v2.yaml"
    calibrator.save_results(str(results_yaml))
    
    # 转换为URDF更新格式
    calib_delta = params_to_urdf_delta(calibrator, calib_model)
    
    # 生成优化后的URDF
    src_urdf = str(cfg["urdf"])
    out_urdf = src_urdf.replace(".urdf", "_calibrated.urdf")
    
    print(f"\n[更新URDF]")
    apply_calibration_results(src_urdf, calib_delta, out_urdf)
    
    print(f"\n[Step 3 v2 完成]")
    print(f"  标定URDF : {out_urdf}")
    print(f"  结果YAML : {results_yaml}")
    print(f"  下一步   : python step4_urdf_to_mujoco.py --robot {args.robot}")


if __name__ == "__main__":
    main()
