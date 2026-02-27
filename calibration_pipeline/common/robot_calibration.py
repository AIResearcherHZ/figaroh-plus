"""
基于 FIGAROH 框架的机器人标定工具
继承 BaseCalibration，实现 Semi_Taks_T1 和 Taks_T1 的几何标定
"""

import sys
import logging
import numpy as np
import yaml
from yaml.loader import SafeLoader
from pathlib import Path
from typing import List, Dict, Any, Optional
from scipy.optimize import least_squares

# 添加 figaroh 路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

# 暂时禁用FIGAROH版本，因为数据格式不兼容
# 后续可以实现数据格式转换层
FIGAROH_AVAILABLE = False
CalibrationError = Exception
def handle_calibration_errors(func):
    return func

# try:
#     from figaroh.calibration.base_calibration import BaseCalibration
#     from figaroh.calibration.calibration_tools import (
#         load_data, calc_updated_fkm, get_param_from_yaml,
#         calculate_base_kinematics_regressor, add_base_name, add_pee_name,
#         initialize_variables
#     )
#     from figaroh.tools.robot import load_robot
#     from figaroh.utils.error_handling import CalibrationError, handle_calibration_errors
#     FIGAROH_AVAILABLE = True
# except ImportError as e:
#     print(f"[警告] FIGAROH 未完整安装: {e}")
#     print("  使用简化版标定器")
#     FIGAROH_AVAILABLE = False
#     CalibrationError = Exception
#     def handle_calibration_errors(func):
#         return func

import pinocchio as pin


# ── 配置解析 ─────────────────────────────────────────────────────────
def load_unified_config(config_path: str) -> Dict[str, Any]:
    """加载统一格式配置文件"""
    with open(config_path, "r") as f:
        config = yaml.load(f, Loader=SafeLoader)
    return config


def unified_to_calib_config(robot, config: Dict, task: str = "calibration") -> Dict:
    """
    将统一配置转换为 FIGAROH calib_config 格式
    """
    task_cfg = config.get("tasks", {}).get(task, {})
    params = task_cfg.get("parameters", {})
    kin = task_cfg.get("kinematics", {})
    meas = task_cfg.get("measurements", {})
    data_cfg = task_cfg.get("data", {})
    
    # 构建 measurability
    markers = meas.get("markers", [])
    if markers:
        measurability = markers[0].get("measurable_dof", [True]*6)
    else:
        measurability = [True, True, True, False, False, False]
    
    # 计算 calibration_index (测量自由度数)
    calibration_index = sum(measurability)
    
    # 获取活动关节
    active_joints = config.get("robot", {}).get("properties", {}).get(
        "joints", {}).get("active_joints", [])
    
    # 构建 calib_config
    calib_config = {
        "robot_name": config.get("robot", {}).get("name", "robot"),
        "calib_model": params.get("calibration_level", "full_params"),
        "non_geom": params.get("include_non_geometric", False),
        "coeff_regularize": params.get("regularization_coefficient", 0.001),
        "outlier_threshold": params.get("outlier_threshold", 0.02),
        
        "start_frame": kin.get("base_frame", "base_link"),
        "end_frame": kin.get("tool_frame", "end_effector"),
        
        "measurability": measurability,
        "calibration_index": calibration_index,
        "NbMarkers": len(markers) if markers else 1,
        "NbSample": data_cfg.get("number_of_samples", 30),
        
        "data_file": data_cfg.get("source_file", ""),
        "sample_configs_file": data_cfg.get("sample_configurations_file", ""),
        
        "active_joints": active_joints,
        "known_baseframe": True,   # 默认基座已知
        "known_tipframe": False,   # 默认末端未知
        
        "PLOT": True,
        "param_name": [],  # 由 create_param_list 填充
    }
    
    # 获取活动关节在模型中的索引
    if hasattr(robot, 'model'):
        model = robot.model
        actJoint_idx = []
        for jname in active_joints:
            if model.existJointName(jname):
                jid = model.getJointId(jname)
                actJoint_idx.append(jid)
        calib_config["actJoint_idx"] = actJoint_idx
    
    return calib_config


# ── 简化版标定器 (不依赖完整FIGAROH) ────────────────────────────────
class SimpleCalibration:
    """
    简化版几何标定器
    当 FIGAROH 不可用时使用，实现核心标定功能
    """
    
    def __init__(self, robot, config_file: str, del_list: List[int] = None):
        self.robot = robot
        self.model = robot.model
        self.data = robot.data
        self.del_list_ = del_list or []
        self.STATUS = "NOT CALIBRATED"
        
        # 加载配置
        config = load_unified_config(config_file)
        self.calib_config = unified_to_calib_config(robot, config)
        self._config_dir = Path(config_file).parent.parent
        
        # 解析数据路径
        data_file = self.calib_config["data_file"]
        if data_file:
            self._data_path = str(self._config_dir / data_file)
        else:
            self._data_path = None
            
        self.q_measured = None
        self.PEE_measured = None
        self.var_ = None
        self.LM_result = None
        
    def initialize(self):
        """初始化标定数据和参数"""
        self._load_data()
        self._create_param_list()
        print(f"  初始化完成: {self.calib_config['NbSample']} 样本, "
              f"{len(self.calib_config['param_name'])} 参数")
        
    def _load_data(self):
        """加载测量数据"""
        import csv
        
        if not self._data_path or not Path(self._data_path).exists():
            raise FileNotFoundError(f"数据文件不存在: {self._data_path}")
            
        # 解析CSV
        q_list, pee_list = [], []
        measurability = self.calib_config["measurability"]
        
        with open(self._data_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                # 提取关节角度 (q_开头的列)
                q = []
                for key in row:
                    if key.startswith("q_"):
                        q.append(float(row[key]))
                        
                # 提取末端位置 (pee_开头的列)
                pee = []
                dof_labels = ["pee_x", "pee_y", "pee_z", "pee_rx", "pee_ry", "pee_rz"]
                for i, label in enumerate(dof_labels):
                    if measurability[i] and label in row:
                        pee.append(float(row[label]))
                        
                q_list.append(q)
                pee_list.append(pee)
                
        self.q_measured = np.array(q_list)
        self.PEE_measured = np.array(pee_list).flatten()
        self.calib_config["NbSample"] = len(q_list)
        
        print(f"  加载数据: {len(q_list)} 样本, q维度={self.q_measured.shape[1]}")
        
    def _create_param_list(self):
        """创建标定参数列表"""
        calib_model = self.calib_config["calib_model"]
        active_joints = self.calib_config.get("active_joints", [])
        
        # 获取运动学链上的关节
        start_frame = self.calib_config["start_frame"]
        end_frame = self.calib_config["end_frame"]
        chain_joints = self._get_kinematic_chain_joints(start_frame, end_frame)
        
        param_names = []
        
        if calib_model == "full_params":
            # 完整6参数/关节: d_x, d_y, d_z, phi_x, phi_y, phi_z
            for jname in chain_joints:
                param_names.extend([
                    f"{jname}_d_x", f"{jname}_d_y", f"{jname}_d_z",
                    f"{jname}_phi_x", f"{jname}_phi_y", f"{jname}_phi_z"
                ])
        else:  # joint_offset
            # 仅关节角度偏置
            for jname in chain_joints:
                param_names.append(f"{jname}_offset")
                
        # 添加基座参数 (如果未知)
        if not self.calib_config.get("known_baseframe", True):
            param_names.extend([
                "base_d_x", "base_d_y", "base_d_z",
                "base_phi_x", "base_phi_y", "base_phi_z"
            ])
            
        # 添加末端参数 (如果未知)
        if not self.calib_config.get("known_tipframe", True):
            param_names.extend([
                "tip_d_x", "tip_d_y", "tip_d_z",
                "tip_phi_x", "tip_phi_y", "tip_phi_z"
            ])
            
        self.calib_config["param_name"] = param_names
        self.calib_config["chain_joints"] = chain_joints
        
    def _get_kinematic_chain_joints(self, start_frame: str, end_frame: str) -> List[str]:
        """获取从 start_frame 到 end_frame 的运动学链关节"""
        model = self.model
        
        # 获取帧ID (处理多个同名帧的情况)
        start_id = 0
        end_id = model.nframes - 1
        
        for i in range(model.nframes):
            frame = model.frames[i]
            if frame.name == start_frame:
                start_id = i
            if frame.name == end_frame:
                end_id = i
            
        # 从末端向基座回溯
        chain_joints = []
        current_joint = model.frames[end_id].parentJoint
        
        while current_joint > 0:
            jname = model.names[current_joint]
            jtype = model.joints[current_joint].shortname()
            
            # 只添加 revolute/prismatic 关节 (JointModelRX/RY/RZ/PX/PY/PZ)
            if any(x in jtype for x in ["RX", "RY", "RZ", "Revolute", "PX", "PY", "PZ", "Prismatic"]):
                chain_joints.append(jname)
                
            # 检查是否到达起始帧
            if model.existFrame(start_frame):
                start_joint = model.frames[start_id].parentJoint
                if current_joint == start_joint:
                    break
                    
            current_joint = model.parents[current_joint]
            
        chain_joints.reverse()
        return chain_joints
        
    def _build_full_q(self, q_joints: np.ndarray, params: np.ndarray = None) -> np.ndarray:
        """构建完整 q 向量，可选叠加参数偏置"""
        q = pin.neutral(self.model)
        
        # 浮动基座
        if self.model.joints[1].shortname() == "JointModelFreeFlyer":
            q[2] = 0.8
            q[6] = 1.0
            
        # 填入关节角
        active_joints = self.calib_config.get("active_joints", [])
        for i, jname in enumerate(active_joints):
            if i < len(q_joints) and self.model.existJointName(jname):
                jid = self.model.getJointId(jname)
                idx = self.model.joints[jid].idx_q
                q[idx] = q_joints[i]
                
        # 叠加关节偏置 (joint_offset 模式)
        if params is not None and self.calib_config["calib_model"] == "joint_offset":
            chain_joints = self.calib_config.get("chain_joints", [])
            for i, jname in enumerate(chain_joints):
                if i < len(params) and self.model.existJointName(jname):
                    jid = self.model.getJointId(jname)
                    idx = self.model.joints[jid].idx_q
                    q[idx] += params[i]
                    
        return q
        
    def _compute_fk(self, q_full: np.ndarray) -> np.ndarray:
        """计算末端位姿"""
        pin.framesForwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        
        end_frame = self.calib_config["end_frame"]
        if self.model.existFrame(end_frame):
            end_id = self.model.getFrameId(end_frame)
        else:
            end_id = self.model.nframes - 1
            
        oMee = self.data.oMf[end_id]
        trans = oMee.translation.tolist()
        orient = pin.rpy.matrixToRpy(oMee.rotation).tolist()
        
        # 按 measurability 过滤
        measurability = self.calib_config["measurability"]
        loc = trans + orient
        return np.array([loc[i] for i, m in enumerate(measurability) if m])
        
    def _calc_updated_fkm(self, params: np.ndarray) -> np.ndarray:
        """
        计算带参数更新的前向运动学
        full_params 模式: 修改关节 placement
        joint_offset 模式: 叠加关节角度偏置
        """
        calib_model = self.calib_config["calib_model"]
        chain_joints = self.calib_config.get("chain_joints", [])
        n_samples = self.calib_config["NbSample"]
        measurability = self.calib_config["measurability"]
        n_meas = sum(measurability)
        
        PEE_est = []
        
        # 备份原始 placement
        original_placements = {}
        for jname in chain_joints:
            if self.model.existJointName(jname):
                jid = self.model.getJointId(jname)
                original_placements[jid] = self.model.jointPlacements[jid].copy()
        
        try:
            if calib_model == "full_params":
                # 应用几何参数修改
                param_idx = 0
                for jname in chain_joints:
                    if not self.model.existJointName(jname):
                        param_idx += 6
                        continue
                    jid = self.model.getJointId(jname)
                    
                    # 提取该关节的6个参数
                    d_xyz = params[param_idx:param_idx+3]
                    phi_xyz = params[param_idx+3:param_idx+6]
                    param_idx += 6
                    
                    # 构建增量 SE3
                    delta_trans = np.array(d_xyz)
                    delta_rot = pin.rpy.rpyToMatrix(phi_xyz[0], phi_xyz[1], phi_xyz[2])
                    delta_SE3 = pin.SE3(delta_rot, delta_trans)
                    
                    # 更新 placement
                    self.model.jointPlacements[jid] = \
                        self.model.jointPlacements[jid] * delta_SE3
                        
            # 计算每个样本的末端位置
            for i in range(n_samples):
                q_joints = self.q_measured[i]
                
                if calib_model == "joint_offset":
                    q_full = self._build_full_q(q_joints, params)
                else:
                    q_full = self._build_full_q(q_joints)
                    
                pee = self._compute_fk(q_full)
                PEE_est.extend(pee.tolist())
                
        finally:
            # 恢复原始 placement
            for jid, placement in original_placements.items():
                self.model.jointPlacements[jid] = placement
                
        return np.array(PEE_est)
        
    def cost_function(self, params: np.ndarray) -> np.ndarray:
        """代价函数: 测量值 - 估计值 + 正则化"""
        coeff_reg = self.calib_config.get("coeff_regularize", 0.001)
        
        # 计算末端位置估计
        PEE_est = self._calc_updated_fkm(params)
        
        # 残差
        residuals = self.PEE_measured - PEE_est
        
        # 正则化项
        reg_residuals = np.sqrt(coeff_reg) * params
        
        return np.concatenate([residuals, reg_residuals])
        
    def solve(self, method: str = "lm", max_iterations: int = 3,
              outlier_threshold: float = 3.0, enable_logging: bool = True,
              plotting: bool = False):
        """执行标定优化"""
        print(f"\n[标定] 方法={method}, 最大迭代={max_iterations}")
        
        n_params = len(self.calib_config["param_name"])
        var_init = np.zeros(n_params)
        
        current_var = var_init.copy()
        outlier_indices = []
        
        for iteration in range(max_iterations):
            if enable_logging:
                print(f"\n  迭代 {iteration+1}/{max_iterations}")
                
            # 优化
            result = least_squares(
                self.cost_function,
                current_var,
                method=method,
                max_nfev=2000,
                verbose=0
            )
            
            if enable_logging:
                print(f"    收敛: {result.success}  cost={result.cost:.6f}")
                
            # 计算残差并检测异常值
            PEE_est = self._calc_updated_fkm(result.x)
            residuals = self.PEE_measured - PEE_est
            
            n_meas = sum(self.calib_config["measurability"])
            n_samples = self.calib_config["NbSample"]
            
            if len(residuals) == n_meas * n_samples:
                res_2d = residuals.reshape(n_samples, n_meas)
                sample_rms = np.sqrt(np.mean(res_2d ** 2, axis=1))
                
                mean_e = np.mean(sample_rms)
                std_e = np.std(sample_rms)
                thresh = mean_e + outlier_threshold * std_e
                
                new_outliers = np.where(sample_rms > thresh)[0].tolist()
                
                if len(new_outliers) == 0:
                    if enable_logging:
                        print(f"    无异常值，收敛")
                    break
                    
                outlier_indices.extend(new_outliers)
                outlier_indices = list(set(outlier_indices))
                
                if enable_logging:
                    print(f"    检测到 {len(new_outliers)} 个异常值")
                    
            current_var = result.x
            
        # 最终评估
        PEE_est = self._calc_updated_fkm(result.x)
        final_residuals = self.PEE_measured - PEE_est
        rmse = np.sqrt(np.mean(final_residuals ** 2))
        mae = np.mean(np.abs(final_residuals))
        
        self.LM_result = result
        self.var_ = result.x
        self.STATUS = "CALIBRATED"
        
        self.evaluation_metrics = {
            "rmse": rmse,
            "mae": mae,
            "n_outliers": len(outlier_indices),
            "n_samples_used": n_samples - len(outlier_indices),
        }
        
        if enable_logging:
            print(f"\n  ── 标定结果 ──")
            print(f"  RMSE: {rmse*1000:.4f} mm")
            print(f"  MAE : {mae*1000:.4f} mm")
            print(f"  异常值: {len(outlier_indices)}")
            
        return result
        
    def get_calibrated_params(self) -> Dict[str, float]:
        """获取标定后的参数字典"""
        if self.var_ is None:
            return {}
        return dict(zip(self.calib_config["param_name"], self.var_))
        
    def save_results(self, output_path: str):
        """保存标定结果到YAML"""
        if self.var_ is None:
            print("[警告] 未完成标定，无法保存")
            return
            
        results = {
            "robot": self.calib_config["robot_name"],
            "status": self.STATUS,
            "rmse_mm": round(self.evaluation_metrics["rmse"] * 1000, 4),
            "mae_mm": round(self.evaluation_metrics["mae"] * 1000, 4),
            "n_samples_used": self.evaluation_metrics["n_samples_used"],
            "n_outliers": self.evaluation_metrics["n_outliers"],
            "calibration_model": self.calib_config["calib_model"],
            "parameters": {},
        }
        
        for name, val in zip(self.calib_config["param_name"], self.var_):
            results["parameters"][name] = float(val)
            
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w") as f:
            yaml.dump(results, f, allow_unicode=True, sort_keys=False)
            
        print(f"  结果已保存: {output_path}")


# ── FIGAROH 版标定器 ─────────────────────────────────────────────────
if FIGAROH_AVAILABLE:
    class TaksCalibration(BaseCalibration):
        """
        Semi_Taks_T1 / Taks_T1 标定类
        继承 FIGAROH BaseCalibration，实现机器人特定的代价函数
        """
        
        @handle_calibration_errors
        def __init__(self, robot, config_file: str, del_list: List[int] = None):
            """初始化标定器"""
            super().__init__(robot, config_file, del_list or [])
            
        def cost_function(self, var: np.ndarray) -> np.ndarray:
            """
            代价函数: 带权重的残差 + 正则化
            """
            coeff_reg = self.calib_config.get("coeff_regularize", 0.001)
            
            # 计算更新后的FK
            PEE_est = calc_updated_fkm(
                self.model, self.data, var,
                self.q_measured, self.calib_config
            )
            
            # 原始残差
            raw_residuals = self.PEE_measured - PEE_est
            
            # 应用测量权重
            weighted_residuals = self.apply_measurement_weighting(
                raw_residuals, pos_weight=1.0, orient_weight=0.5
            )
            
            # 正则化 (排除基座和末端参数)
            n_base = 6 if not self.calib_config.get("known_baseframe", True) else 0
            n_tip = 6 if not self.calib_config.get("known_tipframe", True) else 0
            
            if n_base > 0 or n_tip > 0:
                intermediate_params = var[n_base:-n_tip] if n_tip > 0 else var[n_base:]
            else:
                intermediate_params = var
                
            reg_residuals = np.sqrt(coeff_reg) * intermediate_params
            
            return np.concatenate([weighted_residuals, reg_residuals])


# ── 工厂函数 ─────────────────────────────────────────────────────────
def create_calibrator(robot, config_file: str, del_list: List[int] = None):
    """
    创建标定器实例
    优先使用 FIGAROH 版本，不可用时回退到简化版
    """
    if FIGAROH_AVAILABLE:
        print("  使用 FIGAROH BaseCalibration")
        return TaksCalibration(robot, config_file, del_list)
    else:
        print("  使用简化版标定器")
        return SimpleCalibration(robot, config_file, del_list)
