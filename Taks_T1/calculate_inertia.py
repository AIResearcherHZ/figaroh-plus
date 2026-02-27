"""
Taks_T1机器人惯性参数计算脚本
严格按照刚体力学公式计算各link的惯性张量

惯性张量公式:
- 长方体 (box): 
    Ixx = (1/12) * m * (ly² + lz²)
    Iyy = (1/12) * m * (lx² + lz²)
    Izz = (1/12) * m * (lx² + ly²)
    
- 圆柱体 (cylinder, 沿Z轴):
    Ixx = Iyy = (1/12) * m * (3r² + h²)
    Izz = (1/2) * m * r²
    
- 球体 (sphere):
    Ixx = Iyy = Izz = (2/5) * m * r²

注意: 惯性张量必须满足三角不等式:
    Ixx + Iyy >= Izz
    Iyy + Izz >= Ixx
    Izz + Ixx >= Iyy

旋转惯量转换:
- MuJoCo XML: quat="w x y z" + diaginertia="Ixx Iyy Izz" (主轴坐标系下的对角惯量)
- URDF: rpy="roll pitch yaw" + 完整惯性张量 ixx, ixy, ixz, iyy, iyz, izz
- 转换公式: I_body = R @ I_principal @ R.T
"""

import numpy as np
from scipy.spatial.transform import Rotation


def quat_to_rotation_matrix(quat):
    """四元数转旋转矩阵
    MuJoCo四元数格式: [w, x, y, z]
    """
    w, x, y, z = quat
    # scipy使用 [x, y, z, w] 格式
    r = Rotation.from_quat([x, y, z, w])
    return r.as_matrix()


def quat_to_rpy(quat):
    """四元数转欧拉角 (roll, pitch, yaw)
    MuJoCo四元数格式: [w, x, y, z]
    返回: [roll, pitch, yaw] 弧度
    """
    w, x, y, z = quat
    # scipy使用 [x, y, z, w] 格式
    r = Rotation.from_quat([x, y, z, w])
    # URDF使用 'xyz' 顺序 (roll-pitch-yaw)
    return r.as_euler('xyz')


def rotate_inertia(diag_inertia, quat):
    """将主轴坐标系下的对角惯量旋转到body坐标系
    
    MuJoCo的diaginertia是在主轴坐标系下的对角惯量,
    quat定义了从body坐标系到主轴坐标系的旋转。
    
    转换公式: I_body = R @ I_principal @ R.T
    
    Args:
        diag_inertia: [Ixx, Iyy, Izz] 主轴惯量
        quat: [w, x, y, z] MuJoCo格式四元数
    
    Returns:
        完整的3x3惯性张量矩阵 (body坐标系)
    """
    # 主轴坐标系下的对角惯性张量
    I_principal = np.diag(diag_inertia)
    
    # 旋转矩阵
    R = quat_to_rotation_matrix(quat)
    
    # 转换到body坐标系: I_body = R @ I_principal @ R.T
    I_body = R @ I_principal @ R.T
    
    return I_body


def inertia_tensor_to_urdf(I):
    """从3x3惯性张量矩阵提取URDF格式的6个分量
    
    Returns:
        (ixx, ixy, ixz, iyy, iyz, izz)
    """
    ixx = I[0, 0]
    iyy = I[1, 1]
    izz = I[2, 2]
    ixy = I[0, 1]
    ixz = I[0, 2]
    iyz = I[1, 2]
    return (ixx, ixy, ixz, iyy, iyz, izz)


def inertia_box(mass, lx, ly, lz):
    """长方体惯性张量 (质心在几何中心)
    lx, ly, lz: 沿x, y, z轴的尺寸
    """
    Ixx = (1/12) * mass * (ly**2 + lz**2)
    Iyy = (1/12) * mass * (lx**2 + lz**2)
    Izz = (1/12) * mass * (lx**2 + ly**2)
    return np.array([Ixx, Iyy, Izz])


def inertia_cylinder_z(mass, radius, height):
    """圆柱体惯性张量 (沿Z轴, 质心在几何中心)"""
    Ixx = Iyy = (1/12) * mass * (3 * radius**2 + height**2)
    Izz = (1/2) * mass * radius**2
    return np.array([Ixx, Iyy, Izz])


def inertia_cylinder_y(mass, radius, height):
    """圆柱体惯性张量 (沿Y轴, 质心在几何中心)"""
    Iyy = (1/2) * mass * radius**2
    Ixx = Izz = (1/12) * mass * (3 * radius**2 + height**2)
    return np.array([Ixx, Iyy, Izz])


def inertia_sphere(mass, radius):
    """球体惯性张量"""
    I = (2/5) * mass * radius**2
    return np.array([I, I, I])


def validate_inertia(name, ixx, iyy, izz):
    """验证惯性张量是否满足三角不等式"""
    valid = True
    if ixx + iyy < izz:
        print(f"  警告 {name}: Ixx + Iyy < Izz ({ixx:.6g} + {iyy:.6g} < {izz:.6g})")
        valid = False
    if iyy + izz < ixx:
        print(f"  警告 {name}: Iyy + Izz < Ixx ({iyy:.6g} + {izz:.6g} < {ixx:.6g})")
        valid = False
    if izz + ixx < iyy:
        print(f"  警告 {name}: Izz + Ixx < Iyy ({izz:.6g} + {ixx:.6g} < {iyy:.6g})")
        valid = False
    return valid


# ============================================================================
# 所有Link的数据定义
# 格式: "link_name": {"mass": kg, "shape": type, "dims": [...], "com": [x,y,z]}
# shape类型: "box" -> dims=[lx, ly, lz]
#           "cylinder_z" -> dims=[radius, height] (沿Z轴)
#           "cylinder_y" -> dims=[radius, height] (沿Y轴)
#           "sphere" -> dims=[radius]
# ============================================================================

all_links = {
    # ========== 下肢 (Pelvis + Legs) ==========
    "pelvis": {
        "mass": 3.34,
        "shape": "box",
        "dims": [0.18, 0.16, 0.12],
        "com": [0.00454, 0.0, -0.07726]
    },
    "left_hip_pitch_link": {
        "mass": 0.91,
        "shape": "box",
        "dims": [0.06, 0.06, 0.10],
        "com": [0.0, 0.0383, -0.0209],
        "quat": [0.954862, 0.293964, 0.0302556, 0.030122]
    },
    "left_hip_roll_link": {
        "mass": 0.79,
        "shape": "cylinder_z",
        "dims": [0.04, 0.10],
        "com": [0.0, -0.00737, 0.0739]
    },
    "left_hip_yaw_link": {
        "mass": 1.822,
        "shape": "box",
        "dims": [0.12, 0.10, 0.22],
        "com": [-0.025, 0.0, -0.083]
    },
    "left_knee_link": {
        "mass": 1.138,
        "shape": "box",
        "dims": [0.08, 0.08, 0.22],
        "com": [-0.00222, 0.0109, -0.1568]
    },
    "left_ankle_pitch_link": {
        "mass": 0.23,
        "shape": "box",
        "dims": [0.03, 0.08, 0.03],
        "com": [0.0, 0.0, 0.0]
    },
    "left_ankle_roll_link": {
        "mass": 0.15,
        "shape": "box",
        "dims": [0.22, 0.10, 0.025],
        "com": [0.00161, 0.00307, -0.0384]
    },
    "right_hip_pitch_link": {
        "mass": 0.91,
        "shape": "box",
        "dims": [0.06, 0.06, 0.10],
        "com": [0.0, -0.0383, -0.0209],
        "quat": [0.954862, -0.293964, 0.0302556, -0.030122]
    },
    "right_hip_roll_link": {
        "mass": 0.79,
        "shape": "cylinder_z",
        "dims": [0.04, 0.10],
        "com": [0.0, -0.00737, 0.0739]
    },
    "right_hip_yaw_link": {
        "mass": 1.822,
        "shape": "box",
        "dims": [0.12, 0.10, 0.22],
        "com": [-0.025, 0.0, -0.083]
    },
    "right_knee_link": {
        "mass": 1.138,
        "shape": "box",
        "dims": [0.08, 0.08, 0.22],
        "com": [0.00305, 0.0103, -0.1537]
    },
    "right_ankle_pitch_link": {
        "mass": 0.23,
        "shape": "box",
        "dims": [0.04, 0.03, 0.03],
        "com": [0.024, 0.0, 0.0]
    },
    "right_ankle_roll_link": {
        "mass": 0.15,
        "shape": "box",
        "dims": [0.22, 0.10, 0.025],
        "com": [-0.00161, 0.00308, -0.0384]
    },

    # ========== 上肢 (Waist + Torso) ==========
    "waist_yaw_link": {
        "mass": 0.79,
        "shape": "cylinder_z",
        "dims": [0.05, 0.06],
        "com": [0.000694, 3.7e-05, 0.023535]
    },
    "waist_roll_link": {
        "mass": 1.56,
        "shape": "box",
        "dims": [0.10, 0.10, 0.14],
        "com": [-0.03647, 0.001918, 0.039525]
    },
    "torso_link": {
        "mass": 2.01,
        "shape": "box",
        "dims": [0.20, 0.15, 0.35],
        "com": [-0.000232, 0.028443, 0.216034]
    },

    # ========== 左臂 ==========
    "left_shoulder_pitch_link": {
        "mass": 0.24,
        "shape": "box",
        "dims": [0.05, 0.08, 0.04],
        "com": [-0.003513, 0.038593, 0.000147]
    },
    "left_shoulder_roll_link": {
        "mass": 0.92,
        "shape": "cylinder_z",
        "dims": [0.04, 0.14],
        "com": [-0.034364, 0.004091, -0.054537]
    },
    "left_shoulder_yaw_link": {
        "mass": 0.625,
        "shape": "cylinder_z",
        "dims": [0.04, 0.18],
        "com": [0.000365, -7.2e-05, -0.086229]
    },
    "left_elbow_link": {
        "mass": 0.552,
        "shape": "cylinder_y",
        "dims": [0.03, 0.14],
        "com": [0.060258, -0.018902, -0.00551]
    },
    "left_wrist_roll_link": {
        "mass": 0.508,
        "shape": "cylinder_y",
        "dims": [0.025, 0.13],
        "com": [0.059349, 0.0, 0.000843]
    },
    "left_wrist_yaw_link": {
        "mass": 0.37,
        "shape": "cylinder_z",
        "dims": [0.03, 0.07],
        "com": [0.0, 0.001805, 0.034219]
    },
    "left_wrist_pitch_link": {
        "mass": 0.308,
        "shape": "box",
        "dims": [0.22, 0.08, 0.06],
        "com": [0.076857, -0.017165, 0.011311]
    },

    # ========== 右臂 ==========
    "right_shoulder_pitch_link": {
        "mass": 0.24,
        "shape": "box",
        "dims": [0.05, 0.08, 0.04],
        "com": [-0.003513, -0.038593, 0.000147]
    },
    "right_shoulder_roll_link": {
        "mass": 0.92,
        "shape": "cylinder_z",
        "dims": [0.04, 0.14],
        "com": [-0.027363, -0.004095, -0.054537]
    },
    "right_shoulder_yaw_link": {
        "mass": 0.625,
        "shape": "cylinder_z",
        "dims": [0.04, 0.18],
        "com": [0.000364, 6.6e-05, -0.08623]
    },
    "right_elbow_link": {
        "mass": 0.552,
        "shape": "cylinder_y",
        "dims": [0.03, 0.14],
        "com": [0.060603, -0.027325, -0.005367]
    },
    "right_wrist_roll_link": {
        "mass": 0.508,
        "shape": "cylinder_y",
        "dims": [0.025, 0.13],
        "com": [0.059331, 0.0, 0.000847]
    },
    "right_wrist_yaw_link": {
        "mass": 0.37,
        "shape": "cylinder_z",
        "dims": [0.03, 0.07],
        "com": [0.0, -0.001803, 0.034219]
    },
    "right_wrist_pitch_link": {
        "mass": 0.308,
        "shape": "box",
        "dims": [0.22, 0.08, 0.06],
        "com": [0.076858, 0.017168, 0.01131]
    },

    # ========== 头部 ==========
    "neck_yaw_link": {
        "mass": 0.4,
        "shape": "cylinder_z",
        "dims": [0.04, 0.08],
        "com": [-0.026005, 0.0, 0.044508]
    },
    "neck_roll_link": {
        "mass": 0.026,
        "shape": "sphere",
        "dims": [0.02],
        "com": [0.05706, -0.000635, 0.0]
    },
    "neck_pitch_link": {
        "mass": 0.6,
        "shape": "box",
        "dims": [0.18, 0.18, 0.18],
        "com": [0.00, -0.020599, 0.105269],
        "quat": [0.966, 0.001, 0.087, 0.001]
    },
}


def calculate_inertia(link_data):
    """根据形状计算惯性张量
    
    Returns:
        如果没有quat: 返回 (ixx, iyy, izz, None) - 对角惯量
        如果有quat: 返回 (ixx, iyy, izz, full_tensor) - 对角惯量 + 完整张量
    """
    mass = link_data["mass"]
    shape = link_data["shape"]
    dims = link_data["dims"]

    if shape == "box":
        diag = inertia_box(mass, dims[0], dims[1], dims[2])
    elif shape == "cylinder_z":
        diag = inertia_cylinder_z(mass, dims[0], dims[1])
    elif shape == "cylinder_y":
        diag = inertia_cylinder_y(mass, dims[0], dims[1])
    elif shape == "sphere":
        diag = inertia_sphere(mass, dims[0])
    else:
        raise ValueError(f"Unknown shape: {shape}")
    
    # 如果有四元数，计算旋转后的完整惯性张量
    if "quat" in link_data:
        quat = link_data["quat"]
        I_full = rotate_inertia(diag, quat)
        return (diag[0], diag[1], diag[2], I_full, quat)
    
    return (diag[0], diag[1], diag[2], None, None)


def main():
    print("=" * 70)
    print("Taks_T1 机器人惯性参数计算")
    print("=" * 70)
    print()

    results = {}
    all_valid = True

    for name, data in all_links.items():
        ixx, iyy, izz, I_full, quat = calculate_inertia(data)

        valid = validate_inertia(name, ixx, iyy, izz)
        if not valid:
            all_valid = False

        results[name] = {
            "mass": data["mass"],
            "com": data["com"],
            "diag_inertia": (ixx, iyy, izz),
            "full_inertia": I_full,
            "quat": quat
        }

    print()
    if all_valid:
        print("✓ 所有惯性张量满足三角不等式")
    else:
        print("✗ 存在不满足三角不等式的惯性张量")

    print()
    print("=" * 70)
    print("URDF格式输出")
    print("=" * 70)
    for name, data in results.items():
        com = data["com"]
        ixx, iyy, izz = data["diag_inertia"]
        I_full = data["full_inertia"]
        quat = data["quat"]
        
        print(f'<!-- {name} -->')
        print(f'<inertial>')
        
        if I_full is not None:
            # 有旋转的情况: 两种方法都输出
            rpy = quat_to_rpy(quat)
            urdf_inertia = inertia_tensor_to_urdf(I_full)
            
            # 方法A (推荐): rpy=0, 使用旋转后的完整惯性张量
            print(f'  <!-- 方法A (推荐): rpy=0 + 完整惯性张量 -->')
            print(f'  <origin xyz="{com[0]} {com[1]} {com[2]}" rpy="0 0 0"/>')
            print(f'  <mass value="{data["mass"]}"/>')
            ixx_f, ixy_f, ixz_f, iyy_f, iyz_f, izz_f = urdf_inertia
            print(f'  <inertia ixx="{ixx_f:.6g}" ixy="{ixy_f:.6g}" ixz="{ixz_f:.6g}"')
            print(f'           iyy="{iyy_f:.6g}" iyz="{iyz_f:.6g}" izz="{izz_f:.6g}"/>')
            
            # 方法B: 使用rpy + 对角惯量
            print(f'  <!-- 方法B: rpy旋转 + 对角惯量 -->')
            print(f'  <!-- <origin xyz="{com[0]} {com[1]} {com[2]}"')
            print(f'         rpy="{rpy[0]:.6g} {rpy[1]:.6g} {rpy[2]:.6g}"/> -->')
            print(f'  <!-- <inertia ixx="{ixx:.6g}" ixy="0" ixz="0"')
            print(f'             iyy="{iyy:.6g}" iyz="0" izz="{izz:.6g}"/> -->')
            print(f'  <!-- 四元数 [w,x,y,z]: {quat} -->')
        else:
            # 无旋转的情况: 使用对角惯量
            print(f'  <origin xyz="{com[0]} {com[1]} {com[2]}" rpy="0 0 0"/>')
            print(f'  <mass value="{data["mass"]}"/>')
            print(f'  <inertia ixx="{ixx:.6g}" ixy="0.0" ixz="0.0"')
            print(f'           iyy="{iyy:.6g}" iyz="0.0" izz="{izz:.6g}"/>')
        
        print(f'</inertial>')
        print()

    print("=" * 70)
    print("MuJoCo XML格式输出 (diaginertia + quat)")
    print("=" * 70)
    for name, data in results.items():
        com = data["com"]
        ixx, iyy, izz = data["diag_inertia"]
        quat = data["quat"]
        
        print(f'<!-- {name} -->')
        if quat is not None:
            print(f'<inertial pos="{com[0]} {com[1]} {com[2]}" quat="{quat[0]} {quat[1]} {quat[2]} {quat[3]}" mass="{data["mass"]}"')
        else:
            print(f'<inertial pos="{com[0]} {com[1]} {com[2]}" mass="{data["mass"]}"')
        print(f'  diaginertia="{ixx:.6g} {iyy:.6g} {izz:.6g}"/>')
        print()

    # 打印转换说明
    print("=" * 70)
    print("转换说明")
    print("=" * 70)
    print("""
MuJoCo XML 与 URDF 惯性参数转换:

1. MuJoCo格式:
   <inertial pos="x y z" quat="w x y z" mass="m" diaginertia="Ixx Iyy Izz"/>
   - quat: 从body坐标系到惯量主轴坐标系的旋转 (w, x, y, z)
   - diaginertia: 主轴坐标系下的对角惯量

2. URDF格式:
   <origin xyz="x y z" rpy="roll pitch yaw"/>
   <inertia ixx="..." ixy="..." ixz="..." iyy="..." iyz="..." izz="..."/>
   - rpy: 从body坐标系到惯量主轴坐标系的旋转 (欧拉角)
   - inertia: body坐标系下的完整惯性张量 (包含非对角项)

3. 转换方法:
   方法A (推荐): 保持rpy=0, 使用旋转后的完整惯性张量
      I_body = R @ diag(Ixx, Iyy, Izz) @ R.T
   
   方法B: 使用rpy旋转 + 对角惯量 (等效但需要正确的rpy)
      rpy = quat_to_euler(quat)
      inertia使用对角形式

4. 四元数到欧拉角:
   MuJoCo quat [w,x,y,z] -> scipy quat [x,y,z,w] -> euler 'xyz' (rpy)
""")

    return results


if __name__ == "__main__":
    main()