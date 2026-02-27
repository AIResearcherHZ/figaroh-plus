"""
Semi_Taks_T1机器人惯性参数计算脚本
严格按照刚体力学公式计算各link的惯性张量
"""

import numpy as np
from scipy.spatial.transform import Rotation


def quat_to_rotation_matrix(quat):
    """四元数转旋转矩阵 (MuJoCo格式: [w, x, y, z])"""
    w, x, y, z = quat
    r = Rotation.from_quat([x, y, z, w])
    return r.as_matrix()


def quat_to_rpy(quat):
    """四元数转欧拉角"""
    w, x, y, z = quat
    r = Rotation.from_quat([x, y, z, w])
    return r.as_euler('xyz')


def rotate_inertia(diag_inertia, quat):
    """将主轴坐标系下的对角惯量旋转到body坐标系"""
    I_principal = np.diag(diag_inertia)
    R = quat_to_rotation_matrix(quat)
    I_body = R @ I_principal @ R.T
    return I_body


def inertia_tensor_to_urdf(I):
    """从3x3惯性张量矩阵提取URDF格式的6个分量"""
    return (I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2])


def inertia_box(mass, lx, ly, lz):
    """长方体惯性张量"""
    Ixx = (1/12) * mass * (ly**2 + lz**2)
    Iyy = (1/12) * mass * (lx**2 + lz**2)
    Izz = (1/12) * mass * (lx**2 + ly**2)
    return np.array([Ixx, Iyy, Izz])


def inertia_cylinder_z(mass, radius, height):
    """圆柱体惯性张量 (沿Z轴)"""
    Ixx = Iyy = (1/12) * mass * (3 * radius**2 + height**2)
    Izz = (1/2) * mass * radius**2
    return np.array([Ixx, Iyy, Izz])


def inertia_cylinder_y(mass, radius, height):
    """圆柱体惯性张量 (沿Y轴)"""
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


# Semi_Taks_T1所有Link的数据定义
all_links = {
    # 底座
    "base_link": {
        "mass": 3.2,
        "shape": "box",
        "dims": [0.15, 0.15, 0.10],
        "com": [0.0, 0.0, 0.0]
    },
    # 上身
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
    # 左臂
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
    # 右臂
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
    # 头部
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
    """根据形状计算惯性张量"""
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
    
    if "quat" in link_data:
        quat = link_data["quat"]
        I_full = rotate_inertia(diag, quat)
        return (diag[0], diag[1], diag[2], I_full, quat)
    
    return (diag[0], diag[1], diag[2], None, None)


def main():
    print("=" * 70)
    print("Semi_Taks_T1 机器人惯性参数计算")
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
            rpy = quat_to_rpy(quat)
            urdf_inertia = inertia_tensor_to_urdf(I_full)
            print(f'  <origin xyz="{com[0]} {com[1]} {com[2]}" rpy="0 0 0"/>')
            print(f'  <mass value="{data["mass"]}"/>')
            ixx_f, ixy_f, ixz_f, iyy_f, iyz_f, izz_f = urdf_inertia
            print(f'  <inertia ixx="{ixx_f:.6g}" ixy="{ixy_f:.6g}" ixz="{ixz_f:.6g}"')
            print(f'           iyy="{iyy_f:.6g}" iyz="{iyz_f:.6g}" izz="{izz_f:.6g}"/>')
        else:
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

    return results


if __name__ == "__main__":
    main()
