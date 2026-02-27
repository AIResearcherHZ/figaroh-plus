"""
基于STL mesh文件计算惯性参数 - Semi_Taks_T1专用
使用trimesh库从mesh几何计算惯性张量，支持输出XML和URDF格式

依赖: pip install trimesh numpy scipy
"""

import os
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh


# Semi_Taks_T1各link的质量定义 (kg)
LINK_MASSES = {
    "base_link": 3.2,
    "waist_yaw_link": 0.79,
    "waist_roll_link": 1.56,
    "waist_pitch_link": 2.01,
    "left_shoulder_pitch_link": 0.24,
    "left_shoulder_roll_link": 0.92,
    "left_shoulder_yaw_link": 0.625,
    "left_elbow_link": 0.552,
    "left_wrist_roll_link": 0.508,
    "left_wrist_yaw_link": 0.37,
    "left_wrist_pitch_link": 0.308,
    "right_shoulder_pitch_link": 0.24,
    "right_shoulder_roll_link": 0.92,
    "right_shoulder_yaw_link": 0.625,
    "right_elbow_link": 0.552,
    "right_wrist_roll_link": 0.508,
    "right_wrist_yaw_link": 0.37,
    "right_wrist_pitch_link": 0.308,
    "neck_yaw_link": 0.4,
    "neck_roll_link": 0.026,
    "neck_pitch_link": 0.6,
}


def load_stl(filepath):
    """加载STL文件"""
    mesh = trimesh.load(filepath)
    if not mesh.is_watertight:
        print(f"  警告: {os.path.basename(filepath)} 不是封闭网格，惯性计算可能不准确")
    return mesh


def compute_inertia_from_mesh(mesh, mass):
    """从mesh计算惯性参数"""
    if mesh.volume > 0:
        density = mass / mesh.volume
        mesh.density = density
    
    com = mesh.center_mass
    inertia = mesh.moment_inertia
    
    if mesh.volume <= 0:
        print("  警告: 体积无效，使用包围盒估算")
        bounds = mesh.bounds
        size = bounds[1] - bounds[0]
        com = (bounds[0] + bounds[1]) / 2
        ixx = (1/12) * mass * (size[1]**2 + size[2]**2)
        iyy = (1/12) * mass * (size[0]**2 + size[2]**2)
        izz = (1/12) * mass * (size[0]**2 + size[1]**2)
        inertia = np.diag([ixx, iyy, izz])
    
    return com, inertia


def diagonalize_inertia(inertia):
    """对惯性张量进行对角化"""
    eigenvalues, eigenvectors = np.linalg.eigh(inertia)
    
    if np.linalg.det(eigenvectors) < 0:
        eigenvectors[:, 0] *= -1
    
    idx = np.argsort(eigenvalues)
    principal_inertia = eigenvalues[idx]
    rotation_matrix = eigenvectors[:, idx]
    
    r = Rotation.from_matrix(rotation_matrix)
    quat_xyzw = r.as_quat()
    quat = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
    
    return principal_inertia, rotation_matrix, quat


def inertia_to_urdf(inertia):
    """从3x3惯性张量提取URDF格式的6个分量"""
    return (inertia[0,0], inertia[0,1], inertia[0,2],
            inertia[1,1], inertia[1,2], inertia[2,2])


def validate_inertia(name, ixx, iyy, izz):
    """验证惯性张量是否满足三角不等式"""
    valid = True
    eps = 1e-10
    if ixx + iyy < izz - eps:
        print(f"  警告 {name}: Ixx + Iyy < Izz ({ixx:.6g} + {iyy:.6g} < {izz:.6g})")
        valid = False
    if iyy + izz < ixx - eps:
        print(f"  警告 {name}: Iyy + Izz < Ixx ({iyy:.6g} + {izz:.6g} < {ixx:.6g})")
        valid = False
    if izz + ixx < iyy - eps:
        print(f"  警告 {name}: Izz + Ixx < Iyy ({izz:.6g} + {ixx:.6g} < {iyy:.6g})")
        valid = False
    return valid


def process_stl_files(meshes_dir, masses=None):
    """处理目录下所有STL文件"""
    if masses is None:
        masses = LINK_MASSES
    
    results = {}
    stl_files = sorted([f for f in os.listdir(meshes_dir) if f.lower().endswith('.stl')])
    
    for stl_file in stl_files:
        link_name = os.path.splitext(stl_file)[0]
        filepath = os.path.join(meshes_dir, stl_file)
        
        print(f"\n处理: {link_name}")
        
        mass = masses.get(link_name, 1.0)
        if link_name not in masses:
            print(f"  警告: 未找到{link_name}的质量定义，使用默认值1.0kg")
        
        mesh = load_stl(filepath)
        com, inertia = compute_inertia_from_mesh(mesh, mass)
        principal_inertia, rot_matrix, quat = diagonalize_inertia(inertia)
        validate_inertia(link_name, principal_inertia[0], principal_inertia[1], principal_inertia[2])
        
        results[link_name] = {
            "mass": mass,
            "com": com,
            "inertia_full": inertia,
            "principal_inertia": principal_inertia,
            "rotation_matrix": rot_matrix,
            "quat": quat,
        }
        
        print(f"  质量: {mass:.4f} kg")
        print(f"  质心: [{com[0]:.6f}, {com[1]:.6f}, {com[2]:.6f}]")
        print(f"  主轴惯量: [{principal_inertia[0]:.6g}, {principal_inertia[1]:.6g}, {principal_inertia[2]:.6g}]")
    
    return results


def output_xml_format(results):
    """输出MuJoCo XML格式"""
    print("\n" + "=" * 70)
    print("MuJoCo XML格式输出")
    print("=" * 70)
    
    for name, data in results.items():
        com = data["com"]
        diag = data["principal_inertia"]
        quat = data["quat"]
        
        quat_identity = np.allclose(np.abs(quat), [1, 0, 0, 0], atol=1e-4) or \
                       np.allclose(quat, [1, 0, 0, 0], atol=1e-4)
        
        print(f'<!-- {name} -->')
        if quat_identity:
            print(f'<inertial pos="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" mass="{data["mass"]}"')
        else:
            print(f'<inertial pos="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" quat="{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}" mass="{data["mass"]}"')
        print(f'  diaginertia="{diag[0]:.6g} {diag[1]:.6g} {diag[2]:.6g}"/>')
        print()


def output_urdf_format(results):
    """输出URDF格式"""
    print("\n" + "=" * 70)
    print("URDF格式输出")
    print("=" * 70)
    
    for name, data in results.items():
        com = data["com"]
        inertia = data["inertia_full"]
        ixx, ixy, ixz, iyy, iyz, izz = inertia_to_urdf(inertia)
        
        print(f'<!-- {name} -->')
        print(f'<inertial>')
        print(f'  <origin xyz="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" rpy="0 0 0"/>')
        print(f'  <mass value="{data["mass"]}"/>')
        print(f'  <inertia ixx="{ixx:.6g}" ixy="{ixy:.6g}" ixz="{ixz:.6g}" iyy="{iyy:.6g}" iyz="{iyz:.6g}" izz="{izz:.6g}"/>')
        print(f'</inertial>')
        print()


def output_python_dict(results):
    """输出Python字典格式"""
    print("\n" + "=" * 70)
    print("Python字典格式 (可复制)")
    print("=" * 70)
    print("inertia_data = {")
    
    for name, data in results.items():
        com = data["com"]
        diag = data["principal_inertia"]
        quat = data["quat"]
        inertia = data["inertia_full"]
        ixx, ixy, ixz, iyy, iyz, izz = inertia_to_urdf(inertia)
        
        print(f'    "{name}": {{')
        print(f'        "mass": {data["mass"]},')
        print(f'        "com": [{com[0]:.6f}, {com[1]:.6f}, {com[2]:.6f}],')
        print(f'        "diaginertia": [{diag[0]:.6g}, {diag[1]:.6g}, {diag[2]:.6g}],')
        print(f'        "quat": [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}],')
        print(f'        "urdf_inertia": {{"ixx": {ixx:.6g}, "ixy": {ixy:.6g}, "ixz": {ixz:.6g}, "iyy": {iyy:.6g}, "iyz": {iyz:.6g}, "izz": {izz:.6g}}},')
        print(f'    }},')
    
    print("}")


def main():
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    meshes_dir = os.path.join(script_dir, "meshes")
    
    if not os.path.exists(meshes_dir):
        print(f"错误: 找不到meshes目录: {meshes_dir}")
        return
    
    print("=" * 70)
    print("基于STL Mesh计算惯性参数 - Semi_Taks_T1")
    print("=" * 70)
    
    results = process_stl_files(meshes_dir, LINK_MASSES)
    
    output_xml_format(results)
    output_urdf_format(results)
    output_python_dict(results)
    
    print("\n" + "=" * 70)
    print("完成!")
    print("=" * 70)
    
    return results


if __name__ == "__main__":
    main()
