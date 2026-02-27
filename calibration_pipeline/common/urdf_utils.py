"""
URDF 解析、关节参数读写、更新工具
支持: 读取关节origin/inertial, 写入标定后参数, 转换为MuJoCo用的numpy数组
"""

import xml.etree.ElementTree as ET
import numpy as np
import copy
from pathlib import Path


def load_urdf_tree(urdf_path: str):
    """加载URDF XML树"""
    tree = ET.parse(urdf_path)
    return tree, tree.getroot()


def get_joint_origins(root) -> dict:
    """提取所有关节的 origin xyz/rpy -> dict[joint_name] = {'xyz': [x,y,z], 'rpy': [r,p,y]}"""
    joints = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        origin = joint.find("origin")
        if origin is not None:
            xyz = list(map(float, origin.get("xyz", "0 0 0").split()))
            rpy = list(map(float, origin.get("rpy", "0 0 0").split()))
        else:
            xyz, rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        joints[name] = {"xyz": xyz, "rpy": rpy}
    return joints


def get_link_inertials(root) -> dict:
    """提取所有连杆的质量/惯量/COM -> dict[link_name]"""
    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is None:
            continue
        origin = inertial.find("origin")
        mass_el = inertial.find("mass")
        inertia_el = inertial.find("inertia")
        xyz = list(map(float, origin.get("xyz", "0 0 0").split())) if origin is not None else [0, 0, 0]
        rpy = list(map(float, origin.get("rpy", "0 0 0").split())) if origin is not None else [0, 0, 0]
        mass = float(mass_el.get("value", 0)) if mass_el is not None else 0.0
        inertia = {}
        if inertia_el is not None:
            for key in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
                inertia[key] = float(inertia_el.get(key, 0))
        links[name] = {"com_xyz": xyz, "com_rpy": rpy, "mass": mass, "inertia": inertia}
    return links


def get_revolute_joints(root) -> list:
    """返回所有 revolute 关节名列表（跳过 floating/fixed）"""
    return [j.get("name") for j in root.findall("joint")
            if j.get("type") in ("revolute", "prismatic")]


def update_joint_origin(root, joint_name: str, xyz: list = None, rpy: list = None):
    """就地更新关节 origin xyz/rpy（传入None则不修改该项）"""
    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            origin = joint.find("origin")
            if origin is None:
                origin = ET.SubElement(joint, "origin")
            if xyz is not None:
                origin.set("xyz", " ".join(f"{v:.8f}" for v in xyz))
            if rpy is not None:
                origin.set("rpy", " ".join(f"{v:.8f}" for v in rpy))
            return True
    return False


def update_link_inertial(root, link_name: str, mass: float = None,
                          com_xyz: list = None, inertia_dict: dict = None):
    """就地更新连杆惯量参数"""
    for link in root.findall("link"):
        if link.get("name") == link_name:
            inertial = link.find("inertial")
            if inertial is None:
                inertial = ET.SubElement(link, "inertial")
            if com_xyz is not None:
                origin = inertial.find("origin")
                if origin is None:
                    origin = ET.SubElement(inertial, "origin")
                origin.set("xyz", " ".join(f"{v:.8f}" for v in com_xyz))
            if mass is not None:
                mass_el = inertial.find("mass")
                if mass_el is None:
                    mass_el = ET.SubElement(inertial, "mass")
                mass_el.set("value", f"{mass:.8f}")
            if inertia_dict is not None:
                inertia_el = inertial.find("inertia")
                if inertia_el is None:
                    inertia_el = ET.SubElement(inertial, "inertia")
                for k, v in inertia_dict.items():
                    inertia_el.set(k, f"{v:.10e}")
            return True
    return False


def save_urdf(tree, output_path: str):
    """保存修改后的URDF到文件，保持缩进可读"""
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="unicode", xml_declaration=False)
    # 补充 <?xml ...> 头
    content = '<?xml version="1.0"?>\n' + ET.tostring(tree.getroot(), encoding="unicode")
    Path(output_path).write_text(content)


def apply_calibration_results(urdf_path: str, calib_params: dict, output_path: str):
    """
    把标定结果写入URDF
    calib_params: dict, key 为关节名, value 为 {'xyz_delta': [dx,dy,dz], 'rpy_delta': [dr,dp,dy]}
    """
    tree, root = load_urdf_tree(urdf_path)
    origins = get_joint_origins(root)
    for jname, delta in calib_params.items():
        if jname not in origins:
            print(f"[警告] 关节 {jname} 在URDF中不存在，跳过")
            continue
        orig = origins[jname]
        new_xyz = [orig["xyz"][i] + delta.get("xyz_delta", [0, 0, 0])[i] for i in range(3)]
        new_rpy = [orig["rpy"][i] + delta.get("rpy_delta", [0, 0, 0])[i] for i in range(3)]
        update_joint_origin(root, jname, xyz=new_xyz, rpy=new_rpy)
        print(f"  [更新] {jname}: xyz{new_xyz}  rpy{new_rpy}")
    save_urdf(tree, output_path)
    print(f"[完成] 优化后URDF已保存: {output_path}")


def urdf_to_pinocchio(urdf_path: str, package_dir: str = None, floating_base: bool = True):
    """用 pinocchio 加载URDF, 返回 robot wrapper"""
    import pinocchio as pin
    from pinocchio.robot_wrapper import RobotWrapper
    pkg = package_dir or str(Path(urdf_path).parent)
    if floating_base:
        robot = RobotWrapper.BuildFromURDF(urdf_path, [pkg], root_joint=pin.JointModelFreeFlyer())
    else:
        robot = RobotWrapper.BuildFromURDF(urdf_path, [pkg])
    return robot
