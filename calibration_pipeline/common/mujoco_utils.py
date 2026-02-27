"""
MuJoCo XML 解析与参数同步工具
功能: 读取body/joint的pos/quat/inertial, 把URDF标定结果同步写入XML
"""

import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation


def rpy_to_quat_wxyz(rpy: list) -> list:
    """RPY -> [w,x,y,z] 四元数 (MuJoCo 格式)"""
    r = Rotation.from_euler("xyz", rpy)
    xyzw = r.as_quat()          # scipy: [x,y,z,w]
    return [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]   # -> [w,x,y,z]


def quat_wxyz_to_rpy(quat_wxyz: list) -> list:
    """[w,x,y,z] -> RPY"""
    w, x, y, z = quat_wxyz
    r = Rotation.from_quat([x, y, z, w])
    return r.as_euler("xyz").tolist()


def delta_rpy_to_quat(orig_rpy: list, delta_rpy: list) -> list:
    """把原始rpy加上增量delta_rpy后转成MuJoCo quat [w,x,y,z]"""
    new_rpy = [orig_rpy[i] + delta_rpy[i] for i in range(3)]
    return rpy_to_quat_wxyz(new_rpy)


def load_xml_tree(xml_path: str):
    """加载MuJoCo XML树"""
    tree = ET.parse(xml_path)
    return tree, tree.getroot()


def _find_body_recursive(root_el, body_name: str):
    """递归搜索body元素"""
    for body in root_el.iter("body"):
        if body.get("name") == body_name:
            return body
    return None


def _find_joint_in_body(body_el, joint_name: str):
    """在body内查找joint"""
    for joint in body_el.findall("joint"):
        if joint.get("name") == joint_name:
            return joint
    return None


def get_body_positions(root) -> dict:
    """读取所有body的 pos 和 quat -> dict[body_name]"""
    result = {}
    for body in root.iter("body"):
        name = body.get("name")
        pos_str = body.get("pos", "0 0 0")
        quat_str = body.get("quat", "1 0 0 0")
        pos = list(map(float, pos_str.split()))
        quat = list(map(float, quat_str.split()))
        result[name] = {"pos": pos, "quat": quat}
    return result


def get_joint_ranges(root) -> dict:
    """读取所有关节的 range -> dict[joint_name]"""
    result = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        rng = joint.get("range", "")
        if rng:
            result[name] = list(map(float, rng.split()))
    return result


def update_body_pos_quat(root, body_name: str, pos: list = None, quat: list = None):
    """更新指定body的 pos/quat"""
    body = _find_body_recursive(root, body_name)
    if body is None:
        print(f"[警告] body '{body_name}' 未找到，跳过")
        return False
    if pos is not None:
        body.set("pos", " ".join(f"{v:.8f}" for v in pos))
    if quat is not None:
        body.set("quat", " ".join(f"{v:.8f}" for v in quat))
    return True


def update_body_inertial(root, body_name: str, mass: float = None,
                          pos: list = None, diaginertia: list = None, quat: list = None):
    """更新指定body内的 inertial 元素"""
    body = _find_body_recursive(root, body_name)
    if body is None:
        print(f"[警告] body '{body_name}' 未找到，跳过")
        return False
    inertial = body.find("inertial")
    if inertial is None:
        inertial = ET.SubElement(body, "inertial")
    if pos is not None:
        inertial.set("pos", " ".join(f"{v:.8f}" for v in pos))
    if quat is not None:
        inertial.set("quat", " ".join(f"{v:.8f}" for v in quat))
    if mass is not None:
        inertial.set("mass", f"{mass:.8f}")
    if diaginertia is not None:
        inertial.set("diaginertia", " ".join(f"{v:.10e}" for v in diaginertia))
    return True


def save_xml(tree, output_path: str):
    """保存MuJoCo XML，保持可读缩进"""
    ET.indent(tree, space="  ")
    content = ET.tostring(tree.getroot(), encoding="unicode")
    Path(output_path).write_text(content)


def sync_urdf_to_mujoco(urdf_path: str, xml_path: str, output_xml_path: str,
                         sync_joint_origin: bool = True,
                         sync_inertial: bool = True):
    """
    核心函数: 从URDF读取最新参数, 覆盖写入MuJoCo XML
    - sync_joint_origin: 同步关节 origin xyz/rpy -> body pos/quat
    - sync_inertial: 同步连杆 inertial -> body inertial
    """
    from .urdf_utils import load_urdf_tree, get_joint_origins, get_link_inertials
    import pinocchio as pin
    from scipy.spatial.transform import Rotation

    print(f"\n[同步] URDF -> MuJoCo XML")
    print(f"  源URDF : {urdf_path}")
    print(f"  源XML  : {xml_path}")
    print(f"  输出XML: {output_xml_path}")

    # 加载URDF参数
    _, urdf_root = load_urdf_tree(urdf_path)
    joint_origins = get_joint_origins(urdf_root)
    link_inertials = get_link_inertials(urdf_root)

    # 加载MuJoCo XML
    xml_tree, xml_root = load_xml_tree(xml_path)

    # 同步关节origin -> 对应子body的pos/quat
    if sync_joint_origin:
        print("\n  [同步关节origin -> body pos/quat]")
        # MuJoCo中: 关节的子连杆名 == body name, body的pos就是URDF joint origin xyz
        # quat由joint origin rpy转换
        for joint_el in urdf_root.findall("joint"):
            jtype = joint_el.get("type")
            if jtype in ("floating",):
                continue
            jname = joint_el.get("name")
            child_el = joint_el.find("child")
            if child_el is None:
                continue
            child_link = child_el.get("link")
            origin = joint_el.find("origin")
            if origin is None:
                continue
            xyz = list(map(float, origin.get("xyz", "0 0 0").split()))
            rpy = list(map(float, origin.get("rpy", "0 0 0").split()))
            quat = rpy_to_quat_wxyz(rpy)

            # 判断是否是单位四元数（不设quat则更干净）
            is_identity_rpy = all(abs(v) < 1e-9 for v in rpy)
            success = update_body_pos_quat(
                xml_root, child_link,
                pos=xyz,
                quat=None if is_identity_rpy else quat
            )
            # 如果原来有quat但现在是0，也需要移除
            if is_identity_rpy:
                body = _find_body_recursive(xml_root, child_link)
                if body is not None and body.get("quat") is not None:
                    del body.attrib["quat"]
            if success:
                print(f"    {child_link}: pos={[f'{v:.6f}' for v in xyz]}"
                      + (f"  quat={[f'{v:.6f}' for v in quat]}" if not is_identity_rpy else ""))

    # 同步inertial
    if sync_inertial:
        print("\n  [同步惯量参数 -> body inertial]")
        for link_name, data in link_inertials.items():
            mass = data["mass"]
            com_xyz = data["com_xyz"]
            com_rpy = data["com_rpy"]
            inertia = data["inertia"]

            # diaginertia近似: 用原始ixx iyy izz (实际应做主轴分解, 见注释)
            # 严格做法: 构建3x3惯量矩阵 -> 特征值分解 -> diaginertia + quat
            I_mat = np.array([
                [inertia.get("ixx", 0), inertia.get("ixy", 0), inertia.get("ixz", 0)],
                [inertia.get("ixy", 0), inertia.get("iyy", 0), inertia.get("iyz", 0)],
                [inertia.get("ixz", 0), inertia.get("iyz", 0), inertia.get("izz", 0)],
            ])
            eigenvalues, eigenvectors = np.linalg.eigh(I_mat)
            # 确保特征向量行列式为正 (右手系)
            if np.linalg.det(eigenvectors) < 0:
                eigenvectors[:, 0] *= -1
            rot = Rotation.from_matrix(eigenvectors)
            xyzw = rot.as_quat()
            quat_wxyz = [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]
            diag = eigenvalues.tolist()

            # com有rpy偏转时合并进quat
            if any(abs(v) > 1e-9 for v in com_rpy):
                com_rpy_q = rpy_to_quat_wxyz(com_rpy)
                # 两个旋转合并 (com_rpy先, 然后主轴旋转)
                r1 = Rotation.from_quat([com_rpy_q[1], com_rpy_q[2], com_rpy_q[3], com_rpy_q[0]])
                r2 = Rotation.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
                combined = (r2 * r1).as_quat()
                quat_wxyz = [combined[3], combined[0], combined[1], combined[2]]

            is_identity_quat = (abs(quat_wxyz[0] - 1.0) < 1e-6 and
                                 all(abs(quat_wxyz[i]) < 1e-6 for i in range(1, 4)))

            success = update_body_inertial(
                xml_root, link_name,
                mass=mass,
                pos=com_xyz,
                diaginertia=diag,
                quat=None if is_identity_quat else quat_wxyz
            )
            if success:
                print(f"    {link_name}: mass={mass:.4f}  diag={[f'{v:.6e}' for v in diag]}")

    # 保存
    save_xml(xml_tree, output_xml_path)
    print(f"\n[完成] MuJoCo XML 已更新: {output_xml_path}")
