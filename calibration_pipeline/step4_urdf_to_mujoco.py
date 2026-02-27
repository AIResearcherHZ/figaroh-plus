"""
Step 4: 一键将优化后的URDF参数同步到MuJoCo XML
用法:
  python step4_urdf_to_mujoco.py --robot Semi_Taks_T1
  python step4_urdf_to_mujoco.py --robot Taks_T1
  python step4_urdf_to_mujoco.py --robot Taks_T1 --urdf custom.urdf   # 指定自定义URDF
  python step4_urdf_to_mujoco.py --robot Taks_T1 --no_backup           # 不备份原XML

功能:
  1. 自动寻找最新的 *_calibrated.urdf (Step3产出), 若不存在则用原始URDF
  2. 解析URDF所有 joint/origin 和 link/inertial 参数
  3. 将参数精确写入对应的 MuJoCo XML:
     - joint origin xyz/rpy -> body pos/quat
     - link inertial mass/com/I_mat -> body inertial mass/pos/diaginertia/quat
  4. 备份原始XML -> <robot>.xml.bak.<timestamp>
  5. 写出新XML并打印变更摘要

注意:
  - MuJoCo XML 中 inertial 的 diaginertia 需要主轴分解, 本脚本自动处理
  - actuator / sensor / default 等与参数无关的部分保持不变
"""

import argparse
import sys
import shutil
import numpy as np
from pathlib import Path
from datetime import datetime
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).parent))
from common.urdf_utils import load_urdf_tree, get_joint_origins, get_link_inertials
from common.mujoco_utils import (
    load_xml_tree, save_xml, update_body_pos_quat, update_body_inertial,
    rpy_to_quat_wxyz, _find_body_recursive
)

# ── 配置 ──────────────────────────────────────────────────────────────
ROBOT_CONFIGS = {
    "Semi_Taks_T1": {
        "urdf_dir": Path(__file__).parent.parent / "Semi_Taks_T1",
        "urdf_name": "Semi_Taks_T1.urdf",
        "xml_path": Path(__file__).parent.parent / "Semi_Taks_T1" / "Semi_Taks_T1.xml",
        "robot_name": "Semi_Taks_T1",
    },
    "Taks_T1": {
        "urdf_dir": Path(__file__).parent.parent / "Taks_T1",
        "urdf_name": "Taks_T1.urdf",
        "xml_path": Path(__file__).parent.parent / "Taks_T1" / "Taks_T1.xml",
        "robot_name": "Taks_T1",
    },
}


def find_latest_urdf(cfg: dict, custom_urdf: str = None) -> Path:
    """优先使用标定后URDF, 否则回退到原始URDF"""
    if custom_urdf:
        p = Path(custom_urdf)
        if not p.exists():
            raise FileNotFoundError(f"指定的URDF不存在: {custom_urdf}")
        return p

    calibrated = cfg["urdf_dir"] / cfg["urdf_name"].replace(".urdf", "_calibrated.urdf")
    if calibrated.exists():
        print(f"  使用标定后URDF: {calibrated}")
        return calibrated

    original = cfg["urdf_dir"] / cfg["urdf_name"]
    print(f"  [提示] 未找到标定URDF, 使用原始URDF: {original}")
    print(f"  (如需标定, 请先运行 step3_calibrate_urdf.py)")
    return original


def backup_xml(xml_path: Path) -> Path:
    """备份XML到 .bak.<timestamp>"""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    bak_path = xml_path.with_suffix(f".xml.bak.{ts}")
    shutil.copy2(xml_path, bak_path)
    print(f"  原始XML已备份: {bak_path}")
    return bak_path


def sync_joint_origins(urdf_root, xml_root) -> list:
    """
    同步关节 origin -> MuJoCo body pos/quat
    返回变更记录列表
    """
    changes = []
    for joint_el in urdf_root.findall("joint"):
        jtype = joint_el.get("type")
        if jtype in ("floating", "fixed"):
            continue
        child_el = joint_el.find("child")
        if child_el is None:
            continue
        child_link = child_el.get("link")
        origin = joint_el.find("origin")
        if origin is None:
            continue

        xyz = list(map(float, origin.get("xyz", "0 0 0").split()))
        rpy = list(map(float, origin.get("rpy", "0 0 0").split()))

        # 读取XML中原值
        body = _find_body_recursive(xml_root, child_link)
        if body is None:
            continue

        old_pos_str = body.get("pos", "0 0 0")
        old_quat_str = body.get("quat", "1 0 0 0")
        old_pos = list(map(float, old_pos_str.split()))

        # 判断是否有实质变化 (阈值 1e-7)
        pos_changed = any(abs(xyz[k] - old_pos[k]) > 1e-7 for k in range(3))
        is_identity_rpy = all(abs(v) < 1e-9 for v in rpy)
        quat = rpy_to_quat_wxyz(rpy) if not is_identity_rpy else None

        # 更新body pos
        body.set("pos", " ".join(f"{v:.8g}" for v in xyz))

        # 更新或删除quat
        if is_identity_rpy:
            if "quat" in body.attrib:
                del body.attrib["quat"]
        else:
            body.set("quat", " ".join(f"{v:.8f}" for v in quat))

        if pos_changed or not is_identity_rpy:
            changes.append({
                "body": child_link,
                "old_pos": old_pos,
                "new_pos": xyz,
                "new_quat": quat,
                "rpy": rpy,
            })

    return changes


def compute_diaginertia_quat(inertia_dict: dict):
    """
    3x3惯量矩阵 -> 主轴分解
    返回 (diaginertia: list[3], quat_wxyz: list[4] or None)
    """
    from scipy.spatial.transform import Rotation

    I_mat = np.array([
        [inertia_dict.get("ixx", 0), inertia_dict.get("ixy", 0), inertia_dict.get("ixz", 0)],
        [inertia_dict.get("ixy", 0), inertia_dict.get("iyy", 0), inertia_dict.get("iyz", 0)],
        [inertia_dict.get("ixz", 0), inertia_dict.get("iyz", 0), inertia_dict.get("izz", 0)],
    ])
    eigenvalues, eigenvectors = np.linalg.eigh(I_mat)

    # 确保右手系
    if np.linalg.det(eigenvectors) < 0:
        eigenvectors[:, 0] *= -1

    # 检查对角化是否必要 (off-diagonal分量是否可忽略)
    ixx, iyy, izz = inertia_dict.get("ixx", 0), inertia_dict.get("iyy", 0), inertia_dict.get("izz", 0)
    ixy = inertia_dict.get("ixy", 0)
    ixz = inertia_dict.get("ixz", 0)
    iyz = inertia_dict.get("iyz", 0)
    diag_norm = max(abs(ixx), abs(iyy), abs(izz), 1e-20)
    off_diag_rel = max(abs(ixy), abs(ixz), abs(iyz)) / diag_norm

    diag = eigenvalues.tolist()

    if off_diag_rel < 1e-6:
        # 近似对角，直接用 ixx/iyy/izz，无需旋转
        return [ixx, iyy, izz], None

    rot = Rotation.from_matrix(eigenvectors)
    xyzw = rot.as_quat()
    quat_wxyz = [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]

    return diag, quat_wxyz


def sync_link_inertials(urdf_root, xml_root) -> list:
    """
    同步 link inertial 参数 -> MuJoCo body inertial
    返回变更记录列表
    """
    changes = []
    for link_el in urdf_root.findall("link"):
        link_name = link_el.get("name")
        inertial_el = link_el.find("inertial")
        if inertial_el is None:
            continue

        origin = inertial_el.find("origin")
        mass_el = inertial_el.find("mass")
        inertia_el = inertial_el.find("inertia")

        if mass_el is None:
            continue

        mass = float(mass_el.get("value", 0))
        com_xyz = list(map(float, origin.get("xyz", "0 0 0").split())) if origin is not None else [0.0]*3
        com_rpy = list(map(float, origin.get("rpy", "0 0 0").split())) if origin is not None else [0.0]*3

        inertia_dict = {}
        if inertia_el is not None:
            for k in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
                inertia_dict[k] = float(inertia_el.get(k, 0))

        diag, quat_wxyz = compute_diaginertia_quat(inertia_dict)

        # 若com有rpy偏转, 合并进quat
        if any(abs(v) > 1e-9 for v in com_rpy):
            from scipy.spatial.transform import Rotation
            com_q = rpy_to_quat_wxyz(com_rpy)
            r_com = Rotation.from_quat([com_q[1], com_q[2], com_q[3], com_q[0]])
            if quat_wxyz is not None:
                r_diag = Rotation.from_quat([quat_wxyz[1], quat_wxyz[2],
                                              quat_wxyz[3], quat_wxyz[0]])
                combined = (r_diag * r_com).as_quat()
            else:
                combined = r_com.as_quat()
            quat_wxyz = [combined[3], combined[0], combined[1], combined[2]]

        # 写入XML body
        body = _find_body_recursive(xml_root, link_name)
        if body is None:
            continue

        inertial = body.find("inertial")
        if inertial is None:
            inertial = ET.SubElement(body, "inertial")

        old_mass = inertial.get("mass", "?")
        inertial.set("pos", " ".join(f"{v:.8g}" for v in com_xyz))
        inertial.set("mass", f"{mass:.8g}")
        inertial.set("diaginertia", " ".join(f"{v:.10g}" for v in diag))

        if quat_wxyz is not None:
            inertial.set("quat", " ".join(f"{v:.8f}" for v in quat_wxyz))
        elif "quat" in inertial.attrib:
            del inertial.attrib["quat"]

        changes.append({
            "link": link_name,
            "old_mass": old_mass,
            "new_mass": mass,
            "com_xyz": com_xyz,
            "diaginertia": diag,
        })

    return changes


def print_change_summary(joint_changes: list, inertial_changes: list):
    """打印变更摘要"""
    print(f"\n{'='*62}")
    print(f"  变更摘要")
    print(f"{'='*62}")

    if joint_changes:
        print(f"\n  关节位置变更 ({len(joint_changes)} 个body):")
        print(f"  {'body名称':<35}  {'pos变化(mm)'}")
        print(f"  {'-'*55}")
        for c in joint_changes:
            delta = [abs((c['new_pos'][k] - c['old_pos'][k])*1000) for k in range(3)]
            max_d = max(delta)
            print(f"  {c['body']:<35}  Δmax={max_d:.3f}mm")
    else:
        print(f"\n  关节位置: 无变化")

    if inertial_changes:
        print(f"\n  惯量参数更新 ({len(inertial_changes)} 个link):")
        print(f"  {'link名称':<35}  mass(kg)")
        print(f"  {'-'*50}")
        for c in inertial_changes:
            print(f"  {c['link']:<35}  {c['new_mass']:.4f}")
    else:
        print(f"\n  惯量参数: 无变化")

    print(f"\n{'='*62}")


def main():
    parser = argparse.ArgumentParser(
        description="Step4: 一键同步URDF优化参数到MuJoCo XML"
    )
    parser.add_argument("--robot", choices=list(ROBOT_CONFIGS.keys()), required=True)
    parser.add_argument("--urdf", default=None,
                        help="指定URDF路径 (默认自动寻找_calibrated.urdf)")
    parser.add_argument("--no_backup", action="store_true",
                        help="不备份原XML")
    parser.add_argument("--joint_only", action="store_true",
                        help="只同步关节位置, 跳过惯量参数")
    parser.add_argument("--inertial_only", action="store_true",
                        help="只同步惯量参数, 跳过关节位置")
    parser.add_argument("--output", default=None,
                        help="输出XML路径 (默认覆盖原始XML)")
    args = parser.parse_args()

    cfg = ROBOT_CONFIGS[args.robot]
    print(f"\n[Step 4] URDF -> MuJoCo XML 参数同步")
    print(f"  机器人: {args.robot}")

    # 找到URDF
    urdf_path = find_latest_urdf(cfg, args.urdf)
    xml_path = cfg["xml_path"]

    if not xml_path.exists():
        raise FileNotFoundError(f"MuJoCo XML不存在: {xml_path}")

    # 备份
    if not args.no_backup:
        backup_xml(xml_path)

    # 加载文件
    print(f"\n  解析 URDF: {urdf_path}")
    _, urdf_root = load_urdf_tree(str(urdf_path))

    print(f"  解析 XML : {xml_path}")
    xml_tree, xml_root = load_xml_tree(str(xml_path))

    # 同步
    joint_changes = []
    inertial_changes = []

    if not args.inertial_only:
        print(f"\n  同步关节位置参数...")
        joint_changes = sync_joint_origins(urdf_root, xml_root)
        print(f"  更新了 {len(joint_changes)} 个body的位置/旋转")

    if not args.joint_only:
        print(f"\n  同步惯量参数...")
        inertial_changes = sync_link_inertials(urdf_root, xml_root)
        print(f"  更新了 {len(inertial_changes)} 个body的惯量")

    # 保存
    out_path = args.output or str(xml_path)
    save_xml(xml_tree, out_path)
    print(f"\n  输出XML: {out_path}")

    # 打印摘要
    print_change_summary(joint_changes, inertial_changes)

    print(f"\n[Step 4 完成] MuJoCo XML 已更新")
    print(f"  可用 MuJoCo viewer 验证: python -m mujoco.viewer --mjcf={out_path}")


if __name__ == "__main__":
    main()
