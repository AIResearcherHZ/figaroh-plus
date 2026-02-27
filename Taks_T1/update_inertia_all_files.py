"""
批量更新所有XML和URDF文件的惯性参数
基于STL mesh计算的惯性数据
"""

import os
import re

# 从STL计算得到的惯性数据
INERTIA_DATA = {
    "left_ankle_pitch_link": {
        "mass": 0.23,
        "com": [0.000000, -0.024000, 0.000000],
        "diaginertia": [1.93535e-05, 6.81815e-05, 7.18285e-05],
        "quat": [1.000000, -0.000070, -0.000000, -0.000005],
        "urdf_inertia": {"ixx": 1.93535e-05, "ixy": 4.64957e-10, "ixz": -3.78169e-14, "iyy": 6.81815e-05, "iyz": 5.13599e-10, "izz": 7.18285e-05},
    },
    "left_ankle_roll_link": {
        "mass": 0.15,
        "com": [0.039442, 0.002884, -0.032059],
        "diaginertia": [0.00012722, 0.000547803, 0.000633425],
        "quat": [0.999149, 0.009110, 0.020904, -0.034363],
        "urdf_inertia": {"ixx": 0.000130082, "ixy": 2.85582e-05, "ixz": 2.12936e-05, "iyy": 0.000545875, "iyz": -2.89761e-06, "izz": 0.000632491},
    },
    "left_elbow_link": {
        "mass": 0.552,
        "com": [0.060290, -0.018897, -0.005505],
        "diaginertia": [0.000980685, 0.00143241, 0.00168229],
        "quat": [0.997566, 0.037813, -0.047139, -0.034794],
        "urdf_inertia": {"ixx": 0.000989202, "ixy": 3.44852e-05, "ixz": -6.49941e-05, "iyy": 0.00143131, "iyz": -1.48861e-05, "izz": 0.00167487},
    },
    "left_hip_pitch_link": {
        "mass": 0.91,
        "com": [-0.019957, 0.068375, -0.041781],
        "diaginertia": [0.000513325, 0.00109885, 0.00117005],
        "quat": [-0.588219, -0.416223, 0.640256, 0.266136],
        "urdf_inertia": {"ixx": 0.00116563, "ixy": 2.93984e-05, "ixz": -4.3747e-07, "iyy": 0.000681287, "iyz": 0.000265154, "izz": 0.000935303},
    },
    "left_hip_roll_link": {
        "mass": 0.79,
        "com": [-0.001024, -0.000004, -0.060249],
        "diaginertia": [0.000556908, 0.00104397, 0.00115351],
        "quat": [0.593000, 0.000065, 0.805202, 0.000058],
        "urdf_inertia": {"ixx": 0.00110099, "ixy": 2.68436e-08, "ixz": -0.000169042, "iyy": 0.00104397, "iyz": 8.0016e-08, "izz": 0.000609428},
    },
    "left_hip_yaw_link": {
        "mass": 1.822,
        "com": [-0.042586, 0.024809, -0.158574],
        "diaginertia": [0.00208373, 0.00482553, 0.00602348],
        "quat": [0.611846, -0.534833, -0.395188, -0.428281],
        "urdf_inertia": {"ixx": 0.00454414, "ixy": 5.88587e-05, "ixz": -0.000831833, "iyy": 0.00597854, "iyz": 0.00039918, "izz": 0.00241007},
    },
    "left_knee_link": {
        "mass": 1.138,
        "com": [0.003930, -0.003975, -0.131002],
        "diaginertia": [0.000615815, 0.0048068, 0.00482231],
        "quat": [-0.695925, 0.066490, 0.706374, 0.110923],
        "urdf_inertia": {"ixx": 0.00481921, "ixy": -9.45358e-06, "ixz": 9.43514e-05, "iyy": 0.00479244, "iyz": 0.000252806, "izz": 0.000633261},
    },
    "left_shoulder_pitch_link": {
        "mass": 0.24,
        "com": [-0.003496, 0.038568, 0.000147],
        "diaginertia": [0.000216626, 0.000302215, 0.000438512],
        "quat": [0.942818, 0.005189, 0.001305, -0.333265],
        "urdf_inertia": {"ixx": 0.000250427, "ixy": 4.18386e-05, "ixz": 2.58077e-07, "iyy": 0.000268432, "iyz": -1.77057e-06, "izz": 0.000438494},
    },
    "left_shoulder_roll_link": {
        "mass": 0.92,
        "com": [-0.034326, 0.004104, -0.054527],
        "diaginertia": [0.00160635, 0.00174986, 0.00232011],
        "quat": [-0.466607, 0.541513, 0.529133, 0.457232],
        "urdf_inertia": {"ixx": 0.0017498, "ixy": 3.29232e-07, "ixz": -3.22734e-06, "iyy": 0.0023048, "iyz": -0.000103378, "izz": 0.00162172},
    },
    "left_shoulder_yaw_link": {
        "mass": 0.625,
        "com": [0.000352, -0.000067, -0.086391],
        "diaginertia": [0.000836482, 0.00204864, 0.002069],
        "quat": [-0.705828, -0.040991, 0.706119, -0.039023],
        "urdf_inertia": {"ixx": 0.00206874, "ixy": 2.28383e-06, "ixz": 3.18573e-07, "iyy": 0.00204889, "iyz": 3.39751e-06, "izz": 0.000836491},
    },
    "left_wrist_pitch_link": {
        "mass": 0.308,
        "com": [0.072931, -0.016145, 0.006694],
        "diaginertia": [0.00020361, 0.000360223, 0.000390146],
        "quat": [0.743661, -0.619814, 0.106806, -0.226696],
        "urdf_inertia": {"ixx": 0.000246269, "ixy": 7.5801e-05, "ixz": -1.39804e-05, "iyy": 0.000348519, "iyz": 1.44425e-05, "izz": 0.00035919},
    },
    "left_wrist_roll_link": {
        "mass": 0.508,
        "com": [0.059365, -0.000127, 0.000856],
        "diaginertia": [0.000846714, 0.00108933, 0.00119508],
        "quat": [0.709390, 0.704504, -0.014627, 0.015066],
        "urdf_inertia": {"ixx": 0.000847141, "ixy": -2.35849e-07, "ixz": -1.01761e-05, "iyy": 0.00119508, "iyz": -7.39738e-07, "izz": 0.00108891},
    },
    "left_wrist_yaw_link": {
        "mass": 0.37,
        "com": [0.000001, 0.001808, 0.034229],
        "diaginertia": [0.00015129, 0.000161838, 0.000185039],
        "quat": [0.505557, -0.497530, -0.494299, -0.502538],
        "urdf_inertia": {"ixx": 0.000161838, "ixy": 7.17778e-09, "ixz": -6.5797e-08, "iyy": 0.00018503, "iyz": 5.48824e-07, "izz": 0.000151299},
    },
    "neck_pitch_link": {
        "mass": 0.6,
        "com": [0.009305, -0.020598, 0.085322],
        "diaginertia": [0.00224315, 0.00340582, 0.00427196],
        "quat": [0.551128, 0.557629, 0.439744, -0.438101],
        "urdf_inertia": {"ixx": 0.00334466, "ixy": 1.35608e-06, "ixz": 0.00025961, "iyy": 0.00427182, "iyz": 1.60119e-05, "izz": 0.00230445},
    },
    "neck_roll_link": {
        "mass": 0.026,
        "com": [0.057357, -0.000598, -0.000025],
        "diaginertia": [6.4839e-06, 1.35416e-05, 1.62635e-05],
        "quat": [-0.674875, -0.000416, 0.000618, 0.737931],
        "urdf_inertia": {"ixx": 1.34856e-05, "ixy": -6.26247e-07, "ixz": -3.80287e-09, "iyy": 6.53992e-06, "iyz": 2.51081e-09, "izz": 1.62635e-05},
    },
    "neck_yaw_link": {
        "mass": 0.4,
        "com": [-0.026017, 0.000004, 0.044450],
        "diaginertia": [0.000158273, 0.000510001, 0.000543254],
        "quat": [-0.000003, 0.399078, 0.000064, 0.916917],
        "urdf_inertia": {"ixx": 0.000364466, "ixy": 1.379e-08, "ixz": 0.000192002, "iyy": 0.000510001, "iyz": -8.97171e-09, "izz": 0.000337061},
    },
    "pelvis": {
        "mass": 3.34,
        "com": [-0.000276, 0.004540, -0.078485],
        "diaginertia": [0.00671618, 0.0161253, 0.0176076],
        "quat": [-0.483309, 0.519158, 0.510148, -0.486453],
        "urdf_inertia": {"ixx": 0.017602, "ixy": -6.67118e-05, "ixz": 8.88231e-05, "iyy": 0.00671794, "iyz": 0.000112175, "izz": 0.0161292},
    },
    "right_ankle_pitch_link": {
        "mass": 0.23,
        "com": [0.000000, 0.024000, 0.000000],
        "diaginertia": [1.93534e-05, 6.81821e-05, 7.18293e-05],
        "quat": [1.000000, 0.000071, 0.000000, 0.000004],
        "urdf_inertia": {"ixx": 1.93534e-05, "ixy": -4.10046e-10, "ixz": 4.78957e-14, "iyy": 6.81821e-05, "iyz": -5.20763e-10, "izz": 7.18293e-05},
    },
    "right_ankle_roll_link": {
        "mass": 0.15,
        "com": [0.039477, -0.002886, -0.032098],
        "diaginertia": [0.000127095, 0.000547706, 0.00063363],
        "quat": [0.999151, -0.009249, 0.020679, 0.034410],
        "urdf_inertia": {"ixx": 0.000129945, "ixy": -2.85988e-05, "ixz": 2.1084e-05, "iyy": 0.000545774, "iyz": 2.91541e-06, "izz": 0.000632714},
    },
    "right_elbow_link": {
        "mass": 0.552,
        "com": [0.060609, -0.027320, -0.005391],
        "diaginertia": [0.00103959, 0.00142282, 0.00173271],
        "quat": [0.998910, 0.001174, -0.046413, 0.004845],
        "urdf_inertia": {"ixx": 0.00104558, "ixy": -3.57132e-06, "ixz": -6.39919e-05, "iyy": 0.00142279, "iyz": -1.2025e-06, "izz": 0.00172675},
    },
    "right_hip_pitch_link": {
        "mass": 0.91,
        "com": [-0.019957, -0.068874, -0.041781],
        "diaginertia": [0.000513318, 0.00109902, 0.0011702],
        "quat": [0.416329, 0.588113, 0.266120, 0.640292],
        "urdf_inertia": {"ixx": 0.00116578, "ixy": -2.93673e-05, "ixz": -4.00738e-07, "iyy": 0.000681243, "iyz": -0.000265198, "izz": 0.000935516},
    },
    "right_hip_roll_link": {
        "mass": 0.79,
        "com": [-0.001024, -0.000004, -0.060249],
        "diaginertia": [0.000556908, 0.00104397, 0.00115351],
        "quat": [0.593000, 0.000065, 0.805202, 0.000058],
        "urdf_inertia": {"ixx": 0.00110099, "ixy": 2.68436e-08, "ixz": -0.000169042, "iyy": 0.00104397, "iyz": 8.0016e-08, "izz": 0.000609428},
    },
    "right_hip_yaw_link": {
        "mass": 1.822,
        "com": [-0.042689, -0.024539, -0.158542],
        "diaginertia": [0.00209418, 0.00478277, 0.00597518],
        "quat": [-0.427414, -0.395165, -0.535773, 0.611645],
        "urdf_inertia": {"ixx": 0.00450421, "ixy": -5.60625e-05, "ixz": -0.000819249, "iyy": 0.00593245, "iyz": -0.000386137, "izz": 0.00241546},
    },
    "right_knee_link": {
        "mass": 1.138,
        "com": [0.005297, 0.004887, -0.128398],
        "diaginertia": [0.000660407, 0.00591153, 0.00594623],
        "quat": [-0.082088, 0.692584, 0.126187, 0.705454],
        "urdf_inertia": {"ixx": 0.00593939, "ixy": 1.80972e-05, "ixz": 0.000142701, "iyy": 0.00589622, "iyz": -0.000308921, "izz": 0.000682549},
    },
    "right_shoulder_pitch_link": {
        "mass": 0.24,
        "com": [-0.003496, -0.038568, 0.000147],
        "diaginertia": [0.000216626, 0.000302215, 0.000438512],
        "quat": [0.942818, -0.005194, 0.001309, 0.333265],
        "urdf_inertia": {"ixx": 0.000250427, "ixy": -4.18386e-05, "ixz": 2.59332e-07, "iyy": 0.000268432, "iyz": 1.77275e-06, "izz": 0.000438494},
    },
    "right_shoulder_roll_link": {
        "mass": 0.92,
        "com": [-0.027387, -0.004099, -0.054543],
        "diaginertia": [0.00160664, 0.00174971, 0.00232021],
        "quat": [0.538692, -0.464976, -0.459110, -0.531814],
        "urdf_inertia": {"ixx": 0.00174968, "ixy": 2.23905e-07, "ixz": -1.81555e-06, "iyy": 0.002305, "iyz": 0.000103081, "izz": 0.00162188},
    },
    "right_shoulder_yaw_link": {
        "mass": 0.625,
        "com": [0.000351, 0.000061, -0.086330],
        "diaginertia": [0.000836592, 0.00204945, 0.00206961],
        "quat": [0.706146, 0.036788, -0.706063, 0.038462],
        "urdf_inertia": {"ixx": 0.00206938, "ixy": 2.13039e-06, "ixz": 4.95083e-09, "iyy": 0.00204967, "iyz": -2.87596e-06, "izz": 0.000836599},
    },
    "right_wrist_pitch_link": {
        "mass": 0.308,
        "com": [0.072931, 0.016145, 0.006694],
        "diaginertia": [0.000203609, 0.000360223, 0.000390147],
        "quat": [-0.619812, 0.743663, 0.226696, -0.106804],
        "urdf_inertia": {"ixx": 0.000246269, "ixy": -7.58014e-05, "ixz": -1.39808e-05, "iyy": 0.00034852, "iyz": -1.44428e-05, "izz": 0.000359191},
    },
    "right_wrist_roll_link": {
        "mass": 0.508,
        "com": [0.059348, -0.000124, 0.000828],
        "diaginertia": [0.000846771, 0.00108988, 0.00119553],
        "quat": [0.709304, 0.704564, -0.015028, 0.015864],
        "urdf_inertia": {"ixx": 0.000847235, "ixy": -4.3175e-07, "ixz": -1.06067e-05, "iyy": 0.00119552, "iyz": -7.24785e-07, "izz": 0.00108942},
    },
    "right_wrist_yaw_link": {
        "mass": 0.37,
        "com": [-0.000001, -0.001805, 0.034231],
        "diaginertia": [0.000151278, 0.00016184, 0.000185025],
        "quat": [-0.502813, -0.494486, -0.497307, 0.505320],
        "urdf_inertia": {"ixx": 0.00016184, "ixy": -9.17334e-09, "ixz": -5.61025e-08, "iyy": 0.000185016, "iyz": -5.51309e-07, "izz": 0.000151287},
    },
    "waist_pitch_link": {
        "mass": 2.01,
        "com": [-0.000218, 0.028412, 0.216121],
        "diaginertia": [0.0103593, 0.031659, 0.0398056],
        "quat": [-0.012746, 0.707391, 0.013049, -0.706587],
        "urdf_inertia": {"ixx": 0.0398056, "ixy": 2.45331e-06, "ixz": 3.33211e-05, "iyy": 0.0316306, "iyz": 0.000776359, "izz": 0.0103877},
    },
    "waist_roll_link": {
        "mass": 1.56,
        "com": [-0.036458, 0.001915, 0.039503],
        "diaginertia": [0.00121052, 0.00366195, 0.00366507],
        "quat": [-0.497668, -0.522864, -0.452622, 0.523520],
        "urdf_inertia": {"ixx": 0.00365763, "ixy": 5.23222e-06, "ixz": 0.000103033, "iyy": 0.00365945, "iyz": -0.000116974, "izz": 0.00122046},
    },
    "waist_yaw_link": {
        "mass": 0.79,
        "com": [0.000703, 0.000036, 0.023511],
        "diaginertia": [0.000424112, 0.00115195, 0.00139043],
        "quat": [0.707107, 0.707106, -0.000660, 0.000661],
        "urdf_inertia": {"ixx": 0.000424115, "ixy": -7.88004e-10, "ixz": -1.35964e-06, "iyy": 0.00139043, "iyz": -3.146e-10, "izz": 0.00115195},
    },
}

# torso_link 使用 waist_pitch_link 的数据
INERTIA_DATA["torso_link"] = INERTIA_DATA["waist_pitch_link"]


def update_xml_inertia(content, link_name, data):
    """更新XML文件中的惯性参数 (使用quat + diaginertia)"""
    com = data["com"]
    quat = data["quat"]
    diag = data["diaginertia"]
    
    # 匹配 <body name="link_name" ...> 后面的 <inertial .../>
    pattern = rf'(<body\s+name="{link_name}"[^>]*>.*?<inertial\s+)pos="[^"]*"(\s+quat="[^"]*")?\s+mass="[^"]*"\s*\n?\s*diaginertia="[^"]*"'
    
    # 构建新的惯性参数
    new_inertial = f'\\1pos="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" quat="{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}" mass="{data["mass"]}"\n        diaginertia="{diag[0]:.6g} {diag[1]:.6g} {diag[2]:.6g}"'
    
    new_content, count = re.subn(pattern, new_inertial, content, flags=re.DOTALL)
    return new_content, count


def update_urdf_inertia(content, link_name, data):
    """更新URDF文件中的惯性参数 (使用完整惯性张量)"""
    com = data["com"]
    inertia = data["urdf_inertia"]
    
    # 匹配 <link name="link_name"> 中的 <inertial> 块
    pattern = rf'(<link\s+name="{link_name}">\s*<inertial>\s*)<origin\s+xyz="[^"]*"\s+rpy="[^"]*"\s*/>\s*<mass\s+value="[^"]*"\s*/>\s*<inertia[^/]*/>'
    
    # 构建新的惯性参数
    new_inertial = f'''\\1<origin xyz="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" rpy="0 0 0"/>
      <mass value="{data["mass"]}"/>
      <inertia ixx="{inertia["ixx"]:.6g}" ixy="{inertia["ixy"]:.6g}" ixz="{inertia["ixz"]:.6g}" iyy="{inertia["iyy"]:.6g}" iyz="{inertia["iyz"]:.6g}" izz="{inertia["izz"]:.6g}"/>'''
    
    new_content, count = re.subn(pattern, new_inertial, content, flags=re.DOTALL)
    return new_content, count


def process_xml_file(filepath):
    """处理XML文件"""
    print(f"\n处理XML: {filepath}")
    
    with open(filepath, 'r') as f:
        content = f.read()
    
    total_updates = 0
    for link_name, data in INERTIA_DATA.items():
        content, count = update_xml_inertia(content, link_name, data)
        if count > 0:
            print(f"  更新 {link_name}: {count}")
            total_updates += count
    
    with open(filepath, 'w') as f:
        f.write(content)
    
    print(f"  总计更新: {total_updates}")
    return total_updates


def process_urdf_file(filepath):
    """处理URDF文件"""
    print(f"\n处理URDF: {filepath}")
    
    with open(filepath, 'r') as f:
        content = f.read()
    
    total_updates = 0
    for link_name, data in INERTIA_DATA.items():
        content, count = update_urdf_inertia(content, link_name, data)
        if count > 0:
            print(f"  更新 {link_name}: {count}")
            total_updates += count
    
    with open(filepath, 'w') as f:
        f.write(content)
    
    print(f"  总计更新: {total_updates}")
    return total_updates


def main():
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 仅更新Taks_T1目录下的文件
    xml_files = [
        os.path.join(script_dir, "Taks_T1.xml"),
    ]
    
    urdf_files = [
        os.path.join(script_dir, "Taks_T1.urdf"),
    ]
    
    print("=" * 60)
    print("批量更新惯性参数")
    print("=" * 60)
    
    # 处理XML文件
    for filepath in xml_files:
        if os.path.exists(filepath):
            process_xml_file(filepath)
        else:
            print(f"\n警告: 文件不存在 {filepath}")
    
    # 处理URDF文件
    for filepath in urdf_files:
        if os.path.exists(filepath):
            process_urdf_file(filepath)
        else:
            print(f"\n警告: 文件不存在 {filepath}")
    
    print("\n" + "=" * 60)
    print("完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
