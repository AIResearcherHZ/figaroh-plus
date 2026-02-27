"""从STL文件提取脚底碰撞体坐标点 - 基于凸包实际顶点"""
import numpy as np
import struct
import os
from scipy.spatial import ConvexHull

def read_stl_binary(filepath):
    """读取二进制STL文件，返回所有顶点"""
    vertices = []
    with open(filepath, 'rb') as f:
        f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        for _ in range(num_triangles):
            f.read(12)
            for _ in range(3):
                v = struct.unpack('<fff', f.read(12))
                vertices.append(v)
            f.read(2)
    return np.array(vertices)

def get_convex_hull_points(filepath, z_threshold_ratio=0.05, num_points=9):
    """
    从ankle_roll_link的STL提取脚底碰撞点（基于凸包实际顶点）
    
    步骤：
    1. 读取STL所有顶点
    2. 筛选脚底平面附近的顶点（z最低的5%高度范围）
    3. 计算这些底部顶点在XY平面的2D凸包
    4. 沿凸包边界等弧长采样num_points个点（都是实际mesh上的点）
    """
    vertices = read_stl_binary(filepath)
    unique_verts = np.unique(np.round(vertices, 6), axis=0)
    
    z_min = unique_verts[:, 2].min()
    z_max = unique_verts[:, 2].max()
    z_range = z_max - z_min
    z_cutoff = z_min + z_range * z_threshold_ratio
    bottom_verts = unique_verts[unique_verts[:, 2] <= z_cutoff]
    
    print(f"  文件: {os.path.basename(filepath)}")
    print(f"  总顶点: {len(unique_verts)}, 底部顶点: {len(bottom_verts)}")
    print(f"  Z范围: [{z_min:.5f}, {z_max:.5f}], 截断Z: {z_cutoff:.5f}")
    
    # 2D凸包
    xy = bottom_verts[:, :2]
    hull = ConvexHull(xy)
    hull_verts = xy[hull.vertices]
    
    # 按角度排序凸包顶点
    center = hull_verts.mean(axis=0)
    angles = np.arctan2(hull_verts[:, 1] - center[1], hull_verts[:, 0] - center[0])
    sorted_idx = np.argsort(angles)
    hull_verts = hull_verts[sorted_idx]
    
    # 沿凸包边界等弧长采样
    # 先计算凸包周长上每段的累积长度
    n = len(hull_verts)
    closed = np.vstack([hull_verts, hull_verts[0:1]])  # 闭合
    seg_lengths = np.sqrt(np.sum(np.diff(closed, axis=0)**2, axis=1))
    cum_lengths = np.concatenate([[0], np.cumsum(seg_lengths)])
    total_length = cum_lengths[-1]
    
    # 等间距采样点
    sample_dists = np.linspace(0, total_length, num_points, endpoint=False)
    sampled_points = []
    for d in sample_dists:
        # 找到d落在哪一段
        seg_idx = np.searchsorted(cum_lengths, d, side='right') - 1
        seg_idx = min(seg_idx, n - 1)
        # 在该段上插值
        seg_start = cum_lengths[seg_idx]
        seg_len = seg_lengths[seg_idx]
        if seg_len > 0:
            t = (d - seg_start) / seg_len
        else:
            t = 0
        p = closed[seg_idx] * (1 - t) + closed[seg_idx + 1] * t
        sampled_points.append(p)
    
    sampled_points = np.array(sampled_points)
    
    # 将每个采样点snap到最近的实际mesh底部顶点
    snapped = []
    for sp in sampled_points:
        dists = np.sqrt(np.sum((bottom_verts[:, :2] - sp)**2, axis=1))
        nearest_idx = np.argmin(dists)
        nearest = bottom_verts[nearest_idx]
        snapped.append(nearest)
    snapped = np.array(snapped)
    
    # 去重（可能snap到同一个顶点）
    seen = set()
    unique_snapped = []
    for pt in snapped:
        key = (round(pt[0], 5), round(pt[1], 5), round(pt[2], 5))
        if key not in seen:
            seen.add(key)
            unique_snapped.append(pt)
    snapped = np.array(unique_snapped)
    
    # 输出
    sole_z = z_min
    result = []
    print(f"  脚底Z: {sole_z:.5f}")
    print(f"  凸包顶点数: {len(hull_verts)}, 采样点数: {len(snapped)}")
    for i, pt in enumerate(snapped):
        x, y, z = round(float(pt[0]), 4), round(float(pt[1]), 4), round(float(pt[2]), 4)
        result.append([x, y, z])
        print(f"    点{i}: xyz=\"{x} {y} {z}\"")
    
    return result

def generate_urdf_collision(points, radius=0.005):
    """生成URDF碰撞体XML"""
    lines = []
    for pt in points:
        lines.append(f'    <collision>')
        lines.append(f'      <origin xyz="{pt[0]} {pt[1]} {pt[2]}" rpy="0 0 0"/>')
        lines.append(f'      <geometry>')
        lines.append(f'        <sphere radius="{radius}"/>')
        lines.append(f'      </geometry>')
        lines.append(f'    </collision>')
    return '\n'.join(lines)

def generate_mujoco_collision(points, radius=0.005):
    """生成MuJoCo XML碰撞体"""
    lines = []
    for pt in points:
        lines.append(f'                  <geom size="{radius}" pos="{pt[0]} {pt[1]} {pt[2]}" rgba="1.0 1.0 1.0 1"/>')
    return '\n'.join(lines)

if __name__ == '__main__':
    base = os.path.dirname(os.path.abspath(__file__))
    
    # === 验证g1 ===
    print("=" * 60)
    print("验证: g1 left_ankle_roll_link")
    print("已知碰撞点(URDF):")
    g1_known = [
        (-0.05, 0.025, -0.03), (-0.05, -0.025, -0.03),
        (0.12, 0.03, -0.03), (0.12, -0.03, -0.03),
        (0.05, -0.035, -0.03), (0.05, 0.035, -0.03),
        (0.14, 0.0, -0.03), (-0.0625, 0.0, -0.03),
        (0.1, 0.0, -0.01)
    ]
    for p in g1_known:
        print(f"  ({p[0]}, {p[1]}, {p[2]})")
    g1_path = os.path.join(base, '..', 'g1', 'meshes', 'left_ankle_roll_link.STL')
    if os.path.exists(g1_path):
        pts_g1 = get_convex_hull_points(g1_path, num_points=9)
    
    # === 验证pi_12dof ===
    print("\n" + "=" * 60)
    print("验证: pi_12dof r_ankle_roll_link")
    print("已知碰撞点(URDF):")
    pi_known = [
        (0.03, 0, -0.043), (0.027, 0.03, -0.043),
        (0.027, -0.03, -0.043), (-0.045, -0.032, -0.043),
        (-0.045, 0.032, -0.043), (-0.115, 0.027, -0.043),
        (-0.115, -0.027, -0.043), (-0.132, -0.015, -0.043),
        (-0.132, 0.015, -0.043)
    ]
    for p in pi_known:
        print(f"  ({p[0]}, {p[1]}, {p[2]})")
    pi_path = os.path.join(base, '..', 'pi_12dof', 'meshes', 'r_ankle_roll_link.STL')
    if os.path.exists(pi_path):
        pts_pi = get_convex_hull_points(pi_path, num_points=9)
    
    # === Taks_T1 左脚 ===
    print("\n" + "=" * 60)
    print("目标: Taks_T1 left_ankle_roll_link")
    t1_left = os.path.join(base, 'meshes', 'left_ankle_roll_link.STL')
    if os.path.exists(t1_left):
        pts_l = get_convex_hull_points(t1_left, num_points=9)
        print("\n  URDF碰撞体:")
        print(generate_urdf_collision(pts_l))
        print("\n  MuJoCo碰撞体:")
        print(generate_mujoco_collision(pts_l))
    
    # === Taks_T1 右脚 ===
    print("\n" + "=" * 60)
    print("目标: Taks_T1 right_ankle_roll_link")
    t1_right = os.path.join(base, 'meshes', 'right_ankle_roll_link.STL')
    if os.path.exists(t1_right):
        pts_r = get_convex_hull_points(t1_right, num_points=9)
        print("\n  URDF碰撞体:")
        print(generate_urdf_collision(pts_r))
        print("\n  MuJoCo碰撞体:")
        print(generate_mujoco_collision(pts_r))
