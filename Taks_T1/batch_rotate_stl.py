#!/usr/bin/env python3
"""
批量旋转STL文件工具
使用pymeshlab库，先绕X轴旋转90度，再绕Z轴旋转90度
"""

# python batch_rotate_stl.py /home/xhz/下载/t1_leg_v2/meshes
# python batch_rotate_stl.py file1.stl file2.stl file3.stl

import pymeshlab
import os
import argparse
from pathlib import Path


def rotate_stl(input_path: str, output_path: str = None):
    """
    旋转单个STL文件
    先绕X轴(rotaxis=0)旋转90度，再绕Z轴(rotaxis=2)旋转90度
    
    Args:
        input_path: 输入STL文件路径
        output_path: 输出STL文件路径，默认覆盖原文件
    """
    if output_path is None:
        output_path = input_path
    
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(input_path)
    
    # 先绕X轴旋转90度
    ms.compute_matrix_from_rotation(rotaxis=0, angle=90.0)
    # 再绕Z轴旋转90度
    ms.compute_matrix_from_rotation(rotaxis=2, angle=90.0)
    
    ms.save_current_mesh(output_path)
    print(f"已处理: {input_path} -> {output_path}")


def batch_rotate_folder(folder_path: str, output_folder: str = None):
    """
    批量旋转文件夹中的所有STL文件
    
    Args:
        folder_path: 输入文件夹路径
        output_folder: 输出文件夹路径，默认覆盖原文件
    """
    folder = Path(folder_path)
    stl_files = list(folder.glob("*.stl")) + list(folder.glob("*.STL"))
    
    if not stl_files:
        print(f"文件夹 {folder_path} 中没有找到STL文件")
        return
    
    if output_folder:
        out_dir = Path(output_folder)
        out_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"找到 {len(stl_files)} 个STL文件")
    
    for stl_file in stl_files:
        if output_folder:
            output_path = str(Path(output_folder) / stl_file.name)
        else:
            output_path = str(stl_file)
        rotate_stl(str(stl_file), output_path)
    
    print(f"完成! 共处理 {len(stl_files)} 个文件")


def batch_rotate_files(file_list: list, output_folder: str = None):
    """
    批量旋转指定的STL文件列表
    
    Args:
        file_list: STL文件路径列表
        output_folder: 输出文件夹路径，默认覆盖原文件
    """
    if output_folder:
        out_dir = Path(output_folder)
        out_dir.mkdir(parents=True, exist_ok=True)
    
    for file_path in file_list:
        if not file_path.lower().endswith('.stl'):
            print(f"跳过非STL文件: {file_path}")
            continue
        
        if output_folder:
            output_path = str(Path(output_folder) / Path(file_path).name)
        else:
            output_path = file_path
        rotate_stl(file_path, output_path)
    
    print(f"完成! 共处理 {len(file_list)} 个文件")


def main():
    parser = argparse.ArgumentParser(description="批量旋转STL文件 (X轴90° + Z轴90°)")
    parser.add_argument("input", nargs="+", help="输入文件或文件夹路径")
    parser.add_argument("-o", "--output", help="输出文件夹路径（可选，默认覆盖原文件）")
    
    args = parser.parse_args()
    
    # 判断输入是文件夹还是文件
    if len(args.input) == 1 and os.path.isdir(args.input[0]):
        # 输入是文件夹
        batch_rotate_folder(args.input[0], args.output)
    else:
        # 输入是文件列表
        batch_rotate_files(args.input, args.output)


if __name__ == "__main__":
    main()
