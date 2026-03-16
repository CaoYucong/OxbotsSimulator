#!/usr/bin/env python3
"""
calibrate.py
使用 OpenCV 标定相机（monocular）并保存 camera_info.yaml（ROS-friendly）
用法:
  python calibrate.py --images_dir images --board_cols 8 --board_rows 6 --square 0.025
说明:
  board_cols/board_rows = 内角点数 (例如 8x6)
  square = 单位米 (例如 0.025 表示 25mm)

/opt/anaconda3/envs/Unibots/bin/python calibrate.py --images_dir images \
  --board_cols 9 --board_rows 6 \
  --square 0.0225556 \
  --show_corners --out_yaml camera_info.yaml

此版本改动：
- comparison 去畸变图使用 alpha = 1.0（保留所有像素，不裁切）
- 输出文件名：undistorted_sample_alpha1.png, calib_result_compare_alpha1.png
"""

import glob
import cv2
import numpy as np
import argparse
import yaml
import os
from matplotlib import pyplot as plt

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--images_dir", required=True, help="存放标定图片的目录")
    p.add_argument("--board_cols", type=int, required=True, help="棋盘内角点列数 (例如 8)")
    p.add_argument("--board_rows", type=int, required=True, help="棋盘内角点行数 (例如 6)")
    p.add_argument("--square", type=float, required=True, help="方格边长（米）")
    p.add_argument("--show_corners", action="store_true", help="显示检测到的角点")
    p.add_argument("--out_yaml", default="camera_info.yaml", help="输出的 camera_info yaml 文件名")
    p.add_argument("--alpha", type=float, default=1.0, help="getOptimalNewCameraMatrix 的 alpha 参数 (0..1)，1 表示保留所有像素")
    return p.parse_args()

def main():
    args = parse_args()
    images = sorted(glob.glob(os.path.join(args.images_dir, "*.*")))
    if not images:
        raise SystemExit("没有找到任何图片在: " + args.images_dir)

    cb_size = (args.board_cols, args.board_rows)
    # 世界坐标中的角点（Z=0）
    objp = np.zeros((cb_size[1]*cb_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:cb_size[0], 0:cb_size[1]].T.reshape(-1, 2)
    objp *= args.square

    objpoints = []  # 3d points in world
    imgpoints = []  # 2d points in image
    img_shape = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print("无法读取: ", fname)
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_shape is None:
            img_shape = gray.shape[::-1]

        ret, corners = cv2.findChessboardCorners(gray, cb_size,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret:
            # 亚像素角点精化
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
            objpoints.append(objp)
            imgpoints.append(corners2)
            print(f"检测到角点: {fname}")
            if args.show_corners:
                vis = cv2.drawChessboardCorners(img.copy(), cb_size, corners2, ret)
                cv2.imshow('corners', vis)
                cv2.waitKey(200)
        else:
            print(f"未检测到角点: {fname}")
    if args.show_corners:
        cv2.destroyAllWindows()

    if not objpoints:
        raise SystemExit("没有检测到任何角点，请检查图片和 chessboard 配置（内角点数）")

    # 标定
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)
    print("标定完成: RMS reprojection error = ", ret)
    print("相机矩阵 K:\n", K)
    print("畸变系数 dist:\n", dist.ravel())

    # 计算每张图片的重投影误差
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    mean_error /= len(objpoints)
    print("平均重投影误差 (pixels):", mean_error)

    # 保存 camera_info.yaml (ROS 格式)
    cam_info = {
        'camera_matrix': {
            'rows': 3, 'cols': 3,
            'data': [float(K[0,0]), float(K[0,1]), float(K[0,2]),
                     float(K[1,0]), float(K[1,1]), float(K[1,2]),
                     float(K[2,0]), float(K[2,1]), float(K[2,2])]
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1, 'cols': len(dist.ravel()), 'data': [float(x) for x in dist.ravel()]
        },
        'image_width': int(img_shape[0]),
        'image_height': int(img_shape[1])
    }
    with open(args.out_yaml, 'w') as f:
        yaml.safe_dump(cam_info, f)
    print("保存 camera_info 到:", args.out_yaml)

    # 保存为 npz 以便后续加载 (K, dist)
    np.savez("camera_calib.npz", K=K, dist=dist, rvecs=rvecs, tvecs=tvecs)
    print("保存 camera_calib.npz")

    # 可视化去畸变示例（显示第一张）—— 使用 alpha 参数 (默认 1.0，保留所有像素)
    sample = cv2.imread(images[0])
    h, w = sample.shape[:2]

    # alpha 控制是否保留所有像素：0 = 最大裁切, 1 = 保留所有像素
    alpha = float(args.alpha)
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha, (w, h))
    und = cv2.undistort(sample, K, dist, None, new_K)

    # 保存示例，包含 alpha 标记
    und_fname = f"undistorted_sample_alpha{alpha:.2f}.png"
    cv2.imwrite(und_fname, und)
    print("保存去畸变示例:", und_fname)

    # 画出标定结果：原图 vs 去畸变（alpha=1）
    plt.figure(figsize=(10,5))
    plt.subplot(1,2,1); plt.title('original'); plt.imshow(cv2.cvtColor(sample, cv2.COLOR_BGR2RGB)); plt.axis('off')
    plt.subplot(1,2,2); plt.title(f'undistorted (alpha={alpha})'); plt.imshow(cv2.cvtColor(und, cv2.COLOR_BGR2RGB)); plt.axis('off')
    plt.tight_layout()
    compare_fname = f"calib_result_compare_alpha{alpha:.2f}.png"
    plt.savefig(compare_fname)
    print("保存对比图:", compare_fname)

if __name__ == "__main__":
    main()