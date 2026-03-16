#!/usr/bin/env python3
"""
calibrate_fisheye.py
针对鱼眼/超广角镜头的 OpenCV fisheye 标定脚本。
输出:
  - camera_info.yaml (包含 camera_matrix, distortion_coefficients, image_width/height)
  - camera_calib_fisheye.npz (K, D, rvecs, tvecs)
  - undistorted_sample_alpha{balance:.2f}.png
  - calib_result_compare_alpha{balance:.2f}.png

使用示例:
/opt/anaconda3/envs/Unibots/bin/python calibrate_fisheye.py --images_dir images --board_cols 9 --board_rows 6 --square 0.0225556 --balance 1.0 --show_corners

说明:
- board_cols/board_rows = 内角点数（interior corners），例如 9x6 squares
- square = 方格边长（米）
- balance ∈ [0,1] (同 OpenCV fisheye API): 0 -> 裁切以无黑边，1 -> 保留所有像素（可能有黑边）
"""
import os
import glob
import argparse
import yaml
import numpy as np
import cv2
from matplotlib import pyplot as plt

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--images_dir", required=True, help="存放标定图片的目录")
    p.add_argument("--board_cols", type=int, required=True, help="棋盘内角点列数 (例如 8)")
    p.add_argument("--board_rows", type=int, required=True, help="棋盘内角点行数 (例如 5)")
    p.add_argument("--square", type=float, required=True, help="方格边长（米）")
    p.add_argument("--show_corners", action="store_true", help="显示检测到的角点")
    p.add_argument("--out_yaml", default="camera_info.yaml", help="输出的 camera_info yaml 文件名")
    p.add_argument("--balance", type=float, default=1.0, help="fisheye balance 参数, 0..1。1 保留所有像素")
    p.add_argument("--fov_scale", type=float, default=1.0, help="fisheye fov_scale 参数 (一般 1.0)")
    return p.parse_args()

def collect_corners(images, cb_size, square, show_corners=False):
    objp = np.zeros((cb_size[1] * cb_size[0], 1, 3), np.float32)
    objp[:, 0, :2] = np.mgrid[0:cb_size[0], 0:cb_size[1]].T.reshape(-1, 2)
    objp *= square

    objpoints = []
    imgpoints = []
    img_shape = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print("无法读取:", fname)
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_shape is None:
            img_shape = gray.shape[::-1]  # (w, h)

        ret, corners = cv2.findChessboardCorners(gray, cb_size,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
            objpoints.append(objp.copy())
            imgpoints.append(corners2.astype(np.float32))
            print("检测到角点:", os.path.basename(fname))
            if show_corners:
                vis = cv2.drawChessboardCorners(img.copy(), cb_size, corners2, ret)
                cv2.imshow('corners', vis)
                cv2.waitKey(200)
        else:
            print("未检测到角点:", os.path.basename(fname))
    if show_corners:
        cv2.destroyAllWindows()
    return objpoints, imgpoints, img_shape

def fisheye_calibrate(objpoints, imgpoints, img_shape):
    # OpenCV fisheye 接口要求：
    N_OK = len(objpoints)
    if N_OK < 1:
        raise RuntimeError("没有足够的角点样本进行标定")

    K = np.zeros((3,3))
    D = np.zeros((4,1))
    rvecs = []
    tvecs = []

    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-9)

    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        img_shape,
        K,
        D,
        rvecs,
        tvecs,
        flags=flags,
        criteria=criteria
    )
    return rms, K, D, rvecs, tvecs

def compute_reprojection_error_fisheye(objpoints, imgpoints, rvecs, tvecs, K, D):
    total_err = 0.0
    total_points = 0
    for i in range(len(objpoints)):
        # fisheye projectPoints expects object points as (N,3)
        imgpoints_proj, _ = cv2.fisheye.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        err = cv2.norm(imgpoints[i], imgpoints_proj, cv2.NORM_L2)
        n = len(imgpoints_proj)
        total_err += err**2
        total_points += n
    mean_error = np.sqrt(total_err / total_points) if total_points > 0 else 0
    return mean_error

def save_camera_info_yaml(path, K, D, width, height):
    data = {
        'camera_matrix': {
            'rows': 3, 'cols': 3,
            'data': [float(K[0,0]), float(K[0,1]), float(K[0,2]),
                     float(K[1,0]), float(K[1,1]), float(K[1,2]),
                     float(K[2,0]), float(K[2,1]), float(K[2,2])]
        },
        # 标注为 fisheye，使用者在 ROS 里加载时需确认其接受 fisheye 模型或自行处理去畸变
        'distortion_model': 'fisheye',
        'distortion_coefficients': {
            'rows': 1, 'cols': D.size, 'data': [float(x) for x in D.ravel()]
        },
        'image_width': int(width),
        'image_height': int(height)
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)
    print("保存 camera_info 到:", path)

def undistort_sample_and_save(sample_path, K, D, balance, fov_scale, out_prefix):
    sample = cv2.imread(sample_path)
    if sample is None:
        raise RuntimeError("无法读取 sample 图片: " + sample_path)
    h, w = sample.shape[:2]

    # R = identity (no rectification rotation)
    R = np.eye(3)

    # estimateNewCameraMatrixForUndistortRectify: balance in [0,1], fov_scale 控制缩放
    newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (w, h), R, balance=balance, new_size=(w, h), fov_scale=fov_scale
    )

    und = cv2.fisheye.undistortImage(sample, K, D, Knew=newK)
    und_name = f"{out_prefix}_undistorted_alpha{balance:.2f}.png"
    cv2.imwrite(und_name, und)
    print("保存去畸变示例:", und_name)

    # 对比图
    plt.figure(figsize=(10,5))
    plt.subplot(1,2,1); plt.title('original'); plt.imshow(cv2.cvtColor(sample, cv2.COLOR_BGR2RGB)); plt.axis('off')
    plt.subplot(1,2,2); plt.title(f'undistorted (balance={balance:.2f})'); plt.imshow(cv2.cvtColor(und, cv2.COLOR_BGR2RGB)); plt.axis('off')
    plt.tight_layout()
    compare_name = f"{out_prefix}_calib_result_compare_alpha{balance:.2f}.png"
    plt.savefig(compare_name)
    print("保存对比图:", compare_name)
    plt.close()

def main():
    args = parse_args()
    images = sorted(glob.glob(os.path.join(args.images_dir, "*.*")))
    if not images:
        raise SystemExit("没有找到任何图片在: " + args.images_dir)

    cb_size = (args.board_cols, args.board_rows)
    print("棋盘内角:", cb_size, "方格尺寸 (m):", args.square)
    objpoints, imgpoints, img_shape = collect_corners(images, cb_size, args.square, args.show_corners)
    if not objpoints:
        raise SystemExit("没有检测到任何角点，请检查图片和 chessboard 配置（内角点数）")

    print("图像分辨率 (w,h):", img_shape)
    # 标定
    rms, K, D, rvecs, tvecs = fisheye_calibrate(objpoints, imgpoints, img_shape)
    print("fisheye 标定完成: RMS (OpenCV returned) =", rms)
    print("相机矩阵 K:\n", K)
    print("畸变系数 D (4):\n", D.ravel())

    mean_err = compute_reprojection_error_fisheye(objpoints, imgpoints, rvecs, tvecs, K, D)
    print("平均重投影误差 (pixels, RMSE):", mean_err)

    # 保存 camera_info.yaml (注意 distortion_model = 'fisheye')
    save_camera_info_yaml(args.out_yaml, K, D, img_shape[0], img_shape[1])

    # 保存 npz（便于后续加载和调试）
    np.savez("camera_calib_fisheye.npz", K=K, D=D, rvecs=rvecs, tvecs=tvecs)
    print("保存 camera_calib_fisheye.npz")

    # 选择第一张作为示例做 undistort (balance 参数控制保留像素，1.0 为保留所有)
    sample_path = images[0]
    try:
        undistort_sample_and_save(sample_path, K, D, args.balance, args.fov_scale, out_prefix="fisheye")
    except Exception as e:
        print("去畸变示例失败:", e)

if __name__ == "__main__":
    main()