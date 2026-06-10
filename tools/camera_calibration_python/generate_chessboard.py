#!/usr/bin/env python3
"""
生成 A4 横向棋盘格标定图，供 calibrate.py 使用。

对应 calibrate.py 参数：--board_cols 9 --board_rows 6（内角点）
即 10 列 x 7 行 方格。

打印时选择「实际大小 / 100%」，打印后用尺子实测方格边长，
将米制值填入 calibrate.py 的 --square（例如 25 mm -> 0.025）。
"""

import argparse
import cv2
import numpy as np


def mm_to_px(mm: float, dpi: int) -> int:
    return int(round(mm / 25.4 * dpi))


def make_chessboard(rows: int, cols: int, square_mm: float, dpi: int) -> np.ndarray:
    board_w = mm_to_px(cols * square_mm, dpi)
    board_h = mm_to_px(rows * square_mm, dpi)
    sq_w = board_w // cols
    sq_h = board_h // rows

    board = np.zeros((board_h, board_w), np.uint8)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                board[r * sq_h : (r + 1) * sq_h, c * sq_w : (c + 1) * sq_w] = 255
    return board


def parse_args():
    p = argparse.ArgumentParser(description="生成 A4 棋盘格标定图")
    p.add_argument("--rows", type=int, default=7, help="方格行数（内角点 rows + 1）")
    p.add_argument("--cols", type=int, default=10, help="方格列数（内角点 cols + 1）")
    p.add_argument("--square-mm", type=float, default=25.0, help="方格边长（毫米，设计值）")
    p.add_argument("--dpi", type=int, default=300, help="输出分辨率 DPI")
    p.add_argument("--out", default="chessboard_A4.png", help="输出文件名")
    return p.parse_args()


def main():
    args = parse_args()

    board = make_chessboard(args.rows, args.cols, args.square_mm, args.dpi)
    board_h, board_w = board.shape

    # A4 横向：297 mm x 210 mm
    a4_w = mm_to_px(297, args.dpi)
    a4_h = mm_to_px(210, args.dpi)
    canvas = np.full((a4_h, a4_w), 255, np.uint8)

    y0 = max(0, (a4_h - board_h) // 2)
    x0 = max(0, (a4_w - board_w) // 2)
    if board_h > a4_h or board_w > a4_w:
        raise SystemExit(
            f"棋盘 ({board_w}x{board_h} px) 超出 A4 横向 ({a4_w}x{a4_h} px)，"
            f"请减小 --square-mm（当前 {args.square_mm} mm）"
        )

    canvas[y0 : y0 + board_h, x0 : x0 + board_w] = board
    cv2.imwrite(args.out, canvas)

    inner_cols = args.cols - 1
    inner_rows = args.rows - 1
    print(f"已保存: {args.out}")
    print(f"棋盘: {args.cols}x{args.rows} 方格 -> 内角点 {inner_cols}x{inner_rows}")
    print(f"设计方格边长: {args.square_mm} mm -> calibrate.py --square {args.square_mm / 1000:.6f}")
    print("打印请选 100% 实际大小，打印后再用尺子实测 --square")


if __name__ == "__main__":
    main()
