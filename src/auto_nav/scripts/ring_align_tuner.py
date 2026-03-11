#!/usr/bin/env python3
"""
ring_align_tuner — HSV閾値リアルタイム調整ツール

カメラ映像にHSVフィルタを適用し、trackbar で閾値を調整できる。
調整した値を ring_colors.yaml にコピーして使う。

使い方:
  python3 ring_align_tuner.py [camera_device]
  python3 ring_align_tuner.py 0
"""

import sys

import cv2
import numpy as np


def main():
    device = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"cannot open camera device {device}")
        sys.exit(1)

    cv2.namedWindow("tuner")
    cv2.createTrackbar("H min", "tuner", 0, 180, lambda _: None)
    cv2.createTrackbar("H max", "tuner", 180, 180, lambda _: None)
    cv2.createTrackbar("S min", "tuner", 100, 255, lambda _: None)
    cv2.createTrackbar("S max", "tuner", 255, 255, lambda _: None)
    cv2.createTrackbar("V min", "tuner", 80, 255, lambda _: None)
    cv2.createTrackbar("V max", "tuner", 255, 255, lambda _: None)

    print("q: quit, s: print current values")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h_min = cv2.getTrackbarPos("H min", "tuner")
        h_max = cv2.getTrackbarPos("H max", "tuner")
        s_min = cv2.getTrackbarPos("S min", "tuner")
        s_max = cv2.getTrackbarPos("S max", "tuner")
        v_min = cv2.getTrackbarPos("V min", "tuner")
        v_max = cv2.getTrackbarPos("V max", "tuner")

        lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper = np.array([h_max, s_max, v_max], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        result = cv2.bitwise_and(frame, frame, mask=mask)
        combined = np.hstack([frame, result])
        cv2.imshow("tuner", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            print(f"{{h_min: {h_min}, h_max: {h_max}, "
                  f"s_min: {s_min}, s_max: {s_max}, "
                  f"v_min: {v_min}, v_max: {v_max}}}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
