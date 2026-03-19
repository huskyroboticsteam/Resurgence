#!/usr/bin/env python3
"""Capture images from RealSense camera for dataset collection.

Controls:
    Space  - Take a photo
    1/2/3  - Switch class (1=orange_mallet, 2=rock_pick_hammer, 3=water_bottle)
    q/ESC  - Quit

Usage:
    python capture.py
    python capture.py --output captured_images
"""

import argparse
from datetime import datetime
from pathlib import Path

import cv2
import pyrealsense2 as rs
import numpy as np


CLASS_NAMES = {
    1: "orange_mallet",
    2: "rock_pick_hammer",
    3: "water_bottle",
}


def main():
    parser = argparse.ArgumentParser(description="Capture images from RealSense")
    parser.add_argument("--output", default="captured_images", help="Output directory")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    output = Path(args.output)
    for name in CLASS_NAMES.values():
        (output / name).mkdir(parents=True, exist_ok=True)

    # Start RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    profile = pipeline.start(config)

    # Enable auto-exposure settling - skip first 30 frames
    print("Waiting for auto-exposure to settle...")
    for _ in range(30):
        pipeline.wait_for_frames()

    # Try to enable auto-focus if available
    device = profile.get_device()
    for sensor in device.query_sensors():
        if sensor.supports(rs.option.enable_auto_exposure):
            sensor.set_option(rs.option.enable_auto_exposure, 1)

    current_class = 1
    counts = {name: len(list((output / name).iterdir())) for name in CLASS_NAMES.values()}

    print("=== RealSense Image Capture ===")
    print(f"Output: {output}")
    print()
    print("Controls:")
    print("  Space  - Take a photo")
    print("  1/2/3  - Switch class")
    print("  q/ESC  - Quit")
    print()
    print(f"Current class: [{current_class}] {CLASS_NAMES[current_class]}")

    cv2.namedWindow("Capture", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Capture", args.width, args.height)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            display = frame.copy()

            # Draw UI
            cls_name = CLASS_NAMES[current_class]
            cv2.putText(display, f"[{current_class}] {cls_name}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(display, f"Photos: {counts[cls_name]}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display, "Space=capture  1/2/3=switch  q=quit", (10, args.height - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            cv2.imshow("Capture", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord(' '):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = output / cls_name / f"{timestamp}.jpg"
                cv2.imwrite(str(filename), frame)
                counts[cls_name] += 1
                print(f"  Saved: {cls_name}/{filename.name} (total: {counts[cls_name]})")
            elif key in (ord('1'), ord('2'), ord('3')):
                current_class = key - ord('0')
                print(f"Switched to: [{current_class}] {CLASS_NAMES[current_class]}")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    print("\nCapture summary:")
    for name in CLASS_NAMES.values():
        print(f"  {name}: {counts[name]} photos")
    print(f"\nTo add to dataset:")
    for name in CLASS_NAMES.values():
        if counts[name] > 0:
            print(f"  python add_real_data.py --input {output / name} --class {name}")


if __name__ == "__main__":
    main()
