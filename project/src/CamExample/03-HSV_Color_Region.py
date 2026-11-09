import argparse

import pyrealsense2 as rs  # Intel RealSense 카메라 제어
import numpy as np
import cv2


# OpenCV HSV 범위: H=0~179, S=0~255, V=0~255
# 빨강은 H의 양 끝에 있으므로 두 범위를 합친다.
HSV_RANGES = {
    "red": [((0, 120, 70), (10, 255, 255)),
            ((170, 120, 70), (179, 255, 255))],
    "blue": [((90, 120, 50), (130, 255, 255))],
    "green": [((35, 120, 70), (85, 255, 255))],
}


def create_mask(hsv_image, color):
    """선택한 색상 범위에 포함된 픽셀만 255인 마스크를 만든다."""
    mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
    for lower, upper in HSV_RANGES[color]:
        color_mask = cv2.inRange(
            hsv_image,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        mask = cv2.bitwise_or(mask, color_mask)
    return mask


def main(color):
    """red, blue, green 중 선택한 색만 실시간으로 표시한다."""
    if color not in HSV_RANGES:
        raise ValueError("color는 red, blue, green 중 하나여야 합니다.")

    # 카메라 스트림 설정 및 시작
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            # 색상 판별은 HSV에서, 결과 표시는 원본 BGR에서 수행
            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            mask = create_mask(hsv_image, color)
            mask_img = cv2.bitwise_and(color_image, color_image, mask=mask)

            cv2.imshow(f"HSV Color Region - {color}", mask_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="HSV로 선택한 색상만 검출합니다.")
    parser.add_argument("color", choices=HSV_RANGES, help="검출할 색상")
    args = parser.parse_args()
    main(args.color)


