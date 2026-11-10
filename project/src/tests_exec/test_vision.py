"""블록 검출 시험: 위치·각도를 아는 블록을 그린 합성 사진으로 확인한다 (카메라 불필요)."""
import cv2
import numpy as np
import pytest

from kitbot_exec.vision import BlockFilter, detect_blocks, normalize_square_angle

RED = (30, 30, 200)      # BGR
GREEN = (40, 180, 40)


def canvas(seed=0, noise=6):
    rng = np.random.default_rng(seed)
    img = np.full((480, 640, 3), 120, np.uint8)  # 회색 테이블
    img = np.clip(img.astype(np.int16) + rng.integers(-noise, noise + 1, img.shape), 0, 255).astype(np.uint8)
    return img


def draw_square(img, center, side, angle_deg, color):
    pts = cv2.boxPoints((center, (side, side), angle_deg)).astype(np.int32)
    cv2.fillPoly(img, [pts], color)


@pytest.mark.parametrize("angle", [0, 10, 25, 40, 44, -20, -40])
def test_center_and_angle(angle):
    img = canvas()
    draw_square(img, (321.5, 250.0), 50, angle, RED)
    blocks = detect_blocks(img, "red")
    assert len(blocks) == 1
    b = blocks[0]
    assert abs(b.center_px[0] - 321.5) < 1.0 and abs(b.center_px[1] - 250.0) < 1.0
    assert abs(normalize_square_angle(b.angle_px_deg - angle)) < 2.0


def test_several_blocks_largest_first_and_color_separation():
    img = canvas()
    draw_square(img, (150, 150), 40, 0, RED)
    draw_square(img, (450, 300), 60, 30, RED)
    draw_square(img, (300, 400), 50, 0, GREEN)
    reds = detect_blocks(img, "red")
    assert [round(b.center_px[0]) for b in reds] == [450, 150]
    greens = detect_blocks(img, "green")
    assert len(greens) == 1 and abs(greens[0].center_px[0] - 300) < 1


def test_red_hue_wraparound():
    """빨강은 H가 0 근처와 179 근처 두 곳에 있다. 179 쪽 빨강(보라빛 빨강)도 찾아야 한다."""
    img = canvas()
    hsv_pixel = np.uint8([[[175, 200, 200]]])
    bgr = tuple(int(v) for v in cv2.cvtColor(hsv_pixel, cv2.COLOR_HSV2BGR)[0, 0])
    draw_square(img, (320, 240), 50, 0, bgr)
    assert len(detect_blocks(img, "red")) == 1


def test_rejects_noise_and_non_square():
    img = canvas()
    draw_square(img, (100, 100), 8, 0, RED)                      # 너무 작음 (잡음)
    cv2.rectangle(img, (300, 200), (450, 240), RED, -1)          # 가로로 긴 막대 (가로세로 비율 3.75)
    cv2.circle(img, (500, 400), 2, RED, -1)                      # 점
    assert detect_blocks(img, "red", BlockFilter(min_area_px=300)) == []


def test_small_hole_is_filled():
    """블록 윗면의 반사·글자 같은 작은 구멍이 있어도 한 덩어리로 본다."""
    img = canvas()
    draw_square(img, (320, 240), 60, 0, RED)
    cv2.circle(img, (320, 240), 2, (200, 200, 200), -1)
    blocks = detect_blocks(img, "red")
    assert len(blocks) == 1 and blocks[0].fill > 0.9
