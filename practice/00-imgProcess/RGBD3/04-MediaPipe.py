# 기준 버전: MediaPipe 1.2.2
# 사용 API: MediaPipe Tasks / FaceLandmarker
#
# 실습 목표
# 1. RealSense 컬러 영상에서 얼굴을 인식합니다.
# 2. 얼굴 랜드마크를 연결해 얼굴 메시(스켈레톤)를 표시합니다.
#
# 준비: 이 파일을 실행할 Python 환경에서 설치합니다.
# python -m pip install mediapipe==1.2.2
# 기존 RealSense 예제가 실행된다면 아래 패키지는 이미 있을 수 있습니다.
# python -m pip install pyrealsense2 numpy opencv-python
#
# 최초 실행 시 공식 얼굴 인식 모델을 같은 폴더에 자동 다운로드합니다.
# 파일명: face_landmarker.task
# 최초 다운로드에는 인터넷 연결과 이 폴더의 쓰기 권한이 필요합니다.
# 모델 파일이 있으면 다음 실행부터 다시 다운로드하지 않습니다.
# 카메라를 사용하는 다른 Python 예제나 RealSense Viewer는 종료해주세요.
# 종료: 영상 창에서 Q 키 또는 창의 닫기 버튼
#
# 공식 문서
# https://developers.google.com/edge/mediapipe/solutions/vision/face_landmarker?tab=Python

import time
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
from mediapipe.tasks.python.vision import drawing_utils


# 1. 얼굴 인식 모델 파일 준비
model_path = Path(__file__).resolve().parent / "face_landmarker.task"
model_url = (
    "https://storage.googleapis.com/mediapipe-models/"
    "face_landmarker/face_landmarker/float16/latest/face_landmarker.task"
)

if not model_path.is_file():
    print("얼굴 인식 모델을 다운로드합니다. 최초 실행 시 한 번만 진행됩니다.")
    try:
        with urlopen(model_url, timeout=60) as response:
            model_data = response.read()
        model_path.write_bytes(model_data)
    except (URLError, OSError) as error:
        raise RuntimeError(
            "모델 다운로드/저장에 실패했습니다. 인터넷 연결과 폴더 쓰기 권한을 "
            "확인하거나, 공식 모델을 face_landmarker.task 이름으로 이 파일 옆에 "
            "저장하세요."
        ) from error
    print("모델 다운로드 완료")


# 2. MediaPipe 얼굴 인식 설정
options = mp.tasks.vision.FaceLandmarkerOptions(
    base_options=mp.tasks.BaseOptions(
        model_asset_buffer=model_path.read_bytes()
    ),
    running_mode=mp.tasks.vision.RunningMode.VIDEO,
    num_faces=1,
    min_face_detection_confidence=0.5,
    min_face_presence_confidence=0.5,
    min_tracking_confidence=0.5
)

# 촘촘한 삼각형 메시와 얼굴 주요 윤곽선을 서로 다른 색으로 표시합니다.
face_connections = mp.tasks.vision.FaceLandmarksConnections
mesh_style = drawing_utils.DrawingSpec(
    color=(100, 180, 100),
    thickness=1,
    circle_radius=1
)
contour_style = drawing_utils.DrawingSpec(
    color=(0, 255, 255),
    thickness=2,
    circle_radius=1
)


# 3. RealSense 컬러 카메라 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

window_name = "SSAFY MediaPipe - Face Mesh"
previous_timestamp_ms = -1
pipeline_started = False

with mp.tasks.vision.FaceLandmarker.create_from_options(options) as landmarker:
    try:
        pipeline.start(config)
        pipeline_started = True
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data()).copy()
            height, width = color_image.shape[:2]

            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=rgb_image
            )

            timestamp_ms = time.monotonic_ns() // 1_000_000
            timestamp_ms = max(timestamp_ms, previous_timestamp_ms + 1)
            previous_timestamp_ms = timestamp_ms

            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            for face_landmarks in result.face_landmarks:
                # 얼굴 전체 삼각 메시
                drawing_utils.draw_landmarks(
                    image=color_image,
                    landmark_list=face_landmarks,
                    connections=face_connections.FACE_LANDMARKS_TESSELATION,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=mesh_style
                )
                # 눈, 눈썹, 코, 입, 얼굴 외곽 등 주요 윤곽선
                drawing_utils.draw_landmarks(
                    image=color_image,
                    landmark_list=face_landmarks,
                    connections=face_connections.FACE_LANDMARKS_CONTOURS,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=contour_style
                )

            face_count = len(result.face_landmarks)
            cv2.rectangle(color_image, (10, 10), (300, 50), (0, 0, 0), -1)
            cv2.putText(
                color_image,
                f"Faces: {face_count} | Q: Quit",
                (20, 37),
                cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.65,
                color=(0, 255, 255),
                thickness=2
            )
            cv2.imshow(window_name, color_image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break

    finally:
        if pipeline_started:
            pipeline.stop()
        cv2.destroyAllWindows()
