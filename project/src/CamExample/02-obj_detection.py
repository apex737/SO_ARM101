import pyrealsense2 as rs # Intel RealSense 카메라 제어 라이브러리
import numpy as np # 이미지 데이터를 배열 형태로 처리하는 라이브러리
import cv2 # 영상 처리 및 화면출력 라이브러리

#1. 카메라 파이프라인 설정
pipeline = rs.pipeline() # RealSense 카메라 스트리밍 파이프라인 객체 생성

# 2. 해상도, 스트림 종류, 픽셀 형식, FPS 설정 객체 생성
config = rs.config() # RealSense 카메라 스트리밍 설정 객체 생성

# 3. 컬러 영상 스트림 설정 (해상도: 640x480, 픽셀 형식: BGR8, FPS: 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 4. 설정한 스트림으로 카메라 시작
pipeline.start(config) 

# 5. 실시간 영상 수신
try:
    while True:
        # 5-1. 프레임 수신
        # 카메라에서 새 프레임이 준비될 때까지 기다렸다가 frameset 가져옴
        # frameset은 여러 개의 프레임(컬러, 깊이 등)을 포함하는 객체
        frames = pipeline.wait_for_frames() 
        # frameset 중, 컬러 프레임만 추출
        color_frame = frames.get_color_frame() 

        # 5-2. 프레임 유효성 검사
        if not color_frame: # 컬러 프레임이 유효하지 않으면 루프 재시작
            continue

        # 5-3. 컬러 프레임을 numpy 배열로 변환
        # OpenCV는 이미지 데이터를 numpy 배열 형태로 처리하기 때문
        color_image = np.asanyarray(color_frame.get_data())

        lower_bgr = np.array([0, 0, 150]) # BGR 색상 범위 하한 (어두운 빨간색)
        upper_bgr = np.array([100, 100, 255]) # BGR 색상 범위 상한 (밝은 빨간색)

        # 5-4. 마스킹
        # 입력된 이미지에서, 특정 색 범위에 포함된 픽셀만 흰색(255)으로, 나머지는 검은색(0)으로 변환하여 마스크 생성
        # cv2.inRange() 함수는 이미지에서 특정 색 범위에 해당하는 픽셀을 추출하는데 사용
        mask = cv2.inRange(color_image, lower_bgr, upper_bgr) 

        # 마스크를 적용하여 원본 이미지에서 특정 색상만 추출
        mask_img = cv2.bitwise_and(color_image, color_image, mask=mask) 

        # 5-5. OpenCV를 이용하여 영상 출력
        # cv2.imshow('RealSense Color Stream', color_image) # 원본 컬러 영상 출력
        # cv2.imshow('Color Mask BGR', mask) # 마스크 영상 출력
        cv2.imshow('Color Mask BGR Applied', mask_img) # 마스크 적용 영상 출력

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


finally:
    # 6. 카메라 스트리밍 종료 및 자원 해제
    pipeline.stop() # RealSense 카메라 스트리밍 종료
    cv2.destroyAllWindows() # OpenCV 창 닫기


