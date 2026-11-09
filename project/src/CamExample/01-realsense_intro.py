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
        frames = pipeline.wait_for_frames() 
        # frameset 중, 컬러 프레임만 추출
        color_frame = frames.get_color_frame() 

        # 5-2. 프레임 유효성 검사
        if not color_frame: # 컬러 프레임이 유효하지 않으면 루프 재시작
            continue

        # 5-3. 컬러 프레임을 numpy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data()) 

        # 5-4. OpenCV를 이용하여 영상 출력
        cv2.imshow('RealSense Color Stream', color_image) 

        # 5-5. 'q' 키 입력 시 루프 종료
        # 최소 1ms 동안 키 입력 대기 후, 입력된 키가 'q'인지 확인
        # & 연산 의미
            # cv2.waitKey(1) 함수는 32비트 정수 값을 반환하며, 
            # 이 값의 하위 8비트만을 확인하기 위해 & 0xFF 연산을 수행
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
finally:
    # 6. 카메라 스트리밍 종료 및 자원 해제
    pipeline.stop() # RealSense 카메라 스트리밍 종료
    cv2.destroyAllWindows() # OpenCV 창 닫기


