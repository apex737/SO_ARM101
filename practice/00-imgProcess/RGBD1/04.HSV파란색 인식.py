#Intel RealSense 카메라 제어 라이브러리
import pyrealsense2 as rs
#이미지 데이터를 배열 형태로 처리하는 라이브러리
import numpy as np
#OpenCV, 영상 처리 및 화면 출력 라이브러리
import cv2

#1. 카메라 파이프라인 설정
#카메라 스트리밍을 관리하는 파이프라인 객체 생성
pipeline = rs.pipeline()
#해상도, 스트림 종류, 픽셀 형식, FPS 설정 객체 생성
config = rs.config()

#컬러 영상 스트림 설정
#해상도 : 640x480
#픽셀 형식 : BGR, 8bit 색영역 표현
#프레임 속도: 30FPS (초당 30프레임)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#설정한 스트림으로 카메라 시작
pipeline.start(config)


#--------------------------------------
#2. 실시간 영상 수신
try:
    while True:
        #활성화된 스트림에서 실시간 새로운 프레임 세트를 수신
        frames = pipeline.wait_for_frames()

        #수신한 프레임 세트에서 컬러 프레임만 추출
        color_frame = frames.get_color_frame()

        #컬러 프레임이 없으면 현재 반복을 건너뛰고 다시 수신
        if not color_frame:
            continue

        #컬러 프레임 데이터를 NumPy 배열로 변환
        #OpenCV에서 처리할 수 있는 이미지 배열 생성
        color_image = np.asanyarray(color_frame.get_data())


        #BGR 컬러 이미지를 HSV 색 영역으로 변환
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)


        #--------------------------------------
        #HSV 파란색 영역 설정
        #H: 색상(0~179)
        #S: 채도(0~255)
        #V: 명도(0~255)
        #OpenCV에서는 H 각도 값을 /2 해서 표현합니다.

        lower_blue = np.array([90, 120, 50])  #파란색 최소값
        upper_blue = np.array([130, 255, 255]) #파란색 최대값


        #--------------------------------------
        #파란색 마스크 생성
        #특정 색상 범위에 포함되는 픽셀만 255(흰색)
        #나머지 픽셀은 0(검정색)으로 표현

        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)


        #--------------------------------------
        #마스크 연산
        #마스크에서 255인 영역만 원본 이미지로 유지
        #0인 부분은 검정색으로 제거

        mask_bit = cv2.bitwise_and(
            color_image,
            color_image,
            mask=mask
        )


        #--------------------------------------
        #3. 영상 출력

        #원본 영상
        cv2.imshow("SSAFY RGBD CAMERA STREAM", color_image)

        #파란색 영역만 흰색으로 표시
        cv2.imshow("Blue Mask (HSV)", mask)

        #파란색 영역만 실제 색상으로 표시
        cv2.imshow("Blue Mask Bit Wise", mask_bit)


        #--------------------------------------
        #4. 키보드 입력으로 종료 처리

        #q 키가 입력되면 반복문 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    #5. 프로그램 종료 및 리소스 정리
    pipeline.stop()             #카메라 스트리밍 중지
    cv2.destroyAllWindows()     #모든 OpenCV 창 닫기