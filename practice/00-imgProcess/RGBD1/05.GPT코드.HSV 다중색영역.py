#Intel RealSense 카메라 제어 라이브러리
import pyrealsense2 as rs
#이미지 데이터를 배열 형태로 처리하는 라이브러리
import numpy as np
#OpenCV, 영상 처리 및 화면 출력 라이브러리
import cv2

#1. 카메라 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()

#컬러 영상 스트림 설정
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#설정한 스트림으로 카메라 시작
pipeline.start(config)


#--------------------------------------
#2. 실시간 영상 수신
try:
    while True:
        #실시간 프레임 수신
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        #컬러 프레임을 NumPy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data())

        #BGR 이미지를 HSV 색 영역으로 변환
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)


        #--------------------------------------
        #3. HSV 색상 범위 설정

        #빨간색 (Red)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([179, 255, 255])

        #파란색 (Blue)
        lower_blue = np.array([100, 120, 70])
        upper_blue = np.array([130, 255, 255])

        #초록색 (Green)
        lower_green = np.array([35, 100, 70])
        upper_green = np.array([85, 255, 255])

        #보라색 (Purple)
        lower_purple = np.array([135, 80, 70])
        upper_purple = np.array([165, 255, 255])


        #--------------------------------------
        #4. 색상별 마스크 생성

        #빨간색 마스크
        mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        #파란색 마스크
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

        #초록색 마스크
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

        #보라색 마스크
        mask_purple = cv2.inRange(hsv_image, lower_purple, upper_purple)


        #--------------------------------------
        #5. 모든 색상 마스크 합치기

        mask1 = cv2.bitwise_or(mask_red, mask_blue)
        mask2 = cv2.bitwise_or(mask_green, mask_purple)

        #최종 마스크
        mask_all = cv2.bitwise_or(mask1, mask2)


        #--------------------------------------
        #6. 마스크 연산
        #빨강, 파랑, 초록, 보라 영역만 원본 색상으로 출력
        #나머지 영역은 검정색으로 처리

        mask_bit = cv2.bitwise_and(
            color_image,
            color_image,
            mask=mask_all
        )


        #--------------------------------------
        #7. 영상 출력

        #원본 영상
        cv2.imshow("SSAFY RGBD CAMERA STREAM", color_image)

        #4가지 색상 통합 마스크
        cv2.imshow("Color Mask (HSV)", mask_all)

        #4가지 색상만 실제 색상으로 출력
        cv2.imshow("Color Mask Bit Wise", mask_bit)


        #--------------------------------------
        #8. 키보드 입력으로 종료 처리
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    #9. 프로그램 종료 및 리소스 정리
    pipeline.stop()
    cv2.destroyAllWindows()