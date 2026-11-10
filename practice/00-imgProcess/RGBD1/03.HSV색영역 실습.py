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
        #OpneCV에서 처리할 수 있는 이미지 배열 생성
        color_image = np.asanyarray(color_frame.get_data())


        #BGR 컬러 이미지를 HSV색 영역으로 변환
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)


        
        #---HSV 빨간색----H:색상 값, S:채도값(0~255, 0~100%) V:명도 값(0~255, 0~100%)
        #OpenCV에서는 H 각도 값을 /2 해서 표현 합니다.
        lower_red1 = np.array([0, 120, 70]) #색상의 최소값(어두운 빨강
        upper_red1 = np.array([10, 255, 255]) #색상의 최대값(밝은 빨강)

        lower_red2 = np.array([170, 120, 70]) #색상의 최소값(어두운 빨강)
        upper_red2 = np.array([180, 255, 255]) #색상의 최대값(밝은 빨강)


        #최소값 최대값 마스크 처리
        #입력된 이미지에서, 특정 색 범위에 포함되는 픽셀만 255 값으로 출력(화이트)
        #나머지는 0 값 (블랙으로 표현) 으로 만든 2진 이미지(mask)
        #inRange 조건 생성(마스크만드는 조건)
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        #마스크 연산 후 해당 색상만 출력
        mask3 =cv2.bitwise_or(mask1, mask2)


        
        #마스크 연산
        #mask 에서 만든 레이어를 이용해서 255인 영역만 원본 이미지로 유지
        #0인 부분은 전부 검정으로 제거, 선택된 부분만 실제 이미지로 유지
        #bitwise_and 조건 적용 (이미지에서 추출)
        mask_bit = cv2.bitwise_and(color_image, color_image, mask = mask3)
        


        
        
        #3. 영상출력
        cv2.imshow("SSAFY RGBD CAMERA STREAM", color_image) #원본 영상
        cv2.imshow("Color Mask (HSV)", mask3) #인식된 부분만 255로 표현
        cv2.imshow("Color Mask Bit Wise", mask_bit) #인식된 부분만 실제표현


        #4. 키보드 입력으로 종료 처리

        #waitKey(1): 최소 1ms 동안 키 입력 대기 및 창 이벤트 처리
        #ord('q'): 문자'q'를 정수 코드 113로 변환
        #&연산, 0xFF: 키 코드의 하위 8비트만 추출
        #'q'키가 입력되면 반복문 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    #5. 프로그램 종료 및 리소스 정리
    pipeline.stop()             #카메라 스트리밍 중지
    cv2.destroyAllWindows()     #모든 OpenCV창 닫기
