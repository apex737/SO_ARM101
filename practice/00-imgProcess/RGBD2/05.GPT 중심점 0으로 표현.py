#Intel RealSense 카메라 제어 라이브러리
import pyrealsense2 as rs
#이미지 데이터를 배열 형태로 처리하는 라이브러리
import numpy as np
#OpenCV, 영상 처리 및 화면 출력 라이브러리
import cv2


#--------------------------------------
#1. 카메라 파이프라인 설정

#카메라 스트리밍을 관리하는 파이프라인 객체 생성
pipeline = rs.pipeline()

#해상도, 스트림 종류, 픽셀 형식, FPS 설정 객체 생성
config = rs.config()

#컬러 영상 스트림 설정
#해상도 : 640x480
#픽셀 형식 : BGR, 8bit 색영역 표현
#프레임 속도 : 30FPS
config.enable_stream(
    rs.stream.color,
    640, 480,
    rs.format.bgr8,
    30
)

#설정한 스트림으로 카메라 시작
pipeline.start(config)


#--------------------------------------
#2. 실시간 영상 수신

try:
    while True:

        #활성화된 스트림에서 새로운 프레임 수신
        frames = pipeline.wait_for_frames()

        #컬러 프레임 추출
        color_frame = frames.get_color_frame()

        #컬러 프레임이 없으면 다시 수신
        if not color_frame:
            continue

        #컬러 프레임을 NumPy 배열로 변환
        color_image = np.asanyarray(
            color_frame.get_data()
        )


        #--------------------------------------
        #3. BGR 이미지를 HSV로 변환

        hsv_image = cv2.cvtColor(
            color_image,
            cv2.COLOR_BGR2HSV
        )


        #--------------------------------------
        #4. HSV 빨간색 범위 설정

        #H : 색상 (0~179)
        #S : 채도 (0~255)
        #V : 명도 (0~255)

        #빨간색 첫 번째 범위
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        #빨간색 두 번째 범위
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([179, 255, 255])


        #--------------------------------------
        #5. HSV 마스크 생성

        #지정한 색상 범위는 255(흰색)
        #나머지 영역은 0(검정색)

        mask1 = cv2.inRange(
            hsv_image,
            lower_red1,
            upper_red1
        )

        mask2 = cv2.inRange(
            hsv_image,
            lower_red2,
            upper_red2
        )

        #두 개의 빨간색 마스크 결합
        mask3 = cv2.bitwise_or(mask1, mask2)


        #마스크를 적용한 컬러 이미지
        mask_bit = cv2.bitwise_and(
            color_image,
            color_image,
            mask=mask3
        )


        #--------------------------------------
        #6. 화면 중심 좌표 설정

        #이미지 크기 추출
        height, width = color_image.shape[:2]

        #화면의 정중앙 좌표 계산
        center_x = width // 2
        center_y = height // 2

        #640x480 기준
        #center_x = 320
        #center_y = 240


        #--------------------------------------
        #7. 윤곽선(Contour) 검출

        contours, _ = cv2.findContours(
            mask3,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        #RETR_EXTERNAL : 가장 바깥쪽 윤곽선 추출
        #CHAIN_APPROX_SIMPLE : 윤곽선 좌표 단순화


        #검출된 윤곽선을 하나씩 처리
        for i in contours:

            #윤곽선 내부 면적 계산
            area = cv2.contourArea(i)

            #면적이 5000px²보다 큰 객체만 선택
            if area > 5000:

                #--------------------------------------
                #8. 윤곽선 그리기

                cv2.drawContours(
                    color_image,
                    [i],
                    -1,
                    (0, 255, 0),
                    2
                )


                #--------------------------------------
                #9. 바운딩박스 그리기

                #객체의 외접 사각형 계산
                x, y, w, h = cv2.boundingRect(i)

                #사각형 표현
                cv2.rectangle(
                    color_image,
                    (x, y),
                    (x+w, y+h),
                    (0, 0, 255),
                    2
                )


                #--------------------------------------
                #10. 객체 중심점 계산

                #기존 픽셀 좌표
                cx = x + w // 2
                cy = y + h // 2


                #--------------------------------------
                #11. 중심 좌표계 변환

                #화면 정중앙을 (0, 0)으로 변경
                new_cx = cx - center_x
                new_cy = center_y - cy

                #X축 : 오른쪽(+), 왼쪽(-)
                #Y축 : 위쪽(+), 아래쪽(-)


                #--------------------------------------
                #12. 객체 중심점 마커 표현

                cv2.drawMarker(
                    color_image,
                    (cx, cy),
                    (0, 255, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=10,
                    thickness=2
                )


                #--------------------------------------
                #13. 객체 중심점 좌표 출력

                cv2.putText(
                    color_image,
                    f"({new_cx}, {new_cy})",
                    (cx-50, cy-20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )


        #--------------------------------------
        #14. 화면 중앙에 기준 좌표 표시





        #--------------------------------------
        #15. 영상 출력

        cv2.imshow(
            "SSAFY RGBD CAMERA STREAM",
            color_image
        )

        #HSV 마스크 영상
        #cv2.imshow("Color Mask (HSV)", mask3)

        #마스크 적용 컬러 영상
        #cv2.imshow("Color Mask Bit Wise", mask_bit)


        #--------------------------------------
        #16. 키보드 입력으로 종료

        #q 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


finally:

    #--------------------------------------
    #17. 프로그램 종료 및 리소스 정리

    #카메라 스트리밍 중지
    pipeline.stop()

    #모든 OpenCV 창 닫기
    cv2.destroyAllWindows()