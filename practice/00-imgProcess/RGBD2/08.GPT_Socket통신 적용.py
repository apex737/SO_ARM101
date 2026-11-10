#Intel RealSense 카메라 제어 라이브러리
import pyrealsense2 as rs
#이미지 데이터를 배열 형태로 처리하는 라이브러리
import numpy as np
#OpenCV, 영상 처리 및 화면 출력 라이브러리
import cv2
#Socket TCP/IP 통신 라이브러리
import socket
#시간 간격 제어 라이브러리
import time


#--------------------------------------
#1. Socket Client 설정

HOST = '127.0.0.1'
PORT = 20000

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#서버 접속
client_socket.connect((HOST, PORT))
print(f"서버 접속 완료. 서버 주소는 {HOST} : {PORT}")

#마지막 데이터 전송 시간
last_send_time = time.monotonic()


#--------------------------------------
#2. 카메라 파이프라인 설정

pipeline = rs.pipeline()
config = rs.config()

#컬러 영상 스트림 설정
config.enable_stream(
    rs.stream.color, 640, 480, rs.format.bgr8, 30
)

#카메라 시작
try:
    pipeline.start(config)
except Exception:
    client_socket.close()
    raise


#--------------------------------------
#3. 실시간 영상 수신

try:
    while True:

        #카메라 프레임 수신
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        #NumPy 배열 변환
        color_image = np.asanyarray(color_frame.get_data())


        #--------------------------------------
        #4. HSV 색상 변환

        hsv_image = cv2.cvtColor(
            color_image, cv2.COLOR_BGR2HSV
        )

        #빨간색 HSV 범위 설정
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([179, 255, 255])


        #--------------------------------------
        #5. 마스크 생성

        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        mask3 = cv2.bitwise_or(mask1, mask2)

        #마스크 적용 이미지
        mask_bit = cv2.bitwise_and(
            color_image, color_image, mask=mask3
        )


        #--------------------------------------
        #6. 윤곽선 검출

        contours, _ = cv2.findContours(
            mask3,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        #객체 인식 여부
        detected = False

        #전송할 객체 면적
        pixel_area = 0


        #검출된 윤곽선 처리
        for i in contours:

            #윤곽선 내부 면적 계산
            area = cv2.contourArea(i)

            #5000px²보다 큰 객체만 인식
            if area > 5000:

                detected = True

                #여러 객체 중 가장 큰 객체의 면적 저장
                if area > pixel_area:
                    pixel_area = int(area)

                #윤곽선 그리기
                cv2.drawContours(
                    color_image, [i], -1, (0, 255, 0), 2
                )

                #바운딩박스 계산
                x, y, w, h = cv2.boundingRect(i)

                #바운딩박스 그리기
                cv2.rectangle(
                    color_image,
                    (x, y),
                    (x+w, y+h),
                    (0, 0, 255),
                    2
                )

                #중심점 계산
                cx = x + w // 2
                cy = y + h // 2

                #중심점 마커 표현
                cv2.drawMarker(
                    color_image,
                    (cx, cy),
                    (0, 255, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=10,
                    thickness=2
                )

                #중심점 좌표 출력
                cv2.putText(
                    color_image,
                    f"{cx, cy}px",
                    (cx-50, cy-20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

                #객체 인식 메시지
                cv2.putText(
                    color_image,
                    "Vision Check OK",
                    (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 255, 0),
                    2
                )

                #객체 Pixel Area 출력
                cv2.putText(
                    color_image,
                    f"Pixel Area : {int(area)}",
                    (cx-50, cy-120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (120, 255, 120),
                    2
                )


        #--------------------------------------
        #7. Socket 데이터 전송

        #현재 시간 확인
        current_time = time.monotonic()

        #0.5초가 지났는지 확인
        if current_time - last_send_time >= 0.5:

            #객체가 인식되었을 때만 전송
            if detected:

                #Pixel Area를 문자열로 변환
                message = str(pixel_area) + "\n"

                #서버로 데이터 전송
                client_socket.sendall(
                    message.encode('utf-8')
                )

                print(f"서버 전송 : {pixel_area} px")

            #마지막 전송 시간 갱신
            last_send_time = current_time


        #--------------------------------------
        #8. 영상 출력

        cv2.putText(
            color_image,
            "SSAFY Vision",
            (20, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.imshow(
            "SSAFY RGBD CAMERA STREAM",
            color_image
        )

        #cv2.imshow("Color Mask (HSV)", mask3)
        #cv2.imshow("Color Mask Bit Wise", mask_bit)


        #--------------------------------------
        #9. 키보드 종료

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


except KeyboardInterrupt:
    print("강제로 종료되었습니다.")

except (ConnectionError, OSError) as e:
    print(f"Socket 통신 오류 : {e}")

finally:

    #카메라 종료
    pipeline.stop()

    #OpenCV 창 종료
    cv2.destroyAllWindows()

    #Socket 연결 종료
    client_socket.close()

    print("카메라 및 Socket 연결 종료")