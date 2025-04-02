import cv2

def main():
    # 1. 기본 카메라(인덱스 0)로 VideoCapture 객체 생성
    cap = cv2.VideoCapture(4)

    # 2. 카메라가 정상적으로 열렸는지 확인
    if not cap.isOpened():
        print("오류: 카메라를 열 수 없습니다.")
        return

    # 3. 무한 루프를 돌며 프레임 읽기 및 화면 출력
    while True:
        ret, frame = cap.read()  # 프레임 읽기
        if not ret:
            print("오류: 프레임을 읽을 수 없습니다.")
            break

        # 4. 프레임을 창에 표시
        cv2.imshow('Camera Feed', frame)

        # 5. 'q' 키를 누르면 루프 종료 (키 입력 대기 1ms)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 6. 카메라 객체 해제 및 모든 창 닫기
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()




# export PATH=/usr/bin:$PATH
# export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH


# import cv2

# cap = cv2.VideoCapture("/dev/video4")
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPG로 포맷 설정

# if not cap.isOpened():
#     print("Failed to open /dev/video4")
# else:
#     ret, frame = cap.read()
#     if ret:
#         print("Successfully read a frame from /dev/video4")
#         cv2.imshow("Camera 4", frame)
#         cv2.waitKey(0)
#     else:
#         print("Failed to read frame")
# cap.release()
# cv2.destroyAllWindows()

# import cv2

# cap = cv2.VideoCapture("/dev/video6")
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPG로 포맷 설정

# if not cap.isOpened():
#     print("Failed to open /dev/video6")
# else:
#     ret, frame = cap.read()
#     if ret:
#         print("Successfully read a frame from /dev/video6")
#         cv2.imshow("Camera 6", frame)
#         cv2.waitKey(0)
#     else:
#         print("Failed to read frame")
# cap.release()
# cv2.destroyAllWindows()