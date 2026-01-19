import cv2
import numpy as np


#cap = cv2.VideoCapture(0)  
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

print("웹캠이 켜졌습니다. 'Space'를 눌러 캡처, 'ESC'를 눌러 종료.")

while True:
    ret, frame = cap.read()  # 프레임 읽기
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break
    
    cv2.imshow('Webcam', frame)  # 현재 프레임 표시

    key = cv2.waitKey(1)
    if key == 27:  # ESC 키 종료
        print("종료합니다.")
        break
    elif key == 32:  # 스페이스바로 캡처
        filename = "captured_image.jpg"
        cv2.imwrite(filename, frame)  # 프레임 저장
        print(f"이미지 캡처 및 저장: {filename}")

# 웹캠 해제 및 창 닫기
cap.release()
cv2.destroyAllWindows()