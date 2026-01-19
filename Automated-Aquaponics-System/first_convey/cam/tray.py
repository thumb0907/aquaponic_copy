import cv2
import numpy as np


cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

print("웹캠이 켜졌습니다. 'ESC'를 눌러 종료.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # 1. 그레이스케일
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2. 블러
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # 3. 엣지
    edge = cv2.Canny(blur, 150, 350)

    # 4. 컨투어
    contours, _ = cv2.findContours(
        edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 4000:
            continue

        # 다각형 근사
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        if len(approx) == 4:
            # 사각형 그리기
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)

            # 2D 자세 추정
            rect = cv2.minAreaRect(cnt)
            (cx, cy), (w, h), angle = rect

            # 각도 보정
            if w < h:
                angle += 90

            # 중심점
            cx, cy = int(cx), int(cy)

            # 방향 벡터 시각화
            rad = np.deg2rad(angle)
            length = 100
            dx = int(length * np.cos(rad))
            dy = int(length * np.sin(rad))

            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(frame, (cx, cy), (cx + dx, cy + dy), (0, 0, 255), 2)

            # 각도 텍스트 표시
            text = f"Angle: {angle:.2f} deg"
            cv2.putText(
                frame, text, (cx - 80, cy - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2
            )

    # 출력
    cv2.imshow('Rectangle Pose Estimation (2D)', frame)
    cv2.imshow('Edge', edge)

    if cv2.waitKey(1) == 27:
        print("종료합니다.")
        break

cap.release()
cv2.destroyAllWindows()
