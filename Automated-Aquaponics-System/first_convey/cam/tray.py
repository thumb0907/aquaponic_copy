import cv2
import numpy as np
import time

# pyserial 미설치 시 안내
try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    print("[ERROR] pyserial이 설치되어 있지 않습니다.")
    print("CMD/PowerShell에서 아래를 실행하세요:")
    print("  py -m pip install pyserial")
    raise


# =========================
# 설정
# =========================
CAM_INDEX = 1                  # 웹캠 번호
USE_DSHOW = True               # Windows면 True 추천
BAUD = 115200

# 자동 포트 탐색이 싫으면 여기에 직접 입력: 예) "COM5" 또는 "/dev/ttyACM0"
FORCE_PORT = "COM20"              # None이면 자동 탐색 시도

SEND_INTERVAL = 0.05           # 50ms (너무 빠르면 버퍼 밀릴 수 있음)
SEND_DELTA_DEG = 0.3           # 각도 변화가 이 이상일 때만 전송(노이즈 줄임)

MIN_AREA = 4000                # 사각형 최소 면적
CANNY1, CANNY2 = 150, 350


def find_opencr_port():
    """
    OpenCR로 보이는 포트를 대충 자동 추정.
    - Windows: description/manufacturer에 'OpenCR', 'ROBOTIS', 'STM' 등 힌트
    - Linux: /dev/ttyACM*, /dev/ttyUSB* 중 description 힌트
    """
    ports = list(list_ports.comports())
    if not ports:
        return None

    # 1) description 기반 우선순위
    keywords = ["opencr", "robotis", "open cr", "stm", "cdc", "serial"]
    for p in ports:
        desc = (p.description or "").lower()
        manu = (p.manufacturer or "").lower()
        hwid = (p.hwid or "").lower()
        text = f"{desc} {manu} {hwid}"
        if any(k in text for k in keywords):
            return p.device

    # 2) 그래도 없으면 첫 번째 포트(최후)
    return ports[0].device


def open_serial():
    port = FORCE_PORT if FORCE_PORT else find_opencr_port()
    if not port:
        print("[WARN] 시리얼 포트를 찾지 못했습니다. (OpenCR 연결 확인)")
        return None

    try:
        ser = serial.Serial(port, BAUD, timeout=0.01)
        time.sleep(2.0)  # OpenCR 리셋/안정화 대기
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"[OK] Serial connected: {port} @ {BAUD}")
        print("※ Arduino IDE 시리얼 모니터는 닫아야 합니다(포트 충돌).")
        return ser
    except Exception as e:
        print(f"[WARN] 시리얼 열기 실패: {e}")
        return None


def read_opencr_lines(ser):
    """OpenCR이 보내는 로그를 파이썬 콘솔로 출력(논블로킹)."""
    if ser is None:
        return
    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print("[OPENCR]", line)
    except Exception as e:
        print("[WARN] Serial read failed:", e)


def send_angle(ser, angle_deg):
    """OpenCR로 각도 전송."""
    if ser is None:
        return
    msg = f"ANG,{angle_deg:.2f}\n"
    try:
        ser.write(msg.encode("utf-8"))
    except Exception as e:
        print("[WARN] Serial write failed:", e)


def main():
    # 시리얼 연결
    ser = open_serial()

    # 웹캠 연결
    api = cv2.CAP_DSHOW if USE_DSHOW else cv2.CAP_ANY
    cap = cv2.VideoCapture(CAM_INDEX, api)

    if not cap.isOpened():
        print("[ERROR] 웹캠을 열 수 없습니다.")
        if ser:
            ser.close()
        return

    print("[OK] 웹캠이 켜졌습니다. 'ESC'를 눌러 종료.")

    last_send_time = 0.0
    last_sent_angle = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] 프레임을 읽을 수 없습니다.")
            break

        # OpenCR 로그 읽기(항상)
        read_opencr_lines(ser)

        # 1) 그레이스케일
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2) 블러
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # 3) 엣지
        edge = cv2.Canny(blur, CANNY1, CANNY2)

        # 4) 컨투어
        contours, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0

        # 가장 큰 사각형 하나만 선택(전송 안정)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue

            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            if len(approx) == 4 and area > best_area:
                best = (cnt, approx)
                best_area = area

        if best is not None:
            cnt, approx = best

            # 사각형 그리기
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)

            # 2D 자세 추정
            rect = cv2.minAreaRect(cnt)
            (cx, cy), (w, h), angle = rect

            # 각도 보정
            if w < h:
                angle += 90

            cx, cy = int(cx), int(cy)

            # 방향 벡터 시각화
            rad = np.deg2rad(angle)
            length = 100
            dx = int(length * np.cos(rad))
            dy = int(length * np.sin(rad))

            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(frame, (cx, cy), (cx + dx, cy + dy), (0, 0, 255), 2)

            # 각도 텍스트
            cv2.putText(
                frame, f"Angle: {angle:.2f} deg",
                (cx - 80, cy - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2
            )

            # =========================
            # OpenCR로 전송(주기/변화량 제한)
            # =========================
            now = time.time()
            can_send_by_time = (now - last_send_time) >= SEND_INTERVAL
            can_send_by_delta = (
                last_sent_angle is None or abs(angle - last_sent_angle) >= SEND_DELTA_DEG
            )

            if ser is not None and can_send_by_time and can_send_by_delta:
                send_angle(ser, angle)
                last_send_time = now
                last_sent_angle = angle

        # 출력
        cv2.imshow("Rectangle Pose Estimation (2D)", frame)
        cv2.imshow("Edge", edge)

        key = cv2.waitKey(1)
        if key == 27:
            print("종료합니다.")
            break

    cap.release()
    cv2.destroyAllWindows()
    if ser is not None:
        ser.close()


if __name__ == "__main__":
    main()
