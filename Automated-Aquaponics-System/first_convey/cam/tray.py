# tray.py
from __future__ import annotations

import time
from dataclasses import dataclass

import cv2
import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "pyserial이 설치되어 있지 않습니다. 설치:  py -m pip install pyserial"
    ) from e


@dataclass
class TrayConfig:
    # -----------------
    # Camera
    # -----------------
    cam_index: int = 1
    use_dshow: bool = True  # Windows면 True 추천

    # -----------------
    # Calibration / Undistort
    # -----------------
    calib_path: str | None = "camera_calib.npz" # None이면 왜곡보정 안 함
    alpha: float = 1.0        # 0~1 (0: 최대 crop / 1: 최대 시야 유지)
    crop: bool = True         # ROI로 crop
    use_remap: bool = True    # remap이 가장 빠르고 안정적

    # -----------------
    # Serial(OpenCR)
    # -----------------
    baud: int = 115200
    force_port: str | None = "COM20"  # None이면 자동 탐색

    # -----------------
    # Vision params
    # -----------------
    min_area: int = 4000
    canny1: int = 150
    canny2: int = 350

    # -----------------
    # Send throttling
    # -----------------
    send_interval: float = 0.05   # sec
    send_delta_deg: float = 0.3   # deg (변화량이 작으면 전송 안 함)

    # -----------------
    # UI / Debug
    # -----------------
    show_windows: bool = True
    draw_axis_len: int = 100
    print_angle_to_console: bool = False  # True면 파이썬 콘솔에 angle 찍음


class TrayAngleStreamer:
    """
    - webcam frame -> (optional) undistort -> detect largest rectangle -> angle
    - send to OpenCR: "ANG,xx.xx\\n"
    - also read OpenCR prints and show in python console as [OPENCR] ...
    """

    def __init__(self, cfg: TrayConfig):
        self.cfg = cfg

        self.ser: serial.Serial | None = None
        self.cap: cv2.VideoCapture | None = None

        # undistort related
        self.K: np.ndarray | None = None
        self.dist: np.ndarray | None = None
        self.newK: np.ndarray | None = None
        self.roi: tuple[int, int, int, int] | None = None
        self.map1 = None
        self.map2 = None

        # send throttling
        self.last_send_time = 0.0
        self.last_sent_angle: float | None = None

    # ----------------------------
    # Serial
    # ----------------------------
    def _find_opencr_port(self) -> str | None:
        ports = list(list_ports.comports())
        if not ports:
            return None

        keywords = ["opencr", "robotis", "open cr", "stm", "cdc", "serial"]
        for p in ports:
            desc = (p.description or "").lower()
            manu = (p.manufacturer or "").lower()
            hwid = (p.hwid or "").lower()
            text = f"{desc} {manu} {hwid}"
            if any(k in text for k in keywords):
                return p.device

        return ports[0].device

    def open_serial(self) -> None:
        port = self.cfg.force_port if self.cfg.force_port else self._find_opencr_port()
        if not port:
            print("[WARN] 시리얼 포트를 찾지 못했습니다. OpenCR 연결/드라이버 확인.")
            self.ser = None
            return

        try:
            self.ser = serial.Serial(port, self.cfg.baud, timeout=0.01)
            time.sleep(2.0)  # OpenCR USB 시리얼 안정화
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"[OK] Serial connected: {port} @ {self.cfg.baud}")
            print("※ 같은 COM 포트는 한 프로그램만 사용 가능 → Arduino 시리얼 모니터는 닫아야 함.")
        except Exception as e:
            print(f"[WARN] 시리얼 열기 실패: {e}")
            self.ser = None

    def close_serial(self) -> None:
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _read_opencr_lines(self) -> None:
        if self.ser is None:
            return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    print("[OPENCR]", line)
        except Exception as e:
            print("[WARN] Serial read failed:", e)

    def _send_angle(self, angle_deg: float) -> None:
        if self.ser is None:
            return
        msg = f"ANG,{angle_deg:.2f}\n"
        try:
            self.ser.write(msg.encode("utf-8"))
        except Exception as e:
            print("[WARN] Serial write failed:", e)

    # ----------------------------
    # Camera + Calibration
    # ----------------------------
    def open_camera(self) -> None:
        api = cv2.CAP_DSHOW if self.cfg.use_dshow else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(self.cfg.cam_index, api)
        if not self.cap.isOpened():
            raise RuntimeError("웹캠을 열 수 없습니다. cam_index/연결 상태 확인 필요.")
        print("[OK] Camera opened.")

        # 캘리브레이션 로드/맵 생성(옵션)
        if self.cfg.calib_path:
            self._load_calibration_and_prepare_undistort()

    def close_camera(self) -> None:
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None

    def _load_calibration_and_prepare_undistort(self) -> None:
        assert self.cap is not None

        data = np.load(self.cfg.calib_path)
        keys = list(data.files)

        def pick_key(candidates):
            for k in candidates:
                if k in data:
                    return data[k]
            return None

        K = pick_key(["K", "camera_matrix", "mtx", "cameraMatrix", "intrinsic", "intrinsics"])
        dist = pick_key(["dist", "dist_coeff", "distCoeffs", "dist_coeffs", "distortion", "D"])

        if K is None or dist is None:
            raise RuntimeError(
                f"calib 파일에서 K/dist 키를 찾지 못했습니다.\n"
                f"- 파일: {self.cfg.calib_path}\n"
                f"- keys: {keys}\n"
                f"tip) npz에 저장된 키 이름에 맞춰 후보 목록을 추가해야 합니다."
            )

        self.K = np.array(K, dtype=np.float64)
        self.dist = np.array(dist, dtype=np.float64).reshape(-1, 1)

        # 프레임 크기 확보
        ret, frame0 = self.cap.read()
        if not ret:
            raise RuntimeError("캘리브레이션 준비 중 첫 프레임을 읽지 못했습니다.")
        h, w = frame0.shape[:2]

        self.newK, self.roi = cv2.getOptimalNewCameraMatrix(
            self.K, self.dist, (w, h), self.cfg.alpha, (w, h)
        )

        if self.cfg.use_remap:
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.K, self.dist, None, self.newK, (w, h), cv2.CV_16SC2
            )

        print(f"[OK] Calibration loaded: {self.cfg.calib_path}")
        print(f"     keys={keys}")

    def _apply_undistort(self, frame: np.ndarray) -> np.ndarray:
        if self.K is None or self.dist is None or self.newK is None:
            return frame

        if self.cfg.use_remap and (self.map1 is not None) and (self.map2 is not None):
            frame = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
        else:
            frame = cv2.undistort(frame, self.K, self.dist, None, self.newK)

        if self.cfg.crop and self.roi is not None:
            x, y, w, h = self.roi
            frame = frame[y:y + h, x:x + w]

        return frame

    # ----------------------------
    # Vision
    # ----------------------------
    def _detect_best_rectangle(self, frame_bgr: np.ndarray):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edge = cv2.Canny(blur, self.cfg.canny1, self.cfg.canny2)

        contours, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.cfg.min_area:
                continue

            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            if len(approx) == 4 and area > best_area:
                best = (cnt, approx)
                best_area = area

        if best is None:
            return None

        cnt, approx = best
        (cx, cy), (w, h), angle = cv2.minAreaRect(cnt)

        if w < h:
            angle += 90

        return float(angle), int(cx), int(cy), approx, edge

    # ----------------------------
    # Step / Run
    # ----------------------------
    def step(self) -> bool:
        """
        1 프레임 처리(외부 루프에서 호출 가능)
        return False if frame read failed
        """
        if self.cap is None:
            raise RuntimeError("Camera not opened. call open_camera() first")

        ret, frame = self.cap.read()
        if not ret:
            return False

        # OpenCR 로그
        self._read_opencr_lines()

        # undistort
        frame = self._apply_undistort(frame)

        result = self._detect_best_rectangle(frame)

        if result is not None:
            angle, cx, cy, approx, edge = result

            if self.cfg.print_angle_to_console:
                print(f"[ANGLE] {angle:.2f}")

            # 전송 조건
            now = time.time()
            can_send_by_time = (now - self.last_send_time) >= self.cfg.send_interval
            can_send_by_delta = (
                self.last_sent_angle is None or abs(angle - self.last_sent_angle) >= self.cfg.send_delta_deg
            )

            if self.ser is not None and can_send_by_time and can_send_by_delta:
                self._send_angle(angle)
                self.last_send_time = now
                self.last_sent_angle = angle

            # 화면 표시
            if self.cfg.show_windows:
                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)

                rad = np.deg2rad(angle)
                dx = int(self.cfg.draw_axis_len * np.cos(rad))
                dy = int(self.cfg.draw_axis_len * np.sin(rad))

                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(frame, (cx, cy), (cx + dx, cy + dy), (0, 0, 255), 2)

                cv2.putText(
                    frame, f"Angle: {angle:.2f} deg",
                    (cx - 80, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2
                )

                cv2.imshow("Rectangle Pose Estimation (2D)", frame)
                cv2.imshow("Edge", edge)
        else:
            if self.cfg.show_windows:
                cv2.imshow("Rectangle Pose Estimation (2D)", frame)

        return True

    def run(self) -> None:
        """
        내부 루프 실행 (ESC 종료)
        """
        if self.ser is None:
            self.open_serial()
        if self.cap is None:
            self.open_camera()

        print("[RUN] ESC to exit.")
        try:
            while True:
                ok = self.step()
                if not ok:
                    print("[ERROR] 프레임을 읽을 수 없습니다.")
                    break

                if self.cfg.show_windows:
                    key = cv2.waitKey(1)
                    if key == 27:
                        break
                else:
                    time.sleep(0.001)
        finally:
            self.close()

    def close(self) -> None:
        self.close_camera()
        self.close_serial()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
