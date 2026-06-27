# undistort.py
import os
import time
from dataclasses import dataclass

import cv2
import numpy as np

__all__ = ["UD", "UDConf"]


@dataclass
class UDConf:
    cam: int = 0
    calib: str = "camera_calib.npz"
    alpha: float = 1.0
    crop: bool = True
    use_remap: bool = True
    backend: int = cv2.CAP_DSHOW

    warmup_frames: int = 20
    warmup_sleep: float = 0.05


class UD:
    """
    사용법:
        from undistort import UD, UDConf
        ud = UD(UDConf(cam=0, calib="camera_calib.npz"))
        out = ud.apply(frame)   # 한 프레임 왜곡 보정
        ud.run()                # 테스트용 실시간 창
    """

    def __init__(self, conf: UDConf):
        self.c = conf

        # calib 경로를 undistort.py 위치 기준으로도 찾을 수 있게 처리
        calib_path = conf.calib
        if not os.path.isabs(calib_path):
            here = os.path.dirname(os.path.abspath(__file__))
            calib_path = os.path.join(here, calib_path)

        data = np.load(calib_path)
        self.K = data["camera_matrix"]
        self.dist = data["dist_coeffs"]

        self.newK = None
        self.roi = None
        self.map1 = None
        self.map2 = None
        self._ready = False

    def _prepare(self, frame_shape):
        h, w = frame_shape[:2]
        self.newK, self.roi = cv2.getOptimalNewCameraMatrix(
            self.K, self.dist, (w, h), self.c.alpha, (w, h)
        )

        if self.c.use_remap:
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.K, self.dist, None, self.newK, (w, h), cv2.CV_16SC2
            )

        self._ready = True

    def apply(self, frame):
        """프레임 1장 왜곡 보정해서 반환"""
        if not self._ready:
            self._prepare(frame.shape)

        if self.c.use_remap and self.map1 is not None:
            out = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
        else:
            out = cv2.undistort(frame, self.K, self.dist, None, self.newK)

        if self.c.crop and self.roi is not None:
            x, y, rw, rh = self.roi
            if rw > 0 and rh > 0:
                out = out[y:y + rh, x:x + rw]

        return out

    def run(self, show_orig=True, save_key=True,
            win_orig="Original", win_und="Undistorted+ROI"):
        """테스트용: 실시간 창으로 확인"""
        cap = cv2.VideoCapture(self.c.cam, self.c.backend)
        
        if not cap.isOpened():
            raise RuntimeError(f"카메라 오픈 실패 (index={self.c.cam})")

        # warmup
        frame0 = None
        for _ in range(self.c.warmup_frames):
            ret, frame0 = cap.read()
            if ret and frame0 is not None:
                break
            time.sleep(self.c.warmup_sleep)

        if frame0 is None:
            cap.release()
            raise RuntimeError("프레임 읽기 실패(카메라 점유/인덱스 확인)")

        self._prepare(frame0.shape)

        if show_orig:
            cv2.namedWindow(win_orig, cv2.WINDOW_NORMAL)
        cv2.namedWindow(win_und, cv2.WINDOW_NORMAL)

        print("실시간 왜곡 보정 실행 중")
        print(" - ESC : 종료")
        if save_key:
            print(" - S   : 현재 보정 프레임 저장")

        try:
            while True:
                ret, frame = cap.read()
                if not ret or frame is None:
                    time.sleep(0.05)
                    continue

                out = self.apply(frame)

                if show_orig:
                    cv2.imshow(win_orig, frame)
                cv2.imshow(win_und, out)

                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    break
                if save_key and key in (ord("s"), ord("S")):
                    ts = time.strftime("%Y%m%d_%H%M%S")
                    name = f"und_{ts}.jpg"
                    cv2.imwrite(name, out)
                    print(f"[저장 완료] {name}")
        finally:
            cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    #UD(UDConf(cam=1)).run(show_orig=True, save_key=True)
    UD(UDConf(cam=0)).run(show_orig=True, save_key=True)