import cv2
import numpy as np
import time

# =========================
# 설정
# =========================
CAM_INDEX = 0
CALIB_NPZ = "camera_calib.npz"

# alpha:
#  - 1.0 : 최대한 화면 보존(검은 테두리 생길 수 있음) -> ROI 크롭으로 해결
#  - 0.0 : 검은 테두리 최소(대신 더 많이 잘림)
ALPHA = 1.0


def main():
    # 1) 캘리브레이션 데이터 로드
    data = np.load(CALIB_NPZ)
    K = data["camera_matrix"]
    dist = data["dist_coeffs"]

    # 2) 카메라 열기 (Windows 안정: DSHOW)
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("카메라 오픈 실패")
        return

    # 3) 첫 프레임으로 크기 확인 + newK/roi 계산(고정)
    #    (프레임 크기가 바뀌지 않는 한 매 프레임 계산할 필요 없음)
    ok = False
    frame0 = None
    for _ in range(20):  # 워밍업
        ret, frame0 = cap.read()
        if ret and frame0 is not None:
            ok = True
            break
        time.sleep(0.05)

    if not ok:
        print("프레임 읽기 실패(카메라 점유 프로그램 확인)")
        cap.release()
        return

    h, w = frame0.shape[:2]
    newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), ALPHA, (w, h))

    x, y, rw, rh = roi
    # roi가 (0,0,0,0)처럼 이상하게 나오는 경우 대비
    use_roi_crop = (rw > 0 and rh > 0)

    cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Undistorted+ROI", cv2.WINDOW_NORMAL)

    print("실시간 왜곡 보정 실행 중")
    print(" - ESC : 종료")
    print(" - S   : 현재 보정+크롭 프레임 저장\n")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[WARN] 프레임 읽기 실패")
            time.sleep(0.1)
            continue

        # 4) 왜곡 보정
        und = cv2.undistort(frame, K, dist, None, newK)

        # 5) ROI로 검은 테두리 제거(크롭)
        if use_roi_crop:
            und_crop = und[y:y+rh, x:x+rw]
        else:
            und_crop = und

        # 표시(원본 + 보정크롭)
        cv2.imshow("Original", frame)
        cv2.imshow("Undistorted+ROI", und_crop)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        elif key in (ord('s'), ord('S')):
            ts = time.strftime("%Y%m%d_%H%M%S")
            out_name = f"undist_roi_{ts}.jpg"
            cv2.imwrite(out_name, und_crop)
            print(f"[저장 완료] {out_name}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
