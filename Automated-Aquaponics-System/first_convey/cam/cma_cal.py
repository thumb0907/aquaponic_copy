import os
import time
import glob
import cv2
import numpy as np

# =========================
# 설정
# =========================
CAM_INDEX = 0  # ✅ 외장 웹캠 인덱스
PATTERN_SIZE = (9, 7)       # 내부 코너 (가로, 세로)  ← 네 체커보드3 기준
SQUARE_SIZE_MM = 21.0       # 한 칸 실제 길이(mm) (출력물에서 자로 재서 수정)
OUT_DIR = "calib_images"
CALIB_BASENAME = "camera_calib"
RECOMMENDED_MIN = 15


def ensure_dir(path: str):
    if not os.path.exists(path):
        os.makedirs(path)


def build_object_points(pattern_size, square_size):
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp


def save_calibration_npz_yaml(filename_base, K, dist, img_size, rms,
                              pattern_size, square_size, reproj_error):
    npz_path = f"{filename_base}.npz"
    np.savez(
        npz_path,
        camera_matrix=K,
        dist_coeffs=dist,
        img_width=img_size[0],
        img_height=img_size[1],
        rms=rms,
        reprojection_error=reproj_error,
        pattern_cols=pattern_size[0],
        pattern_rows=pattern_size[1],
        square_size=square_size
    )

    yaml_path = f"{filename_base}.yaml"
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", K)
    fs.write("dist_coeffs", dist)
    fs.write("image_width", int(img_size[0]))
    fs.write("image_height", int(img_size[1]))
    fs.write("rms", float(rms))
    fs.write("reprojection_error", float(reproj_error))
    fs.write("pattern_cols", int(pattern_size[0]))
    fs.write("pattern_rows", int(pattern_size[1]))
    fs.write("square_size", float(square_size))
    fs.release()

    print("\n[저장 완료]")
    print(f"- {npz_path}")
    print(f"- {yaml_path}\n")


def calibrate_from_folder(folder, pattern_size, square_size):
    images = sorted(glob.glob(os.path.join(folder, "*.jpg")) + glob.glob(os.path.join(folder, "*.png")))
    if len(images) == 0:
        print("[ERROR] 캘리브레이션할 이미지가 없습니다.")
        return None

    objp_template = build_object_points(pattern_size, square_size)
    objpoints, imgpoints = [], []
    img_size = None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
             cv2.CALIB_CB_NORMALIZE_IMAGE |
             cv2.CALIB_CB_FAST_CHECK)

    valid = 0

    for path in images:
        img = cv2.imread(path)
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])

        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if not found:
            continue

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objpoints.append(objp_template)
        imgpoints.append(corners2)
        valid += 1

    if valid < 3:
        print(f"[ERROR] 체스보드 검출 성공 이미지가 너무 적습니다: {valid}장")
        print("각도/거리 다르게 더 찍어주세요. (권장 10~20장)")
        return None

    print(f"[INFO] 유효 이미지: {valid}장 / 전체 {len(images)}장")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)

    # 평균 재투영 오차
    total_err2 = 0.0
    total_n = 0
    for i in range(len(objpoints)):
        projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(imgpoints[i], projected, cv2.NORM_L2)
        n = len(projected)
        total_err2 += err * err
        total_n += n
    reproj_error = np.sqrt(total_err2 / total_n) if total_n > 0 else float("inf")

    return {"rms": rms, "K": K, "dist": dist, "img_size": img_size, "reproj_error": reproj_error}


def main():
    ensure_dir(OUT_DIR)

    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("카메라 오픈 실패. 인덱스/점유 프로그램(OBS/줌) 확인.")
        return

    # 워밍업
    ok = False
    for _ in range(10):
        ret, frame = cap.read()
        if ret and frame is not None:
            ok = True
            break
        time.sleep(0.05)

    if not ok:
        print("프레임 읽기 실패. 다른 앱이 카메라를 사용 중인지 확인.")
        cap.release()
        return

    cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL)

    print("\n===== Calibration Capture =====")
    print("Space : 체스보드 검출되면 이미지 저장")
    print("C     : 저장된 이미지로 캘리브레이션 수행 + 결과 저장")
    print("ESC   : 종료\n")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
             cv2.CALIB_CB_NORMALIZE_IMAGE |
             cv2.CALIB_CB_FAST_CHECK)

    saved_count = len(glob.glob(os.path.join(OUT_DIR, "*.jpg"))) + len(glob.glob(os.path.join(OUT_DIR, "*.png")))

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[WARN] 프레임 읽기 실패")
            time.sleep(0.2)
            continue

        view = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, flags)

        status = "FOUND" if found else "NOT FOUND"
        cv2.putText(view, f"Chessboard: {status}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 0) if found else (0, 0, 255), 2)

        cv2.putText(view, f"Saved: {saved_count} (rec {RECOMMENDED_MIN}+)",
                    (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(view, PATTERN_SIZE, corners2, found)

        cv2.imshow("Webcam", view)

        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            print("종료합니다.")
            break

        elif key == 32:  # Space 저장
            if not found:
                print("[저장 실패] 체스보드가 검출되지 않았습니다. 각도/거리/조명 바꿔서 다시.")
                continue

            ts = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(OUT_DIR, f"calib_{ts}.jpg")
            cv2.imwrite(filename, frame)
            saved_count += 1
            print(f"[저장 완료] {filename}  (총 {saved_count}장)")

        elif key in (ord('c'), ord('C')):
            print(f"\n[캘리브레이션 시작] 폴더: {OUT_DIR}")
            result = calibrate_from_folder(OUT_DIR, PATTERN_SIZE, SQUARE_SIZE_MM)
            if result is None:
                continue

            print("\n[결과]")
            print(f"- RMS: {result['rms']}")
            print(f"- Reprojection Error(px): {result['reproj_error']}")
            print(f"- Image Size: {result['img_size']}")
            print(f"- Camera Matrix:\n{result['K']}")
            print(f"- Dist Coeffs:\n{result['dist']}\n")

            save_calibration_npz_yaml(
                CALIB_BASENAME,
                result["K"],
                result["dist"],
                result["img_size"],
                result["rms"],
                PATTERN_SIZE,
                SQUARE_SIZE_MM,
                result["reproj_error"]
            )

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
