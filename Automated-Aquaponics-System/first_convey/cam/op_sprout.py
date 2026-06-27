import cv2
import numpy as np

# ==============================
# 1. 설정값
# ==============================

CAMERA_INDEX = 0

CALIB_PATH = r"/home/kkh/Automated-Aquaponics-System/first_convey/cam/camera_calib.npz"

# 트레이 ROI 수동 설정
# 주의: 이 좌표는 "왜곡 보정 + 가장자리 제거 후 이미지" 기준이다.
ROI_X = 115 - 40
ROI_Y = 110 - 40
ROI_W = 440
ROI_H = 210

# HSV 초록색 범위
LOWER_GREEN = np.array([31, 70, 100])
UPPER_GREEN = np.array([40, 230, 230])

MIN_SPROUT_AREA = 12
KERNEL_SIZE = 2

# 왜곡 보정 후 유효 영역에서 추가로 더 자르고 싶으면 사용
# 보통은 0으로 두면 됨
EXTRA_CROP_MARGIN = 0


# ==============================
# 2. 카메라 캘리브레이션 로드
# ==============================

def load_camera_calibration(calib_path):
    """
    camera_calib.npz에서 카메라 행렬과 왜곡 계수를 불러온다.
    npz 파일 내부 key 이름이 다를 수 있으므로 여러 경우를 처리한다.
    """

    data = np.load(calib_path)

    print("Calibration file keys:", data.files)

    camera_matrix_keys = [
        "camera_matrix",
        "mtx",
        "K",
        "cam_mtx",
        "cameraMatrix"
    ]

    dist_coeff_keys = [
        "dist_coeffs",
        "dist",
        "D",
        "distortion",
        "distCoeffs"
    ]

    camera_matrix = None
    dist_coeffs = None

    for key in camera_matrix_keys:
        if key in data.files:
            camera_matrix = data[key]
            break

    for key in dist_coeff_keys:
        if key in data.files:
            dist_coeffs = data[key]
            break

    if camera_matrix is None:
        raise KeyError("카메라 행렬을 찾을 수 없습니다. npz 내부 key를 확인하세요.")

    if dist_coeffs is None:
        raise KeyError("왜곡 계수를 찾을 수 없습니다. npz 내부 key를 확인하세요.")

    print("Camera Matrix:")
    print(camera_matrix)

    print("Distortion Coefficients:")
    print(dist_coeffs)

    return camera_matrix, dist_coeffs


# ==============================
# 3. 왜곡 보정 맵 생성
# ==============================

def create_undistort_maps(frame, camera_matrix, dist_coeffs):
    """
    첫 프레임 크기를 기준으로 왜곡 보정용 map1, map2를 생성한다.
    valid_roi는 왜곡 보정 후 검은 가장자리를 제외한 유효 영역이다.
    """

    h, w = frame.shape[:2]

    print(f"Camera frame size: {w} x {h}")

    new_camera_matrix, valid_roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        dist_coeffs,
        (w, h),
        alpha=1,
        newImgSize=(w, h)
    )

    print("Valid ROI after undistortion:", valid_roi)

    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix,
        dist_coeffs,
        None,
        new_camera_matrix,
        (w, h),
        cv2.CV_16SC2
    )

    return map1, map2, valid_roi


# ==============================
# 4. 왜곡 보정 + 가장자리 제거
# ==============================

def undistort_and_crop(frame, map1, map2, valid_roi):
    """
    왜곡 보정 후 valid_roi 기준으로 검은 가장자리를 제거한다.
    """

    undistorted = cv2.remap(
        frame,
        map1,
        map2,
        interpolation=cv2.INTER_LINEAR
    )

    x_roi, y_roi, w_roi, h_roi = valid_roi

    if w_roi > 0 and h_roi > 0:
        x1 = x_roi + EXTRA_CROP_MARGIN
        y1 = y_roi + EXTRA_CROP_MARGIN
        x2 = x_roi + w_roi - EXTRA_CROP_MARGIN
        y2 = y_roi + h_roi - EXTRA_CROP_MARGIN

        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(undistorted.shape[1], x2)
        y2 = min(undistorted.shape[0], y2)

        if x2 > x1 and y2 > y1:
            undistorted_crop = undistorted[y1:y2, x1:x2]
        else:
            undistorted_crop = undistorted
    else:
        undistorted_crop = undistorted

    return undistorted, undistorted_crop


# ==============================
# 5. ROI 범위 안전 보정
# ==============================

def clamp_roi(frame, x, y, w, h):
    """
    ROI가 이미지 영역 밖으로 나가지 않도록 보정한다.
    """

    frame_h, frame_w = frame.shape[:2]

    x = max(0, x)
    y = max(0, y)

    w = min(w, frame_w - x)
    h = min(h, frame_h - y)

    if w <= 0 or h <= 0:
        return None

    return x, y, w, h


# ==============================
# 6. 새싹 검출 함수
# ==============================

def detect_sprouts(frame):
    """
    입력:
        frame: 왜곡 보정 + 가장자리 제거가 끝난 BGR 이미지

    출력:
        result_frame: 결과 표시 이미지
        sprout_count: 검출된 새싹 후보 개수
        mask_clean: 후처리된 이진 마스크
    """

    result_frame = frame.copy()

    # ------------------------------
    # ROI 범위 확인
    # ------------------------------
    roi_info = clamp_roi(
        frame,
        ROI_X,
        ROI_Y,
        ROI_W,
        ROI_H
    )

    if roi_info is None:
        print("ROI가 이미지 범위를 벗어났습니다. ROI_X, ROI_Y, ROI_W, ROI_H를 다시 설정하세요.")
        empty_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        return result_frame, 0, empty_mask

    roi_x, roi_y, roi_w, roi_h = roi_info

    # ------------------------------
    # ROI 자르기
    # ------------------------------
    roi = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]

    # ------------------------------
    # BGR → HSV 변환
    # ------------------------------
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # ------------------------------
    # 초록색 영역 마스크 생성
    # ------------------------------
    mask = cv2.inRange(
        hsv,
        LOWER_GREEN,
        UPPER_GREEN
    )

    # ------------------------------
    # morphology 연산
    # ------------------------------
    kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)

    # 작은 점 제거
    mask_clean = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    # 끊어진 영역 연결
    mask_clean = cv2.morphologyEx(
        mask_clean,
        cv2.MORPH_CLOSE,
        kernel
    )

    # ------------------------------
    # contour 검출
    # ------------------------------
    contours, _ = cv2.findContours(
        mask_clean,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    sprout_count = 0

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < MIN_SPROUT_AREA:
            continue

        sprout_count += 1

        x, y, w, h = cv2.boundingRect(contour)

        # ROI 내부 좌표 → 현재 frame 기준 좌표
        global_x = roi_x + x
        global_y = roi_y + y

        cv2.rectangle(
            result_frame,
            (global_x, global_y),
            (global_x + w, global_y + h),
            (0, 255, 0),
            2
        )

        M = cv2.moments(contour)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"]) + roi_x
            cy = int(M["m01"] / M["m00"]) + roi_y

            cv2.circle(
                result_frame,
                (cx, cy),
                4,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                result_frame,
                f"{sprout_count}",
                (cx + 5, cy - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

    # ------------------------------
    # ROI 표시
    # ------------------------------
    cv2.rectangle(
        result_frame,
        (roi_x, roi_y),
        (roi_x + roi_w, roi_y + roi_h),
        (255, 0, 0),
        2
    )

    cv2.putText(
        result_frame,
        f"Sprout Count: {sprout_count}",
        (30, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 0, 255),
        2
    )

    return result_frame, sprout_count, mask_clean


# ==============================
# 7. 메인 실행
# ==============================

def main():
    # ------------------------------
    # 캘리브레이션 파라미터 로드
    # ------------------------------
    camera_matrix, dist_coeffs = load_camera_calibration(CALIB_PATH)

    # ------------------------------
    # 카메라 열기
    # ------------------------------
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    # ------------------------------
    # 첫 프레임으로 보정 맵 생성
    # ------------------------------
    ret, first_frame = cap.read()

    if not ret:
        print("첫 프레임을 읽을 수 없습니다.")
        cap.release()
        return

    map1, map2, valid_roi = create_undistort_maps(
        first_frame,
        camera_matrix,
        dist_coeffs
    )

    print("카메라 실행 시작")
    print("q: 종료")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # ------------------------------
        # 1. 왜곡 보정 + 가장자리 제거
        # ------------------------------
        undistorted_full, undistorted_crop = undistort_and_crop(
            frame,
            map1,
            map2,
            valid_roi
        )

        # ------------------------------
        # 2. 새싹 검출
        # ------------------------------
        result_frame, sprout_count, mask_clean = detect_sprouts(
            undistorted_crop
        )

        # ------------------------------
        # 3. 화면 출력
        # ------------------------------
        #cv2.imshow("Original", frame)
        #cv2.imshow("Undistorted Full", undistorted_full)
        cv2.imshow("Undistorted Cropped + Detection", result_frame)
        cv2.imshow("Green Mask ROI", mask_clean)

        print(f"Detected sprouts: {sprout_count}")

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
