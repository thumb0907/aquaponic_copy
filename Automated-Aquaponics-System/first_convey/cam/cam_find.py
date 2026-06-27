import cv2

for i in range(5):
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"camera index {i}: opened")
            cv2.imshow(f"cam{i}", frame)
            cv2.waitKey(1000)
            cv2.destroyAllWindows()
        else:
            print(f"camera index {i}: opened but no frame")
        cap.release()
    else:
        print(f"camera index {i}: failed")