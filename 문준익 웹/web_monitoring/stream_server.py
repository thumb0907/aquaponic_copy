from flask import Flask, Response
import cv2
import time

app = Flask(__name__)

CAMERA_INDEX = 0
WIDTH = 640
HEIGHT = 360
FPS = 15

camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
camera.set(cv2.CAP_PROP_FPS, FPS)

if not camera.isOpened():
    print("카메라를 열 수 없습니다. CAMERA_INDEX를 확인하세요.")


def generate_frames():
    while True:
        success, frame = camera.read()

        if not success:
            time.sleep(0.1)
            continue

        ret, buffer = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, 70]
        )

        if not ret:
            continue

        frame_bytes = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            frame_bytes +
            b"\r\n"
        )


@app.route("/")
def index():
    return "Raspberry Pi Camera Stream Server"


@app.route("/video")
def video():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)
