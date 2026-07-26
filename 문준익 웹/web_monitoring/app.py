#!/usr/bin/env python3

import os
import time
import threading

from flask import Flask, jsonify, send_from_directory

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# =====================================================
# 기본 경로
# =====================================================

BASE_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

VIDEO_DIR = os.path.join(
    BASE_DIR,
    "videos"
)

app = Flask(
    __name__,
    static_folder=BASE_DIR
)


# =====================================================
# 최신 센서 데이터
# =====================================================

latest_data = {
    "pi3_alive": False,
    "emergency": False,

    "level": None,
    "water_temp": None,
    "ph": None,
    "tds": None,

    "air_temp": None,
    "humidity": None,

    "last_update": None
}

data_lock = threading.Lock()


# =====================================================
# 데이터 변환 함수
# =====================================================

def to_float(value):
    if value is None:
        return None

    try:
        return float(value)

    except (ValueError, TypeError):
        return None


def to_bool(value):
    if value is None:
        return False

    return str(value).strip().lower() in {
        "true",
        "1",
        "yes",
        "on"
    }


def parse_monitor_state(text):
    """
    수신 데이터 예시:

    pi3_alive:True,
    emergency:False,
    sensor_level:320,
    sensor_tds:650,
    sensor_ph:6.85,
    sensor_water_temp:25.4,
    sensor_air_temp:24.8,
    sensor_humidity:58.0
    """

    parsed = {}

    for item in text.split(","):
        item = item.strip()

        if ":" not in item:
            continue

        key, value = item.split(":", 1)

        parsed[
            key.strip()
        ] = value.strip()

    return parsed


# =====================================================
# ROS2 센서값 수신 노드
# =====================================================

class DashboardBridgeNode(Node):
    def __init__(self):
        super().__init__(
            "dashboard_bridge_node"
        )

        self.create_subscription(
            String,
            "/monitor/state",
            self.monitor_callback,
            10
        )

        self.get_logger().info(
            "Dashboard bridge node started"
        )

        self.get_logger().info(
            "Waiting for /monitor/state"
        )

    def monitor_callback(self, msg):
        parsed = parse_monitor_state(
            msg.data
        )

        with data_lock:
            latest_data["pi3_alive"] = (
                to_bool(
                    parsed.get("pi3_alive")
                )
            )

            latest_data["emergency"] = (
                to_bool(
                    parsed.get("emergency")
                )
            )

            latest_data["level"] = (
                to_float(
                    parsed.get("sensor_level")
                )
            )

            latest_data["water_temp"] = (
                to_float(
                    parsed.get(
                        "sensor_water_temp"
                    )
                )
            )

            latest_data["ph"] = (
                to_float(
                    parsed.get("sensor_ph")
                )
            )

            latest_data["tds"] = (
                to_float(
                    parsed.get("sensor_tds")
                )
            )

            latest_data["air_temp"] = (
                to_float(
                    parsed.get(
                        "sensor_air_temp"
                    )
                )
            )

            latest_data["humidity"] = (
                to_float(
                    parsed.get(
                        "sensor_humidity"
                    )
                )
            )

            latest_data["last_update"] = (
                time.strftime(
                    "%Y-%m-%d %H:%M:%S"
                )
            )

        self.get_logger().info(
            f"Sensor data received: {msg.data}"
        )


# =====================================================
# 웹 페이지 파일
# =====================================================

@app.route("/")
def index():
    return send_from_directory(
        BASE_DIR,
        "index.html"
    )


@app.route("/style.css")
def style_css():
    return send_from_directory(
        BASE_DIR,
        "style.css"
    )


@app.route("/script.js")
def script_js():
    return send_from_directory(
        BASE_DIR,
        "script.js"
    )


# =====================================================
# 타임랩스 영상 파일
# =====================================================

@app.route("/videos/<path:filename>")
def video_file(filename):
    return send_from_directory(
        VIDEO_DIR,
        filename,
        conditional=True
    )


# =====================================================
# 센서 API
# =====================================================

@app.route("/api/sensors")
def api_sensors():
    with data_lock:
        response_data = dict(
            latest_data
        )

    return jsonify(
        response_data
    )


# =====================================================
# ROS2 스레드
# =====================================================

def ros_thread_main():
    rclpy.init()

    node = DashboardBridgeNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


# =====================================================
# 프로그램 실행
# =====================================================

def main():
    ros_thread = threading.Thread(
        target=ros_thread_main,
        daemon=True
    )

    ros_thread.start()

    app.run(
        host="0.0.0.0",
        port=8080,
        debug=False,
        threaded=True,
        use_reloader=False
    )


if __name__ == "__main__":
    main()