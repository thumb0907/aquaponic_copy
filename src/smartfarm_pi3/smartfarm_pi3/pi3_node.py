#!/usr/bin/env python3
"""
pi3_node.py  ─  라파3 노드 (ROS2)
=========================================================
PC ←(ROS2)→ Pi3 ←(UART 문자열)→ STM3 (수경재배실 센서허브)
                ←(TCP 스트림)→ CAM2, CAM3 (수경재배실 카메라 2대)

[STM3 UART 출력 형식 — 문자열, 115200bps]
  "tds = 512, ph = 7.20\r\n"     TDS(ppm) + pH
  "DHT: T=25.1 H=60.3\r\n"       기온(℃) + 습도(%)
  "T=25.23\r\n"                   DS18B20 수온(℃)
  "DS18B20: NO PRESENCE\r\n"      수온센서 미감지 (무시)
  "DS18B20 FAIL\r\n"              수온센서 읽기 실패 (무시)

[ROS2 토픽]
  구독: /pi3/uart_cmd       PC → Pi3 명령 (바이너리, 앞에 대상 식별자 포함)
  발행: /pi3/uart_response  STM3 상태/에러 → PC
        /pi3/sensor_data    센서값 → PC / 웹UI 연동용
        /system/heartbeat   1초 생존 신호 ('pi3')

[/pi3/sensor_data 형식]
  즉시 publish (줄 수신 시): "TDS:512.0,PH:7.20"
                              "WATER_TEMP:25.23"
                              "AIR_TEMP:25.1,HUMIDITY:60.3"
  주기 publish (1초, 캐시 전체): "TDS:512.0,PH:7.20,WATER_TEMP:25.23,AIR_TEMP:25.1,HUMIDITY:60.3"

[대상 식별자 — master_node _send_stm3() 와 일치]
  0x01 = STM3
  SOF(0xAA)로 시작 = STM3 fallback (ESTOP/RESET 등)

[카메라 스트림 포트]
  CAM2: TCP 5001  (수경재배실 카메라 왼쪽)
  CAM3: TCP 5002  (수경재배실 카메라 오른쪽)
  Pi3 → PC 방향으로 클라이언트 접속

[포트 설정]
  /dev/stm3  ← 수경재배실 STM32 센서허브
  udev rules 미설정 시 /dev/ttyACM0 등으로 변경
"""
from __future__ import annotations

import re
import time
import threading
import socket
import struct

import cv2
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


# ── 포트 / 네트워크 설정 ──────────────────────
PORT_STM3  = '/dev/stm3'      # 센서허브 STM32
BAUD       = 115200

PC_IP      = '192.168.0.183'  # master_node 실행 PC IP
CAM2_PORT  = 5001             # 수경재배실 카메라 왼쪽
CAM3_PORT  = 5002             # 수경재배실 카메라 오른쪽

CAM2_INDEX = 0                # Pi3 기준 CAM2 장치 번호
CAM3_INDEX = 1                # Pi3 기준 CAM3 장치 번호
CAM_WIDTH  = 640
CAM_HEIGHT = 480

# ── 대상 식별자 ───────────────────────────────
TARGET_STM3 = 0x01
TARGET_RSVD = 0x02            # 예비

# ── 프로토콜 상수 ─────────────────────────────
SOF = 0xAA                    # 바이너리 프레임 시작 (ESTOP/RESET 수신용)

# ── 센서 캐시 키 ──────────────────────────────
# sensor_data 토픽에 쓰는 키 이름 통일
KEY_TDS        = 'TDS'         # TDS (ppm)
KEY_PH         = 'PH'          # pH
KEY_WATER_TEMP = 'WATER_TEMP'  # DS18B20 수온 (℃)
KEY_AIR_TEMP   = 'AIR_TEMP'    # DHT22 기온 (℃)
KEY_HUMIDITY   = 'HUMIDITY'    # DHT22 습도 (%)

# ── 문자열 파서 정규식 ────────────────────────
# "tds = 512, ph = 7.20"
RE_TDS_PH   = re.compile(
    r'tds\s*=\s*(\d+),\s*ph\s*=\s*([\d.]+)', re.IGNORECASE)

# "DHT: T=25.1 H=60.3"
RE_DHT      = re.compile(
    r'DHT:\s*T=([-\d.]+)\s+H=([\d.]+)', re.IGNORECASE)

# "T=25.23"  (DS18B20 수온, DHT 줄과 구분하기 위해 앞에 'DHT'가 없어야 함)
RE_DS18B20  = re.compile(r'^T=([-\d.]+)\s*$')

# 무시할 줄 (파싱 불필요한 에러 메시지)
RE_IGNORE   = re.compile(
    r'(NO PRESENCE|DS18B20 FAIL|=== TEST)', re.IGNORECASE)


# ── 시리얼 연결 헬퍼 ──────────────────────────
def try_connect(port: str, label: str):
    """
    시리얼 포트 연결 시도.
    성공 시 Serial 객체 반환, 실패 시 None.
    DTR=False → STM32 자동 리셋 방지.
    """
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
        try:
            ser.setDTR(False)
        except Exception:
            pass
        time.sleep(1.5)
        print(f'[Serial] {label} 연결 성공 ({port})')
        return ser
    except Exception as e:
        print(f'[Serial] {label} 연결 실패 ({port}): {e}')
        return None


def _safe_close(ser):
    """시리얼 포트 안전하게 닫기"""
    try:
        if ser is not None:
            ser.close()
    except Exception:
        pass


# ══════════════════════════════════════════════
# Pi3 ROS2 노드
# ══════════════════════════════════════════════
class Pi3Node(Node):

    def __init__(self):
        super().__init__('pi3_node')

        # STM3 시리얼 핸들
        self.ser_stm3 = None

        # 센서값 캐시 — None이면 아직 수신 안 된 것
        self.sensor_cache: dict[str, float | None] = {
            KEY_TDS:        None,
            KEY_PH:         None,
            KEY_WATER_TEMP: None,
            KEY_AIR_TEMP:   None,
            KEY_HUMIDITY:   None,
        }
        self.sensor_lock = threading.Lock()

        # ── Publisher ─────────────────────────
        self.pub_uart   = self.create_publisher(
            String, '/pi3/uart_response', 10)
        self.pub_sensor = self.create_publisher(
            String, '/pi3/sensor_data', 10)   # 센서값 전용 토픽
        self.pub_hb     = self.create_publisher(
            String, '/system/heartbeat', 10)

        # ── Subscriber ────────────────────────
        # PC → Pi3 명령 (첫 바이트가 대상 식별자)
        self.create_subscription(
            UInt8MultiArray, '/pi3/uart_cmd',
            self._on_uart_cmd, 10)

        # 1초 주기: heartbeat + 캐시 전체 publish
        self.create_timer(1.0, self._heartbeat)
        self.create_timer(1.0, self._publish_sensor_cache)

        self.get_logger().info('Pi3 노드 시작 (수경재배실 센서허브 + 카메라 2대)')

    # ══════════════════════════════════════════
    # PC → Pi3 명령 → STM3 라우팅
    # ══════════════════════════════════════════
    def _on_uart_cmd(self, msg: UInt8MultiArray):
        """
        master_node에서 오는 명령을 STM3로 전달.

        [패킷 구조]
          [대상식별자(1)] [SOF(1)] [PID(1)] [LEN(1)] [DATA...] [CHK(1)]
          └ 0x01 = STM3

        SOF(0xAA)로 시작하면 식별자 없이 보낸 것 → STM3 fallback
        (ESTOP/RESET 등 긴급 명령)
        """
        raw = bytes(msg.data)
        if not raw:
            return

        # SOF로 시작 → 식별자 없음 → STM3 fallback
        if raw[0] == SOF:
            self._write_to_stm3(raw)
            return

        target  = raw[0]
        payload = raw[1:]   # 식별자 제거 후 실제 프레임

        if target == TARGET_STM3:
            self._write_to_stm3(payload)
        elif target == TARGET_RSVD:
            self.get_logger().warn(
                f'예비 대상(0x02) — 현재 미지원: {payload.hex()}')
        else:
            self.get_logger().warn(
                f'알 수 없는 대상 식별자: 0x{target:02X}')

    def _write_to_stm3(self, payload: bytes):
        """STM3 시리얼 포트로 바이트 전송"""
        if self.ser_stm3 is None or not self.ser_stm3.is_open:
            self.get_logger().error('STM3 미연결 — 전송 불가')
            return
        try:
            self.ser_stm3.write(payload)
            self.ser_stm3.flush()
            self.get_logger().info(f'→ STM3: {payload.hex()}')
        except Exception as e:
            self.get_logger().error(f'STM3 전송 실패: {e}')

    # ══════════════════════════════════════════
    # STM3 문자열 한 줄 파싱 → 캐시 갱신 + publish
    # ══════════════════════════════════════════
    def handle_stm3_line(self, line: str):
        """
        STM3에서 수신한 문자열 한 줄을 파싱해서 센서 캐시 갱신 및
        /pi3/sensor_data 즉시 publish.

        처리 우선순위:
          1. 무시할 줄 (에러 메시지) → 그냥 버림
          2. "tds = N, ph = N.NN"   → TDS + pH
          3. "DHT: T=N H=N"         → 기온 + 습도
          4. "T=N.NN" (줄 전체)     → DS18B20 수온
          5. 그 외                  → /pi3/uart_response 로 그대로 전달
        """
        line = line.strip()
        if not line:
            return

        # 1. 무시할 줄
        if RE_IGNORE.search(line):
            self.get_logger().debug(f'[STM3] 무시: {line}')
            return

        # 2. TDS + pH
        m = RE_TDS_PH.search(line)
        if m:
            tds = float(m.group(1))
            ph  = float(m.group(2))
            updates = {KEY_TDS: tds, KEY_PH: ph}
            self._update_cache_and_publish(updates)
            return

        # 3. DHT22 기온 + 습도
        m = RE_DHT.search(line)
        if m:
            air_temp = float(m.group(1))
            humidity = float(m.group(2))
            updates = {KEY_AIR_TEMP: air_temp, KEY_HUMIDITY: humidity}
            self._update_cache_and_publish(updates)
            return

        # 4. DS18B20 수온 ("T=25.23" — 줄 전체가 이 패턴인 경우만)
        m = RE_DS18B20.match(line)
        if m:
            water_temp = float(m.group(1))
            updates = {KEY_WATER_TEMP: water_temp}
            self._update_cache_and_publish(updates)
            return

        # 5. 그 외 → uart_response 로 전달 (디버그용)
        msg      = String()
        msg.data = f'STM3:PC:RAW:{line}'
        self.pub_uart.publish(msg)
        self.get_logger().info(f'STM3→PC(raw): {line}')

    def _update_cache_and_publish(self, updates: dict):
        """
        캐시 갱신 후 갱신된 값들만 즉시 publish.
        형식: "TDS:512.0,PH:7.20"  /  "WATER_TEMP:25.23"
        """
        with self.sensor_lock:
            self.sensor_cache.update(updates)

        # 갱신된 값만 즉시 publish
        parts = [f'{k}:{v:.2f}' for k, v in updates.items()]
        msg      = String()
        msg.data = ','.join(parts)
        self.pub_sensor.publish(msg)
        self.get_logger().info(f'[Sensor] {msg.data}')

    def _publish_sensor_cache(self):
        """
        1초 주기로 수신된 전체 센서값을 한 번에 publish.
        형식: "TDS:512.0,PH:7.20,WATER_TEMP:25.23,AIR_TEMP:25.1,HUMIDITY:60.3"
        아직 수신 안 된 항목은 제외.
        웹UI처럼 전체 상태를 주기적으로 폴링하는 쪽에서 사용.
        """
        with self.sensor_lock:
            cache = dict(self.sensor_cache)

        parts = [
            f'{k}:{v:.2f}'
            for k, v in cache.items()
            if v is not None
        ]
        if not parts:
            return   # 아직 아무 센서도 수신 안 됨

        msg      = String()
        msg.data = ','.join(parts)
        self.pub_sensor.publish(msg)

    # ══════════════════════════════════════════
    # Heartbeat
    # ══════════════════════════════════════════
    def _heartbeat(self):
        msg      = String()
        msg.data = 'pi3'
        self.pub_hb.publish(msg)


# ══════════════════════════════════════════════
# STM3 수신 루프 (별도 스레드)
# ══════════════════════════════════════════════
def stm3_rx_loop(node: Pi3Node):
    """
    STM3(센서허브) UART 수신 루프.
    STM3가 '\r\n' 종료 문자열을 보내므로 readline() 방식으로 읽음.
    끊기면 자동 재연결.
    """
    while rclpy.ok():
        # 연결 끊기면 재시도
        if node.ser_stm3 is None or not node.ser_stm3.is_open:
            node.ser_stm3 = try_connect(PORT_STM3, 'STM3')
            time.sleep(3.0)
            continue
        try:
            # readline(): '\n' 까지 읽음, timeout=0.1s 설정되어 있어서 블로킹 안 됨
            raw = node.ser_stm3.readline()
            if not raw:
                continue
            # 디코딩 실패 시 무시 (바이너리 노이즈 방어)
            try:
                line = raw.decode('utf-8', errors='ignore')
            except Exception:
                continue
            node.handle_stm3_line(line)
        except OSError:
            _safe_close(node.ser_stm3)
            node.ser_stm3 = None
        except serial.SerialException:
            node.ser_stm3 = None
        # readline 자체가 timeout으로 반환하므로 sleep 불필요
        # 단 CPU 과점유 방지용 최소 슬립
        time.sleep(0.005)


# ══════════════════════════════════════════════
# 카메라 스트림 루프 (카메라별 별도 스레드)
# ══════════════════════════════════════════════
def _camera_stream_loop(cam_index: int, pc_ip: str, port: int, label: str):
    """
    단일 카메라 → PC TCP 스트리밍 루프.
    Pi1의 video_stream_loop과 동일한 방식.
    끊기면 자동 재연결.

    프레임 구조: [4바이트 big-endian 길이][JPEG 바이트]
    PC(master_node)에서 해당 포트를 listen 하고 있어야 함.
    """
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

    if not cap.isOpened():
        print(f'[{label}] 카메라 열기 실패 (index={cam_index})')
        return

    print(f'[{label}] 카메라 초기화 완료 (index={cam_index})')

    while rclpy.ok():
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((pc_ip, port))
            print(f'[{label}] PC 연결: {pc_ip}:{port}')

            while rclpy.ok():
                ok, frame = cap.read()
                if not ok:
                    continue

                # JPEG 인코딩 (품질 80)
                _, jpeg = cv2.imencode(
                    '.jpg', frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 80])
                data = jpeg.tobytes()

                # [4바이트 길이][JPEG] 형식으로 전송
                sock.sendall(struct.pack('>I', len(data)) + data)

        except Exception as e:
            print(f'[{label}] 오류: {e} → 재연결 대기')
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
            time.sleep(1.0)

    cap.release()


def cam2_stream_loop():
    """CAM2 (수경재배실 왼쪽) 스트림 루프"""
    _camera_stream_loop(CAM2_INDEX, PC_IP, CAM2_PORT, 'CAM2')


def cam3_stream_loop():
    """CAM3 (수경재배실 오른쪽) 스트림 루프"""
    _camera_stream_loop(CAM3_INDEX, PC_IP, CAM3_PORT, 'CAM3')


# ══════════════════════════════════════════════
# 진입점
# ══════════════════════════════════════════════
def main():
    rclpy.init()
    node = Pi3Node()

    # STM3 초기 연결 시도
    node.ser_stm3 = try_connect(PORT_STM3, 'STM3')

    # 스레드 구성:
    #   - rclpy.spin      : ROS2 콜백 처리 (uart_cmd 수신 포함)
    #   - stm3_rx_loop    : STM3 UART 문자열 수신 → 파싱 → publish
    #   - cam2_stream_loop: CAM2 영상 → PC TCP 5001 전송
    #   - cam3_stream_loop: CAM3 영상 → PC TCP 5002 전송
    threading.Thread(target=rclpy.spin,       args=(node,), daemon=True).start()
    threading.Thread(target=stm3_rx_loop,     args=(node,), daemon=True).start()
    threading.Thread(target=cam2_stream_loop, daemon=True).start()
    threading.Thread(target=cam3_stream_loop, daemon=True).start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        _safe_close(node.ser_stm3)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()