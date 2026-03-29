#!/usr/bin/env python3
from __future__ import annotations

import time
import threading
import socket
import struct

import cv2
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


#PC_IP       = '192.168.0.183'
PC_IP       = '172.20.10.3'
STREAM_PORT = 5000
SERIAL_PORT = '/dev/ttyACM0'
BAUD        = 115200
CAM_INDEX   = 0
CAM_WIDTH   = 640
CAM_HEIGHT  = 480


def reconnect_serial():
    for i in range(5):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
            try:
                ser.setDTR(False)
            except Exception:
                pass
            time.sleep(1.5)
            print('[Serial] 연결 성공')
            return ser
        except Exception as e:
            print(f'[Serial] 실패 {i+1}/5: {e}')
            time.sleep(1.0)
    return None


class Pi1Node(Node):

    def __init__(self):
        super().__init__('pi1_node')
        self.ser = None

        self.pub_uart = self.create_publisher(String, '/pi1/uart_response', 10)
        self.pub_hb   = self.create_publisher(String, '/system/heartbeat', 10)

        self.pub_pi2 = self.create_publisher(String, '/interpi/pi1_to_pi2', 10)
        
        self.pub_link   = self.create_publisher(String, '/pi1/interpi_rx', 10)


        self.create_subscription(
            String, '/pi1/uart_cmd',
            self._on_uart_cmd, 10
        )
        self.create_subscription(
            String, '/pi1/interpi_send',
            self._on_interpi_send, 10
        )
        self.create_subscription(
            String, '/interpi/pi2_to_pi1',
            self._on_from_pi2, 10
        )

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info('Pi1 노드 시작')

    def _on_uart_cmd(self, msg: String):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('UART 미연결')
            return

        try:
            data = msg.data
            if not data.endswith('\n'):
                data += '\n'

            self.ser.write(data.encode('utf-8'))
            self.ser.flush()
            self.get_logger().info(f'STM32 전달: "{data.strip()}"')

        except Exception as e:
            self.get_logger().error(f'UART 전송 실패: {e}')
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

    def publish_uart_response(self, line: str):
        msg = String()
        msg.data = line
        self.pub_uart.publish(msg)
        self.get_logger().info(f'STM32→PC: "{line}"')

    def _on_interpi_send(self, msg:String):
        payload = msg.data.strip()
        if not payload:
            self.get_logger().warn('Pi1->Pi2 전송 무시: 빈 메시지')
            return
        
        out = String()
        out.data = payload
        self.pub_pi2.publish(out)
        self.get_logger().info(f'Pi1->Pi2: "{payload}') 
    
    def _on_from_pi2(self, msg:String):
        payload = msg.data.strip()
        if not payload:
            return
        
        out = String()
        out.data = payload
        self.pub_link.publish(out)
        self.get_logger().info(f'Pi2->Pi1 수신: "{payload}"') 


    def _heartbeat(self):
        msg = String()
        msg.data = 'pi1'
        self.pub_hb.publish(msg)


def uart_rx_loop(node: Pi1Node):
    buf = ''
    while rclpy.ok():
        if node.ser is None or not node.ser.is_open:
            time.sleep(0.5)
            node.ser = reconnect_serial()
            buf = ''
            continue

        try:
            if node.ser.in_waiting > 0:
                raw = node.ser.read(node.ser.in_waiting)
                buf += raw.decode('utf-8', errors='ignore')

                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if line:
                        node.publish_uart_response(line)

        except OSError:
            try:
                node.ser.close()
            except Exception:
                pass
            node.ser = None
            buf = ''

        except serial.SerialException:
            node.ser = None
            buf = ''

        time.sleep(0.02)


def open_camera_with_retry():
    while rclpy.ok():
        cap = cv2.VideoCapture(CAM_INDEX)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
            print('[Stream] 카메라 연결 성공')
            return cap

        print('[Stream] 카메라 열기 실패 → 3초 후 재시도')
        try:
            cap.release()
        except Exception:
            pass
        time.sleep(3.0)

    return None


def video_stream_loop():
    while rclpy.ok():
        cap = open_camera_with_retry()
        if cap is None:
            break

        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((PC_IP, STREAM_PORT))
            print(f'[Stream] PC 연결 성공: {PC_IP}:{STREAM_PORT}')

            while rclpy.ok():
                ok, frame = cap.read()
                if not ok:
                    print('[Stream] 카메라 프레임 읽기 실패 → 카메라 재오픈')
                    break

                ok_enc, jpeg = cv2.imencode(
                    '.jpg', frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                if not ok_enc:
                    continue

                data = jpeg.tobytes()
                size = struct.pack('>I', len(data))
                sock.sendall(size + data)

        except Exception as e:
            print(f'[Stream] 전송/연결 오류: {e} → 재시도')

        finally:
            try:
                if sock is not None:
                    sock.close()
            except Exception:
                pass

            try:
                cap.release()
            except Exception:
                pass

            time.sleep(1.0)


def spin_thread(node: Pi1Node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    rclpy.init()
    node = Pi1Node()
    node.ser = reconnect_serial()

    threading.Thread(target=spin_thread, args=(node,), daemon=True).start()
    threading.Thread(target=uart_rx_loop, args=(node,), daemon=True).start()
    threading.Thread(target=video_stream_loop, daemon=True).start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser is not None and node.ser.is_open:
                node.ser.close()
        except Exception:
            pass

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