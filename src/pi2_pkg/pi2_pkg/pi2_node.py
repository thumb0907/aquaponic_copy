#!/usr/bin/env python3
"""
pi2_node.py  ─  라파2 노드 (ROS2)
=========================================================
PC ←(ROS2)→ Pi2 ←(GPIO UART 핀)→ STM2

물리 연결 (Pi2 ↔ STM2):
  Pi2 GPIO14(TX) ─── STM32 PA10(RX, USART1_RX)
  Pi2 GPIO15(RX) ─── STM32 PA9(TX, USART1_TX)
  Pi2 GND        ─── STM32 GND
  ※ 3.3V 레벨 동일하므로 레벨 시프터 불필요

STM32 수신 예시:
  STM2:PC:STATE:CONVEY_RUN
  STM2:PC:STATE:HARVESTING
  STM2:PC:DONE:CYCLE2
  STM2:PC:ERR:TIMEOUT
  STM2:PI2:ACK:TRAY_PLACED

라파 → STM32 송신:
  PI2:STM2:EVT:TRAY_PLACED      (스카라가 트레이 올려놓음)
  PI2:STM2:EVT:HARVEST_DONE     (수확 완료)
  PC:STM2:CMD:ESTOP
  PC:STM2:CMD:RESET

GPIO UART 활성화 (한 번만):
  sudo raspi-config → Interface Options → Serial Port
    → login shell: No
    → serial port hardware: Yes
  재부팅 후 /dev/serial0 사용 가능
"""

from __future__ import annotations

import time
import threading

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SERIAL_PORT = '/dev/ttyAMA0'   # GPIO UART (TX=GPIO14/핀8, RX=GPIO15/핀10)
BAUD        = 115200

# 라파 → STM32 허용 커맨드
ALLOWED_CMDS = {
    'PI2:STM2:EVT:TRAY_PLACED',
    'PI2:STM2:EVT:HARVEST_DONE',
    'PC:STM2:CMD:ESTOP',
    'PC:STM2:CMD:RESET',
}


def reconnect_serial():
    """GPIO UART 연결 (핀 직결이므로 DTR 불필요)"""
    for i in range(5):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
            time.sleep(0.3)
            print('[Serial] 연결 성공')
            return ser
        except Exception as e:
            print(f'[Serial] 실패 {i+1}/5: {e}')
            time.sleep(1.0)
    return None


class Pi2Node(Node):

    def __init__(self):
        super().__init__('pi2_node')
        self.ser = None

        # pi1_node와 동일한 토픽 구조
        self.pub_uart = self.create_publisher(String, '/pi2/uart_response', 10)
        self.pub_hb   = self.create_publisher(String, '/system/heartbeat',  10)
        self.pub_pi1  = self.create_publisher(String, '/interpi/pi2_to_pi1', 10)
        self.pub_link = self.create_publisher(String, 'pi2/interpi_rx', 10)

        self.create_subscription(
            String, '/pi2/uart_cmd',
            self._on_uart_cmd, 10
        )
        
        self.create_subscription(
            String, '/pi2/interpi_send',
            self._on_interpi_send, 10
        )

        self.create_subscription(
            String, '/interpi/pi1_to_pi2',
            self._on_from_pi1, 10
        )

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info('Pi2 노드 시작')

    def _on_uart_cmd(self, msg: String):
        """master_node → STM32 명령 전달 (pi1_node와 동일한 구조)"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('UART 미연결')
            return

        cmd = msg.data.strip()
        if cmd not in ALLOWED_CMDS:
            self.get_logger().warn(f'허용되지 않은 커맨드: "{cmd}"')
            return

        try:
            self.ser.write(f'{cmd}\n'.encode('ascii'))
            self.ser.flush()
            self.get_logger().info(f'STM32 전달: "{cmd}"')
        except Exception as e:
            self.get_logger().error(f'UART 전송 실패: {e}')
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

    def publish_uart_response(self, line: str):
        """
        pi1_node.publish_uart_response()와 완전히 동일.
        STM1은 원문을 그대로 발행 → pi2도 동일하게 원문 발행.
        (변환 테이블 불필요 — STM32가 이미 긴 형식으로 보냄)
        """
        msg      = String()
        msg.data = line
        self.pub_uart.publish(msg)
        self.get_logger().info(f'STM32→PC: "{line}"')

    def _heartbeat(self):
        msg      = String()
        msg.data = 'pi2'
        self.pub_hb.publish(msg)
    
    def _on_interpi_send(self, msg:String):
        payload = msg.data.strip()
        if not payload:
            self.get_logger().warn('Pi2->Pi1 전송 무시: 빈 메시지')
            return
        
        out = String()
        out.data = payload
        self.pub_pi1.publish(out)
        self.get_logger().info(f'Pi2->Pi1: "{payload}"')
    
    def _on_from_pi1(self, msg:String):
        payload = msg.data.strip()
        if not payload:
            return
        
        out = String()
        out.data = payload
        self.pub_link.publish(out)
        self.get_logger().info(f'Pi2->Pi1 수신: "{payload}"')
    

def uart_rx_loop(node: Pi2Node):
    """pi1_node.uart_rx_loop()과 동일."""
    buf = ''
    while rclpy.ok():
        if node.ser is None or not node.ser.is_open:
            time.sleep(1.0)
            node.ser = reconnect_serial()
            buf = ''
            continue

        try:
            if node.ser.in_waiting > 0:
                raw  = node.ser.read(node.ser.in_waiting)
                buf += raw.decode('ascii', errors='ignore')

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


def spin_thread(node: Pi2Node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    rclpy.init()
    node     = Pi2Node()
    node.ser = reconnect_serial()

    threading.Thread(target=spin_thread,  args=(node,), daemon=True).start()
    threading.Thread(target=uart_rx_loop, args=(node,), daemon=True).start()

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