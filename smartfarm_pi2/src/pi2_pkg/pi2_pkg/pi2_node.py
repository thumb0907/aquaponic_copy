#!/usr/bin/env python3
"""
pi2_node.py  ─  라파2 노드
=========================================================
pi1_node.py와 완전히 동일한 구조.

차이점:
  pi1_node  : STM32가 긴 문자열 직접 송신 → 원문 그대로 발행
  pi2_node  : STM32가 짧은 코드 송신(C1, D1 …) → 프로토콜 변환 후 발행

담당:
  STM32 #2  /dev/ttyUSB0  (TX/RX 핀 직결, 115200 bps)
  카메라 없음

ROS2 토픽:
  구독  /pi2/uart_cmd      String  master_node → STM32 명령
  발행  /pi2/uart_response String  STM32 상태  → master_node
  발행  /system/heartbeat  String  'pi2' (1 Hz)

UART 변환표:
  수신 (STM32 → 라파)       발행 (/pi2/uart_response)
  C1  컨베이어 구동      →  STM2:PC:STATE:CONVEY_RUN
  I1  IR 감지            →  STM2:PC:STATE:IR_DETECTED
  Z1  Z축 고정 완료      →  STM2:PC:STATE:Z_FIX
  H1  수확 명령 송신     →  STM2:PC:STATE:HARVESTING
  R1  수확 완료          →  STM2:PC:STATE:HARVEST_DONE
  X1  배출 컨베이어 구동 →  STM2:PC:STATE:EJECTING
  X0  배출 완료          →  STM2:PC:STATE:EJECT_DONE
  D1  사이클 완료        →  STM2:PC:DONE:CYCLE
  E1  긴급정지           →  STM2:PC:STATE:ESTOP
  TO  타임아웃           →  STM2:PC:ERR:TIMEOUT
  RS  리셋 완료          →  STM2:PC:STATE:RESET_DONE

  송신 (master_node → STM32)  /pi2/uart_cmd 구독
  S1  시작
  R1  수확완료 확인 or 긴급정지 해제  (STM32가 context로 구분)
  E1  긴급정지
"""

from __future__ import annotations

import time
import threading

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SERIAL_PORT = '/dev/ttyUSB0'   # 환경에 맞게 수정
BAUD        = 115200

STM2_PROTOCOL = {
    'C1': 'STM2:PC:STATE:CONVEY_RUN',
    'I1': 'STM2:PC:STATE:IR_DETECTED',
    'Z1': 'STM2:PC:STATE:Z_FIX',
    'H1': 'STM2:PC:STATE:HARVESTING',
    'R1': 'STM2:PC:STATE:HARVEST_DONE',
    'X1': 'STM2:PC:STATE:EJECTING',
    'X0': 'STM2:PC:STATE:EJECT_DONE',
    'D1': 'STM2:PC:DONE:CYCLE',
    'E1': 'STM2:PC:STATE:ESTOP',
    'TO': 'STM2:PC:ERR:TIMEOUT',
    'RS': 'STM2:PC:STATE:RESET_DONE',
}

ALLOWED_CMDS = {'S1', 'R1', 'E1'}


def reconnect_serial():
    """pi1_node.reconnect_serial()과 동일."""
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


class Pi2Node(Node):

    def __init__(self):
        super().__init__('pi2_node')
        self.ser = None

        self.pub_uart = self.create_publisher(String, '/pi2/uart_response', 10)
        self.pub_hb   = self.create_publisher(String, '/system/heartbeat',  10)

        self.create_subscription(
            String, '/pi2/uart_cmd',
            self._on_uart_cmd, 10
        )

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info('Pi2 노드 시작')

    def _on_uart_cmd(self, msg: String):
        """master_node → STM32 명령 전달 (pi1_node._on_uart_cmd와 동일한 구조)"""
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
        STM32 단순 코드를 프로토콜 문자열로 변환 후 발행.
        pi1_node는 원문 그대로 발행하지만,
        pi2_node는 STM32가 짧은 코드를 보내므로 변환 필요.
        """
        protocol = STM2_PROTOCOL.get(line, f'STM2:PC:UNKNOWN:{line}')
        msg      = String()
        msg.data = protocol
        self.pub_uart.publish(msg)
        self.get_logger().info(f'STM32→PC: "{line}" → "{protocol}"')

    def _heartbeat(self):
        msg      = String()
        msg.data = 'pi2'
        self.pub_hb.publish(msg)


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
