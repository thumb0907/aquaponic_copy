# Aquaponics ROS2 DDS Communication And Protocol Format

이 문서는 현재 코드 기준의 ROS2 통신 구조와 UART 바이너리 프레임 포맷을 그림/코드로 정리한 것이다.

## 생성된 그림

- `docs/ros2_dds_communication_diagram.svg`
  - PC `master_node.py`, ROS2 DDS/RTPS, Pi1/Pi2/Pi3 게이트웨이, STM/SCARA/카메라 연결 흐름
- `docs/ros2_protocol_frame_format.svg`
  - `SOF | ID | LEN | DATA | CHK` 바이트 프레임과 예시

## ROS2 DDS Topic Map

| 방향 | Topic | Type | 역할 |
|---|---|---|---|
| PC -> Pi1 | `/pi1/uart_cmd` | `std_msgs/UInt8MultiArray` | STM1 명령 프레임 전달 |
| Pi1 -> PC | `/pi1/uart_response` | `std_msgs/String` | STM1 상태/완료/에러/플래그 문자열 |
| PC -> Pi2 | `/pi2/uart_cmd` | `std_msgs/UInt8MultiArray` | SCARA/Manipulator/STM2 라우팅 명령 |
| Pi2 -> PC | `/pi2/uart_response` | `std_msgs/String` | SCARA/STM2 상태/완료/에러/플래그 문자열 |
| Pi2 -> PC | `/pi2/flag_update` | `std_msgs/String` | 장치 플래그 업데이트 보조 채널 |
| PC -> Pi3 | `/pi3/uart_cmd` | `std_msgs/UInt8MultiArray` | STM3 센서허브 명령 |
| Pi3 -> PC | `/pi3/uart_response` | `std_msgs/String` | STM3 raw/status 문자열 |
| Pi3 -> PC | `/pi3/sensor_data` | `std_msgs/String` | 수경재배실 센서 데이터 |
| PC -> UI/log | `/smartfarm/monitor` | `std_msgs/String` | master 상태 및 flag 모니터링 |

ROS2의 DDS 계층은 위 Topic들의 discovery, publish, subscribe 전송을 담당한다. 이 프로젝트에서 실제 장치 프로토콜은 DDS payload 안에 들어가는 `UInt8MultiArray` 바이트 배열 또는 `String` 상태 문자열로 표현된다.

## UART Binary Frame

```text
SOF(1) | ID(1) | LEN(1) | DATA(LEN) | CHK(1)
```

```text
SOF = 0xAA
CHK = (ID + LEN + sum(DATA)) & 0xFF
total_length = 4 + LEN
```

## Python Frame Builder

현재 `master_node.py`의 포맷과 같은 형태다.

```python
SOF = 0xAA

def make_frame(pid: int, data: list[int] | None = None) -> list[int]:
    data = data or []
    length = len(data)
    chk = (pid + length + sum(data)) & 0xFF
    return [SOF, pid, length, *data, chk]

def make_flag_u8(pid: int, val: int) -> list[int]:
    return make_frame(pid, [val & 0xFF])

def make_flag_u16(pid: int, val: int) -> list[int]:
    return make_frame(pid, [(val >> 8) & 0xFF, val & 0xFF])
```

## Example Frames

```text
C1F=1 to STM1
PID_C1F = 0x0E
DATA    = 0x01
FRAME   = AA 0E 01 01 10
```

```text
CRF=1 to SCARA through Pi2
TARGET_SCARA = 0x01
PID_CRF      = 0x03
INNER FRAME  = AA 03 01 01 05
ROS2 PAYLOAD = 01 AA 03 01 01 05
```

```text
FF=1 to STM2 through Pi2 fallback
PID_FF = 0x0A
FRAME  = AA 0A 01 01 0C
```

## Pi2 Target Byte

Pi2는 `/pi2/uart_cmd` 수신 시 첫 바이트로 장치를 라우팅한다.

| Target | 장치 |
|---|---|
| `0x01` | SCARA |
| `0x02` | Manipulator |
| `0x03` | STM2 |
| 첫 바이트가 `0xAA` | target 없음, STM2 fallback |

## 주요 PID

| PID | 이름 | 의미 |
|---|---|---|
| `0x01` | `SSF` | SCARA start flag |
| `0x02` | `SMF` | SCARA move flag |
| `0x03` | `CRF` | Cartesian reset flag |
| `0x04` | `UV` | UV tray count, 16-bit |
| `0x05` | `ULF` | UV left sprout done |
| `0x06` | `URF` | UV right sprout done |
| `0x07` | `WCNT` | water room tray count, 16-bit |
| `0x08` | `WLF` | water room left growth flag |
| `0x09` | `WRF` | water room right growth flag |
| `0x0A` | `FF` | fix flag |
| `0x0B` | `UEF` | UV event flag |
| `0x0C` | `WEF` | water event flag |
| `0x0D` | `HF` | harvest flag |
| `0x0E` | `C1F` | conveyor1 flag |
| `0x0F` | `C2F` | conveyor2 flag |
| `0x10` | `ESTOP` | emergency stop |
| `0x11` | `RESET` | reset |
| `0x12` | `HMF` | home flag used by master/SCARA route |
| `0x20` | `STATE` | device state report |
| `0x21` | `DONE` | done report |
| `0x22` | `ERR` | error report |

