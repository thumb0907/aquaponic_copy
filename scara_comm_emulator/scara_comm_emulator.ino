#include <Arduino.h>

/*
 * SCARA communication emulator
 *
 * Pi2 serial protocol:
 *   [SOF] [PID] [LEN] [DATA ...] [CHECKSUM]
 *
 * SOF      = 0xAA
 * CHECKSUM = (PID + LEN + sum(DATA)) & 0xFF
 *
 * Serial 출력은 바이너리 통신 전용이다.
 * Serial.print(), Serial.println()을 사용하면 안 된다.
 */

// ============================================================
// 1. 통신 설정
// ============================================================

static const uint32_t SERIAL_BAUD = 115200;

static const uint8_t SOF = 0xAA;
static const uint8_t MAX_DATA_LEN = 16;

// ============================================================
// 2. Master/Pi2/SCARA 공통 PID
// ============================================================

static const uint8_t PID_SSF       = 0x01;
static const uint8_t PID_SMF       = 0x02;
static const uint8_t PID_CRF       = 0x03;
static const uint8_t PID_UV        = 0x04;

static const uint8_t PID_S2F       = 0x14;
static const uint8_t PID_S3F       = 0x15;
static const uint8_t PID_SCARA_SRC = 0x16;
static const uint8_t PID_SCARA_DST = 0x17;

static const uint8_t PID_ESTOP     = 0x10;
static const uint8_t PID_RESET     = 0x11;
static const uint8_t PID_HMF       = 0x12;

static const uint8_t PID_STATE     = 0x20;

// Pi2의 SCARA_STATE_STR과 일치
static const uint8_t STATE_IDLE  = 0x01;
static const uint8_t STATE_MOVING = 0x02;
static const uint8_t STATE_ESTOP = 0x04;

// ============================================================
// 3. 가상 동작 시간
// ============================================================

// 실제 로봇 대신 기다렸다가 완료 신호를 보낸다.
static const unsigned long HOME_TIME_MS  = 2000;
static const unsigned long SECT1_TIME_MS = 10000;
static const unsigned long SECT2_TIME_MS = 10000;

// 수경실 → 진동부 → C2 전체 시간을 흉내 낸다.
static const unsigned long SECT3_TIME_MS = 15000;

// ============================================================
// 4. 수신 파서 상태
// ============================================================

enum RxState : uint8_t {
  RX_WAIT_SOF,
  RX_WAIT_PID,
  RX_WAIT_LEN,
  RX_WAIT_DATA,
  RX_WAIT_CHECKSUM
};

static RxState rxState = RX_WAIT_SOF;

static uint8_t rxPid = 0;
static uint8_t rxLen = 0;
static uint8_t rxData[MAX_DATA_LEN];
static uint8_t rxIndex = 0;

// ============================================================
// 5. 가상 SCARA 동작 상태
// ============================================================

enum SimAction : uint8_t {
  ACTION_NONE,
  ACTION_HOME,
  ACTION_SECT1,
  ACTION_SECT2,
  ACTION_SECT3
};

static SimAction currentAction = ACTION_NONE;

static unsigned long actionStartedAt = 0;
static unsigned long actionDuration = 0;

// Master가 보낸 슬롯 위치 보관
static uint8_t scaraSource = 0;
static uint8_t scaraDestination = 0;
static uint16_t uvSelector = 0;

// 상승 에지 중복 실행 방지
static uint8_t previousHmf = 0;
static uint8_t previousSsf = 0;
static uint8_t previousS2f = 0;
static uint8_t previousS3f = 0;

// ============================================================
// 6. 프레임 송신 함수
// ============================================================

uint8_t calculateChecksum(
  uint8_t pid,
  uint8_t length,
  const uint8_t* data
) {
  uint16_t sum = pid + length;

  for (uint8_t i = 0; i < length; i++) {
    sum += data[i];
  }

  return (uint8_t)(sum & 0xFF);
}

void sendFrame(
  uint8_t pid,
  const uint8_t* data,
  uint8_t length
) {
  uint8_t checksum = calculateChecksum(
    pid,
    length,
    data
  );

  Serial.write(SOF);
  Serial.write(pid);
  Serial.write(length);

  for (uint8_t i = 0; i < length; i++) {
    Serial.write(data[i]);
  }

  Serial.write(checksum);
  Serial.flush();
}

void sendU8(uint8_t pid, uint8_t value) {
  uint8_t data[1] = {value};

  sendFrame(
    pid,
    data,
    1
  );
}

// ============================================================
// 7. 가상 동작 시작
// ============================================================

void startAction(
  SimAction newAction,
  unsigned long duration
) {
  // 동시에 두 동작을 시작하지 않는다.
  if (currentAction != ACTION_NONE) {
    return;
  }

  currentAction = newAction;
  actionStartedAt = millis();
  actionDuration = duration;

  digitalWrite(LED_BUILTIN, HIGH);

  // SCARA 구동 중
  sendU8(PID_SMF, 1);

  // 선택 사항이지만 모니터 확인에 도움이 된다.
  sendU8(PID_STATE, STATE_MOVING);
}

// ============================================================
// 8. 가상 동작 완료
// ============================================================

void finishCurrentAction() {
  switch (currentAction) {
    case ACTION_HOME:
      /*
       * Master는 HMF=0을 받아야
       * scara_prehome_done=True로 변경한다.
       */
      sendU8(PID_HMF, 0);
      previousHmf = 0;
      break;

    case ACTION_SECT1:
      /*
       * 파종 트레이를 발아실에 놓았다고 가정한다.
       * Master는 SSF=0을 작업 완료로 처리한다.
       */
      sendU8(PID_SSF, 0);
      previousSsf = 0;
      break;

    case ACTION_SECT2:
      /*
       * 발아실에서 수경재배실로 이동 완료.
       */
      sendU8(PID_S2F, 0);
      previousS2f = 0;
      break;

    case ACTION_SECT3:
      /*
       * 수경재배실 → 진동부 → C2 적재 완료.
       *
       * Master가 S3F=0을 받으면
       * STM2에 FF=1을 전송한다.
       */
      sendU8(PID_S3F, 0);
      previousS3f = 0;
      break;

    case ACTION_NONE:
    default:
      return;
  }

  // SCARA 구동 종료
  sendU8(PID_SMF, 0);
  sendU8(PID_STATE, STATE_IDLE);

  currentAction = ACTION_NONE;
  actionStartedAt = 0;
  actionDuration = 0;

  digitalWrite(LED_BUILTIN, LOW);
}

// ============================================================
// 9. 리셋 및 긴급정지
// ============================================================

void resetSimulator() {
  currentAction = ACTION_NONE;
  actionStartedAt = 0;
  actionDuration = 0;

  scaraSource = 0;
  scaraDestination = 0;
  uvSelector = 0;

  previousHmf = 0;
  previousSsf = 0;
  previousS2f = 0;
  previousS3f = 0;

  digitalWrite(LED_BUILTIN, LOW);

  sendU8(PID_STATE, STATE_IDLE);
}

void emergencyStopSimulator() {
  currentAction = ACTION_NONE;
  actionStartedAt = 0;
  actionDuration = 0;

  digitalWrite(LED_BUILTIN, LOW);

  sendU8(PID_SMF, 0);
  sendU8(PID_STATE, STATE_ESTOP);
}

// ============================================================
// 10. 정상 프레임 처리
// ============================================================

void handleFrame(
  uint8_t pid,
  const uint8_t* data,
  uint8_t length
) {
  // RESET과 ESTOP은 DATA가 없는 프레임이다.
  if (pid == PID_RESET && length == 0) {
    resetSimulator();
    return;
  }

  if (pid == PID_ESTOP && length == 0) {
    emergencyStopSimulator();
    return;
  }

  // UV는 현재 마스터 코드에서 2바이트로 전송된다.
  if (pid == PID_UV && length == 2) {
    uvSelector =
      ((uint16_t)data[0] << 8)
      | data[1];

    return;
  }

  // 나머지 시험 대상 값은 1바이트다.
  if (length != 1) {
    return;
  }

  uint8_t value = data[0];

  switch (pid) {
    case PID_SCARA_SRC:
      scaraSource = value;
      break;

    case PID_SCARA_DST:
      scaraDestination = value;
      break;

    case PID_HMF:
      /*
       * HMF 0 → 1 상승 에지에서만 호밍 시작
       */
      if (
        previousHmf == 0
        && value == 1
      ) {
        startAction(
          ACTION_HOME,
          HOME_TIME_MS
        );
      }

      previousHmf = value;
      break;

    case PID_SSF:
      /*
       * 파종부 → 발아실
       */
      if (
        previousSsf == 0
        && value == 1
      ) {
        startAction(
          ACTION_SECT1,
          SECT1_TIME_MS
        );
      }

      previousSsf = value;
      break;

    case PID_S2F:
      /*
       * 발아실 → 수경재배실
       */
      if (
        previousS2f == 0
        && value == 1
      ) {
        startAction(
          ACTION_SECT2,
          SECT2_TIME_MS
        );
      }

      previousS2f = value;
      break;

    case PID_S3F:
      /*
       * 수경재배실 → 진동부 → C2
       */
      if (
        previousS3f == 0
        && value == 1
      ) {
        startAction(
          ACTION_SECT3,
          SECT3_TIME_MS
        );
      }

      previousS3f = value;
      break;

    default:
      // 다른 플래그는 시험에 필요하지 않으므로 저장하지 않는다.
      break;
  }
}

// ============================================================
// 11. 바이트 단위 수신 파서
// ============================================================

void receiveSerialTask() {
  while (Serial.available() > 0) {
    uint8_t value = (uint8_t)Serial.read();

    switch (rxState) {
      case RX_WAIT_SOF:
        if (value == SOF) {
          rxState = RX_WAIT_PID;
        }
        break;

      case RX_WAIT_PID:
        rxPid = value;
        rxState = RX_WAIT_LEN;
        break;

      case RX_WAIT_LEN:
        rxLen = value;
        rxIndex = 0;

        if (rxLen > MAX_DATA_LEN) {
          rxState = RX_WAIT_SOF;
        }
        else if (rxLen == 0) {
          rxState = RX_WAIT_CHECKSUM;
        }
        else {
          rxState = RX_WAIT_DATA;
        }
        break;

      case RX_WAIT_DATA:
        rxData[rxIndex++] = value;

        if (rxIndex >= rxLen) {
          rxState = RX_WAIT_CHECKSUM;
        }
        break;

      case RX_WAIT_CHECKSUM: {
        uint8_t expectedChecksum =
          calculateChecksum(
            rxPid,
            rxLen,
            rxData
          );

        if (value == expectedChecksum) {
          handleFrame(
            rxPid,
            rxData,
            rxLen
          );
        }

        rxState = RX_WAIT_SOF;
        rxIndex = 0;
        break;
      }
    }
  }
}

// ============================================================
// 12. Arduino 기본 함수
// ============================================================

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(SERIAL_BAUD);

  /*
   * Serial 연결 직후 텍스트를 출력하면 안 된다.
   * Pi2 BinaryParser가 사용하는 포트이기 때문이다.
   */
}

void loop() {
  // 통신 수신은 항상 수행한다.
  receiveSerialTask();

  // delay()를 사용하지 않아 동작 중에도 ESTOP/RESET 수신 가능
  if (currentAction != ACTION_NONE) {
    unsigned long elapsed =
      millis() - actionStartedAt;

    if (elapsed >= actionDuration) {
      finishCurrentAction();
    }
  }
}