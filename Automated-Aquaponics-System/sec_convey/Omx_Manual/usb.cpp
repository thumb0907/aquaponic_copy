#include "usb.h"
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

static Stream* g_port = nullptr;

static char   g_line[48];
static size_t g_idx = 0;

static float    g_last_angle = 0.0f;
static uint32_t g_last_rx_ms = 0;

static void resetLine()
{
  g_idx = 0;
  g_line[0] = '\0';
}

static bool parseAngleLine(const char* s, float &outAngle)
{
  // leading spaces 제거
  while (*s && isspace((unsigned char)*s)) s++;

  // "ANG" prefix 허용: ANG,  ANG:  ANG=
  if (strncmp(s, "ANG", 3) == 0) {
    s += 3;
    if (*s == ',' || *s == ':' || *s == '=') s++;
  }

  // 다시 spaces 제거
  while (*s && isspace((unsigned char)*s)) s++;

  char* endp = nullptr;
  float v = strtof(s, &endp);
  if (endp == s) return false;     // 변환 실패(숫자 없음)

  outAngle = v;
  return true;
}

void usb_begin(Stream &port)
{
  g_port = &port;
  resetLine();
  g_last_angle = 0.0f;
  g_last_rx_ms = 0;
}

bool usb_poll_angle(float &outAngleDeg)
{
  if (!g_port) return false;

  bool gotNew = false;

  while (g_port->available() > 0) {
    char c = (char)g_port->read();

    // CR 무시
    if (c == '\r') continue;

    // LF면 한 줄 완성
    if (c == '\n') {
      g_line[g_idx] = '\0';

      float ang = 0.0f;
      if (parseAngleLine(g_line, ang)) {
        g_last_angle = ang;
        g_last_rx_ms = millis();
        outAngleDeg = ang;
        gotNew = true;
      }

      resetLine();
      // 여기서 break 하지 않는 이유: 한 번에 여러 줄 들어올 수도 있어서 계속 처리
      continue;
    }

    // 일반 문자 누적(버퍼 오버플로우 방지)
    if (g_idx < sizeof(g_line) - 1) {
      g_line[g_idx++] = c;
    } else {
      // 너무 길면 버리고 리셋
      resetLine();
    }
  }

  return gotNew;
}

float usb_last_angle(void)
{
  return g_last_angle;
}

uint32_t usb_last_rx_ms(void)
{
  return g_last_rx_ms;
}
