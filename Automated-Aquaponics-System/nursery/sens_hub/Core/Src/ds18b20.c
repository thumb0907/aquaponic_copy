#include "ds18b20.h"

// 1-Wire 기본 동작 (PB0를 Open-Drain Output으로 계속 사용)
static inline void OW_Low(void)     { HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET); }
static inline void OW_Release(void) { HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET); } // OD라서 "놓기"
static inline uint8_t OW_Read(void) { return (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_SET); }

// reset + presence detect
static uint8_t OW_Reset(void)
{
  uint8_t presence;

  // 타이밍 깨질까봐 짧게 인터럽트 막고 통신
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  OW_Low();
  microDelay(500);       // reset low (>=480us)
  OW_Release();
  microDelay(70);        // wait 15~60us 후 presence 확인 타이밍 근처

  presence = !OW_Read(); // 센서가 있으면 presence동안 LOW
  microDelay(410);       // reset slot 마무리

  if (!primask) __enable_irq();
  return presence;       // 1이면 센서 존재
}

static void OW_WriteBit(uint8_t bit)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  OW_Low();
  if (bit) {
    microDelay(6);       // '1' 슬롯: 짧게 LOW
    OW_Release();
    microDelay(64);
  } else {
    microDelay(60);      // '0' 슬롯: 길게 LOW
    OW_Release();
    microDelay(10);
  }

  if (!primask) __enable_irq();
}

static uint8_t OW_ReadBit(void)
{
  uint8_t bit;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  OW_Low();
  microDelay(6);
  OW_Release();
  microDelay(9);         // 샘플 타이밍(대략 15us 근처)

  bit = OW_Read();
  microDelay(55);

  if (!primask) __enable_irq();
  return bit;
}

static void OW_WriteByte(uint8_t data)
{
  for (int i = 0; i < 8; i++) {
    OW_WriteBit(data & 0x01);
    data >>= 1;
  }
}

static uint8_t OW_ReadByte(void)
{
  uint8_t data = 0;
  for (int i = 0; i < 8; i++) {
    data >>= 1;
    if (OW_ReadBit()) data |= 0x80;
  }
  return data;
}

// Convert T 시작 (싱글 센서면 Skip ROM 사용)
uint8_t DS18B20_StartConversion(void)
{
  if (!OW_Reset()) return 0;

  OW_WriteByte(0xCC); // Skip ROM
  OW_WriteByte(0x44); // Convert T
  return 1;
}

// 온도 읽기
uint8_t DS18B20_ReadTemp(float *tempC)
{
  uint8_t lsb, msb;
  int16_t raw;

  if (!OW_Reset()) return 0;

  OW_WriteByte(0xCC); // Skip ROM
  OW_WriteByte(0xBE); // Read Scratchpad

  lsb = OW_ReadByte();
  msb = OW_ReadByte();

  raw = (int16_t)((msb << 8) | lsb);
  *tempC = (float)raw / 16.0f; // 기본 12-bit 기준 (LSB=0.0625°C)

  return 1;
}
