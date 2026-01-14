#pragma once
#include <Arduino.h>
#include <HardwareTimer.h>

// OpenCR: HardwareTimer Timer(TIMER_CH1) 패턴 사용
void step_begin(uint8_t pulPin, uint8_t dirPin, uint8_t enaPin,
                bool enaActiveLow = false);

void step_enable(bool on);
void step_setDir(bool dir);

// 1) 토글 주기(us) 직접 지정 (ISR에서 PUL을 토글)
//    실제 "스텝(상승엣지)" 속도는 대략 1 / (2*period) 입니다.
void step_setTogglePeriodUs(uint32_t periodUs);

// 2) 스텝 속도(steps/sec)로 지정 (내부에서 period 계산)
void step_setStepsPerSec(float stepsPerSec);

void step_start();
void step_stop();
void step_pin();