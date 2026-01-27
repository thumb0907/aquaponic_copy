#ifndef SET_MOTOR_H
#define SET_MOTOR_H

#include <Arduino.h>

//베이스
#define j1_pul  4
#define j1_dir  5
#define j1_en   6
#define j1_A    18
#define j1_B    8
#define j1_Z    9
#define stop_j1 52

//z축
#define STEP_PIN  37
#define DIR_PIN  27
#define j2_en   28
#define PEND_PIN    21
#define ALM_PIN    29
#define stop_z 3

//중간 관절
#define j3_pul  10
#define j3_dir  11
#define j3_en   12
#define j3_A    19
#define j3_B    14
#define j3_Z    15
#define stop_j3 50

//엔드이펙터 회전
#define j4_pul  16
#define j4_dir  17
#define j4_en   23
#define j4_A    20
#define j4_B    24
#define j4_Z    25
#define stop_j4 48

static inline void motor_pin()
{
  pinMode(j1_pul, OUTPUT); pinMode(j1_dir, OUTPUT); pinMode(j1_en, OUTPUT);
  pinMode(j3_pul, OUTPUT); pinMode(j3_dir, OUTPUT); pinMode(j3_en, OUTPUT);
  pinMode(j4_pul, OUTPUT); pinMode(j4_dir, OUTPUT); pinMode(j4_en, OUTPUT);

  digitalWrite(j1_en, HIGH);
  digitalWrite(j3_en, HIGH);
  digitalWrite(j4_en, HIGH);


  pinMode(j1_A, INPUT_PULLUP); pinMode(j1_B, INPUT_PULLUP);
  pinMode(j3_A, INPUT_PULLUP); pinMode(j3_B, INPUT_PULLUP);
  pinMode(j4_A, INPUT_PULLUP); pinMode(j4_B, INPUT_PULLUP);
  
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, HIGH);

  pinMode(PEND_PIN, INPUT_PULLUP);
  pinMode(ALM_PIN, INPUT_PULLUP);

  pinMode(stop_z, INPUT_PULLUP);
  pinMode(stop_j1, INPUT_PULLUP);
  pinMode(stop_j3, INPUT_PULLUP);
  pinMode(stop_j4, INPUT_PULLUP);
}

#endif
