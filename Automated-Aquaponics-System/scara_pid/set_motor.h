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

//z축
#define j2_pul  26
#define j2_dir  27
#define j2_en   28
#define j2_A    21
#define j2_B    29
#define j2_Z    30

//중간 관절
#define j3_pul  10
#define j3_dir  11
#define j3_en   12
#define j3_A    19
#define j3_B    14
#define j3_Z    15

//엔드이펙터 회전
#define j4_pul  16
#define j4_dir  17
#define j4_en   23
#define j4_A    20
#define j4_B    24
#define j4_Z    25

static inline void motor_pin()
{
  pinMode(j1_pul, OUTPUT); pinMode(j1_dir, OUTPUT); pinMode(j1_en, OUTPUT);
  pinMode(j2_pul, OUTPUT); pinMode(j2_dir, OUTPUT); pinMode(j2_en, OUTPUT);
  pinMode(j3_pul, OUTPUT); pinMode(j3_dir, OUTPUT); pinMode(j3_en, OUTPUT);
  pinMode(j4_pul, OUTPUT); pinMode(j4_dir, OUTPUT); pinMode(j4_en, OUTPUT);

  digitalWrite(j1_en, HIGH);
  digitalWrite(j2_en, HIGH);
  digitalWrite(j3_en, HIGH);
  digitalWrite(j4_en, HIGH);


  pinMode(j1_A, INPUT_PULLUP); pinMode(j1_B, INPUT_PULLUP);
  pinMode(j2_A, INPUT_PULLUP); pinMode(j2_B, INPUT_PULLUP);
  pinMode(j3_A, INPUT_PULLUP); pinMode(j3_B, INPUT_PULLUP);
  pinMode(j4_A, INPUT_PULLUP); pinMode(j4_B, INPUT_PULLUP);
}

#endif
