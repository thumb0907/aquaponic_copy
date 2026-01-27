#pragma once
#include <Arduino.h>

extern const int sw;
extern const int PUL;
extern const int DIR;
extern bool dir_state;

<<<<<<< HEAD
void stepPulse(bool dir, int speed);


//////////////////////////// 01.21 리니어레일 구동
void moveMM(bool dir, float mm, int speed);
void moveCM(bool dir, float cm, int speed);
////////////////////////////
=======
extern bool stop_req;

void STOP_ISR();
void STOP_NOW();
void setstep();
void stepPulse(bool dir, int speed);
>>>>>>> 82280afd5b99d0c136265fe565423a1b2d09f277
