#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <TimerFour.h>
#include <TimerFive.h>
#include <math.h>

extern const float cpr;
extern const float en_cnt;
extern volatile bool j1_run, j2_run, j3_run, j4_run;
extern const long PULSES_PER_REV;
extern float J2_LEAD_MM_PER_REV;
extern unsigned long J2_DEFAULT_PPS;
extern volatile bool j2_endstop_hit;
extern volatile long j2_target_steps;
extern volatile long j2_step_count;
extern volatile bool j2_move_done;

typedef struct {
  uint8_t pinA;
  uint8_t pinB;
  volatile long pos;
} encod;

extern encod j1_enc;
//extern encod j2_enc;
extern encod j3_enc;
extern encod j4_enc;
extern float j2_pos_mm;

extern volatile bool j1pulseState;
//extern volatile bool j2pulseState;
extern volatile bool j3pulseState;
extern volatile bool j4pulseState;

extern volatile long pulseInterval;

static const unsigned long J1_DEFAULT_MAX_PPS = 2500UL;
static const unsigned long J3_DEFAULT_MAX_PPS = 5500UL;

float encoder_getAngleDeg(const encod* e);

void set_tim();
void set_int();
///////////////////////////////////////////////////////////////////////////////////
void j1stepPulse();
void j1EncoderA();
bool move_j1(float targetAngle, unsigned long max_pps = J1_DEFAULT_MAX_PPS , float tolDeg = 0.3f);   
bool move_j1_wait(float targetAngle,
                  unsigned long max_pps = J1_DEFAULT_MAX_PPS,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 80000);
void enc_reset_j1();
void j1_home_stop_on_switch(bool dir_to_switch, unsigned long pps);
///////////////////////////////////////////////////////////////////////////////////
void j2stepPulse();
void j2_set_pps(unsigned long pps);
bool j2_home_stop_on_switch(bool toward_home, unsigned long pps);
bool j2_home_precise(bool toward_home, unsigned long fast_pps, unsigned long slow_pps);
void stop_j2_motion(bool disable_after = false);
bool is_j2_endstop_pressed();
void move_j2_continuous(bool dir, unsigned long pps);
bool move_j2(float deg, unsigned long pps);
///////////////////////////////////////////////////////////////////////////////////
void j3stepPulse();
void j3EncoderA();
bool move_j3(float targetAngle, unsigned long max_pps = J3_DEFAULT_MAX_PPS , float tolDeg = 0.3f);
bool move_j3_wait(float targetAngle,
                  unsigned long max_pps = J3_DEFAULT_MAX_PPS,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000);
void enc_reset_j3();
void j3_home_stop_on_switch(bool dir_to_switch, unsigned long pps);
///////////////////////////////////////////////////////////////////////////////////
void j4stepPulse();
void j4EncoderA();
void move_j4_wait(float targetAngle,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000);

bool move_j4(float targetAngle, float tolDeg = 0.3f);
void enc_reset_j4();
bool j4_home_stop_on_switch_safe(bool dir_to_switch, unsigned long pps);

///////////////////////////////////////////////////////////////////////////////////
void j1_enable(bool on);
void j3_enable(bool on);
void j4_enable(bool on);
void motors_enable_all(bool on);

void enc_reset_all();

float j1_getJointDeg();
float j3_getJointDeg();
float j4_getJointDeg();

void railStepPulse();
void rail_set_pps(unsigned long pps);
void moveRail(unsigned long pps, bool dir);
void stopRail(bool disable_after = false);
bool moveRail_untilStop(bool dir, unsigned long pps,
                        uint8_t stop_pin,
                        unsigned long timeout_ms = 80000);

/*
// ===== J4 open-loop (엔코더 미사용) =====
void j4_set_home_zero_deg(float deg = 0.0f);
float j4_get_openloop_deg();
bool move_j4_openloop(float target_deg, unsigned long pps = 1200);
bool move_j4_openloop_rel(float delta_deg, unsigned long pps = 1200);
void stop_j4_openloop(bool disable_after = false);
bool j4_home_openloop(bool dir_to_switch, unsigned long pps = 1200);
*/
void j4stepPulse();
void j4_move(bool dir, unsigned long pps);
void j4_stop();
void j4_home();

// ===== J1 open-loop, 엔코더 미사용 =====
void j1_ol_move(bool dir, unsigned long pps);
void j1_ol_stop(bool disable_after = false);
bool j1_ol_move_steps(bool dir,
                      long steps,
                      unsigned long pps,
                      unsigned long timeout_ms = 8000);
bool j1_ol_move_deg(float delta_joint_deg,
                    unsigned long pps = 2000,
                    unsigned long timeout_ms = 8000);

// ===== J3 open-loop, 엔코더 미사용 =====
void j3_ol_move(bool dir, unsigned long pps);
void j3_ol_stop(bool disable_after = false);
bool j3_ol_move_steps(bool dir,
                      long steps,
                      unsigned long pps,
                      unsigned long timeout_ms = 8000);
bool j3_ol_move_deg(float delta_joint_deg,
                    unsigned long pps = 3000,
                    unsigned long timeout_ms = 8000);

#endif
