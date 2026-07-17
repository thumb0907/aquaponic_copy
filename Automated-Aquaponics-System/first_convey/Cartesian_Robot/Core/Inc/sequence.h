#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <stdbool.h>

typedef enum {
    SYS_IDLE = 0,
    SYS_HOMING,
    SYS_RUN_CYCLE,
    SYS_WAIT_SCARA_PICK,
    SYS_ERROR
} SystemState;

void Sequence_Init(void);
void Sequence_Task(void);

void Sequence_EStop(void);
void Sequence_ResetError(void);


SystemState Sequence_GetState(void);
bool Sequence_IsError(void);

#endif
