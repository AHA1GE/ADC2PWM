/**
 * @file               Fsm.h
 * @author             Harry
 * @brief              通用状态机头文件
 *
 *
 **************************************************************************************************
 * @attention
 * Copyright (c) ZT MODEL Co.,Ltd. All rights reserved.
 *
 **************************************************************************************************
 */

/**
 * Usage:
 * State_t stateExample = {
 *     .stateName = "Example State",
 *     .stateEnter = stateExampleEnter,
 *     .stateRun = stateExampleRun,
 *     .stateExit = stateExampleExit}
 * void stateExampleEnter(void){//do something when entering this state, called once}
 * void stateExampleRun(void){//do something when running this state, called repeatedly until state transition}
 * void stateExampleExit(void){//do something when exiting this state, called once}
 * FSM_t fsm;
 * FSM_Init(&fsm, &stateExample);
 * void appLogic(void){
 *     if( something ){
 *        FSM_Transition(&fsm, &stateAnother);
 *     }
 * }
 * while (1)
 * {
 *    FSM_Run(&fsm);
 *    appLogic();
 * }
 */

#ifndef FSM_H
#define FSM_H

typedef struct
{
    const char *stateName;
    void (*stateEnter)(void);
    void (*stateRun)(void);
    void (*stateExit)(void);
} State_t;

typedef struct
{
    State_t *currentState;
    State_t *nextState;
} FSM_t;

void FSM_Init(FSM_t *fsm, State_t *initialState);
void FSM_Run(FSM_t *fsm);
void FSM_Transition(FSM_t *fsm, State_t *nextState);

#ifdef __cplusplus

class FSM
{
private:
    State_t *currentState;
    State_t *nextState;

public:
    FSM();
    void init(State_t *initialState);
    void run();
    void transition(State_t *nextState);
};

#endif /* __cplusplus */

#endif /* FSM_H */
