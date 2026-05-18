#include "Fsm.h"
#include <stdio.h>

void FSM_Init(FSM_t *fsm, State_t *initialState)
{
    fsm->currentState = initialState;
    fsm->nextState = NULL;
    if (fsm->currentState->stateEnter != NULL)
    {
        fsm->currentState->stateEnter();
    }
}

void FSM_Run(FSM_t *fsm)
{
    if (fsm->nextState != NULL)
    {
        if (fsm->currentState->stateExit)
        {
            fsm->currentState->stateExit();
        }
        fsm->currentState = fsm->nextState;
        fsm->nextState = NULL;
        if (fsm->currentState->stateEnter)
        {
            fsm->currentState->stateEnter();
        }
    }
    if (fsm->currentState->stateRun)
    {
        fsm->currentState->stateRun();
    }
}

void FSM_Transition(FSM_t *fsm, State_t *nextState)
{
    fsm->nextState = nextState;
}

#ifdef __cplusplus

FSM::FSM() : currentState(nullptr), nextState(nullptr) {}

void FSM::init(State_t *initialState) {
    currentState = initialState;
    nextState = nullptr;
    if (currentState && currentState->stateEnter) {
        currentState->stateEnter();
    }
}

void FSM::run() {
    if (nextState != nullptr) {
        if (currentState && currentState->stateExit) {
            currentState->stateExit();
        }
        currentState = nextState;
        nextState = nullptr;
        if (currentState && currentState->stateEnter) {
            currentState->stateEnter();
        }
    }
    if (currentState && currentState->stateRun) {
        currentState->stateRun();
    }
}

void FSM::transition(State_t *nextState) {
    this->nextState = nextState;
}

#endif
