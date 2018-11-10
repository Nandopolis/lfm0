/*
 * fsme.h
 *
 *  Created on: Oct 20, 2018
 *      Author: nando
 */

#ifndef FSME_H_
#define FSME_H_

#include <stdint.h>

#define FSME_DISABLE 0
#define FSME_ENABLE 1
#define FSME_EVENT_TRUE 1
#define FSME_EVENT_FALSE 0
#define FSME_STATE_CHANGED 1
#define FSME_STATE_NOT_CHANGED 0

// function pointer type – for action (output) functions
typedef void (*FSME_PF) (void);

// function pointer type – for event update functions
typedef uint8_t (*FSME_PF_EV) (void);

// transition type
// describes a transition using the following fields:
// Event - a pointer to an event (input) function
// NextState - the next state for the transition
typedef struct {
	FSME_PF_EV Event;
	uint8_t NextState;
} FSME_TRANS;

// state type
// describes a FSM state using the following fields:
// Action - a pointer to an action (output) function;
// the outputs of the FSM that are activated in this state
// TransNO - the number of transitions from the state
// Trans - an array that contains the actual transitions information: pairs (event, next state)
typedef struct {
	FSME_PF Action;
	uint8_t TransNO;
	FSME_TRANS * Trans;
} FSME_STATE;

// FSM type
// describes a FSM using:
// Enable - a flag that indicates the state of FSM: enabled or disabled
// CurrentState - the current state of the FSM
// StatesNO - the number of states of the FSM
// StateChanged - flag that indicates a state change
// States - an array that contains the states and transitions of the FSM
// TransNO - the number of transitions from current state
// Trans - a reference to an array with the transitions form current state
typedef struct {
	uint8_t Enable;
	uint8_t CurrentState;
	uint8_t StatesNO;
	uint8_t StateChanged;
	FSME_STATE * States;
	uint8_t TransNO;
	FSME_TRANS * Trans;
} FSME_FSM;

static FSME_TRANS * _t;
static FSME_STATE * _s;

static void FSME_UpdateState( FSME_FSM * F );
void FSM_Run( FSME_FSM * F );
static void FSME_Action( FSME_FSM * F );
void FSM_Enable( FSME_FSM * F );
void FSM_Disable( FSME_FSM * F );

static void FSME_UpdateState( FSME_FSM * F )
{
	uint8_t _i = 0;
	uint8_t _n;
	// set a variable to point to current transition table
	_t = F->Trans;
	// set a variable to current value of transitions count for current state
	_n = F->TransNO;
	// loop for all possible transitions from current state
	for (_i = 0 ; _i < _n ; _i++ )
	{
		// check if the events have occurred (conditions are true for a transition to take place)
		if ( FSME_EVENT_TRUE == _t[ _i ].Event() )
		{
			// update current state
			F->CurrentState = _t[ _i ].NextState;
			// get a pointer to current state
			_s = & ( F->States[ F->CurrentState ] );
			// update current transition table according to current state
			F->Trans = _s->Trans;
			// update current transitions number according to current state
			F->TransNO = _s->TransNO;
			// set state changed flag
			F->StateChanged = FSME_STATE_CHANGED;
			// leave the for loop and function
			break;
		}
	}
}

void FSM_Run( FSME_FSM * F )
{
	if ( FSME_DISABLE == F->Enable )
	{
		// TODO: may reset the FSM into initial state and deactivate outputs
		return;
	}
	FSME_UpdateState( F );
	FSME_Action( F );
}

static void FSME_Action( FSME_FSM * F )
{
	F->States[F->CurrentState].Action();
}

void FSM_Enable( FSME_FSM * F )
{
	F->Enable = FSME_ENABLE;
}

void FSM_Disable( FSME_FSM * F )
{
	F->Enable = FSME_DISABLE;
}

#endif /* FSME_H_ */
