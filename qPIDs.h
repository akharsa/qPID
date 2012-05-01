#ifndef __qPID_CONTROLLER_H__
#define __qPID_CONTROLLER_H__

#include <stdint.h>

//================================================================
// Types
//================================================================

typedef enum{
	DISABLED=0,
	ENABLED
} qPID_Feature;

typedef enum{
	MANUAL=0,
	AUTOMATIC
} qPID_Mode;

typedef struct{
	float PV_old;
	float Ui_old;
	float Ud_old;

} qPID_Context;

typedef struct{

	// Parameters:
	float K, Ti, Td;	// For use in NON-INT or INT modes

	float OutputMax;	// For windup
	float OutputMin;	// For windup

	float N;			// For derivator
	float b, c;			// For setpoint Weighting

	float Ts;			// General propoerty
	
	// Features:
	qPID_Mode			Mode;
	qPID_Feature 		AntiWindup;
	qPID_Feature 		Bumpless;

	qPID_Context		ctx;

} qPID;

//================================================================
// Prototypes
//================================================================
void qPID_Init(qPID * q);
float qPID_Process(qPID * q, float Input, float ProcessVariable, float terms[]);

#endif
