#ifndef __qPID_CONTROLLER_H__
#define __qPID_CONTROLLER_H__

#include <stdint.h>

//================================================================
// Types
//================================================================

typedef enum{
	INTERACTING,	 	// K, Ti, Td
	NON_INTERACTING 	// Kp, Ki, Kd
} qPID_Architecture;

typedef enum{
	ENABLED,
	DISABLED
} qPID_Feature;

typedef enum{
	MANUAL,
	AUTOMATIC
} qPID_Mode;

typedef struct{

	//float input_old;
	//float output_old;
	//float error_old;

	float PV_old;
	//float CO_old;

	float Ui_old;
	float Ud_old;

} qPID_Context;

typedef struct{

	// Constants
	float K, Ti, Td;
	float N;
	float b, c;
	float Ts;

	// Inputs:
	float SetPoint; 	// For auto mode
	float ManualInput;		// For manual mode
	
	// AntiWindup:
	float OutputMax;
	float OutputMin;
	
	// Features:
	qPID_Feature 		AntiWindup;
	qPID_Feature 		Bumpless;
	qPID_Mode			Mode;
	//qPID_Architecture	Architecture;

	qPID_Context		ctx;
} qPID;

//================================================================
// Prototypes
//================================================================
void qPID_Init(qPID * q);
float qPID_Process(qPID * q, float ProcessVariable);

#endif
