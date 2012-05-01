
#include "qPIDs.h"
#include <stdio.h>


void qPID_Init(qPID * q){
	q->ctx.Ui_old = 0.0;
	q->ctx.Ud_old = 0.0;
	q->ctx.PV_old = 0.0;
}


//float qPID_Process(qPID * q, float PV){
//float qPID_Process(qPID * q, float Input, float PV ){
float qPID_Process(qPID * q, float Input, float PV, float terms[]){


	// For local use
	float ControllerOutput;
	float Up, Ui, Ud;
	float Kp, Ki, Kd_a, Kd_b;


	Kp = q->K;
	Ki = ((q->K) * (q->Ts) )/ (q->Ti);
	Kd_a = q->Td/(q->Td + q->N*q->Ts) ;
	Kd_b = (q->K*q->Td*q->N)/(q->Td + q->N*q->Ts);


	// Proportional gain
	Up = Kp*( ( (q->b)*(Input) ) - PV);

	// Deriative gain with filter
	Ud = Kd_a * (q->ctx.Ud_old) - Kd_b*(PV-q->ctx.PV_old);

	// Get last integral
	Ui =  q->ctx.Ui_old;


	// Calculate controler output for Automatic or manual mode

	if (q->Mode == MANUAL){
		ControllerOutput = Input;

		if (q->Bumpless == ENABLED){
			q->ctx.Ui_old = PV;
		}

	}else if (q->Mode == AUTOMATIC){
		ControllerOutput =  Up + Ui + Ud;
	}

	// Output parameters for debug
	if (terms!=NULL){
		terms[0] = Up;
		terms[1] = Ui;
		terms[2] = Ud;
	}

	// Calc de integral for the next step
	// FIXME: No antiwindup guard
	Ui = q->ctx.Ui_old + Ki*((Input)-PV);

	// Save context for next step.
	q->ctx.Ui_old = Ui;
	q->ctx.Ud_old = Ud;
	q->ctx.PV_old = PV;


	return ControllerOutput;
}
