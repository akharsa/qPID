
#include "qPIDs.h"
#include <stdio.h>


void qPID_Init(qPID * q){
	q->ctx.Ui_old = 0.0;
	q->ctx.Ud_old = 0.0;
	q->ctx.PV_old = 0.0;
}

float qPID_Process(qPID * q, float PV){

	// For local use
	float ControllerOutput;
	float Up, Ui, Ud;
	float Kp, Ki, Kd;


	switch (q->Architecture){
		case NON_INTERACTING:
			Kp = q->K;
			Ki = ((q->K) * (q->Ts) )/ (q->Ti);
			Kd = (q->K)*(q->Td);
			break;

		case INTERACTING:
			Kp = (q->K)*(1+(q->Td)/(q->Ti));
			Ki = ((q->K) * (q->Ts) )/ (q->Ti);
			Kd = (q->K)*(q->Td);
			break;

		case PARALLEL:
			Kp = q->Kp;
			Ki = q->Ki;
			Kd = q->Kd;
			break;
	}


	// Proportional gain
	Up = Kp*( ( (q->b)*(q->SetPoint) ) -PV);

	// Deriative gain with filter
	// FIXME: Derivative not implemented
	//Ud = Td/(Td + N*Ts) * (Ud_old - K*N*(PV-PV_old));
	//Ud = q->Td
	Ud = 0.0;

	// Calculate controler output for Automatic or manual mode
	// FIXME: No bumpless transition
	if (q->Mode == MANUAL){
		ControllerOutput = q->ManualInput;
	}else if (q->Mode == AUTOMATIC){
		ControllerOutput = Up + Ui + Ud;
	}

	// Calc de integral for the next step
	// FIXME: No antiwindup guard
	Ui = q->ctx.Ui_old + Ki*((q->SetPoint)-PV);

	// Save context for next step.
	q->ctx.Ui_old = Ui;
	q->ctx.Ud_old = Ud;
	q->ctx.PV_old = PV;

	return ControllerOutput;
}
