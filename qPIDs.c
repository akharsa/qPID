
#include "qPIDs.h"
#include <stdio.h>


void qPID_Init(qPID * q){
	q->ctx.Ui_old = 0.0;
	q->ctx.Ud_old = 0.0;
	q->ctx.PV_old = 0.0;
}

float qPID_Process(qPID * q, float PV){
	float ControllerOutput;
	float Up, Ui, Ud;

	float Ts = q->Ts;
	float K = q->K;
	float N = q->N;
	float Ti = q->Ti;
	float Td = q->Td;
	float b = q->b;
	float c = q->c;
	float sp = q->SetPoint;

	float Ui_old = (q->ctx).Ui_old;
	float Ud_old = (q->ctx).Ud_old;
	float PV_old = (q->ctx).PV_old;

	Up = K*(sp-PV);
	Ud = Td/(Td + N*Ts) * (Ud_old - K*N*(PV-PV_old));
	Ui = Ui_old+K*Ts/Ti*(sp-PV);

	q->ctx.Ui_old = Ui;
	q->ctx.Ud_old = Ud;
	q->ctx.PV_old = PV;

	ControllerOutput = Up + Ui + Ud;

	return ControllerOutput;
}
