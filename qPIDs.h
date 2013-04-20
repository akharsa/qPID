/* 
 * The MIT License (MIT)
 * Copyright (c) <2012> <Alan Kharsansky>

 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the "Software"), to deal 
 * in the Software without restriction, including without limitation the rights 
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
 * copies of the Software, and to permit persons to whom the Software is furnished 
 * to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all 
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. 
*/

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
	AUTOMATIC,
	RELAY
} qPID_Mode;

typedef struct{
	float PV_old;
	float Ui_old;
	float Ud_old;
	float SP_old;
} qPID_Context;

typedef struct{

	// Parameters:
	float K, Ti, Td;	// For use in NON-INT or INT modes

	float OutputMax;	// For windup
	float OutputMin;	// For windup

	float Nd;			// For derivator
	float b, c;			// For setpoint Weighting

	float Ts;			// General propoerty
	
	// Features:
	qPID_Mode			Mode;
	qPID_Feature 		AntiWindup;
	qPID_Feature 		Bumpless;

	qPID_Context		ctx;

} qPID;

//================================================================
// Defines
//================================================================
#define	EPSILON	0.0000001

//================================================================
// Prototypes
//================================================================
void qPID_Init(qPID * q);
float qPID_Process_(qPID * q, float Input, float ProcessVariable, float terms[]);
#define qPID_Procees(pPID,input,pv) qPID_Process_(pPID,input,pv,NULL);

#endif
