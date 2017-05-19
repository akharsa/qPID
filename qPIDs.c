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

#include <stdio.h>
#include <math.h>
#include <platform/lib/qPIDs.h>

void qPID_Init(qPID *q)
{
    q->ctx.Ui_old = 0.0;
    q->ctx.Ud_old = 0.0;
    q->ctx.PV_old = 0.0;
    q->ctx.SP_old = 0.0;
    q->Mode = OFF;
}

float qPID_Process_(qPID *q, float Input, float PV, float terms[])
{


    // =====================================
    //   MANUAL
    // =====================================
    // Input = manual input -> CO

    // =====================================
    //   AUTOMATIC
    // =====================================
    // Input = Setpoint

    // For local use
    float ControllerOutput;
    float Up, Ui, Ud;
    float Kp, Ki, Kd_a, Kd_b, Kd_c;


    if (fabsf(q->Ti) < EPSILON) {
        q->Ti = EPSILON;
    }
    if (fabsf(q->Nd) < EPSILON) {
        q->Nd = EPSILON;
    }
    Kp = q->K;
    Ki = ((q->K) * (q->Ts)) / (q->Ti);
    Kd_a = q->Td / (q->Td + q->Nd * q->Ts) ;
    Kd_b = (q->K * q->Td * q->Nd) / (q->Td + q->Nd * q->Ts);
    Kd_c = (q->c * q->K * q->Td * q->Nd) / (q->Td + q->Nd * q->Ts);


    // Proportional gain
    Up = Kp * (((q->b) * (Input)) - PV);

    // Deriative gain with filter
    Ud = Kd_a * (q->ctx.Ud_old) - Kd_b * (PV - q->ctx.PV_old) + Kd_c * (Input - q->ctx.SP_old);

    // Get last integral
    Ui =  q->ctx.Ui_old;

    // Calculate controler output for Automatic or manual mode

    switch (q->Mode) {
        case MANUAL:
            ControllerOutput = Input;

            if (q->Bumpless == ENABLED) {
                q->ctx.Ui_old = PV;
            }

            break;

        case AUTOMATIC:
            ControllerOutput =  Up + Ui + Ud;
            if (ControllerOutput > q->OutputMax) {
                ControllerOutput = q->OutputMax;
            } else if (ControllerOutput < q->OutputMin) {
                ControllerOutput = q->OutputMin;
            }
            break;

        case RELAY:
            if ((Input - PV) >= 0) {
                ControllerOutput = q->OutputMax;
            } else {
                ControllerOutput = q->OutputMin;
            }
            break;
        case OFF:
            ControllerOutput = 0;
            break;

        default:
            // ERROR
            ControllerOutput = NAN;
            break;
    }

    // Output parameters for debug
    if (terms != NULL) {
        terms[0] = Up;
        terms[1] = Ui;
        terms[2] = Ud;
    }

    // Anti Windup
    if ((q->AntiWindup == ENABLED) && (q->Mode == AUTOMATIC)) {
        //if (Ui >= q->OutputMax) {
        if (ControllerOutput >= q->OutputMax) {
            // do not integrate anymore
        } else if (ControllerOutput <= q->OutputMin) {
            // do not integrate anymore
        } else {
            Ui = q->ctx.Ui_old + Ki * ((Input) - PV);
        }
    } else {
        // Calc de integral for the next step in any other case
        Ui = q->ctx.Ui_old + Ki * ((Input) - PV);
    }

    // Save context for next step.
    q->ctx.Ui_old = Ui;
    q->ctx.Ud_old = Ud;
    q->ctx.PV_old = PV;
    q->ctx.SP_old = Input;

    return ControllerOutput;
}
