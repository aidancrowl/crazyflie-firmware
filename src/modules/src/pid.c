/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * pid.c - implementation of the PID regulator
 */

#include "pid.h"
#include "num.h"
#include <math.h>
#include <float.h>

//pid control calculates an error value between a desired point and a measured value
//initialize all variables needed for pid control
void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  pid->error         = 0;
  pid->prevError     = 0;
  pid->integ         = 0; //in pid integral accounts for previous error values
  pid->deriv         = 0; //"anticipatory control" based on the error's current rate of change
  pid->desired       = desired; //desired point
  pid->kp            = kp;      //position gain
  pid->ki            = ki;      //integral gain
  pid->kd            = kd;      //derivative gain
  pid->iLimit        = DEFAULT_PID_INTEGRATION_LIMIT; //set in pid.h to 5000
  pid->outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;       //set in pid.h to 0 which means no limit
  pid->dt            = dt;
  pid->enableDFilter = enableDFilter;  //filter for derivative
  if (pid->enableDFilter) //pid controllers use LPF to limit HF gain & noise
  {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq); //in file filter.c, sets cutoff frequency
  }
}

//update pid values based on measured value
float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output = 0.0f;

    if (updateError)
    {
        pid->error = pid->desired - measured; //new error = desired - measured
    }

    pid->outP = pid->kp * pid->error; //proportional output = kp * error
    output += pid->outP;

    //set derivative by checking if we want to filter it and make sure it is a #
    float deriv = (pid->error - pid->prevError) / pid->dt; //derivative = current error - previous error /change in time (slope of error over time)
    if (pid->enableDFilter)
    {
      pid->deriv = lpf2pApply(&pid->dFilter, deriv); //calculates a delay element & checks to make sure it is finite so bad values don't go through filter
    } else {                                         //if yes sets delay element to derivative and returns it
      pid->deriv = deriv;
    }
    if (isnan(pid->deriv)) {
      pid->deriv = 0;
    }
    pid->outD = pid->kd * pid->deriv;  //derivative output = kd * derivative
    output += pid->outD;


    //compute integral value (proportional to error and duration of error)
    pid->integ += pid->error * pid->dt;

    // Constrain the integral (unless the iLimit is zero)
    if(pid->iLimit != 0)
    {
    	pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);
    }

    pid->outI = pid->ki * pid->integ;
    output += pid->outI;

    // Constrain the total PID output (unless the outputLimit is zero)
    if(pid->outputLimit != 0)
    {
      output = constrain(output, -pid->outputLimit, pid->outputLimit);
    }


    pid->prevError = pid->error;

    return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}


void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
  pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
  return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
  bool isActive = true;

  if (pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f)
  {
    isActive = false;
  }

  return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}
void pidSetDt(PidObject* pid, const float dt) {
    pid->dt = dt;
}
