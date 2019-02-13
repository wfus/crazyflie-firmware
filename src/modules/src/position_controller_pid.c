/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * position_estimator_pid.c: PID-based implementation of the position controller
 */

#include <math.h>
#include "num.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

struct pidInit_s {
  float kp;
  float ki;
  float kd;
};

struct pidAxis_s {
  PidObject pid;

  struct pidInit_s init;
    stab_mode_t previousMode;
  float setpoint;

  float output;
};

struct this_s {
  struct pidAxis_s pidVX;
  struct pidAxis_s pidVY;
  struct pidAxis_s pidVZ;

  struct pidAxis_s pidX;
  struct pidAxis_s pidY;
  struct pidAxis_s pidZ;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
  uint16_t thrustMin;  // Minimum thrust value to output
};

// Maximum roll/pitch angle permited
static float rpLimit  = 20;
static float rpLimitOverhead = 1.10f;
// Velocity maximums
static float xyVelMax = 1.0f;
static float zVelMax  = 1.0f;
static float velMaxOverhead = 1.10f;
static const float thrustScale = 1000.0f;

#define DT (float)(1.0f/POSITION_RATE)
#define POSITION_LPF_CUTOFF_FREQ 20.0f
#define POSITION_LPF_ENABLE true

#ifndef UNIT_TEST
//TFMICRO EDIT: WHY THE F WOULD YOU CALL A VARIABLE 'this' !!!!
static struct this_s pid_this = {
  {
    {
      25.0f,
      1.0f,
      0.0f,
    },
    DT,
  },

  {
    {
    25.0f,
    1.0f,
    0.0f,
    },
    DT,
  },

  {
    {
     25,
     15,
     0,
    },
    DT,
  },

  {
    {
      2.0f,
      0,
      0,
    },
    DT,
  },

  {
    {
    2.0f,
    0,
    0,
    },
    DT,
  },

  {
    {
      2.0f,
      0.5,
      0,
    },
    DT,
  },

  36000,
  20000,
};
#endif

void positionControllerInit()
{
  pidInit(&pid_this.pidX.pid, pid_this.pidX.setpoint, pid_this.pidX.init.kp, pid_this.pidX.init.ki, pid_this.pidX.init.kd,
      pid_this.pidX.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&pid_this.pidY.pid, pid_this.pidY.setpoint, pid_this.pidY.init.kp, pid_this.pidY.init.ki, pid_this.pidY.init.kd,
      pid_this.pidY.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&pid_this.pidZ.pid, pid_this.pidZ.setpoint, pid_this.pidZ.init.kp, pid_this.pidZ.init.ki, pid_this.pidZ.init.kd,
      pid_this.pidZ.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);

  pidInit(&pid_this.pidVX.pid, pid_this.pidVX.setpoint, pid_this.pidVX.init.kp, pid_this.pidVX.init.ki, pid_this.pidVX.init.kd,
      pid_this.pidVX.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&pid_this.pidVY.pid, pid_this.pidVY.setpoint, pid_this.pidVY.init.kp, pid_this.pidVY.init.ki, pid_this.pidVY.init.kd,
      pid_this.pidVY.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&pid_this.pidVZ.pid, pid_this.pidVZ.setpoint, pid_this.pidVZ.init.kp, pid_this.pidVZ.init.ki, pid_this.pidVZ.init.kd,
      pid_this.pidVZ.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
}

static float runPid(float input, struct pidAxis_s *axis, float setpoint, float dt) {
  axis->setpoint = setpoint;

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, true);
}

void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  pid_this.pidX.pid.outputLimit = xyVelMax * velMaxOverhead;
  pid_this.pidY.pid.outputLimit = xyVelMax * velMaxOverhead;
  // The ROS landing detector will prematurely trip if
  // pid_this value is below 0.5
  pid_this.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f)  * velMaxOverhead;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  float bodyvx = setpoint->velocity.x;
  float bodyvy = setpoint->velocity.y;

  // X, Y
  if (setpoint->mode.x == modeAbs) {
    setpoint->velocity.x = runPid(state->position.x, &pid_this.pidX, setpoint->position.x, DT);
  } else if (setpoint->velocity_body) {
    setpoint->velocity.x = bodyvx * cosyaw - bodyvy * sinyaw;
  }
  if (setpoint->mode.y == modeAbs) {
    setpoint->velocity.y = runPid(state->position.y, &pid_this.pidY, setpoint->position.y, DT);
  } else if (setpoint->velocity_body) {
    setpoint->velocity.y = bodyvy * cosyaw + bodyvx * sinyaw;
  }
  if (setpoint->mode.z == modeAbs) {
    setpoint->velocity.z = runPid(state->position.z, &pid_this.pidZ, setpoint->position.z, DT);
  }

  velocityController(thrust, attitude, setpoint, state);
}

void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  pid_this.pidVX.pid.outputLimit = rpLimit * rpLimitOverhead;
  pid_this.pidVY.pid.outputLimit = rpLimit * rpLimitOverhead;
  // Set the output limit to the maximum thrust range
  pid_this.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
  //pid_this.pidVZ.pid.outputLimit = (pid_this.thrustBase - pid_this.thrustMin) / thrustScale;

  // Roll and Pitch
  float rollRaw  = runPid(state->velocity.x, &pid_this.pidVX, setpoint->velocity.x, DT);
  float pitchRaw = runPid(state->velocity.y, &pid_this.pidVY, setpoint->velocity.y, DT);

  float yawRad = state->attitude.yaw * (float)M_PI / 180;
  attitude->pitch = -(rollRaw  * cosf(yawRad)) - (pitchRaw * sinf(yawRad));
  attitude->roll  = -(pitchRaw * cosf(yawRad)) + (rollRaw  * sinf(yawRad));

  attitude->roll  = constrain(attitude->roll,  -rpLimit, rpLimit);
  attitude->pitch = constrain(attitude->pitch, -rpLimit, rpLimit);

  // Thrust
  float thrustRaw = runPid(state->velocity.z, &pid_this.pidVZ, setpoint->velocity.z, DT);
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw*thrustScale + pid_this.thrustBase;
  // Check for minimum thrust
  if (*thrust < pid_this.thrustMin) {
    *thrust = pid_this.thrustMin;
  }
}

void positionControllerResetAllPID()
{
  pidReset(&pid_this.pidX.pid);
  pidReset(&pid_this.pidY.pid);
  pidReset(&pid_this.pidZ.pid);
  pidReset(&pid_this.pidVX.pid);
  pidReset(&pid_this.pidVY.pid);
  pidReset(&pid_this.pidVZ.pid);
}

LOG_GROUP_START(posCtl)

LOG_ADD(LOG_FLOAT, targetVX, &pid_this.pidVX.pid.desired)
LOG_ADD(LOG_FLOAT, targetVY, &pid_this.pidVY.pid.desired)
LOG_ADD(LOG_FLOAT, targetVZ, &pid_this.pidVZ.pid.desired)

LOG_ADD(LOG_FLOAT, targetX, &pid_this.pidX.pid.desired)
LOG_ADD(LOG_FLOAT, targetY, &pid_this.pidY.pid.desired)
LOG_ADD(LOG_FLOAT, targetZ, &pid_this.pidZ.pid.desired)

LOG_ADD(LOG_FLOAT, Xp, &pid_this.pidX.pid.outP)
LOG_ADD(LOG_FLOAT, Xi, &pid_this.pidX.pid.outI)
LOG_ADD(LOG_FLOAT, Xd, &pid_this.pidX.pid.outD)

LOG_ADD(LOG_FLOAT, Yp, &pid_this.pidY.pid.outP)
LOG_ADD(LOG_FLOAT, Yi, &pid_this.pidY.pid.outI)
LOG_ADD(LOG_FLOAT, Yd, &pid_this.pidY.pid.outD)

LOG_ADD(LOG_FLOAT, Zp, &pid_this.pidZ.pid.outP)
LOG_ADD(LOG_FLOAT, Zi, &pid_this.pidZ.pid.outI)
LOG_ADD(LOG_FLOAT, Zd, &pid_this.pidZ.pid.outD)

LOG_ADD(LOG_FLOAT, VXp, &pid_this.pidVX.pid.outP)
LOG_ADD(LOG_FLOAT, VXi, &pid_this.pidVX.pid.outI)
LOG_ADD(LOG_FLOAT, VXd, &pid_this.pidVX.pid.outD)

LOG_ADD(LOG_FLOAT, VZp, &pid_this.pidVZ.pid.outP)
LOG_ADD(LOG_FLOAT, VZi, &pid_this.pidVZ.pid.outI)
LOG_ADD(LOG_FLOAT, VZd, &pid_this.pidVZ.pid.outD)

LOG_GROUP_STOP(posCtl)

PARAM_GROUP_START(velCtlPid)

PARAM_ADD(PARAM_FLOAT, vxKp, &pid_this.pidVX.pid.kp)
PARAM_ADD(PARAM_FLOAT, vxKi, &pid_this.pidVX.pid.ki)
PARAM_ADD(PARAM_FLOAT, vxKd, &pid_this.pidVX.pid.kd)

PARAM_ADD(PARAM_FLOAT, vyKp, &pid_this.pidVY.pid.kp)
PARAM_ADD(PARAM_FLOAT, vyKi, &pid_this.pidVY.pid.ki)
PARAM_ADD(PARAM_FLOAT, vyKd, &pid_this.pidVY.pid.kd)

PARAM_ADD(PARAM_FLOAT, vzKp, &pid_this.pidVZ.pid.kp)
PARAM_ADD(PARAM_FLOAT, vzKi, &pid_this.pidVZ.pid.ki)
PARAM_ADD(PARAM_FLOAT, vzKd, &pid_this.pidVZ.pid.kd)

PARAM_GROUP_STOP(velCtlPid)

PARAM_GROUP_START(posCtlPid)

PARAM_ADD(PARAM_FLOAT, xKp, &pid_this.pidX.pid.kp)
PARAM_ADD(PARAM_FLOAT, xKi, &pid_this.pidX.pid.ki)
PARAM_ADD(PARAM_FLOAT, xKd, &pid_this.pidX.pid.kd)

PARAM_ADD(PARAM_FLOAT, yKp, &pid_this.pidY.pid.kp)
PARAM_ADD(PARAM_FLOAT, yKi, &pid_this.pidY.pid.ki)
PARAM_ADD(PARAM_FLOAT, yKd, &pid_this.pidY.pid.kd)

PARAM_ADD(PARAM_FLOAT, zKp, &pid_this.pidZ.pid.kp)
PARAM_ADD(PARAM_FLOAT, zKi, &pid_this.pidZ.pid.ki)
PARAM_ADD(PARAM_FLOAT, zKd, &pid_this.pidZ.pid.kd)

PARAM_ADD(PARAM_UINT16, thrustBase, &pid_this.thrustBase)
PARAM_ADD(PARAM_UINT16, thrustMin, &pid_this.thrustMin)

PARAM_ADD(PARAM_FLOAT, rpLimit,  &rpLimit)
PARAM_ADD(PARAM_FLOAT, xyVelMax, &xyVelMax)
PARAM_ADD(PARAM_FLOAT, zVelMax,  &zVelMax)

PARAM_GROUP_STOP(posCtlPid)
