#pragma once

#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/device/mujoco_force_sensor.h"
#include "bitbot_mujoco/device/mujoco_imu.h"

#define _USE_MATH_DEFINES
#include <math.h>

enum Events
{
  Wait = 1001,
  InitPos = 1002,
  ToFallPos1 = 1003,
  ToFallPos2 = 1004
};

enum class States : bitbot::StateId
{
  Waiting = 2001,
  InitPos = 2002,
  ToFallPos1 = 2003,
  ToFallPos2 = 2004
};

struct UserData
{
  double sin = 10;
};

using Kernel = bitbot::MujocoKernel<UserData, "time", "sin">;

void ConfigFunc(const bitbot::MujocoBus& bus, UserData&);

std::optional<bitbot::StateId> EventWait(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventInitPos(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventToFallPos1(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventToFallPos2(bitbot::EventValue value, UserData& user_data);

void StateWaiting(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);

void StateInitPos(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);

void StateToFallPos1(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);

void StateToFallPos2(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);

void FinishFunc(UserData&);
