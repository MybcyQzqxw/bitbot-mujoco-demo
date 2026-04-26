#include "user_func.h"

#include <chrono>
#include <memory>

bitbot::MujocoJoint *joint_x, *joint_y = nullptr;

void ConfigFunc(const bitbot::MujocoBus& bus, UserData&)
{
  joint_x = bus.GetDevice<bitbot::MujocoJoint>(1).value();
  joint_y = bus.GetDevice<bitbot::MujocoJoint>(2).value();
}

std::optional<bitbot::StateId> EventWait(bitbot::EventValue, UserData&)
{
  return static_cast<bitbot::StateId>(States::Waiting);
}

std::optional<bitbot::StateId> EventInitPos(bitbot::EventValue value, UserData&)
{
  return static_cast<bitbot::StateId>(States::InitPos);
}

std::optional<bitbot::StateId> EventToFallPos1(bitbot::EventValue value, UserData& user_data)
{
  return static_cast<bitbot::StateId>(States::ToFallPos1);
}

std::optional<bitbot::StateId> EventToFallPos2(bitbot::EventValue value, UserData& user_data)
{
  return static_cast<bitbot::StateId>(States::ToFallPos2);
}

void StateWaiting(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data)
{
}

void StateInitPos(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data)
{
}

void StateToFallPos1(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data)
{
}

void StateToFallPos2(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data)
{
}

void FinishFunc(UserData&)
{
  std::cout << "finish" << std::endl;
}
