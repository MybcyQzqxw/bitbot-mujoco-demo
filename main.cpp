#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "bitbot_mujoco/device/mujoco_joint.h"

#include "user_func.h"

#define _USE_MATH_DEFINES
#include <math.h>

int main(int argc, char const *argv[])
{
  // 定义kernel对象，参数为配置文件
  Kernel kernel("../../bitbot_sc.xml");

  // 注册Config函数
  kernel.RegisterConfigFunc(&ConfigFunc);
  // 注册Finish函数
  kernel.RegisterFinishFunc(&FinishFunc);

  // 注册event
  kernel.RegisterEvent("waiting", static_cast<bitbot::StateId>(Events::Wait), &EventWait, true);
  kernel.RegisterEvent("init_pos", static_cast<bitbot::StateId>(Events::InitPos), &EventInitPos);
  kernel.RegisterEvent("to_fall_pos1", static_cast<bitbot::StateId>(Events::ToFallPos1), &EventToFallPos1, true);
  kernel.RegisterEvent("to_fall_pos2", static_cast<bitbot::StateId>(Events::ToFallPos2), &EventToFallPos2, true);

  // 注册state
  kernel.RegisterState("waiting", static_cast<bitbot::StateId>(States::Waiting), &StateWaiting, {static_cast<bitbot::StateId>(Events::InitPos), static_cast<bitbot::StateId>(Events::ToFallPos1), static_cast<bitbot::StateId>(Events::ToFallPos2)});
  kernel.RegisterState("init_pos", static_cast<bitbot::StateId>(States::InitPos), &StateInitPos, {static_cast<bitbot::StateId>(Events::ToFallPos1), static_cast<bitbot::StateId>(Events::ToFallPos2)});
  kernel.RegisterState("to_fall_pos1", static_cast<bitbot::StateId>(States::ToFallPos1), &StateToFallPos1, {});
  kernel.RegisterState("to_fall_pos2", static_cast<bitbot::StateId>(States::ToFallPos2), &StateToFallPos2, {});

  // 设置用户的第一个state
  kernel.SetFirstState(static_cast<bitbot::StateId>(States::Waiting));

  // 运行
  kernel.Run();
  
  return 0;
}
