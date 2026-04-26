#include "user_func.h"
#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/device/mujoco_force_sensor.h"
#include "bitbot_mujoco/device/mujoco_imu.h"

#include <chrono>
#include <iostream>
#include <memory>

bitbot::MujocoJoint *joint1 = nullptr, *joint2 = nullptr, *joint3 = nullptr, *joint4 = nullptr, *joint5 = nullptr, *joint6 = nullptr, *joint7 = nullptr, *joint8 = nullptr, *joint9 = nullptr, *joint10 = nullptr, *joint11 = nullptr, *joint12 = nullptr, *joint13 = nullptr, *joint14 = nullptr, *joint15 = nullptr, *joint16 = nullptr, *joint17 = nullptr, *joint18 = nullptr, *joint19 = nullptr, *joint20 = nullptr;

void ConfigFunc(const bitbot::MujocoBus& bus, UserData&)
{
  joint1 = bus.GetDevice<bitbot::MujocoJoint>(1).value();
  joint2 = bus.GetDevice<bitbot::MujocoJoint>(2).value();
  joint3 = bus.GetDevice<bitbot::MujocoJoint>(3).value();
  joint4 = bus.GetDevice<bitbot::MujocoJoint>(4).value();
  joint5 = bus.GetDevice<bitbot::MujocoJoint>(5).value();
  joint6 = bus.GetDevice<bitbot::MujocoJoint>(6).value();
  joint7 = bus.GetDevice<bitbot::MujocoJoint>(7).value();
  joint8 = bus.GetDevice<bitbot::MujocoJoint>(8).value();
  joint9 = bus.GetDevice<bitbot::MujocoJoint>(9).value();
  joint10 = bus.GetDevice<bitbot::MujocoJoint>(10).value();
  joint11 = bus.GetDevice<bitbot::MujocoJoint>(11).value();
  joint12 = bus.GetDevice<bitbot::MujocoJoint>(12).value();
  joint13 = bus.GetDevice<bitbot::MujocoJoint>(13).value();
  joint14 = bus.GetDevice<bitbot::MujocoJoint>(14).value();
  joint15 = bus.GetDevice<bitbot::MujocoJoint>(15).value();
  joint16 = bus.GetDevice<bitbot::MujocoJoint>(16).value();
  joint17 = bus.GetDevice<bitbot::MujocoJoint>(17).value();
  joint18 = bus.GetDevice<bitbot::MujocoJoint>(18).value();
  joint19 = bus.GetDevice<bitbot::MujocoJoint>(19).value();
  joint20 = bus.GetDevice<bitbot::MujocoJoint>(20).value();
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
  bitbot::MujocoJoint* joints[20] = {
    joint1, joint2, joint3, joint4, joint5, joint6, joint7, joint8, joint9, joint10,
    joint11, joint12, joint13, joint14, joint15, joint16, joint17, joint18, joint19, joint20
  };
  // 下蹲前倾目标角度（单位：弧度）
  double target_angles[20] = {
    0.0,   // lhipYaw
    0.0,   // lhipRoll
   -0.6,   // lhipPitch
    0.4,   // lknee
   -0.6,   // lankle1
    0.0,   // lankle2
    0.0,   // rhipYaw
    0.0,   // rhipRoll
   -0.6,   // rhipPitch
    0.4,   // rknee
   -0.6,   // rankle1
    0.0,   // rankle2
    0.0,   // lshoulderPitch
    0.0,   // lshoulderRoll
    0.0,   // lshoulderYaw
    0.0,   // lelbow
    0.0,   // rshoulderPitch
    0.0,   // rshoulderRoll
    0.0,   // rshoulderYaw
    0.0    // relbow
  };
  static bool initialized = false;
  static double start_angles[20] = {0};
  static double elapsed = 0.0;
  const double duration = 2.0; // 插值时长2秒
  double dt = 0.002; // 默认2ms
  if (!initialized) {
    for (int i = 0; i < 20; ++i) {
      if (joints[i]) {
        start_angles[i] = joints[i]->GetActualPosition();
      } else {
        start_angles[i] = 0.0;
      }
    }
    elapsed = 0.0;
    initialized = true;
  }
  elapsed += dt;
  double t = elapsed / duration;
  if (t > 1.0) t = 1.0;
  double s = 0.5 - 0.5 * cos(M_PI * t); // 正弦插值
  for (int i = 0; i < 20; ++i) {
    if (joints[i]) {
      joints[i]->SetMode(bitbot::MujocoJointMode::POSITION);
      double q_target = start_angles[i] + (target_angles[i] - start_angles[i]) * s;
      joints[i]->SetTargetPosition(q_target);
    }
  }
  if (t >= 1.0) {
    initialized = false; // 插值完成后允许下次重新插值
  }
}

void StateInitPos(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data)
{
  // 关节指针数组，便于遍历
  bitbot::MujocoJoint* joints[20] = {
    joint1, joint2, joint3, joint4, joint5, joint6, joint7, joint8, joint9, joint10,
    joint11, joint12, joint13, joint14, joint15, joint16, joint17, joint18, joint19, joint20
  };

  // 目标角度为0，使用P控制插值复位
  const double Kp = 100.0; // 位置P增益，可根据实际调整
  const double Kd = 1.0;  // 速度D增益，可根据实际调整
  for (int i = 0; i < 20; ++i) {
    if (joints[i]) {
      double pos = joints[i]->GetActualPosition();
      double vel = joints[i]->GetActualVelocity();
      double torque = -Kp * pos - Kd * vel; // 目标0度
      joints[i]->SetTargetTorque(torque);
    }
  }
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
