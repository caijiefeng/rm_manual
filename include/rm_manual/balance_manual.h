//
// Created by yuchen on 2023/4/3.
//

#pragma once

#include "rm_manual/chassis_gimbal_shooter_cover_manual.h"

namespace rm_manual
{
class BalanceManual : public ChassisGimbalShooterCoverManual
{
public:
  BalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);

protected:
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void wPress() override;
  void sPress() override;
  void aPress() override;
  void dPress() override;
  void shiftPress() override;
  void shiftRelease() override;
  void wPressing() override;
  void aPressing() override;
  void sPressing() override;
  void dPressing() override;
  void xPress() override;
  void gPress() override;
  void ctrlZPress() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;

  void sendCommand(const ros::Time& time) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void zPress();
  void ctrlXPress();
  void modeFallen(ros::Duration duration);
  void modeNormalize();
  rm_common::BalanceCommandSender* balance_cmd_sender_{};

private:
  void gyroCPressedCallback();
  void balanceStateCallback(const rm_msgs::BalanceState::ConstPtr& msg);

  ros::Subscriber state_sub_;
  double gyro_scale_, balance_dangerous_angle_;
  ros::Timer gyro_timer_;

  bool flank_ = false, reverse_ = false;
  std::string flank_frame_, reverse_frame_;

  RampFilter<double>*ramp_x_{}, *ramp_y_{};
  InputEvent z_event_, g_event_, ctrl_x_event_, auto_fallen_event_;
};
}  // namespace rm_manual
