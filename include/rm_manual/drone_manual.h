#ifndef RM_MANUAL_DRONE_MANUAL_H_
#define RM_MANUAL_DRONE_MANUAL_H_

#include "rm_manual/common/manual_base.h"
#include <rm_common/decision/calibration_queue.h>
namespace rm_manual
{
    class DroneManual : public ManualBase
    {
    public:
        explicit DroneManual(ros::NodeHandle& nh);
        void run() override;

    protected:
        void sendCommand(const ros::Time& time) override;
        void updateRc() override;
        void updatePc() override;
        void gimbalOutputOn() override;
        void shooterOutputOn() override;
        void remoteControlTurnOff() override;
        void remoteControlTurnOn() override;
        void checkReferee() override;
        void checkKeyboard() override;
        void drawUi(const ros::Time& time) override;
        void rightSwitchDownRise() override;
        void rightSwitchMidRise() override;
        void rightSwitchUpRise() override;
        void leftSwitchDownRise() override;
        void leftSwitchMidRise() override;
        void leftSwitchMidOn(ros::Duration duration);
        void leftSwitchUpRise() override;
        void leftSwitchUpOn(ros::Duration duration);
        void mouseLeftPress();
        void mouseLeftRelease()
        {
            shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
        }
        void mouseRightPress();
        void mouseRightRelease()
        {
            gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        }

        rm_common::GimbalCommandSender* gimbal_cmd_sender_{};
        rm_common::CalibrationQueue* gimbal_calibration_;
        rm_common::ShooterCommandSender* shooter_cmd_sender_{};
        rm_common::CalibrationQueue* shooter_calibration_;
        TimeChangeUi* time_change_ui_{};
        TriggerChangeUi* trigger_change_ui_{};
        FlashUi* flash_ui_{};
        FixedUi* fixed_ui_{};
        double x_scale_{}, y_scale_{};
        double gimbal_scale_{ 1. };
        double gyro_move_reduction_{ 1. };
        double gyro_rotate_reduction_{ 1. };

        bool isCalibration = false;
        InputEvent gimbal_power_on_event_, shooter_power_on_event_, mouse_left_event_, mouse_right_event_;
    };
}

#endif //RM_MANUAL_DRONE_MANUAL_H_
