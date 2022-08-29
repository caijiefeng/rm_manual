#include "rm_manual/drone_manual.h"

namespace rm_manual
{
    DroneManual::DroneManual(ros::NodeHandle& nh) : ManualBase(nh)
    {
        ros::NodeHandle gimbal_nh(nh, "gimbal");
        gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, data_.referee_.referee_data_);
        ros::NodeHandle shooter_nh(nh, "shooter");
        shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_.referee_.referee_data_);
        ros::NodeHandle ui_nh(nh, "ui");
        trigger_change_ui_ = new TriggerChangeUi(ui_nh, data_);
        time_change_ui_ = new TimeChangeUi(ui_nh, data_);
        flash_ui_ = new FlashUi(ui_nh, data_);
        fixed_ui_ = new FixedUi(ui_nh, data_);
        XmlRpc::XmlRpcValue rpc_value;
        nh.getParam("gimbal_calibration", rpc_value);
        gimbal_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
        gimbal_power_on_event_.setRising(boost::bind(&DroneManual::gimbalOutputOn, this));
        nh.getParam("shooter_calibration", rpc_value);
        shooter_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
        shooter_power_on_event_.setRising(boost::bind(&DroneManual::shooterOutputOn, this));
        controller_manager_.startCalibrationControllers();
        left_switch_up_event_.setActiveHigh(boost::bind(&DroneManual::leftSwitchUpOn, this, _1));
        left_switch_mid_event_.setActiveHigh(boost::bind(&DroneManual::leftSwitchMidOn, this, _1));
        mouse_left_event_.setActiveHigh(boost::bind(&DroneManual::mouseLeftPress, this));
        mouse_left_event_.setFalling(boost::bind(&DroneManual::mouseLeftRelease, this));
        mouse_right_event_.setActiveHigh(boost::bind(&DroneManual::mouseRightPress, this));
        mouse_right_event_.setFalling(boost::bind(&DroneManual::mouseRightRelease, this));
    }

    void DroneManual::run()
    {
        ManualBase::run();
        gimbal_calibration_->update(ros::Time::now());
        shooter_calibration_->update(ros::Time::now());
    }

    void DroneManual::sendCommand(const ros::Time& time)
    {
        gimbal_cmd_sender_->sendCommand(time);
        shooter_cmd_sender_->sendCommand(time);
    }

    void DroneManual::updateRc()
    {
        ManualBase::updateRc();
        gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
    }
    void DroneManual::updatePc()
    {
        ManualBase::updatePc();
        gimbal_cmd_sender_->setRate(-data_.dbus_data_.m_x * gimbal_scale_, data_.dbus_data_.m_y * gimbal_scale_);
    }

    void DroneManual::gimbalOutputOn()
    {
        ManualBase::gimbalOutputOn();
        gimbal_calibration_->reset();
    }

    void DroneManual::shooterOutputOn()
    {
        ManualBase::shooterOutputOn();
        shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
        shooter_calibration_->reset();
    }

    void DroneManual::remoteControlTurnOff()
    {
        ManualBase::remoteControlTurnOff();
        gimbal_calibration_->stop();
        shooter_calibration_->stop();
    }

    void DroneManual::remoteControlTurnOn()
    {
        controller_manager_.stopCalibrationControllers();
        ManualBase::remoteControlTurnOn();
    }

    void DroneManual::checkReferee()
    {
        ManualBase::checkReferee();
        gimbal_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_);
        shooter_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_shooter_output_);
    }

    void DroneManual::checkKeyboard()
    {
        ManualBase::checkKeyboard();
        mouse_left_event_.update(data_.dbus_data_.p_l);
        mouse_right_event_.update(data_.dbus_data_.p_r);
    }

    void DroneManual::drawUi(const ros::Time& time){}

    void DroneManual::rightSwitchDownRise()
    {
        ManualBase::rightSwitchDownRise();
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        gimbal_cmd_sender_->setZero();
        shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    }

    void DroneManual::rightSwitchMidRise()
    {
        ManualBase::rightSwitchMidRise();
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    }
    void DroneManual::rightSwitchUpRise()
    {
        ManualBase::rightSwitchUpRise();
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    }

    void DroneManual::leftSwitchDownRise()
    {
        ManualBase::leftSwitchDownRise();
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    }

    void DroneManual::leftSwitchMidRise()
    {
        ManualBase::leftSwitchMidRise();
        shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    }

    void DroneManual::leftSwitchMidOn(ros::Duration duration)
    {
        if (data_.track_data_.id == 0)
            gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        else
            gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    }

    void DroneManual::leftSwitchUpRise()
    {
        ManualBase::leftSwitchUpRise();
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    }

    void DroneManual::leftSwitchUpOn(ros::Duration duration)
    {
        if (data_.track_data_.id == 0)
            gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        else
            gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
        if (duration > ros::Duration(1.))
        {
            shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
            shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
        }
        else if (duration < ros::Duration(0.02))
        {
            shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
            shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
        }
        else
            shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    }

    void DroneManual::mouseLeftPress()
    {
        shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
        if (data_.dbus_data_.p_r)
            shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
    }

    void DroneManual::mouseRightPress()
    {
        if (data_.track_data_.id == 0)
            gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        else
        {
            gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
            gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
        }
    }


}  // namespace rm_manual