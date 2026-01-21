//-----------------------------------------------------------------------------
// File: HunterControlLoop.cpp
// Author: Akisora
// Created: 2026-01-19
//-----------------------------------------------------------------------------

#include "mvr_robot_control/BotHW.h"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <memory>
#include <chrono>

class BotControlLoop
{
public:
    using Clock    = std::chrono::steady_clock;
    using Duration = std::chrono::duration<double>;

    BotControlLoop(ros::NodeHandle& nh, const std::shared_ptr<BotHW>& hw)
        : nh_(nh)
        , hw_(hw)
        , controller_manager_(hw_.get(), nh_)
    {
        ros::NodeHandle pnh("~");

        pnh.param("loop_frequency", loop_hz_, 100.0);           
        pnh.param("cycle_time_error_threshold", cycle_error_thresh_, 0.01); 

        last_time_ros_ = ros::Time::now();
        last_time_     = Clock::now();

        ROS_INFO_STREAM("BotControlLoop started with loop_frequency = "
                        << loop_hz_ << " Hz, cycle_time_error_threshold = "
                        << cycle_error_thresh_ << " s");
    }

    void run()
    {
        ros::Rate rate(loop_hz_);

        while (ros::ok())
        {
            ros::Time   current_time_ros = ros::Time::now();
            Clock::time_point current_time = Clock::now();

            ros::Duration period_ros = current_time_ros - last_time_ros_;
            last_time_ros_ = current_time_ros;

            Duration desired_period(1.0 / loop_hz_);
            Duration actual_period  = std::chrono::duration_cast<Duration>(current_time - last_time_);
            last_time_ = current_time;

            double cycle_time_error = (period_ros - ros::Duration(desired_period.count())).toSec();
            if (cycle_time_error > cycle_error_thresh_)
            {
                ROS_WARN_STREAM("Cycle time exceeded error threshold by: "
                                << (cycle_time_error - cycle_error_thresh_)
                                << " s, cycle time: " << period_ros.toSec()
                                << " s, threshold: " << cycle_error_thresh_ << " s");
            }

            hw_->read(current_time_ros, period_ros);

            controller_manager_.update(current_time_ros, period_ros);

            hw_->write(current_time_ros, period_ros);

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    std::shared_ptr<BotHW> hw_;
    controller_manager::ControllerManager controller_manager_;

    double loop_hz_{100.0};
    double cycle_error_thresh_{0.01};

    ros::Time last_time_ros_;
    Clock::time_point last_time_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mvr_bot_hw");
    ros::NodeHandle nh;
    ros::NodeHandle hw_nh("~");

    ros::AsyncSpinner spinner(3); 
    spinner.start();

    auto bot_hw = std::make_shared<BotHW>();
    if (!bot_hw->init(hw_nh))
    {
        ROS_FATAL("Failed to initialize BotHW.");
        return 1;
    }

    BotControlLoop control_loop(nh, bot_hw);
    control_loop.run();

    return 0;
}