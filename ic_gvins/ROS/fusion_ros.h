/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef FUSION_ROS_H
#define FUSION_ROS_H

#include "ic_gvins/common/types.h"
#include "ic_gvins/ic_gvins.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <memory>

class FusionROS {

public:
    FusionROS() = default;

    void run();

    void setFinished();

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr &imumsg);

    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr &gnssmsg);

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr &imagemsg);

private:
    std::shared_ptr<GVINS> gvins_;

    IMU imu_{.time = 0}, imu_pre_{.time = 0};
    Frame::Ptr frame_;
    GNSS gnss_;

    bool isusegnssoutage_{false};
    double gnssoutagetime_{0};
    double gnssthreshold_{20.0};

    std::queue<IMU> imu_buffer_;
    std::queue<Frame::Ptr> frame_buffer_;
};

#endif // FUSION_ROS_H
