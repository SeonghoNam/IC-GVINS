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

#include "drawer_pangolin.h"

#include "common/logging.h"
#include "common/rotation.h"

#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

DrawerPangolin::DrawerPangolin()
    : isfinished_(false)
    , isframerdy_(false)
    , ismaprdy_(false) {

    frame_id_ = "world";

}

void DrawerPangolin::setFinished() {
    isfinished_ = true;
    update_sem_.notify_one();
}

void DrawerPangolin::run() {
    pangolin::CreateWindowAndBind("vslam", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
                                           pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &vis_display = pangolin::CreateDisplay()
                                      .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                                      .SetHandler(new pangolin::Handler3D(vis_camera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    while (!pangolin::ShouldQuit() && !isfinished_)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        { // scope for mutex
            std::unique_lock<std::mutex> lock(update_mutex_);
            update_sem_.wait(lock);

            // 发布跟踪的图像
            if (isframerdy_) {
                publishTrackingImage();

                isframerdy_ = false;
            }

            // 发布轨迹和地图点
            if (ismaprdy_) {
                publishOdometry();
                FollowCurrentFrame(vis_camera);

                publishMapPoints();

                ismaprdy_ = false;
            }
            pangolin::FinishFrame();
            // auto t1 = std::chrono::steady_clock::now();
            // if (current_frame_)
            // {
            //     DrawFrame(current_frame_, green);
            //     FollowCurrentFrame(vis_camera);

            //     // cv::Mat img = PlotFrameImage();
            //     // cv::imshow("image", img);
            //     // cv::waitKey(1);
            // }

            // if (map_)
            // {
            //     DrawMapPoints();
            // }

            // pangolin::FinishFrame();
            // auto t2 = std::chrono::steady_clock::now();
            // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            // // LOG(INFO) << "viewer time : " << time_used.count();
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
    }
}

void DrawerPangolin::updateFrame(Frame::Ptr frame) {
    std::unique_lock<std::mutex> lock(image_mutex_);

    frame->image().copyTo(raw_image_);

    isframerdy_ = true;
    update_sem_.notify_one();
}

void DrawerPangolin::updateTrackedMapPoints(vector<cv::Point2f> map, vector<cv::Point2f> matched,
                                        vector<MapPointType> mappoint_type) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    pts2d_map_     = std::move(map);
    pts2d_matched_ = std::move(matched);
    mappoint_type_ = std::move(mappoint_type);
}

void DrawerPangolin::updateTrackedRefPoints(vector<cv::Point2f> ref, vector<cv::Point2f> cur) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    pts2d_ref_ = std::move(ref);
    pts2d_cur_ = std::move(cur);
}

void DrawerPangolin::publishTrackingImage() {
    std::unique_lock<std::mutex> lock(image_mutex_);

    Mat drawed;
    drawTrackingImage(raw_image_, drawed);
    cv::imshow("image", drawed);
    cv::waitKey(1);
}

void DrawerPangolin::publishMapPoints() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    const float red[3] = {1.0, 0, 0};
    const float blue[3] = {0, 0, 1.};

    glPointSize(2);
    glBegin(GL_POINTS);
    for (const auto &local : map_->landmarks()) {
        if (local.second && !local.second->isOutlier()) {
            auto pos = local.second->pos();
            glColor3f(red[0], red[1], red[2]);
            glVertex3d(pos[0], pos[1], pos[2]);            
        }
    }

    for (const auto &pts : fixed_mappoints_) {
        glColor3f(blue[0], blue[1], blue[2]);
        glVertex3d(pts[0], pts[1], pts[2]);   
    }
    glEnd();

    // fixed_mappoints_.clear();
}

void DrawerPangolin::publishOdometry() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    float red[3] = {1., 0., 0.};
    float green[3] = {0., 1., 0.};
    DrawFrame(pose_, green);

    // nav_msgs::Odometry odometry;

    // auto quaternion = Rotation::matrix2quaternion(pose_.R);
    // auto stamp      = ros::Time::now();

    // // Odometry
    // odometry.header.stamp            = stamp;
    // odometry.header.frame_id         = frame_id_;
    // odometry.pose.pose.position.x    = pose_.t.x();
    // odometry.pose.pose.position.y    = pose_.t.y();
    // odometry.pose.pose.position.z    = pose_.t.z();
    // odometry.pose.pose.orientation.x = quaternion.x();
    // odometry.pose.pose.orientation.y = quaternion.y();
    // odometry.pose.pose.orientation.z = quaternion.z();
    // odometry.pose.pose.orientation.w = quaternion.w();
    // pose_pub_.publish(odometry);

    // // Path
    // geometry_msgs::PoseStamped pose_stamped;
    // pose_stamped.header.stamp    = stamp;
    // pose_stamped.header.frame_id = frame_id_;
    // pose_stamped.pose            = odometry.pose.pose;

    // path_.header.stamp    = stamp;
    // path_.header.frame_id = frame_id_;
    // path_.poses.push_back(pose_stamped);
    // path_pub_.publish(path_);
}

void DrawerPangolin::addNewFixedMappoint(Vector3d point) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    fixed_mappoints_.push_back(point);
}

void DrawerPangolin::updateMap(const Eigen::Matrix4d &pose) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    pose_.R = pose.block<3, 3>(0, 0);
    pose_.t = pose.block<3, 1>(0, 3);

    ismaprdy_ = true;
    update_sem_.notify_one();
}

// void DrawerPangolin::updateMap(const Pose pose) {
//     std::unique_lock<std::mutex> lock(map_mutex_);

//     pose_.R = pose.R;
//     pose_.t = pose.t;

//     ismaprdy_ = true;
//     update_sem_.notify_one();
// }

void DrawerPangolin::DrawFrame(Pose pose, const float *color)
{
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    // glPointSize(2);
    // glBegin(GL_POINTS);
    // for (auto feat : frame->features())
    // {
    //     auto mp = feat.second->getMapPoint();
    //     if (mp != nullptr)
    //     {
    //         auto pos = mp->pos();
    //         glColor3f(color[0], color[1], color[2]);
    //         glVertex3d(pos[0], pos[1], pos[2]);
    //     }
    // }
    // glEnd();

    glPushMatrix();

    Eigen::Matrix4d Twc;
    Twc.setZero();
    Twc(3, 3) = 1;

    Twc.block<3, 3>(0, 0) = pose_.R;
    Twc.block<3, 1>(0, 3) = pose_.t;

    // pangolin::OpenGlMatrix m(Twc);
    Eigen::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat *)m.data());

    if (color == nullptr)
    {
        glColor3f(1, 0, 0);
    }
    else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void DrawerPangolin::FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera)
{
    Eigen::Matrix4d Twc;
    Twc.setZero();
    Twc(3, 3) = 1;

    Twc.block<3, 3>(0, 0) = pose_.R;
    Twc.block<3, 1>(0, 3) = pose_.t;

    pangolin::OpenGlMatrix m(Twc);
    vis_camera.Follow(m, true);
}