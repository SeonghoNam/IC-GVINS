#include "backend.h"
#if 0
#include "vslam/ceres_helper/reprojection_.hpp"
#include "vslam/ceres_helper/se3_parameterization.hpp"
#else
#include "factors/reprojection_factor.h"
#include "factors/pose_parameterization.h"
#include "factors/relativepose_factor.h"
#include "factors/PositionDEMFactor.h"
#include "factors/MappointDEMFactor.h"
#endif
// #include "vslam/config.h"
#include "tracking/feature.h"
#include "tracking/map.h"
#include "tracking/mappoint.h"
// #include "tracking/utility.h"

struct pose_para
{
    double pose[7];
};

Backend::Backend()
{
    // min_reprojerr_ = Config::Get<double>("min_reprojection_error");

    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    dem_ = std::make_shared<DEM>();
}

void Backend::UpdateMap()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop()
{
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void Backend::BackendLoop()
{
    while (backend_running_.load())
    {
        auto t1 = std::chrono::steady_clock::now();
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);
        Optimize();
        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "backend time : " << time_used.count();
    }
}

void Backend::Optimize()
{

#if 1
    if (map_->landmarks().empty()) {
        return;
    }
    Map::KeyFrames active_kfs = map_->keyframes();
    Map::LandMarks active_landmarks = map_->landmarks();

    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    std::unordered_map<int, struct pose_para> keyframePoses_data;
    for (auto &keyframe : active_kfs)
    {   
        Pose pose = keyframe.second->pose();
        pose_para data;
        memcpy(data.pose, pose.t.data(), sizeof(double) * 3);
        memcpy(data.pose+3, Rotation::matrix2quaternion(pose.R).coeffs().data(), sizeof(double) * 4);

        keyframePoses_data.insert({keyframe.first, data});
        double *para = keyframePoses_data[keyframe.first].pose;

        ceres::LocalParameterization *parameterization = new (PoseParameterization);
        problem.AddParameterBlock(para, 7, parameterization);
    }


    std::unordered_map<ulong, double> invdepthlist_;
    for (const auto &landmark : active_landmarks) {
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            auto frame = mappoint->referenceFrame();
            if (!frame || !(active_kfs.find(frame->keyFrameId()) != active_kfs.end())) { // Original : !map_->isKeyFrameInMap(frame)) {
                continue;
            }

            double depth         = mappoint->depth();
            double inverse_depth = 1.0 / depth;

            // 确保深度数值有效
            // For valid mappoints
            if (std::isnan(inverse_depth)) {
                mappoint->setOutlier(true);
                // LOGE << "Mappoint " << mappoint->id() << " is wrong with depth " << depth << " type "
                //      << mappoint->mapPointType();
                continue;
            }

            invdepthlist_[mappoint->id()] = inverse_depth;
            problem.AddParameterBlock(&invdepthlist_[mappoint->id()], 1);

            mappoint->addOptimizedTimes();
        }
    }
    // // Relative Pose Factor
    // auto sortedKeyframePoses_data = keyframePoses_data;
    // std::sort(sortedKeyframePoses_data.begin(), sortedKeyframePoses_data.end(), [](const pair<int, struct pose_para> &a, const pair<int, struct pose_para> &b) {
    //     return a.first < b.first;
    // });
    // for(int i = 0; i < sortedKeyframePoses_data.size(); i++)
    // {
    //     auto keyframePoses_prv = sortedKeyframePoses_data[i];
    //     auto keyframePoses_cur = sortedKeyframePoses_data[i+1];
    //     Pose Pose_prv_cur;
    //     pose_prv_cur.R = keyframePoses_cur
    // }

 // 外参
    // Extrinsic parameters
    extrinsic_[0] = 0;
    extrinsic_[1] = 0;
    extrinsic_[2] = 0;

    Quaterniond qic = Rotation::matrix2quaternion(Matrix3d::Identity());
    qic.normalize();
    extrinsic_[3] = qic.x();
    extrinsic_[4] = qic.y();
    extrinsic_[5] = qic.z();
    extrinsic_[6] = qic.w();

    ceres::LocalParameterization *parameterization = new (PoseParameterization);
    problem.AddParameterBlock(extrinsic_, 7, parameterization);

    problem.SetParameterBlockConstant(extrinsic_);

    // 时间延时
    // Time delay
    extrinsic_[7] = 0;
    problem.AddParameterBlock(&extrinsic_[7], 1);
    problem.SetParameterBlockConstant(&extrinsic_[7]);

    // // Relative Pose Factor
    // for (auto &keyframe1 : active_kfs)
    // {   
    //     Pose pose1 = keyframe1.second->pose();
    //     if (active_kfs.find(keyframe1.first+1) == active_kfs.end())
    //         continue;
    //     auto keyframe2 = active_kfs[keyframe1.first+1];
    //     Pose pose2 = keyframe2->pose();

    //     Pose pose12;
    //     pose12.t = pose2.R.transpose()*pose1.t - pose2.t;
    //     pose12.R = pose2.R.transpose()*pose1.R;
    //     // normalize
    //     pose12.R = pose12.R + 0.5 * (Eigen::Matrix3d::Identity() - pose12.R * pose12.R.transpose()) * pose12.R;

    //     auto relpose_factor = new RelativePoseFactor(pose12, 1.0);
    //     problem.AddResidualBlock(relpose_factor, loss_function, keyframePoses_data[keyframe1.first].pose, keyframePoses_data[keyframe1.first+1].pose);
    // }

    for (const auto &landmark : active_landmarks) {
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        auto ref_frame = mappoint->referenceFrame();
        if (!(active_kfs.find(ref_frame->keyFrameId()) != active_kfs.end())){  // Original !map_->isKeyFrameInMap(ref_frame)) {
            continue;
        }

        auto ref_frame_pc      = camera_->pixel2cam(mappoint->referenceKeypoint());
        // size_t ref_frame_index = getStateDataIndex(ref_frame->stamp());
        // if (ref_frame_index < 0) {
        //     continue;
        // }

        double *invdepth = &invdepthlist_[mappoint->id()];
        if (*invdepth == 0) {
            *invdepth = 1.0 / MapPoint::DEFAULT_DEPTH;
        }

        auto ref_feature = ref_frame->features().find(mappoint->id())->second;

        // if(dem_->isWithinRegion(mappoint->pos()(0), mappoint->pos()(1)))
        // {
        //    auto mp_factor = new MappointDEMFactorInvDepth(dem_, mappoint->pos()(2), ref_frame_pc, mappoint->pos());
        //    auto mp_residual_block_id = problem.AddResidualBlock(mp_factor, loss_function, keyframePoses_data[ref_frame->keyFrameId()].pose, invdepth);
        // }
        auto observations = mappoint->observations();
        for (auto &observation : observations) {
            auto obs_feature = observation.lock();
            if (!obs_feature || obs_feature->isOutlier()) {
                continue;
            }
            auto obs_frame = obs_feature->getFrame();
            if (!obs_frame || !obs_frame->isKeyFrame() || !(active_kfs.find(obs_frame->keyFrameId()) != active_kfs.end()) /* original : !map_->isKeyFrameInMap(obs_frame) */ || (obs_frame == ref_frame)) {
                continue;
            }

            auto obs_frame_pc      = camera_->pixel2cam(obs_feature->keyPoint());
            // size_t obs_frame_index = getStateDataIndex(obs_frame->stamp());

            // if ((obs_frame_index < 0) || (ref_frame_index == obs_frame_index)) {
            //     // LOGE << "Wrong matched mapoint keyframes " << Logging::doubleData(ref_frame->stamp()) << " with "
            //     //      << Logging::doubleData(obs_frame->stamp());
            //     continue;
            // }

            auto factor = new ReprojectionFactor(ref_frame_pc, obs_frame_pc, ref_feature->velocityInPixel(),
                                                 obs_feature->velocityInPixel(), ref_frame->timeDelay(),
                                                 obs_frame->timeDelay(), 3.0);
            auto residual_block_id =
                problem.AddResidualBlock(factor, loss_function, keyframePoses_data[ref_frame->keyFrameId()].pose,
                                         keyframePoses_data[obs_frame->keyFrameId()].pose, extrinsic_, invdepth, &extrinsic_[7]);
            // residual_ids.push_back(residual_block_id);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 20;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    LOG(INFO) << summary.FullReport();

    
    // Update map from optimzed results
    for (auto &keyframePose: keyframePoses_data)
    {
        Pose pose;
        Quaterniond q;

        memcpy(pose.t.data(), keyframePose.second.pose, sizeof(double) * 3);
        memcpy(q.coeffs().data(), keyframePose.second.pose+3, sizeof(double) * 4);
        pose.R = Rotation::quaternion2matrix(q);
        active_kfs[keyframePose.first]->setPose(pose);
    }

    for (const auto &landmark : active_landmarks) {
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        auto frame = mappoint->referenceFrame();
        if (!frame || !(active_kfs.find(frame->keyFrameId()) != active_kfs.end())){ // orignal !map_->isKeyFrameInMap(frame)) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        double invdepth = invdepthlist_[mappoint->id()];
        double depth    = 1.0 / invdepth;

        auto pc0      = camera_->pixel2cam(mappoint->referenceKeypoint());
        Vector3d pc00 = {pc0.x(), pc0.y(), 1.0};
        pc00 *= depth;

        mappoint->pos() = camera_->cam2world(pc00, mappoint->referenceFrame()->pose());
//        std::cout << mappoint->pos()(2) - dem_->getDEMAtLocation(mappoint->pos()(0), mappoint->pos()(1)) << std::endl;
        mappoint->updateDepth(depth);
    }



    // if (map_->keyframes().empty()) {
    //     return false;
    // }

    // 移除非关键帧中的路标点, 不能在遍历中直接移除, 否则破坏了遍历
    // Find outliers first and remove later
    vector<MapPoint::Ptr> mappoints;
    int num_outliers_mappoint = 0;
    int num_outliers_feature  = 0;
    int num1 = 0, num2 = 0, num3 = 0;
    for (auto &landmark : map_->landmarks()) {
        auto mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        // 未参与优化的无效路标点
        // Only those in the sliding window
        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        // 路标点在滑动窗口内的所有观测
        // All the observations for mappoint
        vector<double> errors;
        for (auto &observation : mappoint->observations()) {
            auto feat = observation.lock();
            if (!feat || feat->isOutlier()) {
                continue;
            }
            auto frame = feat->getFrame();
            if (!frame || !frame->isKeyFrame() || !map_->isKeyFrameInMap(frame)) {
                continue;
            }

            auto pp = feat->keyPoint();

            // 计算重投影误差
            // Calculate the reprojection error
            double error = camera_->reprojectionError(frame->pose(), mappoint->pos(), pp).norm();

            // 大于3倍阈值, 则禁用当前观测
            // Feature outlier
            if (!tracking_->isGoodToTrack(pp, frame->pose(), mappoint->pos(), 3.0)) {
                feat->setOutlier(true);
                mappoint->decreaseUsedTimes();

                // 如果当前观测帧是路标点的参考帧, 直接设置为outlier
                // Mappoint
                if (frame->id() == mappoint->referenceFrameId()) {
                    mappoint->setOutlier(true);
                    mappoints.push_back(mappoint);
                    num_outliers_mappoint++;
                    num1++;
                    break;
                }
                num_outliers_feature++;
            } else {
                errors.push_back(error);
            }
        }

        // 有效观测不足, 平均重投影误差较大, 则为粗差
        // Mappoint outlier
        if (errors.size() < 2) {
            mappoint->setOutlier(true);
            mappoints.push_back(mappoint);
            num_outliers_mappoint++;
            num2++;
        } else {
            double avg_error = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
            if (avg_error > min_reprojerr_) {
                mappoint->setOutlier(true);
                mappoints.push_back(mappoint);
                num_outliers_mappoint++;
                num3++;
            }
        }
    }

    // 移除outliers
    // Remove the mappoint outliers
    for (auto &mappoint : mappoints) {
        map_->removeMappoint(mappoint);
    }

    LOG(INFO) << "Culled " << num_outliers_mappoint << " mappoint with " << num_outliers_feature << " bad observed features "
         << num1 << ", " << num2 << ", " << num3;

    if(map_->isMaximumKeframes())
    {
        auto frame = map_->oldestKeyFrame();
        frame->resetKeyFrame();

        // vector<ulong> keyframeids = map_->orderedKeyFrames();
        // auto latest_keyframe      = map_->latestKeyFrame();

        // latest_keyframe->setKeyFrameState(KEYFRAME_NORMAL);

        // The marginalized mappoints, for visualization
        // frame    = map_->keyframes().at(keyframeids[0]);
        auto features = frame->features();
        for (const auto &feature : features) {
            auto mappoint = feature.second->getMapPoint();
            if (feature.second->isOutlier() || !mappoint || mappoint->isOutlier()) {
                continue;
            }
            auto &pw = mappoint->pos();

            //drawer_->addNewFixedMappoint(pw);

            // Save these mappoints to file 
            //ptsfilesaver_->dump(vector<double>{pw.x(), pw.y(), pw.z()});
        }
        map_->removeKeyFrame(frame, true);
    }
#else
    // Map::ParamsType para_kfs = map_->GetPoseParams();
    // Map::ParamsType para_landmarks = map_->GetPointParams();
    Map::KeyFrames active_kfs = map_->keyframes();
    Map::LandMarks active_landmarks = map_->landmarks();

    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    ceres::LocalParameterization *local_parameterization = new SE3Parameterization();
    std::unordered_map<int, SE3> keyframePoses;
    for (auto &keyframe : active_kfs)
    {   
        Mat33 R = keyframe.second->pose().R;
        R = R + 0.5 * (Mat33::Identity() - R * R.transpose()) * R; // Todo
        SE3 pose(R, keyframe.second->pose().t);
        keyframePoses.insert({keyframe.first, pose});
        double *para = keyframePoses[keyframe.first].data();
        problem.AddParameterBlock(para, SE3::num_parameters, local_parameterization);
    }

    std::unordered_map<int, Vec3> mapPoints;
    for (auto &landmark : active_landmarks)
    {
        if (landmark.second->isOutlier())
            continue;
        mapPoints.insert({landmark.first, landmark.second->pos()});
        double *para = mapPoints[landmark.first].data();
        // double *para = landmark.second->Pos().data();
        // double *para = para_landmarks[landmark.first];
        problem.AddParameterBlock(para, 3);
        auto observations = landmark.second->observations();
        for (auto &obs : observations)
        {
            if (obs.lock() == nullptr)
                continue;
            auto feature = obs.lock();
            if (feature->isOutlier() || feature->getFrame() == nullptr)
                continue;
            auto frame = feature->getFrame();
            auto iter = active_kfs.find(frame->keyFrameId());
            if (iter == active_kfs.end())
                continue;
            auto keyframe = *iter;

            ceres::CostFunction *cost_function;
            cost_function = new ReprojectionError(toVec2(feature->keyPoint()), camera_);
            // problem.AddResidualBlock(cost_function, loss_function, feature->getFrame()->Pose().data(), para);
            problem.AddResidualBlock(cost_function, loss_function, keyframePoses[keyframe.first].data(), para);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 20;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // LOG(INFO) << summary.FullReport();


    // // Update map from optimzed results
    for (auto &keyframePose: keyframePoses)
    {
        Pose pose({keyframePose.second.rotationMatrix(), keyframePose.second.translation()});
        active_kfs[keyframePose.first]->setPose(pose);
    }
    for (auto &mp : mapPoints)
    {
        active_landmarks[mp.first]->pos() = (mp.second);
    }

    // // reject outliers
    // int cnt_outlier = 0, cnt_inlier = 0;
    // for (auto &landmark : active_landmarks)
    // {
    //     if (landmark.second->isOutlier())
    //         continue;
    //     auto observations = landmark.second->observations();
    //     for (auto &obs : observations)
    //     {
    //         if (obs.lock() == nullptr)
    //             continue;
    //         auto feature = obs.lock();
    //         if (feature->isOutlier() || feature->getFrame() == nullptr)
    //             continue;
    //         auto frame = feature->getFrame();
    //         auto iter = active_kfs.find(frame->keyFrameId());
    //         if (iter == active_kfs.end())
    //             continue;
    //         auto keyframe = (*iter).second;

    //         Vec2 error =
    //             toVec2(feature->keyPoint()) - toVec2(camera_->world2pixel(landmark.second->pos(), keyframe->pose()));
    //         if (error.norm() > min_reprojerr_)
    //         {
    //            landmark.second->setOutlier(true);
    //             feature->setOutlier(true);                
    //             landmark.second->removeAllObservations();
    //             cnt_outlier++;
    //         }
    //         else
    //         {
    //             cnt_inlier++;
    //         }
    //     }
    // }
    // LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/" << cnt_inlier;

    #endif
}
