#include "visual_odometry.h"
#include "drawer_pangolin.h"
#include "common/logging.h"

#include <chrono>
#include <fstream>
#include <yaml-cpp/yaml.h>

VisualOdometry::VisualOdometry(std::string &config_path) : config_file_path_(config_path)
{
    const std::string outputpath = "../output";
    // Output files
    // navfilesaver_    = FileSaver::create(outputpath + "/gvins.nav", 11);
    ptsfilesaver_    = FileSaver::create(outputpath + "/mappoint.txt", 3);
    // statfilesaver_   = FileSaver::create(outputpath + "/statistics.txt", 3);
    // extfilesaver_    = FileSaver::create(outputpath + "/extrinsic.txt", 3);
    // imuerrfilesaver_ = FileSaver::create(outputpath + "/IMU_ERR.bin", 7, FileSaver::BINARY);
    trajfilesaver_   = FileSaver::create(outputpath + "/trajectory.csv", 8);

    // if (!navfilesaver_->isOpen() || !ptsfilesaver_->isOpen() || !statfilesaver_->isOpen() || !extfilesaver_->isOpen()) {
    if (!ptsfilesaver_->isOpen() || !trajfilesaver_->isOpen()) {
        LOGE << "Failed to open data file";
        return;
    }
}

bool VisualOdometry::Init()
{
    YAML::Node config;
    // std::string configfile = "./config/default.yaml";
    try {
        config = YAML::LoadFile(config_file_path_);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return false;
    }


        // Camera parameters
    vector<double> intrinsic  = config["cam0"]["intrinsic"].as<std::vector<double>>();
    vector<double> distortion = config["cam0"]["distortion"].as<std::vector<double>>();
    vector<int> resolution    = config["cam0"]["resolution"].as<std::vector<int>>();

    cam_ = Camera::createCamera(intrinsic, distortion, resolution);

    map_    = std::make_shared<Map>(20);
    drawer_ = std::make_shared<DrawerPangolin>();
    drawer_->setMap(map_);

    // if (is_use_visualization_) {
        drawer_thread_ = std::thread(&Drawer::run, drawer_);
    // }

    frontend_ = std::make_shared<Tracking>(cam_, map_, drawer_, config_file_path_, "../output/");

    backend_ = std::make_shared<Backend>();
    
    backend_->SetMap(map_);
    backend_->SetCameras(cam_);
    backend_->SetTracking(frontend_);

    return true;
}

void VisualOdometry::Run()
{
    std::ofstream file_trajectory("../output/00_pred.txt");

    while (1)
    {
        if (false)
        {
            Pose pose = frontend_->getCurrentPose();
            Eigen::Matrix4d Twc;
            Twc.setZero();
            Twc(3, 3) = 1;

            Twc.block<3, 3>(0, 0) = pose.R;
            Twc.block<3, 1>(0, 3) = pose.t;

            file_trajectory << Twc(0,1) << " " << Twc(0,1) << " " << Twc(0,2) << " " << Twc(0,3) << " "
                            << Twc(1,1) << " " << Twc(1,1) << " " << Twc(1,2) << " " << Twc(1,3) << " "
                            << Twc(2,1) << " " << Twc(2,1) << " " << Twc(2,2) << " " << Twc(2,3) << "\n";

        }
        else
        {
            break;
        }
    }
}

void VisualOdometry::Finish()
{
    backend_->Stop();
    drawer_->setFinished();
    drawer_thread_.join();
    LOGI << "VO exit";
}
bool VisualOdometry::Step(Frame::Ptr new_frame)
{
    if (new_frame == nullptr)
        return false;

    TrackState trackstate = frontend_->track(new_frame);
    if(frontend_->isNewKeyFrame() || trackstate == TRACK_FIRST_FRAME || trackstate == TRACK_LOST) 
    {
        map_->insertKeyFrame(new_frame);
        if(trackstate != TRACK_FIRST_FRAME)
            backend_->UpdateMap();

        if(map_->isMaximumKeframes())
        {
            auto frame = map_->oldestKeyFrame();
            // frame->resetKeyFrame();

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

                drawer_->addNewFixedMappoint(pw);

                // Save these mappoints to file 
                ptsfilesaver_->dump(vector<double>{pw.x(), pw.y(), pw.z()});
            }
            // map_->removeKeyFrame(frame, true);
        }
        
        drawer_->updateMap(frontend_->pose2Twc(frontend_->getCurrentPose()));
    }
    return true;
}
