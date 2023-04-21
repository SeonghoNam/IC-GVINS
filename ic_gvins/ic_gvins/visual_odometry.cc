#include "visual_odometry.h"
#include "drawer_pangolin.h"
#include "common/logging.h"

#include <chrono>
#include <fstream>
#include <yaml-cpp/yaml.h>

VisualOdometry::VisualOdometry(std::string &config_path) : config_file_path_(config_path)
{
}

bool VisualOdometry::Init()
{
    YAML::Node config;
    std::string configfile = "./config/default.yaml";
    try {
        config = YAML::LoadFile(configfile);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return false;
    }

    // bool is_use_visualization_ = config["is_use_visualization"].as<bool>();

    DatasetType dataset_type;
    auto str_dataset_type = config["dataset_type"].as<std::string>();
    auto str_dataset_dir = config["dataset_dir"].as<std::string>();

    if (str_dataset_type== "KITTI")
        dataset_ =
            std::static_pointer_cast<Dataset>(std::make_shared<KITTIDataset>(str_dataset_dir));
    else if (str_dataset_type == "AIR")
        dataset_ =
            std::static_pointer_cast<Dataset>(std::make_shared<AIRDataset>(str_dataset_dir));

    // create components and links
    dataset_->Init();


    map_    = std::make_shared<Map>(20);
    drawer_ = std::make_shared<DrawerPangolin>();
    drawer_->setMap(map_);

    // if (is_use_visualization_) {
        drawer_thread_ = std::thread(&Drawer::run, drawer_);
    // }

    frontend_ = std::make_shared<Tracking>(dataset_->GetCamera(0), map_, drawer_, configfile, "./output/");

    // backend_ = Backend::Ptr(new Backend);
    
    // backend_->SetMap(map_);
    // backend_->SetCameras(dataset_->GetCamera(0));

    return true;
}

void VisualOdometry::Run()
{
    std::ofstream file_trajectory("./output/00_pred.txt");

    while (1)
    {
        if (Step())
        {
            Pose pose = frontend_->getCurrentPose();
            Eigen::Matrix4d Twc;
            Twc.setZero();
            Twc(3, 3) = 1;

            Twc.block<3, 3>(0, 0) = pose.R;
            Twc.block<3, 1>(0, 3) = pose.t;

            file_trajectory << Twc.row(0) << " " << Twc.row(1) << " " << Twc.row(2) << "\n";
        }
        else
        {
            break;
        }
    }

    // backend_->Stop();
    LOGI << "VO exit";
}

bool VisualOdometry::Step()
{
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr)
        return false;

    TrackState trackstate = frontend_->track(new_frame);
    if(frontend_->isNewKeyFrame() || trackstate == TRACK_FIRST_FRAME || trackstate == TRACK_LOST) 
    {
        map_->insertKeyFrame(new_frame);
        // if(trackstate != TRACK_FIRST_FRAME)
        //     backend_->UpdateMap();

        while(map_->isMaximumKeframes())
        {
            auto frame = map_->oldestKeyFrame();
            frame->resetKeyFrame();
            map_->removeKeyFrame(frame, false);
        }
        drawer_->updateMap(Tracking::pose2Twc(frontend_->getCurrentPose()));
    }
    return true;
}
