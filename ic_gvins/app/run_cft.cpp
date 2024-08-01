#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include "dataset.h"
#include "ic_gvins2.h"
#include "drawer_pangolin.h"
#include "common/earth.h"
#include "common/rotation.h"

#include "common/logging.h"
int main(int argc, char **argv)
{
    Logging::initialization(argv, true, true);
    std::string config_file = "../config/cft.yaml";
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return false;
    }

    auto str_dataset_dir = config["dataset_dir"].as<std::string>();

    CFTDataset dataset_(str_dataset_dir);

    GVINS2::Ptr gvins_ = nullptr;
    auto outputpath = config["outputpath"].as<std::string>();
    auto is_make_outputdir = config["is_make_outputdir"].as<bool>();
    if (is_make_outputdir) {
        boost::filesystem::create_directory(outputpath);
    }

    Drawer::Ptr drawer = std::make_shared<DrawerPangolin>();
    gvins_             = std::make_shared<GVINS2>(config_file, outputpath, drawer);

    int image_index = 0;
    Vector3d origin (35.6900 * D2R, 127.1850 * D2R, 0.0);
    gvins_->setWorldOrigin(origin);
    Frame::Ptr new_frame = dataset_.CreateFrame(image_index);
    while(new_frame != nullptr)
    {
        if(gvins_->isFrameBufferEmpty())
        {
            PVA pva;
            Pose pose = dataset_.GetPose(image_index);
            pva.time = new_frame->stamp()+0.005;
            pva.blh = Earth::local2global(origin, pose.t);
            pva.att = Rotation::matrix2euler(pose.R);
            new_frame->setPose(pose);
            gvins_->addNewFrame(new_frame);
            gvins_->addNewPVA(pva);
            image_index++;
            new_frame = dataset_.CreateFrame(image_index);
        }
        else
        {
            usleep(3000);
        }

    }
    gvins_->setFinished();

    return 0;
}
