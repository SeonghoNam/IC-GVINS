#include "visual_odometry.h"
#include <yaml-cpp/yaml.h>
#include "dataset.h"
#include "ic_gvins2.h"
#include "drawer_pangolin.h"
#include "common/earth.h"
#include "common/rotation.h"
#include "common/logging.h"
int main(int argc, char **argv)
{
    Logging::initialization(argv, true, true);
    std::string config_file = "../config/kitti.yaml";
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return false;
    }

    auto str_dataset_dir = config["dataset_dir"].as<std::string>();

    KITTIDataset dataset_(str_dataset_dir);

    // VisualOdometry::Ptr vo(new VisualOdometry(config_file));
    // assert(vo->Init() == true);
    // int image_index = 0;
    // while(1)
    // {
    //     Frame::Ptr new_frame = dataset_.CreateFrame(image_index);
    //     if(new_frame == nullptr)
    //         break;
    //     else
    //     {
    //         new_frame->setPose(dataset_.GetPose(image_index));
    //         vo->Step(new_frame);
    //     }
    //     image_index++;
    // }
    // vo->Finish();


    GVINS2::Ptr gvins_ = nullptr;
    std::string outputpath = "../output/";

    Drawer::Ptr drawer = std::make_shared<DrawerPangolin>();
    gvins_             = std::make_shared<GVINS2>(config_file, outputpath, drawer);

    int image_index = 0;
    Vector3d origin (39.9928 * D2R, 116.3529 * D2R, 0.0);
    gvins_->setWorldOrigin(origin);
    while(1)
    {
        Frame::Ptr new_frame = dataset_.CreateFrame(image_index);
        if(new_frame == nullptr)
            break;
        else
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
            }
            else
            {
                usleep(3000);
            }
        }
        image_index++;
    }
    gvins_->setFinished();

    return 0;
}
