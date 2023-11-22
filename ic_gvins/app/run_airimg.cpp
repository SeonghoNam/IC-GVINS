#include "visual_odometry.h"
#include <yaml-cpp/yaml.h>
#include "dataset.h"

int main(int argc, char **argv)
{
    std::string config_file = "../config/airimg.yaml";
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return false;
    }

    auto str_dataset_dir = config["dataset_dir"].as<std::string>();

    AerialImageDataset dataset_(str_dataset_dir);

    VisualOdometry::Ptr vo(new VisualOdometry(config_file));

    
    assert(vo->Init() == true);
    while(1)
    {
        Frame::Ptr new_frame = dataset_.NextFrame();
        if(new_frame == nullptr)
            break;
        else
            vo->Step(new_frame);
    }

    vo->Finish();

    return 0;
}
