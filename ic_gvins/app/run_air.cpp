#include "visual_odometry.h"

int main(int argc, char **argv)
{
    std::string config_file = "./config/air.yaml";
    VisualOdometry::Ptr vo(new VisualOdometry(config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
