
#ifndef vslam_VISUAL_ODOMETRY_H
#define vslam_VISUAL_ODOMETRY_H


#include <thread>

#include "fileio/filesaver.h"
#include "common/types.h"
#include "tracking/tracking.h"
#include "dataset.h"
#include "tracking/drawer.h"
#include "backend.h"

class VisualOdometry
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    /// conclassor with config file
    VisualOdometry(std::string &config_path);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * start vo in the dataset
     */
    void Run();

    /**
     * Make a step forward in dataset
     */
    bool Step();

  private:
    bool inited_ = false;
    std::string config_file_path_;

    Tracking::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Drawer::Ptr drawer_ = nullptr;

    // dataset
    Dataset::Ptr dataset_ = nullptr;

    std::thread drawer_thread_;

    FileSaver::Ptr ptsfilesaver_;
    FileSaver::Ptr trajfilesaver_;

};

#endif // vslam_VISUAL_ODOMETRY_H
