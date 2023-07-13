#ifndef vslam_BACKEND_H
#define vslam_BACKEND_H

#include <thread>
#include <condition_variable>

#include "tracking/frame.h"
#include "tracking/map.h"
#include "tracking/tracking.h"
#include "DEM.h"

class Map;

class Backend
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    Backend();

    void SetCameras(Camera::Ptr left)
    {
        camera_ = left;
    }

    void SetMap(std::shared_ptr<Map> map)
    {
        map_ = map;
    }

    void SetTracking(std::shared_ptr<Tracking> tracking)
    {
        tracking_ = tracking;
    }

    void UpdateMap();

    void Stop();

  private:
    void BackendLoop();

    void Optimize();

    bool CullingOutliers();

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;
    std::shared_ptr<DEM> dem_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr camera_ = nullptr;
    Tracking::Ptr tracking_ = nullptr;

    double extrinsic_[8]{0};
    
    // parmaeter for backend
    double min_reprojerr_ = 1.5;
};

#endif // vslam_BACKEND_H
