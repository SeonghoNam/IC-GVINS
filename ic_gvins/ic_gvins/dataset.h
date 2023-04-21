#ifndef vslam_DATASET_H
#define vslam_DATASET_H

#include <random>
#include "common/types.h"
#include "tracking/camera.h"
#include "tracking/frame.h"


enum class DatasetType
{
    KITTI,
    AIR,
    EUROC
};

class Dataset
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string &dataset_path);

    virtual bool Init() = 0;

    /// create and return the next frame containing the stereo images
    virtual Frame::Ptr NextFrame() = 0;

    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const
    {
        return cameras_.at(camera_id);
    }

  protected:
    std::default_random_engine generator_;

    DatasetType type_;
    std::string dataset_path_;
    int current_image_index_ = 0;
    std::vector<Matrix4d> gt_poses_;
    std::vector<double> time_stamps_;

    std::vector<Camera::Ptr> cameras_;
};

class KITTIDataset : public Dataset
{
  public:
    KITTIDataset(const std::string &dataset_path) : Dataset(dataset_path)
    {
    }
    bool Init();
    Frame::Ptr NextFrame();
};

class AIRDataset : public Dataset
{
  public:
    AIRDataset(const std::string &dataset_path) : Dataset(dataset_path)
    {
    }
    bool Init();
    Frame::Ptr NextFrame();

  private:
    std::vector<std::string> filenames_;
};

#endif
