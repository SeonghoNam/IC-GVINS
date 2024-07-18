#ifndef vslam_DATASET_H
#define vslam_DATASET_H

#include <random>
#include "common/types.h"
#include "tracking/frame.h"

class Dataset
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string &dataset_path);

    virtual bool Init() = 0;

    /// create and return the next frame containing the stereo images
    virtual Frame::Ptr NextFrame() = 0;


  protected:
    std::default_random_engine generator_;

    std::string dataset_path_;
    int current_image_index_ = 0;
    std::vector<Matrix4d> gt_poses_;
    std::vector<double> time_stamps_;
};

class KITTIDataset : public Dataset
{
  public:
    KITTIDataset(const std::string &dataset_path) : Dataset(dataset_path)
    {
      Init();
    }
    bool Init();
    Frame::Ptr NextFrame(){};
    Frame::Ptr CreateFrame(int image_index);
    Pose GetPose(int image_index);
};

class AIRDataset : public Dataset
{
  public:
    AIRDataset(const std::string &dataset_path) : Dataset(dataset_path)
    {
      Init();
    }
    bool Init();
    Frame::Ptr NextFrame();

  private:
    std::vector<std::string> filenames_;
};

class AerialImageDataset : public Dataset
{
  public:
    AerialImageDataset(const std::string &dataset_path) : Dataset(dataset_path)
    {
      Init();
    }
    bool Init();
    Frame::Ptr NextFrame();
    Frame::Ptr CreateFrame(int image_index);
    Pose GetPose(int image_index);
};

class CFTDataset : public Dataset
{
  public:
    CFTDataset(const std::string &dataset_path) : Dataset(dataset_path)
    {
      Init();
    }
    bool Init();
    Frame::Ptr NextFrame();
    Frame::Ptr CreateFrame(int image_index);
    Pose GetPose(int image_index);

  private:
    std::vector<std::string> filenames_;
};

#endif
