#include "dataset.h"
#include "tracking/frame.h"
#include "common/logging.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
using namespace std;


Dataset::Dataset(const std::string &dataset_path) : dataset_path_(dataset_path)
{
}

bool KITTIDataset::Init(std::string config_file)
{
    YAML::Node config;
    // std::string configfile = "./config/default.yaml";
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return false;
    }
    // Camera parameters
    vector<double> intrinsic  = config["cam0"]["intrinsic"].as<std::vector<double>>();
    vector<double> distortion = config["cam0"]["distortion"].as<std::vector<double>>();
    vector<int> resolution    = config["cam0"]["resolution"].as<std::vector<int>>();

    Camera::Ptr camera1 = Camera::createCamera(intrinsic, distortion, resolution);
    cameras_.push_back(camera1);

    // Load timestamps
    boost::format fmt_t("%s/times.txt");
    ifstream file_time((fmt_t % dataset_path_).str());
    if (file_time.is_open())
    {
        double time;
        while (!file_time.eof())
        {
            file_time >> time;
            time_stamps_.emplace_back(time);
        }
    }

    // Load gt
    boost::format fmt("%s/gt.txt");
    ifstream file_gt((fmt % dataset_path_).str());
    if (file_gt.is_open())
    {
        Matrix4d poses;
        std::string line;
        while (!file_gt.eof())
        {
            std::getline(file_gt, line);
            std::istringstream iss(line);
            poses.setIdentity();
            if (iss >> poses(0, 0) >> poses(0, 1) >> poses(0, 2) >> poses(0, 3) >> poses(1, 0) >> poses(1, 1) >>
                poses(1, 2) >> poses(1, 3) >> poses(2, 0) >> poses(2, 1) >> poses(2, 2) >> poses(2, 3))
            {
                gt_poses_.emplace_back(poses);
            }
        }
    }

    // Load timestamp

    current_image_index_ = 0;
    return true;
}

Frame::Ptr KITTIDataset::NextFrame()
{
    cv::Mat image;

    boost::format fmt("%s/image_%d/%06d.png");
    image = cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);

    if (image.data == nullptr)
    {
        LOGW << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

    // normalize rotation matrix due to precision error from reading data
    Matrix3d R = gt_poses_[current_image_index_].block<3, 3>(0, 0);
    R = R + 0.5 * (Matrix3d::Identity() - R * R.transpose()) * R;
    Vector3d t = gt_poses_[current_image_index_].block<3, 1>(0, 3);

    // add noise (to position)
    std::normal_distribution<double> distribution(0, 0.1 * 1/10);
    auto normal = [&] (double) {return distribution(generator_);};
    static Vector3d err;
    err += Vector3d::NullaryExpr(3, normal);
    cout << err << "\n";
    t = t + err;

    // SE3 pose(gt_poses_[current_image_index_]);
    // SE3 pose(R, t);

    auto new_frame = Frame::createFrame(time_stamps_[current_image_index_], image);
    new_frame->setPose(Pose({R,t}));
    current_image_index_++;
    return new_frame;
}

bool AIRDataset::Init(std::string config_file)
{
    Vector3d t(0, 0, 0);
    // first camera
    YAML::Node config;
    // std::string configfile = "./config/default.yaml";
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return false;
    }
    // Camera parameters
    vector<double> intrinsic  = config["cam0"]["intrinsic"].as<std::vector<double>>();
    vector<double> distortion = config["cam0"]["distortion"].as<std::vector<double>>();
    vector<int> resolution    = config["cam0"]["resolution"].as<std::vector<int>>();

    Camera::Ptr camera1 = Camera::createCamera(intrinsic, distortion, resolution);
    cameras_.push_back(camera1);

    // Load gt
    boost::format fmt("%s/metadata.csv");
    ifstream file_gt((fmt % dataset_path_).str());
    if (file_gt.is_open())
    {
        Matrix4d poses;
        double time = 0;
        std::string line;
        std::getline(file_gt, line);
        while (!file_gt.eof())
        {
            std::vector<std::string> items;
            std::getline(file_gt, line);
            std::istringstream iss(line);
            std::string field;
            while (std::getline(iss, field, ','))
            {
                items.emplace_back(field);
            }
            if(items.empty())
                continue;

            // get filename, filename as "image_00000,jpg", remove "" at the begin and end
            // items[0].erase(items[0].begin());
            // items[0].erase(items[0].end() - 1);
            
            filenames_.emplace_back(items[0]);

            // get pose
            poses.setIdentity();
            poses(0, 3) = std::stod(items[8]);
            poses(1, 3) = std::stod(items[9]);
            poses(2, 3) = std::stod(items[10]);
            // qw, qx, qy, qz in Eigen:Quaternion
            Eigen::Quaterniond quat(std::stod(items[14]), std::stod(items[11]), std::stod(items[12]),
                                    std::stod(items[13]));
            // Eigen::Quaterniond quat(std::stod(items[7]), std::stod(items[4]), std::stod(items[5]),
            //                         std::stod(items[6]));
            quat.normalize(); // normalize due to precision error from reading data;
            poses.block<3, 3>(0, 0) = quat.toRotationMatrix().transpose();

            gt_poses_.emplace_back(poses);

            // no time stamps in AIR dataset, so we generate
            time_stamps_.emplace_back(time);
            time += 1. / 50;
        }
    }

    // Load timestamp

    current_image_index_ = 0;
    return true;
}

Frame::Ptr AIRDataset::NextFrame()
{
    cv::Mat image;

    if (current_image_index_ >= filenames_.size())
    {
        LOGW << "End of dataset files";
        return nullptr;
    }

    boost::format fmt("%s/%s");
    image = cv::imread((fmt % dataset_path_ % filenames_[current_image_index_]).str(), cv::IMREAD_GRAYSCALE);

    if (image.data == nullptr)
    {
        LOGW << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

    // normalize rotation matrix due to precision error from reading data
    Matrix3d R = gt_poses_[current_image_index_].block<3, 3>(0, 0);
    R = R + 0.5 * (Matrix3d::Identity() - R * R.transpose()) * R;
    Vector3d t = gt_poses_[current_image_index_].block<3, 1>(0, 3);

    // SE3 pose(gt_poses_[current_image_index_]);
    // SE3 pose(R, t);

    auto new_frame = Frame::createFrame(time_stamps_[current_image_index_], image);
    new_frame->setPose(Pose({R,t}));
    current_image_index_++;
    return new_frame;
}


bool AerialImageDataset::Init(std::string config_file)
{
    Vector3d t(0, 0, 0);
    // first camera
    YAML::Node config;
    // std::string configfile = "./config/default.yaml";
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return false;
    }
    // Camera parameters
    vector<double> intrinsic  = config["cam0"]["intrinsic"].as<std::vector<double>>();
    vector<double> distortion = config["cam0"]["distortion"].as<std::vector<double>>();
    vector<int> resolution    = config["cam0"]["resolution"].as<std::vector<int>>();

    Camera::Ptr camera1 = Camera::createCamera(intrinsic, distortion, resolution);
    cameras_.push_back(camera1);

    // Load gt
    Matrix4d poses;
    double time = 0;
    for(int i = 0; i < 101; i++)
    {
        // get pose
        poses.setIdentity();
        poses(0, 3) = 100*i;
        

        gt_poses_.emplace_back(poses);

        // no time stamps in AIR dataset, so we generate
        time_stamps_.emplace_back(time);
        time += 1;
    }

    current_image_index_ = 0;
    return true;
}

Frame::Ptr AerialImageDataset::NextFrame()
{
    cv::Mat image;

    boost::format fmt("%s/imgseqpng%04d.png");
    image = cv::imread((fmt % dataset_path_ % (current_image_index_+1)).str(), cv::IMREAD_GRAYSCALE);

    if (image.data == nullptr)
    {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

    // normalize rotation matrix due to precision error from reading data
    Matrix3d R = gt_poses_[current_image_index_].block<3, 3>(0, 0);
    // R = R + 0.5 * (Mat33::Identity() - R * R.transpose()) * R;
    Vector3d t = gt_poses_[current_image_index_].block<3, 1>(0, 3);

    // SE3 pose(gt_poses_[current_image_index_]);

    auto new_frame = Frame::createFrame(time_stamps_[current_image_index_], image);
    new_frame->setPose(Pose{R, t});
    current_image_index_++;
    return new_frame;
}