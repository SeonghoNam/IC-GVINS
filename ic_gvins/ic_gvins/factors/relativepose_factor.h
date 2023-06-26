
#ifndef RELATIVEPOSE_FACTOR_H
#define RELATIVEPOSE_FACTOR_H

#include <ceres/ceres.h>

#include <Eigen/Geometry>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "common/rotation.h"
#include "common/types.h"

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

using Sophus::SE3d;
using Sophus::SO3d;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// 给定误差求J_R^{-1}的近似
Matrix6d JRInv(const SE3d &e) {
    Matrix6d J;
    J.block(0, 0, 3, 3) = SO3d::hat(e.so3().log());
    J.block(0, 3, 3, 3) = SO3d::hat(e.translation());
    J.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
    J.block(3, 3, 3, 3) = SO3d::hat(e.so3().log());
    // J = J * 0.5 + Matrix6d::Identity();
    J = Matrix6d::Identity();    // try Identity if you want
    return J;
}

class RelativePoseFactor : public ceres::SizedCostFunction<6, 7, 7> {

public:
    RelativePoseFactor() = delete;

    RelativePoseFactor(Pose poseij, double std)
    { 
        _measure.setRotationMatrix(poseij.R);
        _measure.translation() = poseij.t;
        
        sqrt_info_.setZero();
        sqrt_info_(0, 0) = 1.0 / std;
        sqrt_info_(1, 1) = 1.0 / std;
        sqrt_info_(2, 2) = 1.0 / std;
        sqrt_info_(3, 3) = 1.0 / std;
        sqrt_info_(4, 4) = 1.0 / std;
        sqrt_info_(5, 5) = 1.0 / std;

    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        // 参考帧位姿
        Eigen::Vector3d p0(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond q0(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d p1(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond q1(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        SE3d T_cur(q1, p1);
        SE3d T_prv(q0, p0);

        Eigen::Map<Vector6d> error(residuals);

        error = (_measure.inverse() * T_cur.inverse() * T_prv).log();
        error = sqrt_info_ * error;
        // // 相机外参
        // Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
        // Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        // // 逆深度
        // double id0 = parameters[3][0];

        // // 时间延时
        // double td = parameters[4][0];

        // Eigen::Map<Vector6d> error(residuals);

        // Quaterniond qij = Rotation::matrix2quaternion(poseij_.R);

        // Quaterniond err_q = qij.inverse() * q1.inverse() * q0;
        // Eigen::Vector3d err_t = - (qij.inverse() * poseij_.t)
        //                         - (qij.inverse() * q1.inverse() * p1)
        //                         + (qij.inverse() * q1.inverse() * p0);
        // Eigen::Vector3d err_rot = Rotation::quaternion2vector(err_q);
        // error = Vector6d{err_t.x(), err_t.y(), err_t.z(), err_rot.x(), err_rot.y(), err_rot.z()};
        // error = sqrt_info_ * error;

        if (jacobians)
        {
            Matrix6d J = JRInv(SE3d::exp(error));
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobianXi(jacobians[0]);
                // jacobianXi = -J * T_cur.inverse().Adj();
                // 尝试把J近似为I？
                jacobianXi.setZero();
                jacobianXi.block<6,6>(0,0) = -J * T_cur.inverse().Adj();
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobianXj(jacobians[1]);
                // jacobianXj = J * T_cur.inverse().Adj();
                jacobianXj.setZero();
                jacobianXj.block<6,6>(0,0) = J * T_cur.inverse().Adj();
            }
        }
        return true;
    }

private:
    SE3d _measure;
    Pose poseij_;
    
    Matrix6d sqrt_info_;
};

#endif // RELATIVEPOSE_FACTOR_H
