#pragma once
#include "DEM.h"
#include <ceres/ceres.h>

#include "common/rotation.h"

class MappointDEMFactor: public ceres::SizedCostFunction<1,3>
{
  public:
    MappointDEMFactor(std::shared_ptr<DEM> pDEM, double height)
        :pDEM_(pDEM), height_(height)
    {
        sqrt_information_ = 1./10.;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {      
        Eigen::Map<Eigen::Vector3d const> xyz(parameters[0]);
        // Eigen::Map<double> residual(residuals);

        std::cout << "parameter : " << xyz << "\n";
        *residuals = sqrt_information_ * (height_ - pDEM_->getDEMAtLocation(xyz(0), xyz(1)));
        std::cout << "residual : " << *residuals << "\n";
        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian(jacobians[0]);
                // jacobian is gradiant of DEM
                jacobian(0, 0) = pDEM_->getJacobianXAt(xyz(0), xyz(1)); 
                jacobian(0, 1) = pDEM_->getJacobianYAt(xyz(0), xyz(1));
                jacobian(0, 2) = 0.; // altitude jacobian

                jacobian = sqrt_information_ * jacobian;
                 std::cout << "jacobian : " << jacobian << "\n";
                // jacobian = sqrt_information_ * jaco_res_2_Pc * jaco_Pc_2_Pose;
            }
        }

        return true;
    }
  private:
    Eigen::Vector3d pos_;
    double height_;
    std::shared_ptr<DEM> pDEM_;
    double sqrt_information_;
};

class MappointDEMFactorInvDepth: public ceres::SizedCostFunction<1,7,1>
{
  public:
    MappointDEMFactorInvDepth(std::shared_ptr<DEM> pDEM, double height, Eigen::Vector3d pts0, Eigen::Vector3d pos)
        :pDEM_(pDEM), height_(height), pts0_(pts0), pos_(pos)
    {
        sqrt_information_ = 1./1.;
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {      
        // parameters[0] : reference frame pose(p0, q0) (dim 7)
        // parameters[1] : inverse depth (dim 1)
        Eigen::Vector3d p0(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond q0(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        double inv_depth = parameters[1][0];


        // Eigen::Vector3d pts_0_td = pts0_ - (td - td0_) * vel0_;
        // Eigen::Vector3d pts_1_td = pts1_ - (td - td1_) * vel1_;

        Eigen::Vector3d pts_c_0 = pts0_ / inv_depth;
        // Eigen::Vector3d pts_b_0 = qic * pts_c_0 + tic;
        Eigen::Vector3d pts_n   = q0 * pts_c_0 + p0;
        // Eigen::Vector3d pts_b_1 = q1.inverse() * (pts_n - p1);
        // Eigen::Vector3d pts_1   = qic.inverse() * (pts_b_1 - tic);

        Eigen::Vector3d xyz = pts_n;

        // Eigen::Map<Eigen::Vector3d const> xyz(parameters[0]);
        // Eigen::Map<double> residual(residuals);

        *residuals = sqrt_information_ * (height_ - pDEM_->getDEMAtLocation(xyz(0), xyz(1)));
        std::cout << (height_ - pDEM_->getDEMAtLocation(xyz(0), xyz(1))) << "\n";
        if (jacobians)  // jacobians of residual z-h(x) not h(x)!!!!
        {
            Eigen::Matrix3d rotation_matrix = q0.toRotationMatrix();
            Eigen::Matrix<double, 1, 3> jaco_h_mappoint;

            // jacobian is gradiant of DEM
            jaco_h_mappoint(0, 0) = pDEM_->getJacobianXAt(xyz(0), xyz(1)); 
            jaco_h_mappoint(0, 1) = pDEM_->getJacobianYAt(xyz(0), xyz(1));
            jaco_h_mappoint(0, 2) = 0.; // altitude jacobian


            if (jacobians[0])   // pose
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jaco_ref_pose(jacobians[0]);
                jaco_ref_pose.block<1,3>(0,0) = -jaco_h_mappoint;
                // below two lines are the same
                // jaco_ref_pose.block<1,3>(0,3) = jaco_h_mappoint * Rotation::skewSymmetric(rotation_matrix * 1/inv_depth * pts0_);
                jaco_ref_pose.block<1,3>(0,3) = jaco_h_mappoint * Rotation::skewSymmetric(q0 * pts_c_0);
                jaco_ref_pose(0,6) = 0.;

                // no rotation jacobian (no optimization on rotation)
                // jaco_ref_pose.block<1,4>(0,3) = Eigen::Matrix<double, 1, 4>::Zero();
            }

            if (jacobians[1])   // inv_depth
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jaco_invdepth(jacobians[1]);
                // jaco_invdepth = (1. / (inv_depth * inv_depth) ) * jaco_h_mappoint * rotation_matrix * pts0_;
                jaco_invdepth = (1. / inv_depth ) * jaco_h_mappoint * (q0 * pts_c_0);

                // jaco_invdepth(0,0) = 0.;
            }
        }

        return true;
    }
  private:
    Eigen::Vector3d pts0_; // [u_c v_c 1]
    Eigen::Vector3d pos_;
    double height_;
    std::shared_ptr<DEM> pDEM_;
    double sqrt_information_;
};