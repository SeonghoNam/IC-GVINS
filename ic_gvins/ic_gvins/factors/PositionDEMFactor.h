#pragma once
#include "DEM.h"
#include <ceres/ceres.h>


class PositionDEMFactor: public ceres::SizedCostFunction<1,3>
{
  public:
    PositionDEMFactor(std::shared_ptr<DEM> pDEM, Eigen::Vector3d mappoint_pos)
        :pDEM_(pDEM), mappoint_pos_(mappoint_pos)
    {
        sqrt_information_ = 1./10.;
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {      
        Eigen::Map<Eigen::Vector3d const> xyz(parameters[0]);
        double mp_x = mappoint_pos_(0) + xyz(0);
        double mp_y = mappoint_pos_(1) + xyz(1);
        // Eigen::Map<double> residual(residuals);

        std::cout << "parameter : " << xyz << "\n";
        *residuals = sqrt_information_ * (mappoint_pos_(2) - pDEM_->getDEMAtLocation(mp_x, mp_y));
        std::cout << "residual : " << *residuals << "\n";
        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian(jacobians[0]);
                // jacobian is gradiant of DEM
                jacobian(0, 0) = pDEM_->getJacobianXAt(mp_x, mp_y); 
                jacobian(0, 1) = pDEM_->getJacobianYAt(mp_x, mp_y);
                jacobian(0, 2) = 0.; // altitude jacobian

                jacobian = sqrt_information_ * jacobian;
                 std::cout << "jacobian : " << jacobian << "\n";
                // jacobian = sqrt_information_ * jaco_res_2_Pc * jaco_Pc_2_Pose;
            }
        }

        return true;
    }
  private:
    Eigen::Vector3d mappoint_pos_;
    std::shared_ptr<DEM> pDEM_;
    double sqrt_information_;
};

