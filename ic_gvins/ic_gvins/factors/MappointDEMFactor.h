#pragma once
#include "DEM.h"
#include <ceres/ceres.h>


class MappointDEMFactor: public ceres::SizedCostFunction<1,3>
{
  public:
    MappointDEMFactor(std::shared_ptr<DEM> pDEM, double height)
        :pDEM_(pDEM), height_(height)
    {
        sqrt_information_ = 1./10.;
    }
    ~MappointDEMFactor()
    {
        std::cout<< "~MappointdEMFactor" << "\n";
    };

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