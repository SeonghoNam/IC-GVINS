#pragma once
#include <array>
#include <unordered_map>
#include <fstream>
#include <cmath>
class DEM
{
public:
	double getDEMAtLocation(double x, double y) const
	{
		//assert(isWithinRegion == true);
		if(!isWithinRegion(x, y))
			return 0.;
		// bi-linear interpolation
		double xidx = x2Index(x);
		double yidx = y2Index(y);

		int x1 = std::floor(xidx);
		int x2 = x1 + 1;
		int y1 = std::floor(yidx);
		int y2 = y1 + 1;

		double q11 = demData_[y1][x1];
		double q12 = demData_[y1][x2];
		double q21 = demData_[y2][x1];
		double q22 = demData_[y2][x2];

		double rx = xidx - x1;
		double interpolatedX1 = (1 - rx) * q11 + rx * q12;
		double interpolatedX2 = (1 - rx) * q21 + rx * q22;

		double ry = yidx - y1;
		double interpolatedValue = (1 - ry) * interpolatedX1 + ry * interpolatedX2;

		return interpolatedValue;
	}

	double getJacobianXAt(double x, double y) const
	{
		if(!isWithinRegion(x, y))
			return 0.;

		// double xidx = x2Index(x);

		// int x1 = std::floor(xidx);
		// int x2 = x1 + 1;
	
		// double q1 = getDEMAtLocation(x1, y);
		// double q2 = getDEMAtLocation(x2, y);
		
		// return (q1-q2)/dx_;

		// bi-linear interpolation
		double xidx = x2Index(x);
		double yidx = y2Index(y);

		int x1 = std::floor(xidx);
		int x2 = x1 + 1;
		int y1 = std::floor(yidx);
		int y2 = y1 + 1;

		double q11 = demData_[y1][x1];
		double q12 = demData_[y1][x2];
		double q21 = demData_[y2][x1];
		double q22 = demData_[y2][x2];

		double ry = yidx - y1;
		double interpolatedX1 = (1 - ry) * q11 + ry * q21;
		double interpolatedX2 = (1 - ry) * q12 + ry * q22;

		return (interpolatedX1 - interpolatedX2) / dx_;
	}

	double getJacobianYAt(double x, double y) const
	{
		if(!isWithinRegion(x, y))
			return 0.;
		// double yidx = y2Index(y);

		// int y1 = std::floor(yidx);
		// int y2 = y1 + 1;
	
		// double q1 = getDEMAtLocation(x, y1);
		// double q2 = getDEMAtLocation(x, y2);
		
		// return (q1-q2)/dy_;

		// bi-linear interpolation
		double xidx = x2Index(x);
		double yidx = y2Index(y);

		int x1 = std::floor(xidx);
		int x2 = x1 + 1;
		int y1 = std::floor(yidx);
		int y2 = y1 + 1;

		double q11 = demData_[y1][x1];
		double q12 = demData_[y1][x2];
		double q21 = demData_[y2][x1];
		double q22 = demData_[y2][x2];

		double rx = xidx - x1;
		double interpolatedY1 = (1 - rx) * q11 + rx * q12;
		double interpolatedY2 = (1 - rx) * q21 + rx * q22;

		return (interpolatedY1 - interpolatedY2) / dy_;
	}

	DEM()
		:dx_(30.0659), dy_(-30.0028)
	{
		for (int i = 0; i < XSize_; i++)
			xData_[i] = -7.3060e3 + dx_ * i;

		for (int i = 0; i < YSize_; i++)
			yData_[i] = 4.5604e3 + dy_ * i;

		std::ifstream fdem;
		fdem.open("../config/DEM.txt");
		if(!fdem.is_open())
		{
			std::cout << "DEM file open error \n";
		}
		float temp;
		for (int i = 0; i < YSize_; i++)
		{
			for (int j = 0; j < XSize_; j++)
			{
				fdem >> temp;
				demData_[i][j] = temp;			
			}
		}

		fdem.close();
	}
	~DEM()
	{
		std::cout << "~DEM" << "\n";
	}
	bool isWithinRegion(double x, double y) const
	{
		if (x < xData_[0] || x > xData_[XSize_-1] || y > yData_[0] || y < yData_[YSize_-1])
		{
			std::cout << "DEM is not defined in this region \n";
			return false;
		}
		else
			return true;
	}

	double x2Index(double x) const
	{
		return (x - xData_[0]) / dx_;
	}
	double y2Index(double y) const
	{
		return (y - yData_[0]) / dy_;
	}

	static const int XSize_ = 487;
	static const int YSize_ = 305;

	// DEM is defined in xyz coordinate (lat-lon-hgt should be converted to xyz coordinate)
	// DEM[0][0] is upper left conner
	std::array<float, XSize_> xData_; // xData[0] is left
	std::array<float, YSize_> yData_; // yData[0] is up
	std::array<std::array<float, XSize_>, YSize_> demData_; //DEM[0][0] is upper left conner.

	// double xData_[XSize_];
	// double yData_[YSize_];
	// double demData_[XSize_][YSize_]; 

	double dx_, dy_;

	// PointCloud
	std::unordered_map<int, double> pointColudDEM_;

};