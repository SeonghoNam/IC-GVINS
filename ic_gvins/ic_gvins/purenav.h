//#pragma once
#ifndef __CPURENAV
#define __CPURENAV

#include "common/types.h"

#define LATCHED_NAV_MAX	5U

using namespace std;

typedef struct NavData
{
	double Lat;
	double Lon;
	double Hei;
	double Ve;
	double Vn;
	double Vu;
	Matrix3d Cbn;
	Quaterniond Qbn;
	double Roll;
	double Pitch;
	double Yaw;
	double Ae;
	double An;
	double Au;
} CNavData;

class CPureNav
{
private:
	int nNavCalcHz;			// Navigation Calculation Frequency in Hz
	double dNavCalcT;		// Navigation Calculation Period in sec

	int nNavRunningCnt;		// Navigation Routine Running Count ( is integrated )
		
	double dModeTime;
	
	CNavData NavData;
	
	Matrix3d Cbn_prev;
	
	Vector3d Wie_n, Wen_n;

	Vector3d _delAb, _delVb;	// Angle Increments and Velocity Increments in body frame after coning & sculling compensation
	
	Vector3d delVn;
	double delVuFeedback;

	bool bNavMode;
	bool bZVSNavMode;
	
	// CIntegrator intAe[LATCHED_NAV_MAX], intAn[LATCHED_NAV_MAX], intAu[LATCHED_NAV_MAX];
public:
	CPureNav(void);
	~CPureNav(void);

	void Initialize(void);
	
    void UpdatePureNav(const Vector3d _delVb, const Vector3d _delAb);
	void FeedbackNavErr(const CNavData &ErrNavData, double dAu = 0.);
	// void LatchNavInfo(unsigned int ch, double dT = 0.);

	void SetInitPosition(double initLat, double initLon, double initHei);
	void SetInitAttitude(const Matrix3d& dcm);
	void SetInitAttitude(const Quaterniond& Quaternion);
	void SetControlRate(Vector3d iWcr);
	void SetNavCalcHz(int NavFrequency);
	void SetNavMode(bool mode);
	void SetZVSMode(bool mode);
	void operator = (const CPureNav &m);

private:
	void UpdateAttitude(void);
	void UpdateVelocity(void);
	void UpdatePosition(void);
	void DelvBody2Nav(void);
};

#endif