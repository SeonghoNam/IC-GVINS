#include "purenav.h"
#include "common/rotation.h"
#include "common/earth.h"

CPureNav::CPureNav(void)
{
	SetNavCalcHz(200);
	
	// NavData.Initialize();

	nNavRunningCnt = 0;	
	
	_delAb.Zero();
	_delVb.Zero();
	delVn.Zero();

	delVuFeedback = 0.;

	Cbn_prev.Identity();

	Wie_n.Zero();
	Wen_n.Zero();

	//_Wcr.Zero();
	//bNewWcr = false;
	
	SetNavMode(false);
	SetZVSMode(false);
	
	dModeTime = 0;
	
	// for (unsigned int i = 0U; i < LATCHED_NAV_MAX; i++)
	// {
	// 	intAe[i].Reset();
	// 	intAn[i].Reset();
	// 	intAu[i].Reset();
	// }
}

CPureNav::~CPureNav(void)
{
}

void CPureNav::Initialize()
{
	SetNavCalcHz(200);
	
	// NavData.Initialize();
	
	nNavRunningCnt = 0;
	
	_delAb.Zero();
	_delVb.Zero();
	delVn.Zero();
	
	delVuFeedback = 0.;
	
	Cbn_prev.Identity();
	
	Wie_n.Zero();
	Wen_n.Zero();
	
	//_Wcr.Zero();
	//bNewWcr = false;
	
	SetNavMode(false);
	SetZVSMode(false);
	
	dModeTime = 0;
	
	// for (unsigned int i = 0U; i < LATCHED_NAV_MAX; i++)
	// {
	// 	intAe[i].Reset();
	// 	intAn[i].Reset();
	// 	intAu[i].Reset();
	// }
}

void CPureNav::UpdatePureNav(const Vector3d _delVb, const Vector3d _delAb)
{
    nNavRunningCnt++;
    
    dModeTime += dNavCalcT;
    
    this->_delVb = _delVb;
    this->_delAb = _delAb;
    
    UpdateAttitude();
    
    DelvBody2Nav();
    
    if( bNavMode )
	{
        UpdateVelocity();
        UpdatePosition();
    }
}

void CPureNav::UpdateAttitude(void)
{
	Quaterniond DelLambda, DelOmega;
	Vector3d omega;
	
	DelLambda = Rotation::rotvec2quaternion(_delAb);
	
	/*
	if ( bNewWcr == true )
	{
		omega = Wie_n + Wen_n + _Wcr;
		
		bNewWcr = false;
	}
	else
	{
		omega = Wie_n + Wen_n;
	}
	*/
	omega = (Wie_n + Wen_n) * dNavCalcT;
	
	DelOmega = Rotation::rotvec2quaternion(omega);
	
	NavData.Qbn = DelOmega.conjugate() * NavData.Qbn * DelLambda;
	
	NavData.Qbn.normalize();
	
	Cbn_prev = NavData.Cbn;
	NavData.Cbn = Rotation::quaternion2matrix(NavData.Qbn);
	auto euler = Rotation::matrix2euler(NavData.Cbn.transpose());
	NavData.Yaw = euler(0);
	NavData.Pitch = euler(1);
	NavData.Roll = euler(2);
	NavData.Yaw *= -1.;
}

void CPureNav::UpdateVelocity(void)
{	
	double tmpVe, tmpVn, tmpVu;
	Vector3d tmpWn;
	
	Wie_n(0) = 0.;
	Wie_n(1) = WGS84_WIE * cos(NavData.Lat);
	Wie_n(2) = WGS84_WIE * sin(NavData.Lat);
	
	tmpWn = (2. * Wie_n) + Wen_n;
	
	if ( bZVSNavMode )
	{
		tmpVe = NavData.Ve + (delVn(0)) + (NavData.Vn * (tmpWn(2)) * dNavCalcT);
		tmpVn = NavData.Vn + (delVn(1)) - (NavData.Ve * (tmpWn(2)) * dNavCalcT);
	}
	else 
	{
		tmpVe = (NavData.Ve + delVn(0)) + (((NavData.Vn * tmpWn(2)) - (NavData.Vu * tmpWn(1))) * dNavCalcT);
		tmpVn = (NavData.Vn + delVn(1)) - (((NavData.Ve * tmpWn(2)) - (NavData.Vu * tmpWn(0))) * dNavCalcT);
	}

    double dGravity;
	dGravity = Earth::gravity(Vector3d(NavData.Lat, NavData.Lon, NavData.Hei));

	tmpVu = ( (NavData.Vu + delVn(2)) - delVuFeedback) - (((NavData.Vn * tmpWn(0)) - (NavData.Ve * tmpWn(1)) + dGravity) * dNavCalcT);

	NavData.Ve = tmpVe;
	NavData.Vn = tmpVn;
	NavData.Vu = tmpVu;
}

void CPureNav::UpdatePosition(void)
{
    double R_phi, R_lam;
	
	Eigen::Vector2d rmn = Earth::meridianPrimeVerticalRadius(NavData.Lat);

    if( bZVSNavMode)
    {
		R_phi = rmn(0);
		R_lam = rmn(1);			
    }
    else
    {
		R_phi = rmn(0) + NavData.Hei;
		R_lam = (rmn(1) + NavData.Hei);
    }
    
    if ( fabs(R_phi) < 1. )
    {
    	R_phi = 6378137.;
    }
    else
    {
    }

	if ( fabs(R_lam) < 1. )
    {
    	R_lam = 6378137.;
    }
    else
    {
    }    		
    
	Wen_n(0) = -NavData.Vn / R_phi;
	Wen_n(1) =  NavData.Ve / R_lam;
	Wen_n(2) =  (NavData.Ve * tan(NavData.Lat)) / R_lam;

	NavData.Lat += (NavData.Vn / R_phi) * dNavCalcT;
	NavData.Lon += (NavData.Ve / R_lam / cos(NavData.Lat)) * dNavCalcT;
	NavData.Hei += NavData.Vu * dNavCalcT;
}

void CPureNav::DelvBody2Nav(void)
{
    Vector3d PhiV;
    Vector3d PhiV2;
    Vector3d delV2;
    double PhiNorm;
    
    PhiNorm = _delAb.norm();
    PhiNorm = PhiNorm * PhiNorm;
    
    PhiV = Rotation::skewSymmetric(_delAb) * _delVb;
    PhiV2 = Rotation::skewSymmetric(_delAb) * PhiV;
    
    PhiV = PhiV * ((1. / 2.) - (PhiNorm / 24.));
    PhiV2 = PhiV2 * ((1. / 6.) - (PhiNorm / 120.));
    delV2 = _delVb + PhiV + PhiV2;
    
    delVn = Cbn_prev * delV2;
    //delVn = NavData.Cbn * _delVb;
	
	// for (unsigned int i = 0U; i < LATCHED_NAV_MAX; i++)
	// {
	// 	intAe[i] << (delVn(1) / dNavCalcT);
	// 	intAn[i] << (delVn(2) / dNavCalcT);
	// 	intAu[i] << (delVn(3) / dNavCalcT);
	// }
}

// void CPureNav::LatchNavInfo(unsigned int ch, double dT)
// {
// 	LatchedNavData[ch].Lat = NavData.Lat;
// 	LatchedNavData[ch].Lon = NavData.Lon;
// 	LatchedNavData[ch].Hei = NavData.Hei;
	
// 	LatchedNavData[ch].Ve = NavData.Ve;
// 	LatchedNavData[ch].Vn = NavData.Vn;
// 	LatchedNavData[ch].Vu = NavData.Vu;
	
// 	LatchedNavData[ch].Ae = intAe[ch].GetAverage();
// 	LatchedNavData[ch].An = intAn[ch].GetAverage();
// 	LatchedNavData[ch].Au = intAu[ch].GetAverage();
	
// 	LatchedNavData[ch].Qbn = NavData.Qbn;
// 	LatchedNavData[ch].Cbn = NavData.Cbn;
	
// 	intAe[ch].Reset();
// 	intAn[ch].Reset();
// 	intAu[ch].Reset();
// }

void CPureNav::SetInitPosition(double initLat, double initLon, double initHei)
{
	NavData.Lat = initLat;
	NavData.Lon = initLon;
	NavData.Hei = initHei;
	
	NavData.Ve = 0;
	NavData.Vn = 0;
	NavData.Vu = 0;
	
	Wie_n(0) = 0.;
	Wie_n(1) = WGS84_WIE * cos(NavData.Lat);
	Wie_n(2) = WGS84_WIE * sin(NavData.Lat);
	
	Wen_n.Zero();
	
	// for (unsigned int i = 0U; i < LATCHED_NAV_MAX; i++)
	// {
	// 	this->LatchNavInfo(i);
	// }
}

void CPureNav::SetInitAttitude(const Matrix3d & dcm)
{
	NavData.Cbn = dcm;
	NavData.Qbn = Rotation::matrix2quaternion(dcm);
	
	Cbn_prev = NavData.Cbn;
}

void CPureNav::SetInitAttitude(const Quaterniond & Quaternion)
{
	NavData.Qbn = Quaternion;
	NavData.Cbn = Rotation::quaternion2matrix(Quaternion);
	
	Cbn_prev = NavData.Cbn;
}

void CPureNav::SetControlRate(Vector3d Wcr)
{
	Quaterniond DelOmega;
	Vector3d omega;

	omega = Wcr * dNavCalcT;
	
	DelOmega = Rotation::rotvec2quaternion(omega);
	
	NavData.Qbn = DelOmega.conjugate() * NavData.Qbn;
	
	NavData.Qbn.normalize();
	
	Cbn_prev = NavData.Cbn;
	NavData.Cbn = Rotation::quaternion2matrix(NavData.Qbn);
	
	auto euler = Rotation::matrix2euler(NavData.Cbn.transpose());
	NavData.Yaw = euler(0);
	NavData.Pitch = euler(1);
	NavData.Roll = euler(2);
	NavData.Yaw *= -1.;

}

void CPureNav::SetNavMode(bool mode)
{
	bNavMode = mode;
}

void CPureNav::SetZVSMode(bool mode)
{
	bZVSNavMode = mode;
}

void CPureNav::SetNavCalcHz(int NavFrequency)
{
	nNavCalcHz = NavFrequency;
	
	dNavCalcT = 1. / (double)nNavCalcHz;
	
	// for (unsigned int i = 0U; i < LATCHED_NAV_MAX; i++)
	// {
	// 	intAe[i].SetDelTime(dNavCalcT);
	// 	intAn[i].SetDelTime(dNavCalcT);
	// 	intAu[i].SetDelTime(dNavCalcT);
	// }
}

void CPureNav::FeedbackNavErr(const CNavData &ErrNavData, double dAu)
{

	NavData.Lat = NavData.Lat - ErrNavData.Lat;
	NavData.Lon = NavData.Lon - ErrNavData.Lon;
	NavData.Hei = NavData.Hei - ErrNavData.Hei;
	
	NavData.Ve = NavData.Ve - ErrNavData.Ve;
	NavData.Vn = NavData.Vn - ErrNavData.Vn;
	NavData.Vu = NavData.Vu - ErrNavData.Vu;
	
	Vector3d APhi;
	
	APhi(0) = ErrNavData.Pitch;
	APhi(1) = ErrNavData.Roll;
	APhi(2) = ErrNavData.Yaw;
	
	NavData.Qbn = Rotation::rotvec2quaternion(APhi) * NavData.Qbn;
	
	NavData.Cbn = Rotation::quaternion2matrix(NavData.Qbn);

	delVuFeedback += dAu * dNavCalcT;
}

void CPureNav::operator = (const CPureNav &m)
{
	SetNavCalcHz(m.nNavCalcHz);
		
	NavData = m.NavData;

	nNavRunningCnt = m.nNavRunningCnt;	
	
	_delAb = m._delAb;
	_delVb = m._delVb;
	delVn  = m.delVn;

	delVuFeedback = m.delVuFeedback;

	Cbn_prev = m.Cbn_prev;

	Wie_n = m.Wie_n;
	Wen_n = m.Wen_n;
	
	SetNavMode(m.bNavMode);
	SetZVSMode(m.bZVSNavMode);
	
	dModeTime = m.dModeTime;
	
	// for (unsigned int i = 0U; i < LATCHED_NAV_MAX; i++)
	// {
	// 	intAe[i] = m.intAe[i];
	// 	intAn[i] = m.intAn[i];
	// 	intAu[i] = m.intAu[i];
		
	// 	LatchedNavData[i] = m.LatchedNavData[i];
	// }
}