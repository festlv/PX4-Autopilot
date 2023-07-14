#include "ekf_wrapper.h"

EkfWrapper::EkfWrapper(std::shared_ptr<Ekf> ekf):
	_ekf{ekf}
{
	_ekf_params = _ekf->getParamHandle();
}

EkfWrapper::~EkfWrapper()
{
}

void EkfWrapper::setBaroHeightRef()
{
	_ekf_params->height_sensor_ref = HeightSensor::BARO;
}

void EkfWrapper::enableBaroHeightFusion()
{
	_ekf_params->baro_ctrl = 1;
}

void EkfWrapper::disableBaroHeightFusion()
{
	_ekf_params->baro_ctrl = 0;
}

bool EkfWrapper::isIntendingBaroHeightFusion() const
{
	return _ekf->control_status_flags().baro_hgt;
}

void EkfWrapper::setGpsHeightRef()
{
	_ekf_params->height_sensor_ref = HeightSensor::GNSS;
}

void EkfWrapper::enableGpsHeightFusion()
{
	_ekf_params->gnss_ctrl |= GnssCtrl::VPOS;
}

void EkfWrapper::disableGpsHeightFusion()
{
	_ekf_params->gnss_ctrl &= ~GnssCtrl::VPOS;
}

bool EkfWrapper::isIntendingGpsHeightFusion() const
{
	return _ekf->control_status_flags().gps_hgt;
}

void EkfWrapper::setRangeHeightRef()
{
	_ekf_params->height_sensor_ref = HeightSensor::RANGE;
}

void EkfWrapper::enableRangeHeightFusion()
{
	_ekf_params->rng_ctrl = RngCtrl::ENABLED;
}

void EkfWrapper::disableRangeHeightFusion()
{
	_ekf_params->rng_ctrl = RngCtrl::DISABLED;
}

bool EkfWrapper::isIntendingRangeHeightFusion() const
{
	return _ekf->control_status_flags().rng_hgt;
}

void EkfWrapper::setExternalVisionHeightRef()
{
	_ekf_params->height_sensor_ref = HeightSensor::EV;
}

void EkfWrapper::enableExternalVisionHeightFusion()
{
	_ekf_params->ev_ctrl |= static_cast<int32_t>(EvCtrl::VPOS);
}

bool EkfWrapper::isIntendingExternalVisionHeightFusion() const
{
	return _ekf->control_status_flags().ev_hgt;
}

void EkfWrapper::enableBetaFusion()
{
	_ekf_params->beta_fusion_enabled = true;
}

void EkfWrapper::disableBetaFusion()
{
	_ekf_params->beta_fusion_enabled = false;
}

bool EkfWrapper::isIntendingBetaFusion() const
{
	return _ekf->control_status_flags().fuse_beta;
}

void EkfWrapper::enableGpsFusion()
{
	_ekf_params->gnss_ctrl |= GnssCtrl::HPOS | GnssCtrl::VEL;
}

void EkfWrapper::disableGpsFusion()
{
	_ekf_params->gnss_ctrl &= ~(GnssCtrl::HPOS | GnssCtrl::VEL);
}

bool EkfWrapper::isIntendingGpsFusion() const
{
	return _ekf->control_status_flags().gps;
}

void EkfWrapper::enableGpsHeadingFusion()
{
	_ekf_params->gnss_ctrl |= GnssCtrl::YAW;
}

void EkfWrapper::disableGpsHeadingFusion()
{
	_ekf_params->gnss_ctrl &= ~GnssCtrl::YAW;
}

bool EkfWrapper::isIntendingGpsHeadingFusion() const
{
	return _ekf->control_status_flags().gps_yaw;
}

void EkfWrapper::enableFlowFusion()
{
	_ekf_params->flow_ctrl = 1;
}

void EkfWrapper::disableFlowFusion()
{
	_ekf_params->flow_ctrl = 0;
}

bool EkfWrapper::isIntendingFlowFusion() const
{
	return _ekf->control_status_flags().opt_flow;
}

void EkfWrapper::setFlowOffset(const Vector3f &offset)
{
	_ekf_params->flow_pos_body = offset;
}

void EkfWrapper::enableExternalVisionPositionFusion()
{
	_ekf_params->ev_ctrl |= static_cast<int32_t>(EvCtrl::HPOS);
}

void EkfWrapper::disableExternalVisionPositionFusion()
{
	_ekf_params->ev_ctrl &= ~static_cast<int32_t>(EvCtrl::HPOS);
}

bool EkfWrapper::isIntendingExternalVisionPositionFusion() const
{
	return _ekf->control_status_flags().ev_pos;
}

void EkfWrapper::enableExternalVisionVelocityFusion()
{
	_ekf_params->ev_ctrl |= static_cast<int32_t>(EvCtrl::VEL);
}

void EkfWrapper::disableExternalVisionVelocityFusion()
{
	_ekf_params->ev_ctrl &= ~static_cast<int32_t>(EvCtrl::VEL);
}

bool EkfWrapper::isIntendingExternalVisionVelocityFusion() const
{
	return _ekf->control_status_flags().ev_vel;
}

void EkfWrapper::enableExternalVisionHeadingFusion()
{
	_ekf_params->ev_ctrl |= static_cast<int32_t>(EvCtrl::YAW);
}

void EkfWrapper::disableExternalVisionHeadingFusion()
{
	_ekf_params->ev_ctrl &= ~static_cast<int32_t>(EvCtrl::YAW);
}

bool EkfWrapper::isIntendingExternalVisionHeadingFusion() const
{
	return _ekf->control_status_flags().ev_yaw;
}

bool EkfWrapper::isIntendingMagHeadingFusion() const
{
	return _ekf->control_status_flags().mag_hdg;
}

bool EkfWrapper::isIntendingMag3DFusion() const
{
	return _ekf->control_status_flags().mag_3D;
}

void EkfWrapper::setMagFuseTypeNone()
{
	_ekf_params->mag_fusion_type = MagFuseType::NONE;
}

void EkfWrapper::enableMagStrengthCheck()
{
	_ekf_params->mag_check |= static_cast<int32_t>(MagCheckMask::STRENGTH);
}
}

bool EkfWrapper::isWindVelocityEstimated() const
{
	return _ekf->control_status_flags().wind;
}

void EkfWrapper::enableTerrainRngFusion()
{
	_ekf_params->terrain_fusion_mode |= TerrainFusionMask::TerrainFuseRangeFinder;
}

void EkfWrapper::disableTerrainRngFusion()
{
	_ekf_params->terrain_fusion_mode &= ~TerrainFusionMask::TerrainFuseRangeFinder;
}

bool EkfWrapper::isIntendingTerrainRngFusion() const
{
	terrain_fusion_status_u terrain_status;
	terrain_status.value = _ekf->getTerrainEstimateSensorBitfield();
	return terrain_status.flags.range_finder;
}

void EkfWrapper::enableTerrainFlowFusion()
{
	_ekf_params->terrain_fusion_mode |= TerrainFusionMask::TerrainFuseOpticalFlow;
}

void EkfWrapper::disableTerrainFlowFusion()
{
	_ekf_params->terrain_fusion_mode &= ~TerrainFusionMask::TerrainFuseOpticalFlow;
}

bool EkfWrapper::isIntendingTerrainFlowFusion() const
{
	terrain_fusion_status_u terrain_status;
	terrain_status.value = _ekf->getTerrainEstimateSensorBitfield();
	return terrain_status.flags.flow;
}

Eulerf EkfWrapper::getEulerAngles() const
{
	return Eulerf(_ekf->getQuaternion());
}

float EkfWrapper::getYawAngle() const
{
	const Eulerf euler(_ekf->getQuaternion());
	return euler(2);
}

matrix::Vector4f EkfWrapper::getQuaternionVariance() const
{
	return matrix::Vector4f(_ekf->orientation_covariances().diag());
}

int EkfWrapper::getQuaternionResetCounter() const
{
	float tmp[4];
	uint8_t counter;
	_ekf->get_quat_reset(tmp, &counter);
	return static_cast<int>(counter);
}

matrix::Vector3f EkfWrapper::getDeltaVelBiasVariance() const
{
	return _ekf->covariances_diagonal().slice<3, 1>(13, 0);
}

void EkfWrapper::enableDragFusion()
{
	_ekf_params->drag_ctrl = 1;
}

void EkfWrapper::disableDragFusion()
{
	_ekf_params->drag_ctrl = 0;
}

void EkfWrapper::setDragFusionParameters(const float &bcoef_x, const float &bcoef_y, const float &mcoef)
{
	_ekf_params->bcoef_x = bcoef_x;
	_ekf_params->bcoef_y = bcoef_y;
	_ekf_params->mcoef = mcoef;
}
