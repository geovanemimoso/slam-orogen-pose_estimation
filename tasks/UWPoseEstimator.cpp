/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "UWPoseEstimator.hpp"

using namespace pose_estimation;

UWPoseEstimator::UWPoseEstimator(std::string const& name)
    : UWPoseEstimatorBase(name)
{
}

UWPoseEstimator::UWPoseEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : UWPoseEstimatorBase(name, engine)
{
}

UWPoseEstimator::~UWPoseEstimator()
{
}

void UWPoseEstimator::depth_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_samples_sample)
{
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberZ] = 1;
    handleMeasurement(ts, depth_samples_sample, config, _pressure_sensor2body);
}

void UWPoseEstimator::orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberRoll] = 1;
    config.measurement_mask[BodyStateMemberPitch] = 1;
    config.measurement_mask[BodyStateMemberYaw] = 1;
    config.measurement_mask[BodyStateMemberVroll] = 1;
    config.measurement_mask[BodyStateMemberVpitch] = 1;
    config.measurement_mask[BodyStateMemberVyaw] = 1;
    handleMeasurement(ts, orientation_samples_sample, config, _imu2body);
}

void UWPoseEstimator::position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &position_samples_sample)
{
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberX] = 1;
    config.measurement_mask[BodyStateMemberY] = 1;
    config.measurement_mask[BodyStateMemberZ] = 1;
    handleMeasurement(ts, position_samples_sample, config, _lbl2body);
}

void UWPoseEstimator::velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample)
{
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberVx] = 1;
    config.measurement_mask[BodyStateMemberVy] = 1;
    config.measurement_mask[BodyStateMemberVz] = 1;
    handleMeasurement(ts, velocity_samples_sample, config, _dvl2body);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See UWPoseEstimator.hpp for more detailed
// documentation about them.

bool UWPoseEstimator::configureHook()
{
    if (! UWPoseEstimatorBase::configureHook())
        return false;
    return true;
}
bool UWPoseEstimator::startHook()
{
    if (! UWPoseEstimatorBase::startHook())
        return false;
    return true;
}
void UWPoseEstimator::updateHook()
{
    UWPoseEstimatorBase::updateHook();
    
    // update and write new state
    updateState();
}
void UWPoseEstimator::errorHook()
{
    UWPoseEstimatorBase::errorHook();
}
void UWPoseEstimator::stopHook()
{
    UWPoseEstimatorBase::stopHook();
}
void UWPoseEstimator::cleanupHook()
{
    UWPoseEstimatorBase::cleanupHook();
}
