/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FastSlowPoseEstimator.hpp"

using namespace pose_estimation;

FastSlowPoseEstimator::FastSlowPoseEstimator(std::string const& name)
    : FastSlowPoseEstimatorBase(name)
{
}

FastSlowPoseEstimator::FastSlowPoseEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : FastSlowPoseEstimatorBase(name, engine)
{
}

FastSlowPoseEstimator::~FastSlowPoseEstimator()
{
}

bool FastSlowPoseEstimator::resetState()
{
    return bool();
}

void FastSlowPoseEstimator::depth_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_pressure_sensor2body, ts, sensorInBody))
         return;

    if(!base::isNaN(depth_samples_sample.position.z()) && !base::isNaN(depth_samples_sample.cov_position(2,2)))
    {
        fast_filter->predictionStep(ts);

        try
        {
            // apply sensorInBody transformation to measurement
            Eigen::Matrix<double, 1, 1> depth;
            depth << depth_samples_sample.position.z() - sensorInBody.translation().z();
            PoseUKF::ZMeasurement measurement;
            measurement.mu = depth;
            measurement.cov(0,0) = depth_samples_sample.cov_position(2,2);
            fast_filter->getPoseEstimator()->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add depth measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Depth measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void FastSlowPoseEstimator::dvl_velocity_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dvl_velocity_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_dvl2body, ts, sensorInBody))
        return;

    if(dvl_velocity_samples_sample.hasValidVelocity() && dvl_velocity_samples_sample.hasValidVelocityCovariance())
    {
        fast_filter->predictionStep(ts);
        try
        {
            // apply sensorInBody transformation to measurement, assuming that the velocity is expressed in the DVL device frame
            base::Vector3d velocity = sensorInBody.rotation() * dvl_velocity_samples_sample.velocity;
            PoseUKF::State current_state;
            if(fast_filter->getPoseEstimator()->getCurrentState(current_state))
            {
                velocity -= current_state.angular_velocity.cross(sensorInBody.translation());
            }

            // apply new velocity measurement
            PoseUKF::VelocityMeasurement measurement;
            measurement.mu = velocity;
            measurement.cov = sensorInBody.rotation() * dvl_velocity_samples_sample.cov_velocity * sensorInBody.rotation().transpose();
            fast_filter->getPoseEstimator()->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add DVL measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Info) << "DVL measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void FastSlowPoseEstimator::gps_position_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_position_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_gps2body, ts, sensorInBody))
        return;

    if(gps_position_samples_sample.position.block(0,0,2,1).allFinite() && gps_position_samples_sample.cov_position.block(0,0,2,2).allFinite())
    {
        fast_filter->predictionStep(ts);
        try
        {
            // apply sensorInBody transformation to measurement
            Eigen::Affine3d gpsInWorld = Eigen::Affine3d::Identity();
            gpsInWorld.translation() = Eigen::Vector3d(gps_position_samples_sample.position.x(), gps_position_samples_sample.position.y(), 0.0);
            Eigen::Affine3d bodyInWorld = gpsInWorld * sensorInBody.inverse();

            Eigen::Quaterniond rotation(sensorInBody.rotation());
            Eigen::Matrix2d yaw_rot = Eigen::Rotation2Dd(base::getYaw(rotation)).toRotationMatrix();

            PoseUKF::XYMeasurement measurement;
            measurement.mu = bodyInWorld.translation().block(0,0,2,1);
            measurement.cov = yaw_rot * gps_position_samples_sample.cov_position.block(0,0,2,2) * yaw_rot.transpose();
            fast_filter->getPoseEstimator()->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add GPS measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "GPS position measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void FastSlowPoseEstimator::lbl_position_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &lbl_position_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_lbl2body, ts, sensorInBody))
        return;

    if(lbl_position_samples_sample.hasValidPosition() && lbl_position_samples_sample.hasValidPositionCovariance())
    {
        fast_filter->predictionStep(ts);
        try
        {
            // apply sensorInBody transformation to measurement
            Eigen::Affine3d modemInWorld = Eigen::Affine3d::Identity();
            modemInWorld.translation() = lbl_position_samples_sample.position;
            Eigen::Affine3d bodyInWorld = modemInWorld * sensorInBody.inverse();

            PoseUKF::PositionMeasurement measurement;
            measurement.mu = bodyInWorld.translation();
            measurement.cov = sensorInBody.rotation() * lbl_position_samples_sample.cov_position * sensorInBody.rotation().transpose();
            fast_filter->getPoseEstimator()->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add LBL/USBL measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "LBL/USBL measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void FastSlowPoseEstimator::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_imu2body, ts, sensorInBody))
        return;

    if(orientation_samples_sample.hasValidOrientation() && orientation_samples_sample.hasValidOrientationCovariance())
    {
        fast_filter->predictionStep(ts);
        try
        {
            // apply sensorInBody transformation to measurement
            Eigen::Quaterniond transformed_orientation((orientation_samples_sample.orientation * sensorInBody.inverse()).linear());

            PoseUKF::OrientationMeasurement measurement;
            measurement.mu = MTK::SO3<double>::log(transformed_orientation);
            measurement.cov = sensorInBody.rotation() * orientation_samples_sample.cov_orientation * sensorInBody.rotation().transpose();
            fast_filter->getPoseEstimator()->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add orientation measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Orientation measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void FastSlowPoseEstimator::xy_position_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xy_position_samples_sample)
{
    if(xy_position_samples_sample.position.block(0,0,2,1).allFinite() && xy_position_samples_sample.cov_position.block(0,0,2,2).allFinite())
    {
        fast_filter->predictionStep(ts);
        try
        {
            PoseUKF::XYMeasurement measurement;
            measurement.mu = xy_position_samples_sample.position.block(0,0,2,1);
            measurement.cov = xy_position_samples_sample.cov_position.block(0,0,2,2);
            fast_filter->getPoseEstimator()->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add XY position measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "XY position measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FastSlowPoseEstimator.hpp for more detailed
// documentation about them.

bool FastSlowPoseEstimator::configureHook()
{
    if (! FastSlowPoseEstimatorBase::configureHook())
        return false;
    return true;

    fast_filter->setupFilter(_initial_state.value(),_process_noise.value(), _max_time_delta.get());
}
bool FastSlowPoseEstimator::startHook()
{
    if (! FastSlowPoseEstimatorBase::startHook())
        return false;
    return true;
}
void FastSlowPoseEstimator::updateHook()
{
    FastSlowPoseEstimatorBase::updateHook();
}
void FastSlowPoseEstimator::errorHook()
{
    FastSlowPoseEstimatorBase::errorHook();
}
void FastSlowPoseEstimator::stopHook()
{
    FastSlowPoseEstimatorBase::stopHook();
}
void FastSlowPoseEstimator::cleanupHook()
{
    FastSlowPoseEstimatorBase::cleanupHook();
}

bool FastSlowPoseEstimator::getSensorInBodyPose(const transformer::Transformation& sensor2body_transformer, const base::Time& ts, Eigen::Affine3d& sensorInBody)
{
    // receive sensor to body transformation
    if (!sensor2body_transformer.get(ts, sensorInBody))
    {
        RTT::log(RTT::Error) << "skip, have no " << sensor2body_transformer.getSourceFrame() << "In" << sensor2body_transformer.getTargetFrame() << " transformation sample!" << RTT::endlog();
        //new_state = MISSING_TRANSFORMATION;
        return false;
    }
    return true;
}

//BASE TASkS

void FastSlowPoseEstimator::SetupStreams(){

    _stream_aligner.clear();
    _stream_aligner.setTimeout( base::Time::fromSeconds( _aggregator_max_latency.value()) );

    //Register orientation samples to the stream aligner
    const double orientation_samplesPeriod = _orientation_samples_period.value();
    orientation_samples_idx = _stream_aligner.registerStream< ::base::samples::RigidBodyState >(
    boost::bind( &FastSlowPoseEstimator::orientation_samplesCallback, this, _1, _2 ),
    2.0* ceil( 0.1/orientation_samplesPeriod),
    base::Time::fromSeconds( orientation_samplesPeriod ) );
    _lastStatusTime = base::Time();

    //Register deth sammples to the stream aligner
    const double depth_samplesPeriod = _depth_samples_period.value();
    depth_samples_idx = _stream_aligner.registerStream< ::base::samples::RigidBodyState >(
    boost::bind( &FastSlowPoseEstimator::depth_samplesCallback, this, _1, _2 ),
    2.0* ceil( 0.1/depth_samplesPeriod),
    base::Time::fromSeconds( depth_samplesPeriod ) );
    _lastStatusTime = base::Time();

    //Register dvl samples to the stream aligner
    const double dvl_velocity_samplesPeriod = _dvl_velocity_samples_period.value();
    dvl_velocity_samples_idx = _stream_aligner.registerStream< ::base::samples::RigidBodyState >(
    boost::bind( &FastSlowPoseEstimator::dvl_velocity_samplesCallback, this, _1, _2 ),
    2.0* ceil( 0.1/dvl_velocity_samplesPeriod),
    base::Time::fromSeconds( dvl_velocity_samplesPeriod ) );
    _lastStatusTime = base::Time();

    //Register lbl samples to the stream aligner
    const double lbl_position_samplesPeriod = _lbl_position_samples_period.value();
    lbl_position_samples_idx = _stream_aligner.registerStream< ::base::samples::RigidBodyState >(
    boost::bind( &FastSlowPoseEstimator::lbl_position_samplesCallback, this, _1, _2 ),
    2.0* ceil( 0.1/lbl_position_samplesPeriod),
    base::Time::fromSeconds( lbl_position_samplesPeriod ) );
    _lastStatusTime = base::Time();

    //Register XY position samples to the stream aligner
    const double xy_position_samplesPeriod = _xy_position_samples_period.value();
    xy_position_samples_idx = _stream_aligner.registerStream< ::base::samples::RigidBodyState >(
    boost::bind( &FastSlowPoseEstimator::xy_position_samplesCallback, this, _1, _2 ),
    2.0* ceil( 0.1/xy_position_samplesPeriod),
    base::Time::fromSeconds( xy_position_samplesPeriod ) );
    _lastStatusTime = base::Time();

    //Register gps position samples to the stream aligner
    const double gps_position_samplesPeriod = _gps_position_samples_period.value();
    gps_position_samples_idx = _stream_aligner.registerStream< ::base::samples::RigidBodyState >(
    boost::bind( &FastSlowPoseEstimator::gps_position_samplesCallback, this, _1, _2 ),
    2.0* ceil( 0.1/gps_position_samplesPeriod),
    base::Time::fromSeconds( gps_position_samplesPeriod ) );
    _lastStatusTime = base::Time();
}

void FastSlowPoseEstimator::pullPorts()
{
    bool keepGoing = true;
    bool hasData[6] = { true, true, true, true, true, true };

    while(keepGoing)
    {
        keepGoing = false;

        if(hasData[0] && _orientation_samples.read(port_listener_orientation_samples_sample, false) == RTT::NewData )
        {

                _stream_aligner.push(orientation_samples_idx, aggregator::determineTimestamp(port_listener_orientation_samples_sample), port_listener_orientation_samples_sample);
            keepGoing = true;
        }
        else
            hasData[0] = false;
        if(hasData[1] && _depth_samples.read(port_listener_depth_samples_sample, false) == RTT::NewData )
        {

                _stream_aligner.push(depth_samples_idx, aggregator::determineTimestamp(port_listener_depth_samples_sample), port_listener_depth_samples_sample);
            keepGoing = true;
        }
        else
            hasData[1] = false;
        if(hasData[2] && _dvl_velocity_samples.read(port_listener_dvl_velocity_samples_sample, false) == RTT::NewData )
        {

                _stream_aligner.push(dvl_velocity_samples_idx, aggregator::determineTimestamp(port_listener_dvl_velocity_samples_sample), port_listener_dvl_velocity_samples_sample);
            keepGoing = true;
        }
        else
            hasData[2] = false;
        if(hasData[3] && _lbl_position_samples.read(port_listener_lbl_position_samples_sample, false) == RTT::NewData )
        {

                _stream_aligner.push(lbl_position_samples_idx, aggregator::determineTimestamp(port_listener_lbl_position_samples_sample), port_listener_lbl_position_samples_sample);
            keepGoing = true;
        }
        else
            hasData[3] = false;
        if(hasData[4] && _xy_position_samples.read(port_listener_xy_position_samples_sample, false) == RTT::NewData )
        {

                _stream_aligner.push(xy_position_samples_idx, aggregator::determineTimestamp(port_listener_xy_position_samples_sample), port_listener_xy_position_samples_sample);
            keepGoing = true;
        }
        else
            hasData[4] = false;
        if(hasData[5] && _gps_position_samples.read(port_listener_gps_position_samples_sample, false) == RTT::NewData )
        {

                _stream_aligner.push(gps_position_samples_idx, aggregator::determineTimestamp(port_listener_gps_position_samples_sample), port_listener_gps_position_samples_sample);
            keepGoing = true;
        }
        else
            hasData[5] = false;

	while(_stream_aligner.step())
	{
	    ;
	}
    }
}
