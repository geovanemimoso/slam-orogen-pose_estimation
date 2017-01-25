/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef POSE_ESTIMATION_FASTSLOWPOSEESTIMATOR_TASK_HPP
#define POSE_ESTIMATION_FASTSLOWPOSEESTIMATOR_TASK_HPP

#include "pose_estimation/FastSlowPoseEstimatorBase.hpp"
#include "pose_estimation/BaseFilter.hpp"
#include <aggregator/DetermineSampleTimestamp.hpp>
#include <aggregator/StreamAligner.hpp>

namespace pose_estimation{

    /*! \class FastSlowPoseEstimator
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','pose_estimation::FastSlowPoseEstimator')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class FastSlowPoseEstimator : public FastSlowPoseEstimatorBase
    {
	friend class FastSlowPoseEstimatorBase;
        boost::shared_ptr<BaseFilter> slow_filter;
        boost::shared_ptr<BaseFilter> fast_filter;
    protected:

        virtual void depth_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_samples_sample);

        virtual void dvl_velocity_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dvl_velocity_samples_sample);

        virtual void gps_position_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_position_samples_sample);

        virtual void lbl_position_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &lbl_position_samples_sample);

        virtual void orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);

        virtual void xy_position_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xy_position_samples_sample);

        /* resets the current state to the initial state.
         */
        virtual bool resetState();
        virtual void pullPorts();

        void SetupStreams();
        bool getSensorInBodyPose(const transformer::Transformation& sensor2body_transformer, const base::Time &ts, Eigen::Affine3d& sensorInBody);

        aggregator::StreamAligner _stream_aligner;
        int depth_samples_idx;
        int dvl_velocity_samples_idx;
        int gps_position_samples_idx;
        int lbl_position_samples_idx;
        int orientation_samples_idx;
        int xy_position_samples_idx;
        ::base::samples::RigidBodyState port_listener_depth_samples_sample;
        ::base::samples::RigidBodyState port_listener_dvl_velocity_samples_sample;
        ::base::samples::RigidBodyState port_listener_gps_position_samples_sample;
        ::base::samples::RigidBodyState port_listener_lbl_position_samples_sample;
        ::base::samples::RigidBodyState port_listener_orientation_samples_sample;
        ::base::samples::RigidBodyState port_listener_xy_position_samples_sample;

    public:
        /** TaskContext constructor for FastSlowPoseEstimator
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        FastSlowPoseEstimator(std::string const& name = "pose_estimation::FastSlowPoseEstimator");

        /** TaskContext constructor for FastSlowPoseEstimator
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        FastSlowPoseEstimator(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of FastSlowPoseEstimator
         */
	    ~FastSlowPoseEstimator();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif
