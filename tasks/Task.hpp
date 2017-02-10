/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef NORTH_SEEKING_TASK_TASK_HPP
#define NORTH_SEEKING_TASK_TASK_HPP

#include "north_seeking/TaskBase.hpp"
#include "north_seeking/north_seekingTypes.hpp"

#include <math.h>
#include <queue>

namespace north_seeking{

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        States last_state;
        States new_state;
        double rotation_angle;
        int count_acquire;
        std::queue<point> dead_buffer_queue;
        std::vector<point> dead_sync;
        std::vector<point> gps_sync;

        /*
        * What do i do ?
        * @param ts -
        * @param gps_position_samples_sample -
        */
        virtual void gps_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_position_samples_sample);

        /*
        * What do i do ?
        * @param ts -
        * @param gps_position_samples_sample -
        */
        virtual void pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample);

        /*
        * What do i do ?
        * @param ts -
        * @param gps_position_samples_sample -
        */
        double getPositiveAngle(double angle);

        /*
        * What do i do ?
        * @param ts -
        * @param gps_position_samples_sample -
        */
        double findRotationHistPeak();

        /*
        * What do i do ?
        * @param ts -
        * @param gps_position_samples_sample -
        */
        bool matchSampleTime(base::Time sample_time, point& dead_near_time);

        /*
        * What do i do ?
        * @param ts -
        * @param gps_position_samples_sample -
        */
        double calcOverlapCircleArea(const point &gps, const point &dead_reckoning, const double &points_distance);

        /*
        * What do i do ?
        * @param ts -
        * @param gps_position_samples_sample -
        */
        void findBestFitAngle(double peak);

        /*
        * What do i do ?
        * @param ts -
        * @param gps_position_samples_sample -
        */
        void translatePointsOrigin();


    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "north_seeking::Task");

        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
        ~Task();

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
} /* namespace north_seeking */

#endif
