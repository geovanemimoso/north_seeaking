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
        * Receives gps position samples and try to fing a match in time for it in dead_buffer_queue,
        * if it finds put both samples aligned into dead_sync, gps_sync vectors and update the
        * count_acquire counter for keep track of the number_samples_to_acquire config.
        * @param ts - Time reference for the gps sample.
        * @param gps_position_samples_sample - X, Y position and associated error to the sample.
        */
        virtual void gps_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_position_samples_sample);

        /*
        * Receives dead reckoing position samples and put then at the dead_buffer_queue
        * until the acquire process is done.
        * @param ts -  Time reference for the dead reckoning sample.
        * @param pose_samples_sample - Dead reckoning position sample.
        */
        virtual void pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample);

        /*
        * Receves a angle and if it's negative returns the correspondent positive angle
        * if its positive, retuns the angle raveived.
        * @param angle - angle in radians.
        */
        double getPositiveAngle(double angle);

        /*
        * Get the rotations between the correspondent position points at dead_sync and gps_sync,
        * set's up a histogram of this rotations and retunrs the peak angle of the histogram.
        */
        double findRotationHistPeak();

        /*
        * Get a sample time as reference and try to find the least difference of time at
        * the dead_reckoning current buffer until the dead_buffer_queue or the sync_time_tolerance is achive.
        * @param sample_time - time of the sample that has to be matched.
        * @param dead_near_time - pointer to the dead reckonig point to be placed at the dead_sync vector.
        */
        bool matchSampleTime(base::Time sample_time, point& dead_near_time);

        /*
        * Calculates the overlapping area between two circles, defined as the position points (x,y)
        * and its associated error (r);
        * @param gps - gps point that has x,y postion its error and sample time.
        * @param dead_reckoning - dead reckoning point that has x,y position its error and sample time.
        * @param points_distance - the euclidean distance between the received points.
        */
        double calcOverlapCircleArea(const point &gps, const point &dead_reckoning, const double &points_distance);

        /*
        * Search around the peak with the range defined by angle_search_range and 0.1 degree setps, and returnts
        * the rotation angle that maximaze the total overlapping area betwwen the set of gps and dead points and minimize
        * its total points distance.
        * @param peak - peak of the rotaion histogram
        */
        void findBestFitAngle(double peak);

        /*
        * Translate the dead_sync and gps_sync set of points to orign.
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
