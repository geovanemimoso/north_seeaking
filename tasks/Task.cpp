/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace north_seeking;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    count_acquire = 0;
    last_state = PRE_OPERATIONAL;
    new_state = ACQUIRING;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::gps_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_position_samples_sample)
{

    if(new_state == ACQUIRING)
    {
        if(_number_samples_to_acquire.get() > count_acquire)
        {
            if(gps_position_samples_sample.position.block(0,0,2,1).allFinite() && gps_position_samples_sample.cov_position.block(0,0,2,2).allFinite())
            {
                try
                {
                    // Get gps data alredy at the point structure
                    point gps_current;
                    point dead_near_time;

                    gps_current.sample_time = gps_position_samples_sample.time;
                    gps_current.x =  gps_position_samples_sample.position.x();
                    gps_current.y =  gps_position_samples_sample.position.y();
                    gps_current.error = gps_position_samples_sample.cov_position(0,0);

                    if(matchSampleTime(gps_current.sample_time, dead_near_time))
                    {
                        gps_sync.push_back(gps_current);
                        dead_sync.push_back(dead_near_time);

                        std::cout << "Sample inside: " << count_acquire << std::endl;

                        count_acquire++;
                    }
                    else{
                        RTT::log(RTT::Error) << "Dead rechoning buffer deque is empety or no match for this sample time was found!" << RTT::endlog();
                    }

                }
                catch(const std::runtime_error& e)
                {
                    RTT::log(RTT::Error) << "Failed to add GPS measurement: " << e.what() << RTT::endlog();
                }
            }
            else{
                RTT::log(RTT::Error) << "GPS position measurement contains NaN's, it will be skipped!" << RTT::endlog();
            }
        }else{
            new_state = COMPUTING;
            translatePointsOrigin();
            findBestFitAngle(findRotationHistPeak());
        }
    }
}

void Task::pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
    if(new_state == ACQUIRING)
    {
        if(pose_samples_sample.hasValidPosition() && pose_samples_sample.hasValidPositionCovariance())
        {
            try
            {
                // Get dead reckoning data alredy at the point structure
                point dead_current;

                dead_current.sample_time = pose_samples_sample.time;
                dead_current.x =  pose_samples_sample.position.x();
                dead_current.y =  pose_samples_sample.position.y();
                dead_current.error = pose_samples_sample.cov_position(0,0);

                dead_buffer_queue.push(dead_current);
            }
            catch(const std::runtime_error& e)
            {
                RTT::log(RTT::Error) << "Failed to add dead reckoning measurement: " << e.what() << RTT::endlog();
            }
        }
        else
            RTT::log(RTT::Error) << "Dead reckoning measurement contains NaN's, it will be skipped!" << RTT::endlog();
    }
}

// Get a sample time as reference and try to find the least difference of time at the dead_reckoning current buffer.
bool Task::matchSampleTime(base::Time sample_time, point& dead_near_time){

    int64_t time_diff_min = std::numeric_limits<int64_t>::max();
    int64_t time_diff = 0;

    if(!dead_buffer_queue.empty())
    {
        while(_sync_time_tolerance.get().toMicroseconds()<time_diff_min && !dead_buffer_queue.empty())
        {
            time_diff = abs((sample_time - dead_buffer_queue.front().sample_time).toMicroseconds());

            if(time_diff < time_diff_min )
            {
               time_diff_min = time_diff;
               dead_near_time = dead_buffer_queue.front();
            }
            dead_buffer_queue.pop();
        }

        if(_sync_time_tolerance.get().toMicroseconds()<time_diff_min){
            return false;
        }
        return true;
    }
    return false;
}

double Task::getPositiveAngle(double angle)
{
    if (angle < 0)
        return angle += 2 * M_PI;
    return angle;
}

double Task::findRotationHistPeak()
{
    int rot_range = 0;
    double peak_index = 0;
    double peak_max = std::numeric_limits<double>::min();
    std::vector<double> points_rotation;

    // Calculate the rotations
    for (int i = 1; i < _number_samples_to_acquire.get(); ++i)
        points_rotation.push_back(getPositiveAngle(atan2(gps_sync[i].y, gps_sync[i].x)) - getPositiveAngle(atan2(dead_sync[i].y, dead_sync[i].x)));

    // Get the range for the acc/histogram (1 degree step)
    rot_range = int(base::Angle::rad2Deg(*std::max_element(points_rotation.begin(),points_rotation.end())) - base::Angle::rad2Deg(*std::min_element(points_rotation.begin(),points_rotation.end())));

    // Instaciate a acc using the range from the rotations between the points
    accumulator rotations_acc(tag::density::num_bins = rot_range, tag::density::cache_size = _number_samples_to_acquire.get());

    // Update de density accumulator
    for (int i = 0; i < _number_samples_to_acquire.get(); ++i)
        rotations_acc(points_rotation[i]);

    histogram_type hist = density(rotations_acc);

    // Get the peak from the histogram and return its index
    for( uint i = 0; i < hist.size(); i++ )
    {
        if( peak_max < hist[i].second)
        {
            peak_max = hist[i].second;
            peak_index = hist[i].first;
        }
    }
    return peak_index;
}

void Task::translatePointsOrigin()
{
    point dead_init = dead_sync[0];
    point gps_init = gps_sync[0];

    //bring points origin to (0,0);
    for (int i = 0; i < _number_samples_to_acquire.get(); ++i)
    {
        dead_sync[i].x -= dead_init.x;
        dead_sync[i].y -= dead_init.y;
        gps_sync[i].x -= gps_init.x;
        gps_sync[i].y -= gps_init.y;
    }
}

void  Task::findBestFitAngle(double peak){

    double min_total_points_dist = std::numeric_limits<double>::max();
    double max_total_overlap_area = std::numeric_limits<double>::min();
    double total_overlap_area, total_points_dist = 0;

    double search_range_angle = peak - base::Angle::deg2Rad(_angle_search_range.get());
    double search_range_end = peak + base::Angle::deg2Rad(_angle_search_range.get());
    double search_increment = base::Angle::deg2Rad(0.1); //0.1 degrre in rad

    double bestFitAngle;
    double points_distance;
    point dead_rotated;

    std::cout << "Peak: " << peak << std::endl;
    std::cout << "search_range_angle: " << search_range_angle << std::endl;
    std::cout << "search_range_end: " << search_range_end << std::endl;

    while(search_range_angle < search_range_end)
    {
        for (int i = 0; i < _number_samples_to_acquire.get(); ++i)
        {
            dead_rotated = dead_sync[i];
            dead_rotated.x = dead_sync[i].x*cos(search_range_angle)  - dead_sync[i].y*sin(search_range_angle);
            dead_rotated.y = dead_sync[i].x*sin(search_range_angle)  + dead_sync[i].y*cos(search_range_angle);

            // Get the points distance to keep track of the min sum of it.
            points_distance =  sqrt(pow(gps_sync[i].x - dead_rotated.x,2) + pow(gps_sync[i].y - dead_rotated.y,2));

            // Keep track of the total points distance and overlap area of all point for the current rotation in avaluation.
            total_points_dist += points_distance;
            total_overlap_area += calcOverlapCircleArea(gps_sync[i],dead_rotated, points_distance);
        }

        // Get tha max area with the min distance of points
        if(total_overlap_area >= max_total_overlap_area)
        {
            if(total_points_dist < min_total_points_dist )
            {
                min_total_points_dist = total_points_dist;
                bestFitAngle = search_range_angle;
            }
            max_total_overlap_area = total_overlap_area;
        }

        // std::cout << "search_range_angle: " << search_range_angle << std::endl;
        // std::cout << "total_overlap_area: " << total_overlap_area << std::endl;
        // std::cout << "total_points_dist: " << total_points_dist << std::endl;

        total_overlap_area = 0;
        total_points_dist = 0;
        search_range_angle += search_increment;
    }
    rotation_angle = bestFitAngle;
    new_state = ROTATION_FOUND;
}

double Task::calcOverlapCircleArea(const point &gps, const point &dead_reckoning, const double &points_distance)
{
    // In case they don't have a overlap, Area = 0
    if( points_distance >= dead_reckoning.error + gps.error )
        return 0;

    // In case that the circles have the same origin R1<R2, R1>R2, R1=R2
     if(points_distance == 0)
     {
        if(dead_reckoning.error < gps.error)
            return M_PI*pow(dead_reckoning.error,2);
        else
            return M_PI*pow(gps.error,2);
     }

    // In case they are inside it other
    if(gps.error > dead_reckoning.error)
    {
        if(points_distance < gps.error - dead_reckoning.error)
            return M_PI*pow(dead_reckoning.error,2);
    }else{
        if(points_distance < dead_reckoning.error - gps.error)
            return M_PI*pow(gps.error,2);
    }

    // Function of overlap discribed at 4 eq.15 (Area of two overlapping circles - ben breech)
     double overlap_a = sqrt((-points_distance + gps.error + dead_reckoning.error)*
                          (points_distance - gps.error + dead_reckoning.error)*
                          (points_distance + gps.error - dead_reckoning.error)*
                          (points_distance + gps.error + dead_reckoning.error))/points_distance;

     double overlap_s1 = (overlap_a + 2*gps.error)/2;
     double overlap_s2 = (overlap_a + 2*dead_reckoning.error)/2;

     // Calculate the area of each segment of the overlap
     double gps_segment = pow(gps.error,2)*asin(overlap_a/(2*gps.error)) -
                          sqrt(overlap_s1*(overlap_s1 - overlap_a)*pow(overlap_s1 - gps.error,2));

     double dead_segment = pow(dead_reckoning.error,2)*asin(overlap_a/(2*dead_reckoning.error)) -
                           sqrt(abs(overlap_s2*(overlap_s2 - overlap_a)*pow(overlap_s2 - dead_reckoning.error,2)));

     // Return the area of the overlap
     return (gps_segment + dead_segment);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if(new_state == ROTATION_FOUND)
        _yaw_rotation.write(rotation_angle);

    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
