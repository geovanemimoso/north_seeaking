/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace north_seeking;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    count_acquire = 0;
    acquire_done = false;
    best_fit_done = false;
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
    if(_number_samples_to_acquire.get() > count_acquire)
    {
        if(gps_position_samples_sample.position.block(0,0,2,1).allFinite() && gps_position_samples_sample.cov_position.block(0,0,2,2).allFinite())
        {
            try
            {
                //Get gps data alredy at the point structure
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
    }
    else{
        //Get the best rotation and update the output variable
        if(!best_fit_done)
        {
            acquire_done = true;
            translatePointsOrigin();
            findBestFitAngle(rotationHistPeak()); //Botar algum retorno de verificação !! (boolean na saida da função)
        }
    }
}

void Task::pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
    //std::cout << "Callback dead" << std::endl;
    if(!acquire_done)
    {
        if(pose_samples_sample.hasValidPosition() && pose_samples_sample.hasValidPositionCovariance())
        {
            try
            {
                //Get dead reckoning data alredy at the point structure
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

double Task::pointRotation(const point &gps, const point &dead_reckoning){

    double angle;

    // Pegar a media do atan2 de dead e gps e guardar (definir direção da rotação) #####
    angle = atan2(gps.y, gps.x) - atan2(dead_reckoning.y, dead_reckoning.x);
    if (angle < 0) angle += 2 * M_PI;

    return angle;
}

//Get a sample time as reference and try to find the least difference of time at the dead_reckoning current buffer.
bool Task::matchSampleTime(base::Time sample_time, point& dead_near_time){

    int64_t time_diff_min = 10000000;
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

double Task::rotationHistPeak()
{
    std::vector<double> rotations;
    double rot_min = 100000;
    double rot_max = -100000;
    int rot_range = 0;

    for (int i = 0; i < _number_samples_to_acquire.get(); ++i)
    {
        rotations.push_back(pointRotation(gps_sync[i],dead_sync[i]));

        if(rot_max < rotations[i])
            rot_max = rotations[i];

        if(rot_min > rotations[i])
            rot_min = rotations[i];
    }

    ////rad to degree to get the range
    rot_max = (rot_max*180)/M_PI;
    rot_min = (rot_min*180)/M_PI;

    rot_range = int(rot_max - rot_min);

    //if(rot_range > 0) ?
    double peak_index, peak_max = 0;
    accumulator rotations_acc(tag::density::num_bins = rot_range, tag::density::cache_size = _number_samples_to_acquire.get());

    for (int i = 0; i < _number_samples_to_acquire.get(); ++i)
        rotations_acc(rotations[i]);

    histogram_type hist = density(rotations_acc);

    for( int i = 0; i < hist.size(); i++ )
    {
        if( peak_max < hist[i].second)
        {
            peak_max = hist[i].second;
            peak_index = hist[i].first;
        }
    }

    return peak_index;
}

//Put the trajectores at orign
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

    std::cout << "Translation done" << std::endl;
}

void  Task::findBestFitAngle(double peak){

    double search_range_angle = peak - (_angle_search_range.get()*M_PI)/180;
    double search_range_end = peak + (_angle_search_range.get()*M_PI)/180;
    double search_increment = (0.1*M_PI)/180; //0.1 degrre in rad
    double total_lens_area, max_total_lens_area, total_points_dist = 0;
    double min_total_points_dist = 1000000; //<< -- Checar esse idexadores
    double bestFitAngle;
    double points_distance;
    point dead_rotated;

    std::cout << "Peak: " << peak << std::endl;
    std::cout << "search_range_angle: " << search_range_angle << std::endl;
    std::cout << "search_range_end: " << search_range_end << std::endl;

    while( search_range_angle < search_range_end)
    {
        for (int i = 0; i < _number_samples_to_acquire.get(); ++i)
        {
            dead_rotated = dead_sync[i];

            dead_rotated.x = dead_sync[i].x*cos(search_range_angle)  - dead_sync[i].y*sin(search_range_angle);
            dead_rotated.y = dead_sync[i].x*sin(search_range_angle)  + dead_sync[i].y*cos(search_range_angle);

            //Get the points distance to keep track of the min sum of it.
            points_distance =  sqrt(pow(gps_sync[i].x - dead_rotated.x,2) + pow(gps_sync[i].y - dead_rotated.y,2));

            total_points_dist += points_distance;
            total_lens_area += lensArea(gps_sync[i],dead_rotated, points_distance);
        }

        //checar a BUSCA de max and min EM TODAS AS FUNÇOES (VALOR INICIAL) e melhor forma de fazer! <<<<<<<<
        if(total_lens_area >= max_total_lens_area)
        {
            if(total_points_dist < min_total_points_dist )
            {
                min_total_points_dist = total_points_dist;
                bestFitAngle = search_range_angle;
            }
            max_total_lens_area = total_lens_area;
        }

        std::cout << "search_range_angle: " << search_range_angle << std::endl;
        std::cout << "total_lens_area: " << total_lens_area << std::endl;
        std::cout << "total_points_dist: " << total_points_dist << std::endl;

        total_lens_area = 0;
        total_points_dist = 0;
        search_range_angle += search_increment;
    }

    rotation_angle = bestFitAngle;
    best_fit_done = true;
}

double Task::lensArea(const point &gps, const point &dead_reckoning, const double &points_distance)
{
    //std::cout << "points_distance: " << points_distance << std::endl;

    //In case they don't have a lens
    if( points_distance >= dead_reckoning.error + gps.error )
        return 0;

    //std::cout << "dead_reckoning.error: " << dead_reckoning.error << std::endl;
    //std::cout << "GPS.error: " << gps.error << std::endl;

    //In case that the circles have the same origin R1<R2, R1>R2, R1=R2
     if(points_distance == 0)
     {
        if(dead_reckoning.error < gps.error)
            return M_PI*pow(dead_reckoning.error,2);
        else
            return M_PI*pow(gps.error,2);
     }

    //in case they are inside it other
    if(gps.error > dead_reckoning.error)
    {
        if(points_distance < gps.error - dead_reckoning.error)
            return M_PI*pow(dead_reckoning.error,2);
    }else{
        if(points_distance < dead_reckoning.error - gps.error)
            return M_PI*pow(gps.error,2);
    }

    //Function of lens discribed at 4 eq.15 (Area of two overlapping circles - ben breech)
     double lens_a = sqrt((-points_distance + gps.error + dead_reckoning.error)*
                          (points_distance - gps.error + dead_reckoning.error)*
                          (points_distance + gps.error - dead_reckoning.error)*
                          (points_distance + gps.error + dead_reckoning.error))/points_distance;

     //std::cout << "lens_a: " << lens_a << std::endl;

     double lens_s1 = (lens_a + 2*gps.error)/2;
     double lens_s2 = (lens_a + 2*dead_reckoning.error)/2;

     //std::cout << "lens_s1: " << lens_s1 << std::endl;
     //std::cout << "lens_s2: " << lens_s2 << std::endl;

     //calculate the area of each segment of the lens
     double gps_segment = pow(gps.error,2)*asin(lens_a/(2*gps.error)) -
                          sqrt(lens_s1*(lens_s1 - lens_a)*pow(lens_s1 - gps.error,2));

     //std::cout << "gps_segment: " << gps_segment << std::endl;

     double dead_segment = pow(dead_reckoning.error,2)*asin(lens_a/(2*dead_reckoning.error)) -
                           sqrt(abs(lens_s2*(lens_s2 - lens_a)*pow(lens_s2 - dead_reckoning.error,2)));

     //std::cout << "dead_segment: " << dead_segment << std::endl;

     //return the area of the lens
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

    if(best_fit_done)
        _yaw_rotation.write(rotation_angle);
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
