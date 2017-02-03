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
        // receive sensor to body transformation
        Eigen::Affine3d sensorInBody;
        if (!getSensorInBody(_gps2body, ts, sensorInBody))
            return;

        if(gps_position_samples_sample.position.block(0,0,2,1).allFinite() && gps_position_samples_sample.cov_position.block(0,0,2,2).allFinite())
        {
            try
            {
                // apply sensorInBody transformation to measurement
                Eigen::Affine3d gpsInWorld = Eigen::Affine3d::Identity();
                gpsInWorld.translation() = Eigen::Vector3d(gps_position_samples_sample.position.x(), gps_position_samples_sample.position.y(), 0.0);
                Eigen::Affine3d bodyInWorld = gpsInWorld * sensorInBody.inverse();

                //Get gps data alredy at the point structure
                point gps_current;
                point dead_near_time;

                gps_current.sample_time = gps_position_samples_sample.time;
                gps_current.x =  bodyInWorld.translation()[0];
                gps_current.y =  bodyInWorld.translation()[1];
                gps_current.error = gps_position_samples_sample.cov_position(0,0);

                if(matchSampleTime(gps_current.sample_time, dead_near_time))
                {
                    gps_sync[count_acquire] = gps_current;
                    dead_sync[count_acquire] = dead_near_time;
                    count_acquire++;
                }
                else{
                    RTT::log(RTT::Error) << "Dead rechoning buffer deque is empety or no match for this salpme time was found!" << RTT::endlog();
                }

            }
            catch(const std::runtime_error& e)
            {
                RTT::log(RTT::Error) << "Failed to add GPS measurement: " << e.what() << RTT::endlog();
            }
        }
        else
            RTT::log(RTT::Error) << "GPS position measurement contains NaN's, it will be skipped!" << RTT::endlog();
    }else{
        acquire_done = true;
    }
}

void Task::pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
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

bool Task::getSensorInBody(const transformer::Transformation& sensor2body_transformer, const base::Time& ts, Eigen::Affine3d& sensorInBody)
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

double Task::pointRotation(const point &gps, const point &dead_reckoning){

    double angle;

    // Pegar a media do atan2 de dead e gps e guardar (definir direção da rotação) #####

    angle = atan2(gps.y, gps.x) - atan2(dead_reckoning.y, dead_reckoning.x);
    if (angle < 0) angle += 2 * M_PI;
    //fazer a saida ser em graus;
    return angle;
}

//Get a sample time as reference and try to find the least difference of time at the dead_reckoning current buffer.
bool Task::matchSampleTime(base::Time sample_time, point& dead_near_time){

    int64_t time_diff_min = 10000000;
    int64_t time_diff = 0;

    if(!dead_buffer_queue.empty())
    {
        while(!dead_buffer_queue.empty())
        {
            time_diff = abs((sample_time - dead_buffer_queue.front().sample_time).toMicroseconds());

            if(time_diff < time_diff_min )
            {
               time_diff_min = time_diff;
               dead_near_time = dead_buffer_queue.front();
            }
            dead_buffer_queue.pop();
        }

        if(_sync_time_tolerance.get().toMicroseconds()<time_diff)
            return false;
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
        rotations[i] = pointRotation(gps_sync[i],dead_sync[i]);

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

//
double Task::findBestFitAngle(double peak){

    double search_range_angle = peak - (_angle_search_range.get()*M_PI)/180;
    double search_range_end = peak + (_angle_search_range.get()*M_PI)/180;

    //0.1 degrre in rad
    double search_increment = (0.1*M_PI)/180;
    point dead_rotated;
    double total_lens_area, max_total_lens_area = 0;
    double bestFitAngle;

    while( search_range_angle < search_range_end)
    {
        for (int i = 0; i < _number_samples_to_acquire.get(); ++i)
        {
            dead_rotated = dead_sync[i];

            dead_rotated.x = dead_sync[i].x*cos(search_range_angle)  - dead_sync[i].y*sin(search_range_angle);
            dead_rotated.y = dead_sync[i].x*sin(search_range_angle)  + dead_sync[i].y*cos(search_range_angle);

            total_lens_area += lensArea(gps_sync[i],dead_rotated);
        }

        //checar max and min BUSCA EM TODAS AS FUNÇOES (VALOR INICIAL)! <<<<<<<<
        if(total_lens_area > max_total_lens_area)
        {
            bestFitAngle = search_range_angle;
            max_total_lens_area = total_lens_area;
        }

        search_range_angle += search_increment;
    }

    return bestFitAngle;
}

//
double Task::lensArea(const point &gps, const point &dead_reckoning)
{
    double points_distance = sqrt(pow(gps.x - dead_reckoning.x,2) + pow(gps.y - dead_reckoning.y,2));

    //In case they don't have a lens
    if( points_distance > dead_reckoning.error + gps.error)
        return 0;

    //In case that the circles have the same origin R1<R2, R1>R2, R1=R2
    if(dead_reckoning.x == gps.x && dead_reckoning.y == gps.y)
    {
        if(dead_reckoning.error < gps.error)
            return M_PI*pow(dead_reckoning.error,2);
        else
            return M_PI*pow(gps.error,2);
    }

    //Function of lens discribed at 4 eq.15 (Area of two overlapping circles - ben breech)

    //Repetble parts of the function
     double lens_a = sqrt((-points_distance + gps.error + dead_reckoning.error)*
                         (points_distance - gps.error + dead_reckoning.error)*
                         (points_distance + gps.error - dead_reckoning.error)*
                         (points_distance + gps.error + dead_reckoning.error))
                         /points_distance;

     double lens_s1 = (lens_a + 2*gps.error)/2;
     double lens_s2 = (lens_a + 2*dead_reckoning.error)/2;

     //calculate the area of each segment of the lens
     double gps_segment = pow(gps.error,2)*asin(lens_a/2*gps.error) -
                          sqrt(lens_s1*(lens_s1 - lens_a)*pow(lens_s1 - gps.error,2));

     double dead_segment = pow(dead_reckoning.error,2)*asin(lens_a/2*dead_reckoning.error) -
                           sqrt(lens_s2*(lens_s2 - lens_a)*pow(lens_s2 - dead_reckoning.error,2));

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

    if(acquire_done)
    {
        if(!best_fit_done)
        {
            point dead_init =  dead_sync[0];
            point gps_init = gps_sync[0];

            //bring points origin to (0,0);
            for (int i = 0; i < _number_samples_to_acquire.get(); ++i)
            {
                dead_sync[i].x -= dead_init.x;
                dead_sync[i].y -= dead_init.y;
                gps_sync[i].x -= gps_init.x;
                gps_sync[i].y -= gps_init.y;

                // Put two output here to debug..
            }

            //Get the best rotation fit around the histogram peak
            rotation_angle = findBestFitAngle(rotationHistPeak());

            best_fit_done = true;
            _yaw_rotation.write(rotation_angle);

        }else
            _yaw_rotation.write(rotation_angle);
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
