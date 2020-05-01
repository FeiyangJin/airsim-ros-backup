#include "common_mav.h"

#include <iostream>
#include <fstream>
#include <string>
#include <deque>
#include <algorithm>
#include <cmath>
#include <cstdarg>

#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>


template <class T>
static T magnitude(T a, T b, T c) {
    return std::sqrt(a*a + b*b + c*c);
}

static trajectory_t append_trajectory (trajectory_t first, const trajectory_t& second);

static multiDOFpoint reverse_point(multiDOFpoint mdp) {
    multiDOFpoint result = mdp;

    result.vx = -mdp.vx;
    result.vy = -mdp.vy;
    result.vz = -mdp.vz;

    return result;
}

static trajectory_t append_trajectory (trajectory_t first, const trajectory_t& second) {
    first.insert(first.end(), second.begin(), second.end());
    return first;
}

// check if we finish flying with the trajectory
bool trajectory_done(trajectory_t& trajectory) {
    return trajectory.size() == 0;
}

// Follows trajectory, popping commands off the front of it and returning those commands in reverse order
void follow_trajectory(AirsimROSWrapper& airsim_ros_wrapper, trajectory_t * traj,trajectory_t * reverse_traj, yaw_strategy_t yaw_strategy, bool check_position, float max_speed, float time){
    //ROS_INFO("follow trajectory !");
	trajectory_t reversed_commands;

    static double max_speed_so_far = 0;
    static int ctr = 0;
    
    ros::Time start_hook_t;
    while (time > 0 && traj->size() > 0) {
         
        start_hook_t = ros::Time::now();  
        multiDOFpoint p = traj->front();

        // Calculate the velocities we should be flying at
        double v_x = p.vx;
        double v_y = p.vy;
        double v_z = p.vz;

        auto pos = airsim_ros_wrapper.getPosition();
        if (check_position) {
            v_x += 0.2*(p.x-pos.y());
            v_y += 0.2*(p.y-pos.x());
            v_z += 0.5*(p.z+pos.z());
        }

        // Calculate the yaw we should be flying with
        float yaw = p.yaw;
        if (yaw_strategy == ignore_yaw)
            yaw = YAW_UNCHANGED;
        else if (yaw_strategy == face_forward)
            yaw = FACE_FORWARD;
        else if (yaw_strategy == face_backward) {
            yaw = FACE_BACKWARD;
        }
        else if (yaw_strategy == follow_yaw){
            // do nothing
        }

        // Make sure we're not going over the maximum speed
        double speed = std::sqrt((v_x*v_x + v_y*v_y + v_z*v_z));
        double scale = 1;
        if (speed > max_speed) {
            scale = max_speed / speed;
            
            v_x *= scale;
            v_y *= scale;
            v_z *= scale;
           speed = std::sqrt((v_x*v_x + v_y*v_y + v_z*v_z));
        }

        // Calculate the time for which these flight commands should run
        double flight_time = p.duration <= time ? p.duration : time;
        double scaled_flight_time = flight_time / scale;

        // Fly for flight_time seconds
        auto segment_start_time = std::chrono::system_clock::now();
        airsim_ros_wrapper.fly_velocity(v_x, v_y, v_z, yaw, scaled_flight_time); 

        std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>(scaled_flight_time));

        // Push completed command onto reverse-command stack
        multiDOFpoint rev_point = reverse_point(p);
        rev_point.duration = flight_time;
        reversed_commands.push_front(rev_point);

        // Update trajectory
        traj->front().duration -= flight_time;
        if (traj->front().duration <= 0){
            traj->pop_front();
        }

        time -= flight_time;
    }


    if (reverse_traj != nullptr){
        *reverse_traj = append_trajectory(reversed_commands, *reverse_traj);
    }

}


float yawFromQuat(geometry_msgs::Quaternion q)
{
	float roll, pitch, yaw;

	// Formulas for roll, pitch, yaw
	// roll = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y) );
	// pitch = asin(2*(q.w*q.y - q.z*q.x));
	yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    yaw = (yaw*180)/3.14159265359;

    return (yaw <= 180 ? yaw : yaw - 360);
}


trajectory_t create_trajectory(const trajectory_msgs::MultiDOFJointTrajectory& t, bool face_forward)
{
    trajectory_t result;
    for (auto it = t.points.begin(); it+1 != t.points.end(); ++it) {
        multiDOFpoint mdp;

        mdp.x = it->transforms[0].translation.x;
        mdp.y = it->transforms[0].translation.y;
        mdp.z = it->transforms[0].translation.z;

        mdp.vx = it->velocities[0].linear.x;
        mdp.vy = it->velocities[0].linear.y;
        mdp.vz = it->velocities[0].linear.z;

        mdp.ax = it->accelerations[0].linear.x;
        mdp.ay = it->accelerations[0].linear.y;
        mdp.az = it->accelerations[0].linear.z;


        if (face_forward) {
            if (mdp.vx == 0 && mdp.vy == 0)
                mdp.yaw = YAW_UNCHANGED;
            else
                mdp.yaw = 90 - atan2(mdp.vy, mdp.vx)*180.0/3.14;
        } else {
            mdp.yaw = yawFromQuat(it->transforms[0].rotation);
        }

        mdp.duration = ((it+1)->time_from_start - it->time_from_start).toSec();

        result.push_back(mdp);
    }
    return result;
}

trajectory_msgs::MultiDOFJointTrajectory get_multiDOF_trajectory(mav_trajectory_generation::Trajectory scanning_trajectory){
    
    trajectory_msgs::MultiDOFJointTrajectory result_trajectory;
    // Sample trajectory
    mav_msgs::EigenTrajectoryPoint::Vector states;
    
    double sampling_interval = 0.1;
    mav_trajectory_generation::sampleWholeTrajectory(scanning_trajectory, sampling_interval, &states);

    int state_index = 0;
    for (const auto& s : states) {
        trajectory_msgs::MultiDOFJointTrajectoryPoint point;

        geometry_msgs::Transform pos;
        graph::node current;
        pos.translation.x = current.x = s.position_W.x();
        pos.translation.y = current.y = s.position_W.y();
        pos.translation.z = current.z = s.position_W.z();

        geometry_msgs::Twist vel;
        vel.linear.x = s.velocity_W.x();
        vel.linear.y = s.velocity_W.y();
        vel.linear.z = s.velocity_W.z();

        geometry_msgs::Twist acc;
        acc.linear.x = s.acceleration_W.x();
        acc.linear.y = s.acceleration_W.y();
        acc.linear.z = s.acceleration_W.z();

        ros::Duration dur(float(s.time_from_start_ns) / 1e9);

        point.transforms.push_back(pos);
        point.velocities.push_back(vel);
        point.accelerations.push_back(acc);
        point.time_from_start = dur;

        // if (res.unknown != -1 &&
        //         !known(octree, current.x, current.y, current.z)
        //         && dist(start, current) > safe_radius) {
        //     ROS_WARN("Trajectory enters unknown space.");
        //     res.unknown = state_index;
        // }

        result_trajectory.points.push_back(point);
        
        state_index++;
    }


    return result_trajectory;
}

trajectory_t create_panic_trajectory(AirsimROSWrapper& airsim_ros_wrapper, const geometry_msgs::Vector3& panic_velocity)
{
    multiDOFpoint p;

    p.yaw = airsim_ros_wrapper.get_yaw();

    p.vx = panic_velocity.x * std::cos(M_PI/90 - p.yaw*M_PI/180);
    p.vy = panic_velocity.y * std::cos(p.yaw*M_PI/180);
    p.vz = panic_velocity.z + 0.1; // Counter-act AirSim's slight downward drift

    p.duration = std::numeric_limits<double>::infinity();

    trajectory_t result;
    result.push_back(p);

    return result;
}


trajectory_t create_slam_loss_trajectory(AirsimROSWrapper& airsim_ros_wrapper, trajectory_t& normal_traj, const trajectory_t& rev_normal_traj)
{
    trajectory_t result;
    float current_yaw = airsim_ros_wrapper.get_yaw();
    auto current_pos = airsim_ros_wrapper.getPosition();

    // Add pause to trajectory
    multiDOFpoint pause_p;
    pause_p.x = current_pos.x();
    pause_p.y = current_pos.y();
    pause_p.z = current_pos.z();
    pause_p.vx = pause_p.vy = pause_p.vz = 0;
    pause_p.duration = 2.0;
    pause_p.yaw = current_yaw;

    result.push_back(pause_p);

    // Add spinning around to trajectory
    const float scanning_width = 45;

    multiDOFpoint scan_p;
    scan_p.x = current_pos.x();
    scan_p.y = current_pos.y();
    scan_p.z = current_pos.z();
    scan_p.vx = scan_p.vy = scan_p.vz = 0;
    scan_p.duration = scanning_width / airsim_ros_wrapper.maxYawRateDuringFlight();

    float yaws[] = {current_yaw - scanning_width, current_yaw, current_yaw + scanning_width, current_yaw};

    for (float y : yaws) {
        scan_p.yaw = y;
        result.push_back(scan_p);
    }

    // Add backtrack to trajectory
    double distance_left = 500.0; // TODO: make this a reasonable number
    const double safe_v = 1.0;

    for (multiDOFpoint p : rev_normal_traj) {
        double v = magnitude(p.vx, p.vy, p.vz);

        if (v*p.duration > distance_left) {
            p.duration = distance_left / v;
        }

        distance_left -= v*p.duration;
        
        double scale = v > safe_v ? safe_v/v : 1;

        p.vx *= scale;
        p.vy *= scale;
        p.vz *= scale;
        p.duration /= scale;

        result.push_back(p);

        if (distance_left <= 0)
            break;
    }

    // Add one last pause
    multiDOFpoint last_pause_p;
    last_pause_p.x = result.back().x;
    last_pause_p.y = result.back().y;
    last_pause_p.z = result.back().z;
    last_pause_p.vx = last_pause_p.vy = last_pause_p.vz = 0;
    last_pause_p.duration = 2.0;
    last_pause_p.yaw = YAW_UNCHANGED;

    result.push_back(last_pause_p);

    // Slow down normal_traj
    const double max_a = 1.0;
    double max_v = safe_v;

    for (multiDOFpoint& p : normal_traj) {
        double v = magnitude(p.vx, p.vy, p.vz);

        double scale = v > max_v ? max_v/v : 1;

        p.vx *= scale;
        p.vy *= scale;
        p.vz *= scale;
        p.duration /= scale;

        max_v += max_a*p.duration;
    }

    return result;
}

float distance(float x, float y, float z) {
  return std::sqrt(x*x + y*y + z*z);
}

void spin_around(AirsimROSWrapper& airsim_ros_wrapper){
    airsim_ros_wrapper.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ROS_INFO("Spinning around...");
    ros::Time last_time;
    float init_yaw = airsim_ros_wrapper.get_yaw();
    double start_z = airsim_ros_wrapper.pose().position.z; // Get drone's current position


    int angle_corrected;
    for (int i = 0; i <= 360; i += 90) {
        int angle = init_yaw + i;
        angle_corrected  = (angle <= 180 ? angle : angle - 360);
        airsim_ros_wrapper.set_yaw_at_z(angle_corrected, start_z);
    }

    // to correct 
    double dz = start_z - airsim_ros_wrapper.pose().position.z;
    double vz = dz > 0 ? 1 : -1;
    double dt = dz > 0 ? dz : -dz;
    
    airsim_ros_wrapper.fly_velocity(0, 0, vz, YAW_UNCHANGED, dt);
    std::this_thread::sleep_for(std::chrono::milliseconds(int(dt*1000.0)));
    airsim_ros_wrapper.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}