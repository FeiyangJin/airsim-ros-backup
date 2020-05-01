#ifndef COMMON_MAV_H
#define COMMON_MAV_H

#include <string>
#include <limits>
#include <deque>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Vector3.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "graph.h"

#include "airsim_ros_wrapper.h"

double max_velocity = 2.5;
double max_accelaration = 5;

// Functions and classes to manipulate and follow trajectories
struct multiDOFpoint {
    double x, y, z;
    double vx, vy, vz;
    double ax, ay, az;
    double yaw;
    double duration;
};
typedef std::deque<multiDOFpoint> trajectory_t;
enum yaw_strategy_t { ignore_yaw, face_forward, face_backward, follow_yaw };

bool trajectory_done(trajectory_t& trajectory);

void follow_trajectory(AirsimROSWrapper& airsim_ros_wrapper, trajectory_t * traj,
        trajectory_t * reverse_traj,
        yaw_strategy_t yaw_strategy = ignore_yaw,
        bool check_position = true,
        float max_speed = std::numeric_limits<double>::infinity(),
        //float max_speed = 3,
        float time = 2);

float yawFromQuat(geometry_msgs::Quaternion q);

trajectory_t create_trajectory(const trajectory_msgs::MultiDOFJointTrajectory& t, bool face_forward);

trajectory_msgs::MultiDOFJointTrajectory get_multiDOF_trajectory(mav_trajectory_generation::Trajectory scanning_trajectory);


// Recovery methods
trajectory_t create_panic_trajectory(AirsimROSWrapper& airsim_ros_wrapper, const geometry_msgs::Vector3& panic_dir);
trajectory_t create_future_col_trajectory(const trajectory_t& normal_traj, double stopping_distance);
trajectory_t create_slam_loss_trajectory(AirsimROSWrapper& airsim_ros_wrapper, trajectory_t& normal_traj, const trajectory_t& rev_normal_traj);



// Spinning commands
void spin_around(AirsimROSWrapper& airsim_ros_wrapper);
//void scan_around(Drone &drone, int angle);


// Utility functions
float distance(float x, float y, float z);

#endif