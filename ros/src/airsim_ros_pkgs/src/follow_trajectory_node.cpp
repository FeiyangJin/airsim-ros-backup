#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <signal.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include "common_mav.h"
#include <mav_msgs/default_topics.h>    
#include <iostream>
#include <fstream>
#include <std_srvs/SetBool.h>
#include <airsim_ros_pkgs/multiDOF.h>
#include <airsim_ros_pkgs/multiDOF_array.h>
#include <airsim_ros_pkgs/follow_trajectory_status_srv.h>
#include <airsim_ros_pkgs/BoolPlusHeader.h>

using namespace std;

// add by feiyang jin
	bool fly_back = false;
    std_msgs::Bool fly_back_msg;
    bool global_stop_fly = false;

bool slam_lost = false;


trajectory_t normal_traj;
trajectory_t rev_normal_traj;
float g_localization_status = 1.0;
float g_v_max = 2.5;

float g_max_yaw_rate = 90;
float g_max_yaw_rate_during_flight = 90;

bool g_trajectory_done;

bool should_panic = false;
geometry_msgs::Vector3 panic_velocity;
bool g_got_new_trajectory = false;

int traj_id = 0;


void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}

void panic_callback(const std_msgs::Bool::ConstPtr& msg) {
    should_panic = msg->data;
}

void panic_velocity_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    panic_velocity = *msg;
}


void callback_trajectory(const airsim_ros_pkgs::multiDOF_array::ConstPtr& msg)
{
	ROS_INFO("call back trajectory");
    if(!fly_back){
        normal_traj.clear(); 
        rev_normal_traj.clear();
        for (auto point : msg->points){
            multiDOFpoint traj_point;
            traj_point.x = point.x;
            traj_point.y = point.y;
            traj_point.z = point.z;
            traj_point.vx = point.vx;
            traj_point.vy = point.vy;
            traj_point.vz = point.vz;
            traj_point.yaw = point.yaw;
            traj_point.duration = point.duration;
            normal_traj.push_back(traj_point);
        }
    }
    else{
        rev_normal_traj.clear();
        normal_traj.clear();
        for (auto point : msg->points){
            multiDOFpoint traj_point;
            traj_point.x = point.x;
            traj_point.y = point.y;
            traj_point.z = point.z;
            traj_point.vx = point.vx;
            traj_point.vy = point.vy;
            traj_point.vz = point.vz;
            traj_point.yaw = point.yaw;
            traj_point.duration = point.duration;
            rev_normal_traj.push_back(traj_point);
        }
    }

    traj_id = msg->traj_id;

    g_got_new_trajectory = true; 
}


bool trajectory_done(const trajectory_t& trajectory) {
    g_trajectory_done = (trajectory.size() == 0);
    return g_trajectory_done;
}


bool follow_trajectory_status_cb(airsim_ros_pkgs::follow_trajectory_status_srv::Request &req, 
    airsim_ros_pkgs::follow_trajectory_status_srv::Response &res)
{
    //res.success.data = true;
    //ROS_INFO("trajectory done: %i", g_trajectory_done);
    res.success.data = g_trajectory_done;

    const multiDOFpoint& current_point =
        normal_traj.empty() ? rev_normal_traj.front() : normal_traj.front();
    
    geometry_msgs::Twist last_velocity;
    last_velocity.linear.x = current_point.vx;
    last_velocity.linear.y = current_point.vy;
    last_velocity.linear.z = current_point.vz;
    res.twist = last_velocity;
  
    geometry_msgs::Twist last_acceleration;
    last_acceleration.linear.x = current_point.ax;
    last_acceleration.linear.y = current_point.ay;
    last_acceleration.linear.z = current_point.az;
    res.acceleration = last_acceleration;
    return true;
}


airsim_ros_pkgs::multiDOF_array next_steps_msg(const trajectory_t& traj, const int true_id) {
    airsim_ros_pkgs::multiDOF_array array_of_point_msg;

    for (const auto& point : traj){
        airsim_ros_pkgs::multiDOF point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;
        point_msg.z = point.z;
        point_msg.vx = point.vx;
        point_msg.vy = point.vy;
        point_msg.vz = point.vz;
        point_msg.ax = point.ax;
        point_msg.ay = point.ay;
        point_msg.az = point.az;
        point_msg.yaw = point.yaw;
        point_msg.duration = point.duration;
        array_of_point_msg.points.push_back(point_msg); 
    }

    array_of_point_msg.traj_id = true_id;
    array_of_point_msg.header.stamp = ros::Time::now();
    return array_of_point_msg;
}


void stop_fly_callback(const std_msgs::Bool::ConstPtr& msg){
    bool stop_fly_local = msg->data;
    global_stop_fly = stop_fly_local;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "follow_trajectory", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    // airsim setup 
	    AirsimROSWrapper airsim_ros_wrapper(n, n_private);
	    if (airsim_ros_wrapper.is_used_img_timer_cb_queue_)
	        {
	            airsim_ros_wrapper.img_async_spinner_.start();
	        }

	        if (airsim_ros_wrapper.is_used_lidar_timer_cb_queue_)
	        {
	            airsim_ros_wrapper.lidar_async_spinner_.start();
	        }

	        airsim_ros_wrapper.takeoff_jin();

    //variable
	    std::string localization_method; 
	    std::string mav_name;
	    ros::Time cur_t, last_t;
	    float cur_z, last_z = -9999;
	    trajectory_t panic_traj;
	    trajectory_t slam_loss_traj;
	    bool created_slam_loss_traj = false;
	    bool created_future_col_traj = false;
	    trajectory_t future_col_traj;
	    ros::Rate loop_rate(50);
        g_trajectory_done = false;

    //publisher and subscriber
	    ros::ServiceServer trajectory_done_service = n.advertiseService("follow_trajectory_status", follow_trajectory_status_cb);

	    ros::Publisher next_steps_pub = n.advertise<airsim_ros_pkgs::multiDOF_array>("/next_steps", 1);

	    ros::Subscriber panic_sub =  n.subscribe<std_msgs::Bool>("panic_topic", 1, panic_callback);
	    ros::Subscriber panic_velocity_sub = n.subscribe<geometry_msgs::Vector3>("panic_velocity", 1, panic_velocity_callback);
	    

		ros::Subscriber slam_lost_sub = n.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
	    ros::Subscriber trajectory_follower_sub = n.subscribe<airsim_ros_pkgs::multiDOF_array>("normal_traj", 1, callback_trajectory);

        ros::Publisher fly_back_pub = n.advertise<std_msgs::Bool>("/fly_back", 1);

        ros::Subscriber stop_fly_sub = 
            n.subscribe<std_msgs::Bool>("/stop_fly", 1, stop_fly_callback);

    bool app_started = false;  //decides when the first planning has occured
                               //this allows us to activate all the
                               //functionaliy in follow_trajecotry accordingly

    yaw_strategy_t yaw_strategy = face_forward;
    
    while (ros::ok()) {
        
    	ros::spinOnce();

        trajectory_t * forward_traj = nullptr;
        trajectory_t * rev_traj = nullptr;
        bool check_position = true;
        
        // panic
	        if (should_panic) {
	            ROS_INFO("Panicking!");
	            ROS_INFO("Panicking!");
	            ROS_INFO("Panicking!");
	            ROS_INFO("Panicking!");
	            panic_traj = create_panic_trajectory(airsim_ros_wrapper, panic_velocity);
	            normal_traj.clear(); // Replan a path once we're done
	        } 
	        else {
	            panic_traj.clear();
	        }

       	// slam lost
	        if (slam_lost) {
	            ROS_WARN("SLAM lost!");
	            if (!created_slam_loss_traj)
	                slam_loss_traj = create_slam_loss_trajectory(airsim_ros_wrapper, normal_traj, rev_normal_traj);
	            	created_slam_loss_traj = true;
	        } 
	        else {
	            slam_loss_traj.clear();
	            created_slam_loss_traj = false;
	        }

        // setup trajectory
        if (!panic_traj.empty()) {
            forward_traj = &panic_traj;
            rev_traj = nullptr;
            check_position = false;
            yaw_strategy = ignore_yaw;
        } else if (!slam_loss_traj.empty()) {
            forward_traj = &slam_loss_traj;
            rev_traj = &normal_traj;
            check_position = false;
        }
        else {
            forward_traj = &normal_traj;
            rev_traj = &rev_normal_traj;
        }

        
        if ((!fly_back && normal_traj.size() > 0) || (fly_back && rev_normal_traj.size() > 0)) {
            app_started = true;
        }


        // make sure if new traj come, we do not conflict
        const int id_for_this_traj = traj_id;
        if(app_started && !global_stop_fly){
            // Back up if no trajectory was found
            if (!forward_traj->empty()){
                follow_trajectory(airsim_ros_wrapper, forward_traj, nullptr, yaw_strategy, check_position, g_v_max);
            }
            else {
                follow_trajectory(airsim_ros_wrapper, rev_traj, nullptr, yaw_strategy, true, g_v_max);
            }

            if (forward_traj->size() > 0){
                next_steps_pub.publish(next_steps_msg(*forward_traj,id_for_this_traj));
            }
            else if(rev_traj->size() > 0){
                next_steps_pub.publish(next_steps_msg(*rev_traj,id_for_this_traj));
            }
        }


        if (slam_lost && created_slam_loss_traj && trajectory_done(slam_loss_traj)){
            ROS_INFO_STREAM("slam loss");
            g_localization_status = 0; 
        }
        else if(app_started && fly_back && trajectory_done(*rev_traj)){
        	ROS_INFO_STREAM("mission completed");
        	g_trajectory_done = true;
            app_started = false;
        }
        else if (app_started && !fly_back && trajectory_done(*forward_traj)){
            fly_back = true;
            fly_back_msg.data = true;
            fly_back_pub.publish(fly_back_msg);
            app_started = false;
            g_trajectory_done = false;
            ROS_INFO("front trajectory done");
            loop_rate.sleep();
        }

        g_got_new_trajectory = false;

    }
	return 0;
}