// Standard headers
#include <cstdlib>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
#include <iostream>
#include <thread>
#include <functional>
#include <limits>
#include <signal.h>
#include "std_msgs/Bool.h"

// My headers
#include "common_mav.h"
#include "graph.h"
#include "airsim_ros_pkgs/get_trajectory.h"

// Octomap specific headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_server/OctomapServer.h>

// Trajectory smoothening headers
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

// OMPL specific headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

using namespace std;
using namespace octomap;
using piecewise_trajectory = std::vector<graph::node>;
using smooth_trajectory = mav_trajectory_generation::Trajectory;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;


namespace ob = ompl::base;
namespace og = ompl::geometric;

// TODO: change the bound to something meaningful
double x__low_bound__global = -200, x__high_bound__global = 200;
double y__low_bound__global = -200 , y__high_bound__global = 200;
double z__low_bound__global = 0, z__high_bound__global = 40;
double sampling_interval__global = 0.5;
double v_max__global = 3, a_max__global = 5;
float g_planning_budget = 4;
std::string motion_planning_core_str;

octomap::OcTree * octree = nullptr;
trajectory_msgs::MultiDOFJointTrajectory traj_topic;
bool g_requested_trajectory = false;
bool path_found = false;

double drone_height__global = 0.6;
double drone_radius__global = 2;


std::function<piecewise_trajectory (geometry_msgs::Point, geometry_msgs::Point, int, int , int, octomap::OcTree *)> motion_planning_core;

void setup();
bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr = nullptr);
bool out_of_bounds(const graph::node& pos);
// *** F:DN Optimize and smoothen a piecewise path without causing any new collisions.
smooth_trajectory smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree, Eigen::Vector3d initial_velocity, Eigen::Vector3d initial_acceleration);


// ***F:DN Build the response to the service from the smooth_path
void create_response(airsim_ros_pkgs::get_trajectory::Response &res, smooth_trajectory& smooth_path);


// ***F:DN Use the PRM sampling method to find a piecewise path
piecewise_trajectory PRM(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);


// ***F:DN Use the RRT sampling method to find a piecewise path
piecewise_trajectory RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);


// ***F:DN Use the RRT sampling method from OMPL to find a piecewise path
piecewise_trajectory OMPL_RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);


// ***F:DN Use bi-directonal RRT from OMPL to find a piecewise path
piecewise_trajectory OMPL_RRTConnect(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);


// ***F:DN Use the PRM sampling method from OMPL to find a piecewise path
piecewise_trajectory OMPL_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);

piecewise_trajectory OMPL_RRTstar(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);

#ifdef INFLATE
  bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr)
  {
      // First, check if anything goes underground
      if (n1.z <= 0 || n2.z <= 0)
          return true;
                
      double dx = n2.x - n1.x;
      double dy = n2.y - n1.y;
      double dz = n2.z - n1.z;

      double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

      octomap::point3d start(n1.x, n1.y, n1.z);
      octomap::point3d direction(dx, dy, dz);
      octomap::point3d end;

      bool collided = octree->castRay(start, direction, end, true, distance);

      if (end_ptr != nullptr && collided) {
          end_ptr->x = end.x();
          end_ptr->y = end.y();
          end_ptr->z = end.z();
      }

      return collided;
  }
#else
  bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr)
  {
      
      // First, check if anything goes underground
      if (n1.z <= 0 || n2.z <= 0)
          return true;
              
      const double pi = 3.14159265359;

      // The drone is modeled as a cylinder.
      // Angles are in radians and lengths are in meters.
      
      double height = drone_height__global; 
      double radius = drone_radius__global; 

      const double angle_step = pi/4;
      const double radius_step = radius/3;
      const double height_step = height/2;

      double dx = n2.x - n1.x;
      double dy = n2.y - n1.y;
      double dz = n2.z - n1.z;

      double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

      octomap::point3d direction(dx, dy, dz);
      octomap::point3d end;

      for (double h = -height/2; h <= height/2; h += height_step) {
        for (double r = 0; r <= radius; r += radius_step) {
          for (double a = 0; a <= pi*2; a += angle_step) {
            octomap::point3d start(n1.x + r*std::cos(a), n1.y + r*std::sin(a), n1.z + h);

            if (octree->castRay(start, direction, end, true, distance)) {

                        if (end_ptr != nullptr) {
                            end_ptr->x = end.x();
                            end_ptr->y = end.y();
                            end_ptr->z = end.z();
                        }
              return true;
              }
          }
        }
      }

      return false;
  }
#endif

bool out_of_bounds(const graph::node& pos) {
    return (pos.x < x__low_bound__global
            || pos.x > x__high_bound__global
            || pos.y < y__low_bound__global
            || pos.y > y__high_bound__global
            || pos.z < z__low_bound__global
            || pos.z > z__high_bound__global);
}

bool occupied(octomap::OcTree * octree, double x, double y, double z){
  const double OCC_THRESH = 0.5;
  octomap::OcTreeNode * otn = octree->search(x, y, z);
  // if(otn == nullptr){
  //   return false;
  // }
  // else{
  //   return otn->getOccupancy() >= OCC_THRESH;
  // }
    // Debug
    // cout << x << "  " << y << " " << z << "   ";
    // if(otn != nullptr){
    //   cout << otn->getOccupancy() << endl;
    // }
    // else{
    //   cout << "null pointer" << endl;
    // }
  
  // if it is nullptr, return false, meanning it is not occupied
  // if it is not null ptr, check the probability, return occupied if 
  // the prob is bigger than the threashould.
  return otn != nullptr && otn->getOccupancy() >= OCC_THRESH;
}


bool OMPLStateValidityChecker(const ompl::base::State * state)
{
    const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

    double x = pos->values[0];
    double y = pos->values[1];
    double z = pos->values[2];

    return !occupied(octree, x, y, z);
}


class OMPLMotionValidator : public ompl::base::MotionValidator
{
	public:
	    OMPLMotionValidator(const ompl::base::SpaceInformationPtr &si)
	        : ompl::base::MotionValidator(si)
	    {
	    }

	    bool checkMotion(const ompl::base::State *s1,
	            const ompl::base::State *s2) const override
	    {
	        const auto *pos1 = s1->as<ob::RealVectorStateSpace::StateType>();
	        const auto *pos2 = s2->as<ob::RealVectorStateSpace::StateType>();

	        double x1 = pos1->values[0], x2 = pos2->values[0];
	        double y1 = pos1->values[1], y2 = pos2->values[1];
	        double z1 = pos1->values[2], z2 = pos2->values[2];

	        graph::node end;
	        return !collision(octree, {x1,y1,z1}, {x2,y2,z2}, &end);
	    }

	    bool checkMotion(const ompl::base::State *s1,
	            const ompl::base::State *s2,
	            std::pair<ompl::base::State*, double>& lastValid) const override
	    {
	        namespace ob = ompl::base;

	        const auto *pos1 = s1->as<ob::RealVectorStateSpace::StateType>();
	        const auto *pos2 = s2->as<ob::RealVectorStateSpace::StateType>();

	        double x1 = pos1->values[0], x2 = pos2->values[0];
	        double y1 = pos1->values[1], y2 = pos2->values[1];
	        double z1 = pos1->values[2], z2 = pos2->values[2];

	        graph::node end;
	        bool valid = !collision(octree, {x1,y1,z1}, {x2,y2,z2}, &end);

	        if (!valid) {
	            auto *end_pos = lastValid.first->as<ob::RealVectorStateSpace::StateType>();
	            end_pos->values[0] = end.x;
	            end_pos->values[1] = end.y;
	            end_pos->values[2] = end.z;

	            double dx = x2-x1, dy = y2-y1, dz = z2-z1;
	            double end_dx = end.x-x1, end_dy = end.y-y1, end_dz = end.z-z1;

	            if (dx != 0)
	                lastValid.second = end_dx / dx;
	            else if (dy != 0)
	                lastValid.second = end_dy / dy;
	            else if (dz != 0)
	                lastValid.second = end_dz / dz;
	            else
	                lastValid.second = 0;
	        }

	        return valid;
	    }

};


void postprocess(piecewise_trajectory& path)
{
    // We use a greedy approach to shorten the path here.
    // We connect non-adjacent nodes in the path that do not have collisions.
    for (auto it = path.begin(); it != path.end()-1; ) {
        bool shortened = false;
        for (auto it2 = path.end()-1; it2 != it+1 && !shortened; --it2) {
            if (!collision(octree, *it, *it2)) {
                it = path.erase(it+1, it2);
                shortened = true;
            }
        }

        if (!shortened)
            ++it;
    }
}


// function for generating octree from octomap msgs
void generate_octomap(const octomap_msgs::Octomap& msg)
{
    //cout << "we are generating octomap from octomap binary" << endl;
    if (octree != nullptr) {
        delete octree;
    }
    octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(msg);
     octree = dynamic_cast<octomap::OcTree*> (tree);

    if (octree == nullptr) {
        ROS_ERROR("Octree could not be pulled.");
    }
}



smooth_trajectory smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree, Eigen::Vector3d initial_velocity, Eigen::Vector3d initial_acceleration)
{
    // Variables for visualization for debugging purposes
    double distance = 0.5; 

    // Setup optimizer
    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  
    // Convert roadmap path to optimizer's path format
    mav_trajectory_generation::Vertex start_v(dimension), end_v(dimension);
    start_v.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, initial_velocity);
    start_v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(piecewise_path.front().x, piecewise_path.front().y, piecewise_path.front().z));
    

    end_v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(piecewise_path.back().x, piecewise_path.back().y, piecewise_path.back().z));
    end_v.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));

    vertices.push_back(start_v);
    for (auto it = piecewise_path.begin()+1; it+1 != piecewise_path.end(); ++it) {
      mav_trajectory_generation::Vertex v(dimension);
      v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(it->x, it->y, it->z));
      vertices.push_back(v);
    }
    vertices.push_back(end_v);

    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);

    // Optimize until no collisions are present
    bool col;
    do {
      col = false;
      // Estimate the time the drone should take flying between each node
      auto segment_times = estimateSegmentTimes(vertices, v_max__global, a_max__global);
  
      std::vector<double>  times;
      for (auto el :segment_times) {
          times.push_back(.8*el); 
      }


      // Optimize and create a smooth path from the vertices
      opt.setupFromVertices(vertices, times, derivative_to_optimize);
      opt.solveLinear();

      // Return all the smooth segments in the path
      // (Each segment goes from one of the original nodes to the next one in the path)
      mav_trajectory_generation::Segment::Vector segments;
      opt.getSegments(&segments);

      // Loop through the vector of segments looking for collisions
      for (int i = 0; !col && i < segments.size(); ++i) {
        const double time_step = 0.1;
        double segment_len = segments[i].getTime();

        auto segment_start = *(piecewise_path.begin() + i);
        auto segment_end = *(piecewise_path.begin() + i + 1);

        // Step through each individual segment, at increments of "time_step" seconds, looking for a collision
        for (double t = 0; t < segment_len - time_step; t += time_step) {
            auto pos1 = segments[i].evaluate(t);
            auto pos2 = segments[i].evaluate(t + time_step);

            graph::node n1 = {pos1.x(), pos1.y(), pos1.z()};
            graph::node n2 = {pos2.x(), pos2.y(), pos2.z()};

            // Check for a collision between two near points on the segment
            if (out_of_bounds(n1) || out_of_bounds(n2) || collision(octree, n1, n2)) {     
                // Add a new vertex in the middle of the segment we are currently on
                mav_trajectory_generation::Vertex middle(dimension);

                double middle_x = (segment_start.x + segment_end.x) / 2;
                double middle_y = (segment_start.y + segment_end.y) / 2;
                double middle_z = (segment_start.z + segment_end.z) / 2;

                middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(middle_x, middle_y, middle_z));

                vertices.insert(vertices.begin()+i+1, middle);

                // Add a new node to the piecewise path where the vertex is
                graph::node middle_node = {middle_x, middle_y, middle_z};
                piecewise_path.insert(piecewise_path.begin()+i+1, middle_node);

                col = true;

                break;
            }
        }
      }
  } while (col);

  // Return the collision-free smooth trajectory
  mav_trajectory_generation::Trajectory traj;
  opt.getTrajectory(&traj);

  return traj;
}


bool get_trajectory_fun(airsim_ros_pkgs::get_trajectory::Request &req, airsim_ros_pkgs::get_trajectory::Response &res)
{
    g_requested_trajectory = true;
    x__low_bound__global = std::min(x__low_bound__global, req.start.x);
    x__high_bound__global = std::max(x__high_bound__global, req.start.x);
    y__low_bound__global = std::min(y__low_bound__global, req.start.y);
    y__high_bound__global = std::max(y__high_bound__global, req.start.y);
    z__low_bound__global = std::min(z__low_bound__global, req.start.z);
    z__high_bound__global = std::max(z__high_bound__global, req.start.z);


    piecewise_trajectory piecewise_path;
    smooth_trajectory smooth_path;

    //----------------------------------------------------------------- 
    // *** F:DN Body 
    //----------------------------------------------------------------- 

    if (motion_planning_core_str != "lawn_mower"){
        if (octree == nullptr) {
            ROS_ERROR("Octomap is not available.");
            res.path_found = false;
            return true;
        }
    }

    req.start.x += req.twist.linear.x*g_planning_budget;
    req.start.y += req.twist.linear.y*g_planning_budget;
    req.start.z += req.twist.linear.z*g_planning_budget;

    piecewise_path = motion_planning_core(req.start, req.goal, req.width, req.length ,req.n_pts_per_dir, octree);

    if (piecewise_path.size() == 0) {
        ROS_ERROR("Empty path returned");
        res.path_found = false;
        path_found = false;
        return true;
    }

    if (motion_planning_core_str != "lawn_mower") {
        postprocess(piecewise_path);
    }


    //Smoothen the path and build the multiDOFtrajectory response
    smooth_path = smoothen_the_shortest_path(piecewise_path, octree, 
                                    Eigen::Vector3d(req.twist.linear.x,
                                        req.twist.linear.y,
                                        req.twist.linear.z), 
                                    Eigen::Vector3d(req.acceleration.linear.x,
                                                    req.acceleration.linear.y,
                                                    req.acceleration.linear.z));
  
    create_response(res, smooth_path);

    // Publish the trajectory (for debugging purposes)
    traj_topic = res.multiDOFtrajectory;
    traj_topic.header.stamp = ros::Time::now();
    res.path_found = true;
    path_found = true;  
    return true;
}


void create_response(airsim_ros_pkgs::get_trajectory::Response &res, smooth_trajectory& smooth_path)
{
    const double safe_radius = 1.0;

    // Sample trajectory
    mav_msgs::EigenTrajectoryPoint::Vector states;
  
    mav_trajectory_generation::sampleWholeTrajectory(smooth_path, sampling_interval__global, &states);

    // Get starting position
    graph::node start = {states[0].position_W.x(), states[0].position_W.y(), states[0].position_W.z()};

    // Convert sampled trajectory points to MultiDOFJointTrajectory response
    res.multiDOFtrajectory.joint_names.push_back("base");
    res.unknown = -1;

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

      res.multiDOFtrajectory.points.push_back(point);
          
      state_index++;
    }
}


// set for package delivery
void setup(){
  // set up our planner
  ros::param::get("/motion_planner/motion_planning_core", motion_planning_core_str);
  if (motion_planning_core_str == "OMPL-RRT")
      motion_planning_core = OMPL_RRT;
  else if (motion_planning_core_str == "OMPL-RRTConnect")
      motion_planning_core = OMPL_RRTConnect;
  else if (motion_planning_core_str == "OMPL-PRM")
      motion_planning_core = OMPL_PRM;
  else if (motion_planning_core_str == "OMPL_RRTstar")
      motion_planning_core = OMPL_RRTstar;

  ros::param::get("/planning_budget", g_planning_budget);
  ros::param::get("/motion_planner/sampling_interval", sampling_interval__global);
  ros::param::get("/motion_planner/planner_drone_radius", drone_radius__global);
  ros::param::get("/motion_planner/planner_drone_height", drone_height__global);
  ros::param::get("/motion_planner/v_max", v_max__global);
  ros::param::get("/motion_planner/a_max", a_max__global);
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;

    motion_planning_core = OMPL_RRTConnect;
    setup();

    ros::ServiceServer service = nh.advertiseService("get_trajectory_srv", get_trajectory_fun);
    ros::Subscriber octomap_sub = nh.subscribe("octomap_binary", 1, generate_octomap);
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("multidoftraj", 1);


    enum State {publish_trajectory, idle};
    State state, next_state;
    next_state = state = idle;

    ros::Rate pub_rate(30);
    while (ros::ok())
    {
      if(state == idle){
        ros::spinOnce();
        if (g_requested_trajectory) {
            next_state = publish_trajectory;
        }
      }
      else if(state == publish_trajectory){
            if (path_found) { 
                traj_pub.publish(traj_topic);
            } 
            g_requested_trajectory = false;
            next_state = idle;
      }
        state = next_state;
        pub_rate.sleep();
    }
    ros::spin();
    return 0;
}



template<class PlannerType>
piecewise_trajectory OMPL_plan(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
#ifndef INFLATE

    piecewise_trajectory result;

    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // Set bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, x__low_bound__global);
    bounds.setHigh(0, x__high_bound__global);
    bounds.setLow(1, y__low_bound__global);
    bounds.setHigh(1, y__high_bound__global);
    bounds.setLow(2, z__low_bound__global);
    bounds.setHigh(2, z__high_bound__global);

    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    // Setup collision checker
    ob::SpaceInformationPtr si = ss.getSpaceInformation();
    si->setStateValidityChecker(OMPLStateValidityChecker);
    si->setMotionValidator(std::make_shared<OMPLMotionValidator>(si));
    si->setup();

    // Set planner
    //ob::PlannerPtr planner(new og::RRTstar(si));
    ob::PlannerPtr planner(new PlannerType(si));
    ss.setPlanner(planner);

    ob::ScopedState<> start_state(space);
    start_state[0] = start.x;
    start_state[1] = start.y;
    start_state[2] = start.z;

    ob::ScopedState<> goal_state(space);
    goal_state[0] = goal.x;
    goal_state[1] = goal.y;
    goal_state[2] = goal.z;

    ss.setStartAndGoalStates(start_state, goal_state);

    ss.setup();

    // Solve for path
    ob::PlannerStatus solved = ss.solve(g_planning_budget);

    if (solved)
    {
        //ROS_INFO("Solution found!");
        ss.simplifySolution();

        for (auto state : ss.getSolutionPath().getStates()) {
            const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

            double x = pos->values[0];
            double y = pos->values[1];
            double z = pos->values[2];

            result.push_back({x, y, z});
        }
    }
    else
        ROS_ERROR("Path not found!");

    return result;
#else
    ROS_ERROR("OMPL-based planners cannot be compiled together with inflation!");
#endif
}


piecewise_trajectory OMPL_RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    return OMPL_plan<ompl::geometric::RRT>(start, goal, width, length, n_pts_per_dir, octree);
}


piecewise_trajectory OMPL_RRTConnect(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    return OMPL_plan<ompl::geometric::RRTConnect>(start, goal, width, length, n_pts_per_dir, octree);
}


piecewise_trajectory OMPL_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    return OMPL_plan<ompl::geometric::PRM>(start, goal, width, length, n_pts_per_dir, octree);
}

piecewise_trajectory OMPL_RRTstar(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    return OMPL_plan<ompl::geometric::RRTstar>(start, goal, width, length, n_pts_per_dir, octree);
}