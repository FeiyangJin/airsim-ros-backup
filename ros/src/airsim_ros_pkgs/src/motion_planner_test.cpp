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

namespace ob = ompl::base;
namespace og = ompl::geometric;

octomap::OcTree * octree = nullptr;

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
      
      double height = 1; 
      double radius = 1; 

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

bool occupied(octomap::OcTree * octree, double x, double y, double z){
  const double OCC_THRESH = 0.5;
  octomap::OcTreeNode * otn = octree->search(x, y, z);
  cout << x << "  " << y << " " << z << "   ";
  if(otn != nullptr){
    cout << otn->getOccupancy() << endl;
  }
  else{
    cout << "null pointer" << endl;
  }

  // if(otn == nullptr){
  //   return true;
  // }
  // else if(otn->getOccupancy() >= OCC_THRESH){
  //   return true;
  // }
  // else{
  //   return false;
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


int main(int argc, char ** argv)
{
	  ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;

    //octomap and ompl testing
	  OcTree tree ("../launch/as.bt");  // create empty tree with resolution 0.1
		cout << "Octomap testing: " << tree.volume() << endl;
		octree = &tree;


		cout << "OMPL testing: " << endl;
		//auto space(std::make_shared<ob::SE3StateSpace>());
		auto space(std::make_shared<ob::RealVectorStateSpace>(3));
		ob::RealVectorBounds bounds(3);
		bounds.setLow(-500);
		bounds.setHigh(500);
		space->setBounds(bounds);

		og::SimpleSetup ss(space);
		// Setup collision checker
		ob::SpaceInformationPtr si = ss.getSpaceInformation();
		si->setStateValidityChecker(OMPLStateValidityChecker);
		si->setMotionValidator(std::make_shared<OMPLMotionValidator>(si));
		si->setup();
		

		ob::ScopedState<> start(space);
		start[0] = 20;
		start[1] = 30;
		start[2] = 5;

		ob::ScopedState<> goal(space);
		goal[0] = 10;
		goal[1] = 10;
		goal[2] = 10;

		ss.setStartAndGoalStates(start, goal);
		//ss.setPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
		ob::PlannerStatus solved = ss.solve(10.0);
		if (solved)
		{
		  std::cout << "Found solution:" << std::endl;
		  // print the path to screen
		  ss.simplifySolution();
		  ss.getSolutionPath().print(std::cout);
		}
	return 0;
}