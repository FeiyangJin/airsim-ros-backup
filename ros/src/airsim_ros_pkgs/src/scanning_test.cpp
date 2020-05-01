#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	mav_trajectory_generation::Vertex::Vector vertices;
	const int dimension = 3;
	const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
	mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

	start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
	vertices.push_back(start);

	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,2,3));
	vertices.push_back(middle);

	end.makeStartOrEnd(Eigen::Vector3d(2,1,5), derivative_to_optimize);
	vertices.push_back(end);

	std::vector<double> segment_times;
	const double v_max = 2.0;
	const double a_max = 2.0;
	segment_times = estimateSegmentTimes(vertices, v_max, a_max);

	const int N = 10;
	mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
	opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
	opt.solveLinear();


	// optain the polynomial segments
	mav_trajectory_generation::Segment::Vector segments;
	opt.getSegments(&segments);

	cout << "--------------------------------" << endl;
	cout << "--------------------------------" << endl;
	cout << endl;

	// get trajectory from optimization result
	mav_trajectory_generation::Trajectory trajectory;
	opt.getTrajectory(&trajectory);

	mav_msgs::EigenTrajectoryPoint::Vector statesV;
	    
	double sampling_interval = 0.1;
	//mav_trajectory_generation::sampleWholeTrajectory(scanning_trajectory, sampling_interval, &states);
	auto result = mav_trajectory_generation::sampleTrajectoryInRange(trajectory,trajectory.getMinTime(),
	    trajectory.getMaxTime(), sampling_interval, &statesV);

	cout << "size of states after sampling: " << statesV.size() << endl;


  return 0;
}

