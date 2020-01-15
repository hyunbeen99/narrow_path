#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <cmath>


#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace std;

class NarrowPath {

private:

	// Value
	double steer_angle_;
	int start_distance_;
	int end_distance_;
	int start_signal_;

	// Flag
	bool approach_flag_;
	bool start_flag_;
	bool finish_flag_;

	// Node
	ros::NodeHandle nh_;
	ros::Publisher ackermann_pub_;
	ros::Subscriber sub_, sub1_, sub2_, sub3_;

	// Message
	geometry_msgs::Point wayPoint_;
	ackermann_msgs::AckermannDriveStamped ackerData_;


public:
	void initSetup();
	void setPoint(float x, float y, float z);

	void approachCallback(const obstacle_detector::Obstacles data);
	void obstacleCallback(const obstacle_detector::Obstacles data);
	void escapeCallback(const obstacle_detector::Obstacles obs);

	void narrowPathing(const obstacle_detector::Obstacles data);
	void narrowPathingApproach(const obstacle_detector::Obstacles data);
	
	float calcDistance(geometry_msgs::Point point1, geometry_msgs::Point point2);
	geometry_msgs::Point farthestPoint(const obstacle_detector::Obstacles data);

};
