#include "narrow_path.h"

using namespace std;

void NarrowPath::initSetup() {
	approach_flag_ = false;
	start_flag_ = true;
	finish_flag_ = false;
	steer_angle_ = 0.0;
	
	setPoint(1, 0, 0);

	start_signal_ = 0;

	start_distance_ = 1;
	end_distance_ = 1;

	ackermann_msgs::AckermannDriveStamped ackerData_;
	sub1_ = nh_.subscribe("narrow_path_raw_obstacles", 1, &NarrowPath::obstacleCallback,this);
	sub2_ = nh_.subscribe("narrow_path_approach_raw_obstacles", 1, &NarrowPath::approachCallback,this);
	sub3_ = nh_.subscribe("narrow_path_escape_raw_obstacles", 1, &NarrowPath::escapeCallback,this);
	ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ctrl_cmd", 10);
}

void NarrowPath::setPoint(float x, float y, float z) {
	wayPoint_.x = x;
	wayPoint_.y = y;
	wayPoint_.z = z;	
}

void NarrowPath::approachCallback(const obstacle_detector::Obstacles data) {

	if (start_flag_){
		ROS_INFO("approache_cb");
		narrowPathingApproach(data);
		int nearest_x_ = 100.0;

		for(auto segment_data : data.segments) {
			if (nearest_x_ > segment_data.first_point.x) {
				nearest_x_ = segment_data.first_point.x;
			}

			if (nearest_x_ > segment_data.last_point.x) {
				nearest_x_ = segment_data.last_point.x;
			}
		}

		if (nearest_x_ < start_distance_) {
			start_flag_ = false;
			approach_flag_ = true;
		}
	}
}

void NarrowPath::obstacleCallback(const obstacle_detector::Obstacles data) {
	if (approach_flag_) {
		ROS_INFO("obstacleCallback called");

		narrowPathing(data);

		int farthest_x = 100;
		
		for (auto segment_data : data.segments) {
			if (farthest_x < segment_data.first_point.x) {
				farthest_x = segment_data.first_point.x;
			}

			if (farthest_x <  segment_data.last_point.x) {
				farthest_x = segment_data.last_point.x;
			}
		}

		if (farthest_x < end_distance_) {
			finish_flag_ = true;
		}
		
	}
}


void NarrowPath::escapeCallback(const obstacle_detector::Obstacles obs) {
	geometry_msgs::Point finish_point = farthestPoint(obs);


	if (finish_flag_) {
		if (fabs(calcDistance(wayPoint_, finish_point)) < 0.7) {
			ackerData_.drive.steering_angle = -10;
			ackerData_.drive.speed = 3;
			approach_flag_ = false;
			ackermann_pub_.publish(ackerData_);
			ROS_INFO("##############################");
			ros::shutdown();
		}
		
	}
}

void NarrowPath::narrowPathing(const obstacle_detector::Obstacles data){
	ackerData_.drive.speed = 3; // throttle
	
	//##################### check if segments is empty or not############
	int size_segments = sizeof(data.segments) / sizeof(data.segments[0]);
	
	if (size_segments >= 2){
		int x_center = 0;
		int y_center = 0;

		for (auto segment_data : data.segments){
			x_center = x_center + segment_data.first_point.x;
			x_center = x_center + segment_data.last_point.x;
			y_center = y_center + segment_data.first_point.y;
			y_center = y_center + segment_data.last_point.y;
		}

		x_center = x_center / size_segments;
		y_center = y_center / size_segments;
		
		setPoint(x_center, y_center, 0);
		start_signal_ = 1;
		steer_angle_ = atan((wayPoint_.y / wayPoint_.x));
		ackerData_.drive.steering_angle = int((steer_angle_ / M_PI) * 104);
	} else {
		ackerData_.drive.steering_angle = int((steer_angle_ / M_PI)* 104);
	}

	if (ackerData_.drive.steering_angle > 26) {
		ackerData_.drive.steering_angle = 26;
	}
	else if (ackerData_.drive.steering_angle < -26) {
		ackerData_.drive.steering_angle = -26;
	}

	if (!finish_flag_) {
		ackermann_pub_.publish(ackerData_);
	}
}

void NarrowPath::narrowPathingApproach(const obstacle_detector::Obstacles data){
			
	double x_center = 0;	
	double y_center = 0;
	double nearest_x = 50.0;
	double nearest_y_1 = -100.0;
	double nearest_y_2 = 100.0;

	for(auto segment_data : data.segments){

		if (nearest_x > segment_data.first_point.x) {
			nearest_x = segment_data.first_point.x;
		}
	
		if (nearest_x > segment_data.last_point.x) {
			nearest_x = segment_data.last_point.x;
		}

		if (segment_data.first_point.y < 0) {
			if (nearest_y_1 < segment_data.first_point.y) {
				nearest_y_1 = segment_data.first_point.y;
			}
			if (nearest_y_1 < segment_data.last_point.y) {
				nearest_y_1 = segment_data.last_point.y;
			}
		}

		if (segment_data.first_point.y >= 0) {
			if (nearest_y_2 > segment_data.first_point.y) {
				nearest_y_2 = segment_data.first_point.y;
			}
			if (nearest_y_2 > segment_data.last_point.y) {
				nearest_y_2 = segment_data.last_point.y;
			}
		}
	}

	x_center = nearest_x;
	y_center = nearest_y_1 + nearest_y_2;

	x_center = x_center;
	y_center = y_center/2;
	setPoint(x_center, y_center, 0);
	
	start_signal_ = 1;
	ackerData_.drive.speed = 3;	
	steer_angle_ = atan(wayPoint_.y/wayPoint_.x);

	ackerData_.drive.steering_angle = int((-steer_angle_/M_PI) * 104);

	if (ackerData_.drive.steering_angle > 26) {
		ackerData_.drive.steering_angle = 26;
	}
	else if (ackerData_.drive.steering_angle < -26)	{
		ackerData_.drive.steering_angle = -26;
	}

	if (!finish_flag_) {
		ackermann_pub_.publish(ackerData_);
	}

			
}

geometry_msgs::Point NarrowPath::farthestPoint(const obstacle_detector::Obstacles data) {
	double x_center = 0;
	double y_center = 0;
	double far_x = 0;
	double far_y_1 = 0;
	double far_y_2 = 0;

	for (auto segment_data : data.segments) {
		if (far_x < segment_data.first_point.x){
			far_x = segment_data.first_point.x;
	}
		if (far_x < segment_data.last_point.x) {
			far_x = segment_data.last_point.x;
	}

		if (segment_data.first_point.y < 0) {
			if (far_y_1 > segment_data.first_point.y) {
				far_y_1 = segment_data.first_point.y;
			}
			if (far_y_1 > segment_data.last_point.y) {
				far_y_1 = segment_data.last_point.y;
			}	
	}

		if (segment_data.first_point.y >= 0) {
			if (far_y_2 < segment_data.first_point.y) {
				far_y_2 = segment_data.first_point.y;
			}
			if (far_y_2 < segment_data.last_point.y) {
				far_y_2 = segment_data.last_point.y;
			}
		}
	}

	x_center = far_x;
	y_center = far_y_1 + far_y_2;
	
	x_center = x_center;
	y_center = y_center/2;
	setPoint(x_center, y_center ,0);

	return wayPoint_;
	
}

float NarrowPath::calcDistance(geometry_msgs::Point point1, geometry_msgs::Point point2) {
	return pow((point1.x- point2.x), 2) + pow((point1.y - point2.y), 2);
}



