#include "narrow_path.h"


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "narrow_path_node");
	NarrowPath narrowPath;
	ros::spin();
	return 0;
	
}
