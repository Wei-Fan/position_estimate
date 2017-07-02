#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ardrone_autonomy/navdata_altitude.h"
#include "ardrone_autonomy/Navdata.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "position_estimate/points.h"
#include "point_feature.h"

using namespace cv;
using namespace std;

class Pos_Estimate
{
public:
	Pos_Estimate();
private:
	ros::NodeHandle node;
	ros::Subscriber green_sub;
	ros::Subscriber red_sub;
	ros::Publisher pos_pub;

	void greenCallback(const geometry_msgs::Point &msg);
	void redCallback(const position_estimate::points &msg);
};

Pos_Estimate::Pos_Estimate()
{
	green_sub = node.subscribe("green_point", 1, &Pos_Estimate::greenCallback, this);
	red_sub = node.subscribe("red_real_points", 1, &Pos_Estimate::redCallback, this);
}

void Pos_Estimate::greenCallback(const geometry_msgs::Point &msg)
{

}

void Pos_Estimate::redCallback(const position_estimate::points &msg)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pos_estimate");
	Pos_Estimate pe;
	ros::spin();
	return 0;
}