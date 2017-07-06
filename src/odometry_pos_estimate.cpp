#include <ros/ros.h>
#include "ardrone_autonomy/Navdata.h"
#include <geometry_msgs/Point.h>

using namespace std;

class OdometryPosEstimate
{
public:
	OdometryPosEstimate();
private:
	ros::NodeHandle node;
	ros::Subscriber odometry_sub;
	ros::Publisher pos_pub;

	void odometryCallback(const ardrone_autonomy::Navdata &msgs);

	geometry_msgs::Point current_pos;
	
};

OdometryPosEstimate::OdometryPosEstimate()
{
	odometry_sub = node.subscribe("/ardrone/navdata", 1, &OdometryPosEstimate::odometryCallback, this);
	pos_pub = node.advertise<geometry_msgs::Point>("ardrone_position", 1000);
}

void OdometryPosEstimate::odometryCallback(const ardrone_autonomy::Navdata &msg)
{
	static bool start = true;
	static float last_time = 0;
	if (start)
	{
		start = false;
		last_time = msg.tm;
		current_pos.x = 0;
		current_pos.y = 0;
	}
	float dt = (msg.tm - last_time)/1000000.0;
	last_time = msg.tm;
	current_pos.x += msg.vx * dt/1000.0;
	current_pos.y += msg.vy * dt/1000.0;
	pos_pub.publish(current_pos);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_pos_estimate");
	OdometryPosEstimate o;
	ros::spin();
	return 0;
}