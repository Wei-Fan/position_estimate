#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml.hpp>
#include "ardrone_autonomy/navdata_altitude.h"
#include "ardrone_autonomy/Navdata.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Empty.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "position_estimate/points.h"
#include "point_feature.h"

using namespace cv;
using namespace std;

#define MEASURE_POS_PATH "/home/wade/catkin_ws/src/position_estimate/test_file/measure_position.csv"
#define FEATURE_VEC_PATH "/home/wade/catkin_ws/src/position_estimate/test_file/feature_vectors.csv"
#define POINT_NUM 7

class Pos_Estimate
{
public:
	Pos_Estimate();
private:
	ros::NodeHandle node;
	ros::Subscriber green_sub;
	ros::Subscriber red_sub;
	ros::Subscriber pos_sub;
	ros::Subscriber correct_sub;
	ros::Publisher pos_pub;
	ros::Publisher renew_pub;

	void greenCallback(const geometry_msgs::Point &msg);
	void redCallback(const position_estimate::points &msg);
	void posCallback(const ardrone_autonomy::Navdata &msgs);
	void correctCallback(const geometry_msgs::Point &msgs);
	bool read_csv(char *filepath, Mat &image);

	Mat measured_points = Mat(Size(2, POINT_NUM), CV_32FC1);
	Mat feature_vectors = Mat(Size(30,POINT_NUM), CV_32FC1);
	bool isGreenFound;
	bool isRenew;
	int current_point;
	geometry_msgs::Point current_pos;
	float delt;
	float beta = 0.1;
	geometry_msgs::Point preset_pos;
};

Pos_Estimate::Pos_Estimate()
{
	green_sub = node.subscribe("green_point", 1, &Pos_Estimate::greenCallback, this);
	red_sub = node.subscribe("red_real_points", 1, &Pos_Estimate::redCallback, this);
	pos_sub = node.subscribe("/ardrone/navdata", 1, &Pos_Estimate::posCallback, this);
	correct_sub = node.subscribe("delt", 1, &Pos_Estimate::correctCallback, this);
	pos_pub = node.advertise<geometry_msgs::Point>("ardrone_position", 1000);
	renew_pub = node.advertise<std_msgs::Empty>("is_renew", 1000);
	read_csv(MEASURE_POS_PATH, measured_points);
	read_csv(FEATURE_VEC_PATH, feature_vectors);
	isRenew = false;
}

void Pos_Estimate::posCallback(const ardrone_autonomy::Navdata &msg)
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

	if (msg.vx*msg.vx+msg.vy*msg.vy <= 0.01)
	{
		isRenew = true;
		ros::spin();
	}

	current_pos.x += msg.vx * dt/1000.0;
	current_pos.y += msg.vy * dt/1000.0;
	pos_pub.publish(current_pos);
}

void Pos_Estimate::correctCallback(const geometry_msgs::Point &msg)
{
	current_pos.x = current_pos.x - beta * (current_pos.x - preset_pos.x - msg.x);
	current_pos.y = current_pos.y - beta * (current_pos.y - preset_pos.y - msg.y);
}

void Pos_Estimate::greenCallback(const geometry_msgs::Point &msg)
{
	isGreenFound = 1;
}

void Pos_Estimate::redCallback(const position_estimate::points &msg)
{
	if (isRenew == false)
		ros::spin();
	/*get the feature vector according to image*/
	vector<float> x;
	vector<float> y;
	vector<float> red_feature;
	for (int i = 0; i < msg.point.size(); ++i)
	{
		x.push_back(msg.point[i].x);
		y.push_back(msg.point[i].y);
	}
	//cout << "-----------------------------------\n";
	//cout << Mat(x) << endl << Mat(y) << endl;
	p_feature_extraction(x, y, 30, red_feature);

	/*a straight line or not*/
	bool line_flag = 0;
	vector<int> c;
	for (int i = 0; i < red_feature.size(); ++i)
	{
		if (red_feature[i] != 0)
			c.push_back(i);
	}
	if (c.size() == 2 && c.back()-c.front() == 15)
	{
		line_flag = 1;
		ros::spin();
	}

	/*convert feature vectors to 2D vector*/
	vector<vector<float> > feature_vec(POINT_NUM);
	for (int i = 0; i < feature_vectors.rows; ++i)
	{
		for (int j = 0; j < feature_vectors.cols; ++j)
		{
			feature_vec[i].push_back(feature_vectors.at<float>(i,j));
		}
	}

	/*get the current point number by comparing feature vector distances*/
	vector<float> distant;
	vector<int> distant_num;
	float angle;
	for (int i = 0; i < feature_vec.size(); ++i)
	{
		distant.push_back(p_feature_sdistance(feature_vec[i], red_feature, 30, angle));
		distant_num.push_back(i);
	}
	current_pos.z = angle;

	/*find the most 2 possible points*/
	for (int i = 0; i < POINT_NUM; ++i)
	{
		for (int j = 0; j < POINT_NUM-i-1; ++j)
		{
			if (distant[j] > distant[j+1])
			{
				float tmp = distant[j];
				int tmp_num = distant_num[j];
				distant[j] = distant[j+1];
				distant_num[j] = distant_num[j+1];
				distant[j+1] = tmp;
				distant_num[j+1] = tmp_num;
			}
		}
	}
	/*condition for using feature match*/
	cout << "--------------------------------------\n";
	if (fabs(distant[0]-distant[1]) <= 0.4)
		ros::spin();
	else {
		//cout << msg.point[0].x << '\t' << msg.point[0].y << endl;
		cout << "current point : " << distant_num[0] << '\t' << distant_num[1] << endl;
		cout << "distant : " << distant[0] << '\t' << distant[1] << endl;
		
		/*publish current position*/
		current_pos.x = -msg.point[0].x + measured_points.at<float>(distant_num[0],0);
		current_pos.y = -msg.point[0].y + measured_points.at<float>(distant_num[0],1);
		cout << current_pos.x << '\t' << current_pos.y << endl;
		pos_pub.publish(current_pos);
	}
	isRenew = false;
	std_msgs::Empty e;
	renew_pub.publish(e);
}

bool Pos_Estimate::read_csv(char *filepath, Mat &image)  
{   
    string pixel;  
    ifstream file(filepath, ifstream::in);  
    if (!file) 
    {
    	cout << "CSV read fail" << endl;
    	return false;
	}  

    int nc = image.cols*image.rows;
    int eolElem = image.cols - 1;
    int elemCount = 0;  

	for (int j = 0; j < nc; j++)  
    {    
        if(elemCount == eolElem){  
            getline(file,pixel,'\n');
            image.at<float>((int)(j/image.cols), elemCount) = (float)atof(pixel.c_str());
            //cout << (int)(j/image.cols) << '\t' << elemCount << '\t' << image.at<float>((int)(j/image.cols), elemCount) << endl;
            elemCount = 0;
        } else {  
            getline(file,pixel,',');
            image.at<float>((int)(j/image.cols), elemCount) = (float)atof(pixel.c_str());
      	    //cout << (int)(j/image.cols) << '\t' << elemCount << '\t' << image.at<float>((int)(j/image.cols), elemCount) << endl;
            elemCount++;  
        }  
	}
    return true;  
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pos_estimate");
	Pos_Estimate pe;
	ros::spin();
	return 0;
}