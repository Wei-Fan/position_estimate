#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml.hpp>
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

#define MEASURE_POS_PATH "/home/wade/catkin_ws/src/position_estimate/test_file/measure_position.csv"
#define FEATURE_VEC_PATH "/home/wade/catkin_ws/src/position_estimate/test_file/feature_vectors.csv"
#define POINT_NUM 10

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
	bool read_csv(char *filepath, Mat &image);

	Mat measured_points = Mat(Size(POINT_NUM,2), CV_32FC1);
	Mat feature_vectors = Mat(Size(POINT_NUM,30), CV_32FC1);
	bool isGreenFound;
	int current_point;
	geometry_msgs::Point current_pos;
};

Pos_Estimate::Pos_Estimate()
{
	current_point = 0;
	green_sub = node.subscribe("green_point", 1, &Pos_Estimate::greenCallback, this);
	red_sub = node.subscribe("red_real_points", 1, &Pos_Estimate::redCallback, this);
	pos_pub = node.advertise<geometry_msgs::Point>("ardrone_position", 1000);
	read_csv(MEASURE_POS_PATH, measured_points);
	read_csv(FEATURE_VEC_PATH, feature_vectors);
}

void Pos_Estimate::greenCallback(const geometry_msgs::Point &msg)
{
	isGreenFound = 1;
}

void Pos_Estimate::redCallback(const position_estimate::points &msg)
{
	/*get the feature vector according to image*/
	vector<float> x;
	vector<float> y;
	vector<float> red_feature;
	for (int i = 0; i < msg.point.size(); ++i)
	{
		x.push_back(msg.point[i].x);
		y.push_back(msg.point[i].y);
	}
	//cout << Mat(x) << endl << Mat(y) << endl;
	p_feature_extraction(x, y, 30, red_feature);

	/*convert Mat to 2D vector*/
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
	float angle;
	for (int i = 0; i < feature_vec.size(); ++i)
		distant.push_back(p_feature_sdistance(feature_vec[i], red_feature, 60, angle));
	current_pos.z = angle;
	float min = distant[0];
	current_point = 0;
	for (int i = 1; i < distant.size(); ++i)
	{
		if (distant[i] < min)
			current_point = i;
	}

	current_pos.x = msg.point[current_point].x;
	current_pos.y = msg.point[current_point].y;
	pos_pub.publish(current_pos);
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
      
    int nc;
    int eolElem = image.cols - 1;//每行最后一个元素的下标  
    int elemCount = 0;  
    if (image.isContinuous())  
    {     
        nc= image.cols*image.rows;// then no padded pixels     
        image.rows= 1;// it is now a 1D array     
    }    
    for (int i = 0; i<image.rows; i++)  
    {  
        float* data = (float*)image.ptr<ushort>(i);    
        for (int j = 0; j < nc; j++)  
        {    
            if(elemCount == eolElem){  
                getline(file,pixel,'\n');//任意地读入，直到读到delim字符 '\n',delim字符不会被放入buffer中  
                data[j] = (float)atof(pixel.c_str());//将字符串str转换成一个双精度数值并返回结果  
                elemCount = 0;//计数器置零  
            }  
            else{  
                getline(file,pixel,',');//任意地读入，直到读到delim字符 ','delim字符不会被放入buffer中  
                data[j] = (float)atof(pixel.c_str());//将字符串str转换成一个双精度数值并返回结果  
                elemCount++;  
            }  
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