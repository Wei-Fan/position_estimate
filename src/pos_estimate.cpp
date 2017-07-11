#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml.hpp>
#include "ardrone_autonomy/navdata_altitude.h"
#include "ardrone_autonomy/Navdata.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Int8.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "position_estimate/points.h"
#include "position_estimate/renew.h"
#include "point_feature.h"

using namespace cv;
using namespace std;

#define PRESET_POS_PATH "/home/wade/catkin_ws/src/position_estimate/test_file/setpoint.csv"
#define RENEW_POS_PATH "/home/wade/catkin_ws/src/position_estimate/test_file/preset_renew_position.csv"
#define FEATURE_VEC_PATH "/home/wade/catkin_ws/src/position_estimate/test_file/preset_feature.csv"
#define POINT_NUM 3

class Pos_Estimate
{
public:
	Pos_Estimate();
private:
	ros::NodeHandle node;
	ros::Subscriber yellow_sub;
	ros::Subscriber red_sub;
	ros::Subscriber pos_sub;
	ros::Subscriber correct_sub;
	ros::Subscriber index_sub;
	ros::Publisher pos_pub;
	ros::Publisher renew_pub;

	void yellowCallback(const geometry_msgs::Point &msg);
	void redCallback(const position_estimate::points &msg);
	void posCallback(const ardrone_autonomy::Navdata &msgs);
	void correctCallback(const geometry_msgs::Point &msgs);
	void indexCallback(const std_msgs::Int8 &msg);
	bool read_csv(char *filepath, Mat &image);

	Mat renew_points = Mat(Size(2,POINT_NUM), CV_32FC1);
	Mat feature_vectors = Mat(Size(30,POINT_NUM), CV_32FC1);
	Mat preset_position = Mat(Size(4,292), CV_32FC1);

	bool enableRenew;
	bool isRenew;
	bool isYellowFound;
	int current_index;
	geometry_msgs::Point current_pos;
	geometry_msgs::Point current_v;
	geometry_msgs::Point preset_pos;
	float delt;
	float beta = 0.05;
};

Pos_Estimate::Pos_Estimate()
{
	yellow_sub = node.subscribe("/yellow_point", 1, &Pos_Estimate::yellowCallback, this);
	red_sub = node.subscribe("/red_real_points", 1, &Pos_Estimate::redCallback, this);
	pos_sub = node.subscribe("/ardrone/navdata", 1, &Pos_Estimate::posCallback, this);
	correct_sub = node.subscribe("/delt", 1, &Pos_Estimate::correctCallback, this);
	index_sub = node.subscribe("/index", 1, &Pos_Estimate::indexCallback, this);
	pos_pub = node.advertise<geometry_msgs::Point>("/ardrone_position", 1000);
	renew_pub = node.advertise<position_estimate::renew>("/is_renew", 1000);
	read_csv(RENEW_POS_PATH, renew_points);
	read_csv(FEATURE_VEC_PATH, feature_vectors);
	read_csv(PRESET_POS_PATH, preset_position);
	current_index = 0;
	enableRenew = false;
	isRenew = true;
	isYellowFound = false;
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

	current_v.x = msg.vx;
	current_v.y = msg.vy;
	current_pos.x += msg.vx * dt/1000.0;
	current_pos.y += msg.vy * dt/1000.0;
	pos_pub.publish(current_pos);	//^^^
}

void Pos_Estimate::indexCallback(const std_msgs::Int8 &msg)
{
	current_index = msg.data;
}

void Pos_Estimate::correctCallback(const geometry_msgs::Point &msg)
{
	if (!isRenew)
	{
		preset_pos.x = preset_position.at<float>(current_index, 2);
		preset_pos.y = preset_position.at<float>(current_index, 3);
		current_pos.x = current_pos.x - beta * (current_pos.x - preset_pos.x - msg.x);
		current_pos.y = current_pos.y - beta * (current_pos.y - preset_pos.y - msg.y);	
	}
}

void Pos_Estimate::yellowCallback(const geometry_msgs::Point &msg)
{
	if (!isYellowFound)
	{
		float d = sqrt(msg.x*msg.x + msg.y*msg.y);
		float v = sqrt(current_v.x*current_v.x+current_v.y*current_v.y);
		if ((d >= 0.2) )//|| (v >= 0.4))
		{
			current_pos.x = -msg.x;
			current_pos.y = -msg.y;
			ROS_INFO("this yellow %f, %f", current_pos.x, current_pos.y);
		} else {
			isYellowFound = true;
			cout << "isYellowFound-----------------------------------\n\n";
			position_estimate::renew r;
			isRenew = false;
			r.isRenew = isRenew;
			renew_pub.publish(r);
		}
	}
}

void Pos_Estimate::redCallback(const position_estimate::points &msg)
{
	if (isYellowFound == true)
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
			position_estimate::renew r;
			r.isRenew = false;
			renew_pub.publish(r);
		} else {

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
				float d = p_feature_sdistance(feature_vec[i], red_feature, 30, angle);
				d += 0.2*(renew_points.at<float>(i,0) - current_pos.x);
				distant.push_back(d);
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
			//cout << "--------------------------------------\n";
			if (fabs(distant[0])/fabs(distant[1]) > 0.5 || distant[0] > 1.0)//(fabs(distant[0]-distant[1]) <= 0.4)//wait for trying  //^^^^
			{	
				position_estimate::renew r;
				r.isRenew = false;
				renew_pub.publish(r);
			} else {
				//cout << msg.point[0].x << '\t' << msg.point[0].y << endl;
				cout << "current point : " << distant_num[0] << '\t' << distant_num[1] << endl;
				cout << "distant : " << distant[0] << '\t' << distant[1] << endl;
				
				/*publish current position*/
				current_pos.x = -msg.point[0].x + renew_points.at<float>(distant_num[0],0);
				current_pos.y = -msg.point[0].y + renew_points.at<float>(distant_num[0],1);
				cout << "match position : " << current_pos.x << '\t' << current_pos.y << endl;
				
				float d = sqrt(msg.point[0].x*msg.point[0].x + msg.point[0].y*msg.point[0].y);
				float v = sqrt(current_v.x*current_v.x+current_v.y*current_v.y);
				if ((d >= 0.1) || (v >= 0.05))
				{
					position_estimate::renew r;
					isRenew = true;
					r.isRenew = isRenew;
					r.index = distant_num[0];
					renew_pub.publish(r);
				} else {
					position_estimate::renew r;
					r.isRenew = false;
					renew_pub.publish(r);
				}
			}
		}
	}
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