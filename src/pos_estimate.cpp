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
#include "std_msgs/Bool.h"
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
#define POINT_NUM 9

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
	ros::Publisher yellow_pub;
	ros::Publisher blue_pub;

	void blueCallback(const geometry_msgs::Point &msg);
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
	int next_feature_index;
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
	//correct_sub = node.subscribe("/delt", 1, &Pos_Estimate::correctCallback, this);
	index_sub = node.subscribe("/index", 1, &Pos_Estimate::indexCallback, this);
	pos_pub = node.advertise<geometry_msgs::Point>("/ardrone_position", 2);
	renew_pub = node.advertise<position_estimate::renew>("/is_renew", 2);
	yellow_pub = node.advertise<std_msgs::Bool>("/is_yellow", 2);
	read_csv(RENEW_POS_PATH, renew_points);
	read_csv(FEATURE_VEC_PATH, feature_vectors);
	read_csv(PRESET_POS_PATH, preset_position);
	//current_index = 0;
	current_pos.x = 0;
	current_pos.y = 0;
	next_feature_index = 0;
	enableRenew = false;
	isRenew = true;
	isYellowFound = false;//false; //true;//test
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
	if (fabs(msg.tm-last_time) <= 1000000)
	{
		float dt = (msg.tm - last_time)/1000000.0;
		last_time = msg.tm;

		current_v.x = msg.vx;
		current_v.y = msg.vy;
		current_pos.x += msg.vx * dt/1000.0; //&&&&&
		current_pos.y += msg.vy * dt/1000.0;
	}
	pos_pub.publish(current_pos);//^^^
	//cout << "position = " << current_pos.x << '\t' << current_pos.y << endl;
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
		//float v = sqrt(current_v.x*current_v.x+current_v.y*current_v.y);
		if ((d >= 0.15) )//|| (v >= 0.4))
		{
			current_pos.x = -msg.x;
			current_pos.y = -msg.y;
			ROS_INFO("this yellow %f, %f", current_pos.x, current_pos.y);
		} else {
			isYellowFound = true;
			cout << "isYellowFound-----------------------------------\n\n";
			std_msgs::Bool b;
			b.data = true;
			yellow_pub.publish(b);
		}
	}
}

void Pos_Estimate::redCallback(const position_estimate::points &msg)
{
	//cout << "redCallback start!\n";
	if (isYellowFound == true)
	{
		vector<vector<float> > feature_vec(POINT_NUM);
		for (int i = 0; i < feature_vectors.rows; ++i)
		{
			for (int j = 0; j < feature_vectors.cols; ++j)
			{
				feature_vec[i].push_back(feature_vectors.at<float>(i,j));
			}
		}

		//cout << "453!\n";

		int p_index = msg.point.size();
		int f_index = feature_vec.size(); 
		int min_p, min2_p;
		float min_d = 100;
		float min2_d = 100;

		for (int i = 0; i < p_index; ++i)
		{
			//point list
			vector<float> p_vec_x;
			vector<float> p_vec_y;
			p_vec_x.push_back(msg.point[i].x);
			p_vec_y.push_back(msg.point[i].y);
			for (int k = 0; k < p_index; ++k)
			{
				float dx = msg.point[i].x-msg.point[k].x;
				float dy = msg.point[i].y-msg.point[k].y;
				float d = sqrt(dx*dx + dy*dy);
				if (d <= 0.3 && k!=i)
				{
					p_vec_x.push_back(msg.point[k].x);
					p_vec_y.push_back(msg.point[k].y);
				}
			}

			//cout << "66!\n";
			if (p_vec_x.size() >= 3)
			{
				vector<float> red_feature;
				p_feature_extraction(p_vec_x, p_vec_y, 30, red_feature);
				float angle;
				float f_d = p_feature_sdistance(feature_vec[next_feature_index], red_feature, 30, angle);
				float f_dx = (renew_points.at<float>(next_feature_index,0) - current_pos.x);
				float f_dy = (renew_points.at<float>(next_feature_index,1) - current_pos.y); 
				//f_d += 0.2*sqrt(f_dx*f_dx + f_dy*f_dy);
				//cout << "No." << i << ',' << j << endl;
				//cout << "distant = " << f_d << endl;
				if (f_d < min_d)
				{
					min2_d = min_d;
					min_d = f_d;
					min2_p = min_p;
					min_p = i;
				} else if (f_d < min2_d)
				{
					min2_d = f_d;
					min2_p = i;
				}
			}
		}

		/*condition for using feature match*/

		if(min_d < 100.0)
		{
			//cout << "match raw position : " << renew_points.at<float>(min_p_f,0) << '\t' <<renew_points.at<float>(min2_p_f,1) << endl;
			//cout << "--------------------------------------\n";
			if (min_d/min2_d > 0.4 || min_d > 1.2)//(fabs(distant[0]-distant[1]) <= 0.4)//wait for trying  //^^^^
			{	
				position_estimate::renew r;
				r.isRenew = false;
				renew_pub.publish(r);
			} else {
				/*publish current position*/
				current_pos.x = -msg.point[min_p].x + renew_points.at<float>(next_feature_index,0);
				current_pos.y = -msg.point[min_p].y + renew_points.at<float>(next_feature_index,1);
				//cout << "*************************\nmatch position : " << current_pos.x << '\t' << current_pos.y << endl;
				cout << "*********************************\n";
				cout << "*******feature distant : " << min_d << '\t' << min2_d << endl;
				cout << "*******next feature index = " << next_feature_index << endl;

				position_estimate::renew r;
				isRenew = true;
				r.isRenew = isRenew;
				r.index = next_feature_index;
				
				if (next_feature_index < POINT_NUM)
				{
					next_feature_index++;
				}
				cout << "^^^^^^^^^match feature index = " << next_feature_index << endl;
				renew_pub.publish(r);
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