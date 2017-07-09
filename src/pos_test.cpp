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

#define MEASURE_POS "/home/wade/catkin_ws/src/position_estimate/test_file/measure_position.csv"
#define FEATURE_VEC "/home/wade/catkin_ws/src/position_estimate/test_file/feature_vectors.csv"
#define FIELD_MEASURE "/home/wade/catkin_ws/src/position_estimate/test_file/field_measure.csv"

class Test
{
public:
	Test();
private:
	ros::NodeHandle node;
	ros::Subscriber green_sub;
	ros::Subscriber red_sub;
	ros::Publisher test_red_pub;
	ros::Publisher pos_pub;

	void greenCallback(const geometry_msgs::Point &msg);
	void redCallback(const position_estimate::points &msg);

	void pos_estimate_test();
	void find_circle_test();
	void get_feature_vector();
	void get_measure_point();

	bool save_csv(const vector<vector<float> > &v, char *filename);
	bool read_csv(char *filepath, Mat &image);
};

Test::Test()
{
	//get_feature_vector();
	get_measure_point();
	green_sub = node.subscribe("green_point", 1, &Test::greenCallback, this);
	red_sub = node.subscribe("red_real_points", 1, &Test::redCallback, this);
	test_red_pub = node.advertise<position_estimate::points>("red_real_points", 1000);
	//pos_estimate_test();
}

void Test::pos_estimate_test()
{
	ros::Rate loop_rate(10);
	int count = 0;
	while(ros::ok())
	{	
		position_estimate::points test_msg;
		geometry_msgs::Point p[5];
		for (int i = 0; i < 5; ++i)
		{
			p[i].x = 0.1*i;
			p[i].y = 0.2*i;
			test_msg.point.push_back(p[i]);
		}
		test_red_pub.publish(test_msg);
		cout << "I have published!\t" << count << endl;

		loop_rate.sleep();
		count++;
	}
}

void Test::find_circle_test()
{

}

void Test::get_measure_point()
{
	Mat xy = Mat(Size(2,69), CV_32FC1);
	read_csv(FIELD_MEASURE, xy);
	//cout << format(xy, Formatter::FMT_CSV) << endl;
	vector<vector<float> > field_vectors;
	field_vectors.resize(69);
	for (int i = 0; i < 69; ++i)
		field_vectors[i].resize(2);
	for (int i = 0; i < xy.rows; ++i)
	 {
	 	float a = xy.at<Vec2f>(i,0)[0];
	 	//cout << a << endl;
	 	float b = xy.at<Vec2f>(i,0)[1];
	 	field_vectors[i][0] = (a*a + 1.76*1.76 - b*b) / (2*1.76);
	 	field_vectors[i][1] = b / fabs(b) * sqrt(a*a - field_vectors[i][0]*field_vectors[i][0]);
	 } 
	 save_csv(field_vectors, MEASURE_POS);
	 cout << "save!\n";
}

void Test::get_feature_vector()
{
	vector<vector<float> > feature_vectors;
	vector<float> x;
	vector<float> y;
	float threshold = 0.45;//determined by image size and height

	/*measure and input points' coordinate according to points in fleid*/
	Mat xy = Mat(Size(2,7), CV_32FC1);
	read_csv(MEASURE_POS, xy);
	cout << format(xy, Formatter::FMT_CSV) << endl;
	for (int i = 0; i < xy.rows; ++i)
	{
		x.push_back(xy.at<Vec2f>(i,0)[0]);
		y.push_back(xy.at<Vec2f>(i,0)[1]);
	}
	//cout << Mat(x) << "\n" << Mat(y) << endl;
    /*get the feature vectors*/
    p_feature_calculate(x, y, threshold, 30, feature_vectors);
   	save_csv(feature_vectors, FEATURE_VEC);
} 

bool Test::save_csv(const vector<vector<float> > &v, char *filename)
{
	Mat fv = Mat(v.size(), v[0].size(), CV_32FC1);
	for (int i = 0; i < v.size(); ++i)
	{
		for (int j = 0; j < v[0].size(); ++j)
		{
			fv.at<float>(i,j) = v[i][j];
		}
	}

    ofstream file(filename);
	file << format(fv, Formatter::FMT_CSV);
	file.close();
	return true;
	//cout << "I have saved!\n";
}

bool Test::read_csv(char *filepath, Mat &image)  
{   
    string pixel;  
    ifstream file(filepath, ifstream::in);  
    if (!file) 
    {
    	cout << "CSV read fail" << endl;
    	return false;
	}  
      
    int nc = image.cols*image.rows;
    int eolElem = image.cols - 1;//每行最后一个元素的下标  
    int elemCount = 0;  
    /*if (image.isContinuous())  
    {     
        nc= image.cols*image.rows;// then no padded pixels     
        image.rows= 1;// it is now a 1D array     
    }*/
	for (int j = 0; j < nc; j++)  
    {    
        if(elemCount == eolElem){  
            getline(file,pixel,'\n');//任意地读入，直到读到delim字符 '\n',delim字符不会被放入buffer中  
            image.at<float>((int)(j/image.cols), elemCount) = (float)atof(pixel.c_str());
            //cout << (int)(j/image.cols) << '\t' << elemCount << '\t' << image.at<float>((int)(j/image.cols), elemCount) << endl;
            elemCount = 0;//计数器置零  
        } else {  
            getline(file,pixel,',');//任意地读入，直到读到delim字符 ','delim字符不会被放入buffer中  
            image.at<float>((int)(j/image.cols), elemCount) = (float)atof(pixel.c_str());
      	    //cout << (int)(j/image.cols) << '\t' << elemCount << '\t' << image.at<float>((int)(j/image.cols), elemCount) << endl;
            elemCount++;  
        }  
	}
    return true;  
}

void Test::greenCallback(const geometry_msgs::Point &msg)
{

}

void Test::redCallback(const position_estimate::points &msg)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pos_test");
	Test t;
	ros::spin();
	return 0;
}