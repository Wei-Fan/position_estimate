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
#define POINT_NUM 77
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
	get_feature_vector();
	//get_measure_point();
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
	Mat xy = Mat(Size(2,POINT_NUM), CV_32FC1);
	read_csv(FIELD_MEASURE, xy);
	//cout << format(xy, Formatter::FMT_CSV) << endl;
	vector<vector<float> > field_vectors;
	field_vectors.resize(POINT_NUM);
	for (int i = 0; i < POINT_NUM; ++i)
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
	Mat img = imread("/home/wade/catkin_ws/src/position_estimate/test_file/5.jpg");
	Mat imgThresholded = Mat(img.size(), CV_8UC1);

	vector<vector<float> > feature_vectors;
	vector<float> x;
	vector<float> y;

	for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			//if (tmp2 > 80 && tmp2 < 180 && tmp1 > 0 && tmp1 < 50 && tmp0 > 0 && tmp0 < 70)
			if((tmp2-tmp1) >= 40 && (tmp2-tmp0) >= 40)
			{
				imgThresholded.at<uchar>(i,j) = 255;
				continue;
			}
			imgThresholded.at<uchar>(i,j) = 0;
		}
	}

	//image operation
	Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);	
	erode(imgThresholded, imgThresholded, element);
	dilate(imgThresholded, imgThresholded, element);
	imgThresholded = 255 - imgThresholded;
	imshow("red", imgThresholded);
	if (char(waitKey(1)) == 'o')
		destroyWindow("red");
	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Rect> rb;
	position_estimate::points real_msg;
	position_estimate::points image_msg;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	
	//erase unqualified contours
	vector<vector<Point> >::iterator it = contours.begin();
	while(it != contours.end())
	{
		if (contourArea(*it, true) < 200)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));
			rb.push_back(r);
			rectangle(img, r, Scalar(0, 0, 255), 3);//testing
			++it;
		}
	}

	if (!rb.empty())
	{
		for (int i = 1; i < rb.size(); ++i)
		{
			double d0 = pow(0.5*(rb[0].tl().x+rb[0].br().x-img.cols),2)+pow(0.5*(rb[0].tl().y+rb[0].br().y-img.rows),2);
			double di = pow(0.5*(rb[i].tl().x+rb[i].br().x-img.cols),2)+pow(0.5*(rb[i].tl().y+rb[i].br().y-img.rows),2);
		
			if (d0 > di)
			{
				Rect tmp;
				tmp = rb[0];
				rb[0] = rb[i];
				rb[i] = tmp;
			}
		}
		rectangle(img, rb[0], Scalar(255, 0, 0), 3);
		for (int i = 0; i < rb.size(); ++i)
		{
			float h = 0.8;
			float img_data_x = 0.5*(rb[i].tl().x + rb[i].br().x) - 0.5*img.cols;
			float img_data_y = 0.5*(rb[i].tl().y + rb[i].br().y) - 0.5*img.rows;
			float real_data_x = (1.55 * h + 0.01078) / 1000 * img_data_x;
			float real_data_y = (1.55 * h + 0.01078) / 1000 * img_data_y;
			float result_x = -real_data_y;
			float result_y = -real_data_x;
			x.push_back(result_x);
			y.push_back(result_y);
		}
	}

	imshow("init_img", img);
	waitKey(0);

	vector<float> red_feature;
	p_feature_extraction(x, y, 30, red_feature);
   	
	ofstream outFile;
	outFile.open("/home/wade/catkin_ws/src/position_estimate/test_file/feature_vec_test.csv", ios::out);
	for (int i = 0; i < red_feature.size(); ++i)
	{
		if (i != red_feature.size()-1)
			outFile << red_feature[i] << ',';
		else
			outFile << red_feature[i] << endl;

	}
	outFile.close();
   	destroyWindow("init_img");
   	cout << "save!\n";
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
	cout << 'A' << endl;
	ros::spin();
	cout << 'B' << endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pos_test");
	Test t;
	ros::spin();
	return 0;
}