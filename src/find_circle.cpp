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

using namespace cv;
using namespace std;

#define DATA_SIZE 600
#define R_DFNT 20
#define G_DFNT 10

class FindCircles
{
public:
	FindCircles();
	void measurement(double &x, double &y, double u, double v, double pitch, double roll, double h);
	void revmeasurement(double x, double y, double &u, double &v, double pitch, double roll, double h);
	void writeData();
private:
	ros::NodeHandle node;
	ros::Subscriber img_sub;
	ros::Subscriber altitude_sub;
	ros::Subscriber navdata_sub;
	ros::Publisher green_pub;
	ros::Publisher red_real_pub;
	ros::Publisher red_image_pub; 

	void imageCallback(const sensor_msgs::Image &msg);
	void altitudeCallback(const ardrone_autonomy::navdata_altitude &msg);
	void navdataCallback(const ardrone_autonomy::Navdata &msg);
	void image_process(Mat img);
	void find_green(Mat &img, Mat imgThresholded);
	void find_red(Mat &img, Mat imgThresholded);

	int data_count = 0;
	double height;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	double current_pos[2];
	cv::Point2d ardrone_pos;
	double distant[DATA_SIZE][2];
	double pose[DATA_SIZE][3];
};

FindCircles::FindCircles()
{
	img_sub = node.subscribe("/ardrone/image_raw", 1, &FindCircles::imageCallback, this);//"/ardrone/image_raw" "CamImage"
	altitude_sub = node.subscribe("/ardrone/navdata_altitude", 1, &FindCircles::altitudeCallback, this);
	navdata_sub = node.subscribe("/ardrone/navdata", 1, &FindCircles::navdataCallback, this);
	green_pub = node.advertise<geometry_msgs::Point>("green_point", 1000); //the message of centers is uncertain.
	red_real_pub = node.advertise<position_estimate::points>("red_real_points", 1000);
	red_image_pub = node.advertise<position_estimate::points>("red_image_points", 1000);
	//Mat src_img = imread("/home/wade/catkin_ws/src/find_circles/src/t.jpg", 1);//testing
	//image_process(src_img);//testing 
}

void FindCircles::writeData()
{
	/*save for evaluating the error of dsistant*/
	Mat distantm = Mat(DATA_SIZE, 2, CV_64FC1, distant);
	Mat posem = Mat(DATA_SIZE, 3, CV_64FC1, pose);
	ofstream f1("/home/wade/catkin_ws/src/find_circles/test_file/distant.csv");
	f1 << format(distantm, Formatter::FMT_CSV);
	ofstream f2("/home/wade/catkin_ws/src/find_circles/test_file/pose.csv");
	f2 << format(posem, Formatter::FMT_CSV);
	f1.close();
	f2.close();
}

void FindCircles::altitudeCallback(const ardrone_autonomy::navdata_altitude &msg)
{
	height = msg.altitude_vision/1000.0;
	current_pos[0] = -msg.altitude_vision/1000.0 * tan(roll);
	current_pos[1] = msg.altitude_vision/1000.0 * tan(pitch);
	//cout << "position: " << height << '\t' << current_pos[0] << '\t' << current_pos[1] << endl;
}

void FindCircles::navdataCallback(const ardrone_autonomy::Navdata &msg)
{
	roll = msg.rotX/180.0*3.1416;
	pitch = msg.rotY/180.0*3.1416;
	yaw = msg.rotZ/180.0*3.1416;
	//cout << "angles: " << roll << '\t' << pitch << '\t' << yaw << endl;
}

void FindCircles::imageCallback(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	image_process(cv_ptr->image);
}

void FindCircles::image_process(Mat img)
{
	ros::Rate loop_rate(10);
	/*evaluation*/
	/*if (data_count == DATA_SIZE)
	{
		writeData();
		exit(0);
	}*/
	Size image_size = img.size();

	/* to get the actual relative position*/
	measurement(ardrone_pos.x, ardrone_pos.y, current_pos[0], current_pos[1], pitch, roll, height);
	ardrone_pos.x = -ardrone_pos.x + image_size.width/2;
	ardrone_pos.y = -ardrone_pos.y + image_size.height/2 - 38;
	cout << "ardrone_pos in image: " << ardrone_pos.x << '\t' << ardrone_pos.y << endl;
	circle(img, ardrone_pos, 10, Scalar(0, 255, 255), 1, 8);

	/*color abstract*/
	Mat imgThresholded = Mat(image_size, CV_8UC1);
	Mat img_cp = img.clone();

	find_green(img, imgThresholded);
	find_red(img_cp, imgThresholded);
	//imshow("monitor0", img);//testing
	imshow("monitor1", img_cp);//testing
	/*if (char(waitKey(1)) == 'o')
		destroyWindow("monitor0");*/
	if (char(waitKey(1)) == 'o')
		destroyWindow("monitor1");
	
	loop_rate.sleep();
}

void FindCircles::find_green(Mat &img, Mat imgThresholded)
{
	for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			if ((tmp1-tmp0)>=G_DFNT && (tmp1-tmp2)>=G_DFNT)
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

	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	
	//erase unqualified contours
	vector<vector<Point> >::iterator it = contours.begin();
	while(it != contours.end())
	{
		if (contourArea(*it, true) < 500)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));//testing
			rectangle(img, r, Scalar(0, 255, 0), 3);//testing
			++it;
		}
	}
	
	if (!contours.empty())
	{
		Rect rb = boundingRect(Mat(contours[0]));
		geometry_msgs::Point midP;
		midP.x = 0.5*(rb.tl().x + rb.br().x);
		midP.y = 0.5*(rb.tl().y + rb.br().y);
		//cout << "circle center: " << midP.x << '\t' << midP.y << endl;
	
		midP.x -= ardrone_pos.x;
		midP.y -= ardrone_pos.y;
		geometry_msgs::Point dist;
		revmeasurement(midP.x, midP.y, dist.x, dist.y, pitch, roll, height);
		green_pub.publish(dist);
	}
}

void FindCircles::find_red(Mat &img, Mat imgThresholded)
{
	for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			if ((tmp2-tmp1)>=R_DFNT && (tmp2-tmp0)>=R_DFNT)
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
		if (contourArea(*it, true) < 500)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));
			rb.push_back(r);
			rectangle(img, r, Scalar(0, 255, 0), 3);//testing
			++it;
		}
	}

	if (!rb.empty())
	{
		for (int i = 1; i < rb.size(); ++i)
		{
			double d0 = pow(0.5*(rb[0].tl().x+rb[0].br().x-img.rows),2)+pow(0.5*(rb[0].tl().y+rb[0].br().y-img.cols),2);
			double di = pow(0.5*(rb[i].tl().x+rb[i].br().x-img.rows),2)+pow(0.5*(rb[i].tl().y+rb[i].br().y-img.cols),2);
		
			if (d0 > di)
			{
				Rect tmp;
				tmp = rb[0];
				rb[0] = rb[i];
				rb[i] = tmp;
			}
		}
		
		for (int i = 0; i < rb.size(); ++i)
		{
			geometry_msgs::Point data;
			data.x = 0.5*(rb[i].tl().x + rb[i].br().x);
			data.y = 0.5*(rb[i].tl().y + rb[i].br().y);
			//cout << "circle center: " << midP.x << '\t' << midP.y << endl;
			geometry_msgs::Point img_data;
			img_data.x = data.x - 0.5*img.rows;
			img_data.y = data.y - 0.5*img.cols;
			image_msg.point.push_back(img_data);

			geometry_msgs::Point dist;
			data.x -= ardrone_pos.x;
			data.y -= ardrone_pos.y;
			revmeasurement(data.x, data.y, dist.x, dist.y, pitch, roll, height);
			real_msg.point.push_back(dist);
		}
		red_image_pub.publish(image_msg);
		red_real_pub.publish(real_msg);	
	}
}

void FindCircles::measurement(double &x, double &y, double u, double v, double pitch, double roll, double h)
{
	double m = u * h / (h/cos(roll) + u*sin(roll));
	double n = v * h / (h/cos(pitch) + u*sin(pitch));
	x = 1000 * m / (1.55 * h + 0.01078);
	y = 1000 * n / (1.55 * h + 0.01078);
}
void FindCircles::revmeasurement(double x, double y, double &u, double &v, double pitch, double roll, double h)
{
	double m = (1.55 * h + 0.01078) / 1000 * x;
	double n = (1.55 * h + 0.01078) / 1000 * y;
	u = m * h/cos(roll) / (h-m*sin(roll));
	v = n * h/cos(pitch) / (h-n*sin(pitch));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_circle");
	FindCircles fc;
	ros::spin();
	return 0;
}