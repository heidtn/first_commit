/*
* optical_flow.cpp
*
*  Created on: Mar 16, 2016
*      Author: Nicolas
*/

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <typeinfo>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "SnapCam.h"

image_transport::Publisher image_pub;

// TODO(heidt) we can probably get these from ROS
float F_X = 279.60814589461154;
float F_Y = 280.1693782018111;
float C_X = 222.49106441516423;
float C_Y = 317.7691476613439;


void rot90(cv::Mat &matImage, int rotflag){
  //1=CW, 2=CCW, 3=180
  if (rotflag == 1){
    transpose(matImage, matImage);  
    flip(matImage, matImage,1); //transpose+flip(1)=CW
  } else if (rotflag == 2) {
    transpose(matImage, matImage);  
    flip(matImage, matImage,0); //transpose+flip(0)=CCW     
  } else if (rotflag ==3){
    flip(matImage, matImage,-1);    //flip(-1)=180          
  } else if (rotflag != 0){ //if not 0,1,2,3:
    cout  << "Unknown rotation flag(" << rotflag << ")" << endl;
  }
}


void filterNoise(cv::Mat &img) {
	std::vector<vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	cv::findContours(img.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

	std::vector<std::vector<Point>> good_contours;
	for (int i = 0; i< contours.size(); i++)
	{
	    double area = cv::contourArea(contours[i]);
	    if (area > 200)
	    {
	        good_contours.push_back(contours[i]);
	    }
	}
	drawContours(img, contours, savedContour, Scalar(255), CV_FILLED, 8);
}


void findCorners(const cv::Mat &img) {
	cv::Mat hsv, mask, dilated;

	// Convert to HSV and threshold
	cv::cvtColor(img, hsv, CV_BGR2HSV);
	cv::inRange(hsv, cv::Scalar(6, 80, 130), cv::Scalar(16, 255, 255), mask);

	// Dilate and remove noise
	// TODO(heidt) the morphology method might be improved
	cv::Mat element = getStructuringElement(MORPH_ELLIPSE,
                                        cv::Size( 2*5 + 1, 2*5 + 1 ),
                                        cv::Point(5, 5));
	cv::dilate(mask, dilated, element);


}

void imageCallback(const cv::Mat &img, uint64_t time_stamp) {
	// convert OpenCV image to ROS message

	rot90(img, 1);
	findCorners(img);
/*	cv_bridge::CvImage cvi;
	cvi.header.stamp = ros::Time::now();
	cvi.header.frame_id = "image";
	cvi.image = img;

	if (img.channels() == 1) { // optical flow
		cvi.encoding = "mono8";

	} else { // highres
		cvi.encoding = "bgr8";
	}

	sensor_msgs::Image im;
	cvi.toImageMsg(im);
	image_pub.publish(im);*/
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "snap_cam_publisher");
	ros::NodeHandle nh("~");
	image_transport::ImageTransport it(nh);

	std::string topic_name;

	if (!nh.getParam("topic_name", topic_name)) {
		topic_name = "image";
		ROS_WARN("No topic name parameter provided. Defaulting to: %s.", topic_name.c_str());
	}

	image_pub = it.advertise(topic_name.c_str(), 1);

	std::string res;

	if (!nh.getParam("camera_resolution", res)) {
		res = "VGA";
		ROS_WARN("No resolution parameter provided. Defaulting to %s.", res.c_str());
	}

	std::string camera_type;

	if (!nh.getParam("camera_type", camera_type)) {
		camera_type = "highres";
		ROS_WARN("No camera type parameter provided. Defaulting to %s.", camera_type.c_str());
	}

	int camera_fps;

	if (!nh.getParam("camera_fps", camera_fps)) {
		camera_fps = 30;
		ROS_WARN("No camera fps idx parameter provided. Defaulting to %d.", camera_fps);
	}

	int exposure;

	if (!nh.getParam("exposure", exposure)) {
		exposure = 100;
		ROS_WARN("No exposure parameter provided. Defaulting to %d.", exposure);
	}

	int camera_gain;

	if (!nh.getParam("gain", camera_gain)) {
		camera_gain = 0;
		ROS_WARN("No gain parameter provided. Defaulting to %d.", camera_gain);
	}

	bool auto_exposure;

	if (!nh.getParam("auto_exposure", auto_exposure)) {
		auto_exposure = false;
		ROS_WARN("Defaulting to no auto exposure");
	}

	CamConfig cfg;

	if (camera_type == "highres") {
		cfg.func = CAM_FUNC_HIRES;

	} else if (camera_type == "optflow") {
		cfg.func = CAM_FUNC_OPTIC_FLOW;

	} else {
		ROS_ERROR("Invalid camera type %s. Defaulting to highres.", camera_type.c_str());
		cfg.func = CAM_FUNC_HIRES;
	}

	if (res == "4k") {
		cfg.pSize = CameraSizes::UHDSize();

	} else if (res == "1080p") {
		cfg.pSize = CameraSizes::FHDSize();

	} else if (res == "720p") {
		cfg.pSize = CameraSizes::HDSize();

	} else if (res == "VGA") {
		cfg.pSize = CameraSizes::VGASize();

	} else if (res == "QVGA") {
		cfg.pSize = CameraSizes::QVGASize();

	} else if (res == "stereoVGA") {
		cfg.pSize = CameraSizes::stereoVGASize();

	} else if (res == "stereoQVGA") {
		cfg.pSize = CameraSizes::stereoQVGASize();

	} else {
		ROS_ERROR("Invalid resolution %s. Defaulting to VGA\n", res.c_str());
		cfg.pSize = CameraSizes::stereoVGASize();
	}

	cfg.fps = camera_fps;
	cfg.exposureValue = exposure;
	cfg.gainValue = camera_gain;

	SnapCam cam(cfg);
	cam.setListener(imageCallback);
	if (auto_exposure) {
		ROS_INFO("Using auto exposure");
		cam.setAutoExposure(auto_exposure);
	}

	ros::spin();

	return 0;
}
