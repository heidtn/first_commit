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
double F_X = 279.60814589461154;
double F_Y = 280.1693782018111;
double C_X = 222.49106441516423;
double C_Y = 317.7691476613439;
double CORNERINESS = .04;


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


void filterNoise(cv::Mat &img, int min_area=200) {
	cv::Mat result(img.size(), CV_8UC1, cv::Scalar(0));
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(img.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

	std::vector<std::vector<cv::Point>> good_contours;
	for (int i = 0; i< contours.size(); i++)
	{
	    double area = cv::contourArea(contours[i]);
	    if (area > min_area)
	    {
	        good_contours.push_back(contours[i]);
	    }
	}
	cv::drawContours(result, good_contours, -1, cv::Scalar(255), CV_FILLED, 8);
	std::cout << "adning" << result.size() << " " << img.size() << std::endl;

	img &= result;
}


void thinningIteration(cv::Mat& im, int iter)
{
    cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

    for (int i = 1; i < im.rows-1; i++)
    {
        for (int j = 1; j < im.cols-1; j++)
        {
            uchar p2 = im.at<uchar>(i-1, j);
            uchar p3 = im.at<uchar>(i-1, j+1);
            uchar p4 = im.at<uchar>(i, j+1);
            uchar p5 = im.at<uchar>(i+1, j+1);
            uchar p6 = im.at<uchar>(i+1, j);
            uchar p7 = im.at<uchar>(i+1, j-1);
            uchar p8 = im.at<uchar>(i, j-1);
            uchar p9 = im.at<uchar>(i-1, j-1);

            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + 
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) + 
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(i,j) = 1;
        }
    }

    im &= ~marker;
}

/**
 * Function for thinning the given binary image
 *
 * @param  im  Binary image with range = 0-255
 */
void thinning(cv::Mat& im)
{
    im /= 255;

    cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(im, 0);
        thinningIteration(im, 1);
        cv::absdiff(im, prev, diff);
        im.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);

    im *= 255;
}



std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


void findCorners(cv::Mat &img) {
	cv::Mat hsv, mask, dilated, diltaed_thinned, cleaned_thin, corners, big_corners, corner_thresh;

	std::cout << "convert and threshold" << std::endl;
	// Convert to HSV and threshold
	cv::cvtColor(img, hsv, CV_BGR2HSV);
	cv::inRange(hsv, cv::Scalar(6, 80, 130), cv::Scalar(16, 255, 255), mask);

	// Dilate and remove noise
	// TODO(heidt) the morphology method might be improved
	cv::dilate(mask, dilated, cv::Mat());

	std::cout << "filter noise" << std::endl;

	filterNoise(dilated);

	cv::imwrite("/home/linaro/temp_ims/dilated.jpg", dilated);

	std::cout << "thin image" << std::endl;

	thinning(dilated);
	cv::imwrite("/home/linaro/temp_ims/thinned.jpg", dilated);

	std::cout << "dilate" << std::endl;

	cv::dilate(dilated, diltaed_thinned, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	filterNoise(diltaed_thinned, 100);
	cv::imwrite("/home/linaro/temp_ims/diltaed_thinned.jpg", diltaed_thinned);

	std::cout << "Harris corners" << std::endl;

	cornerHarris(diltaed_thinned, corners, 16, 1, 0.04);
	cv::imwrite("/home/linaro/temp_ims/corners.jpg", corners);

	cv::dilate(corners, big_corners, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

	double min, max;
	cv::minMaxLoc(big_corners, &min, &max);
	std::cout << "minmax " << min << " " << max << std::endl;

	// TODO(heidt) fix bad corners)
	threshold(big_corners, corner_thresh, 0.5*max, 255, CV_THRESH_BINARY);
	corner_thresh.convertTo(corner_thresh, CV_8UC1);

	cv::imwrite("/home/linaro/temp_ims/corners.jpg", corner_thresh);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(corner_thresh.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

	/// Get the moments
	std::vector<cv::Moments> mu(contours.size() );
	for( int i = 0; i < contours.size(); i++ ) { 
		mu[i] = moments( contours[i], false ); 
	}

	///  Get the mass centers:
	std::vector<cv::Point2f> mc( contours.size() );
	for( int i = 0; i < contours.size(); i++ ) { 
		mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
	}

	std::cout << "Found " << mc.size() << " courners!" << std::endl;

}

void imageCallback(const cv::Mat &img_ret, uint64_t time_stamp) {
	// convert OpenCV image to ROS message
	cv::Mat img = img_ret.clone();
	std::cout << "rotating image" << std::endl;
	rot90(img, 1);
	std::cout << "finding corners" << std::endl;
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

/*	if (res == "4k") {
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
	}*/
	cfg.pSize = CameraSizes::stereoVGASize();

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
