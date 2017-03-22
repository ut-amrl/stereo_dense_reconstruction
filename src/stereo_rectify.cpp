#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <ctime>

using namespace std;
using namespace cv;

Mat XR, XT, Q, P1, P2;
Mat R1, R2, K1, K2, D1, D2, R;
Vec3d T;
Mat lmapx, lmapy, rmapx, rmapy;
FileStorage calib_file;
Size out_img_size(320, 240); //kitti - 489x180, 1392x512
Size calib_img_size(640, 480);

image_transport::Publisher pub_img_left;
image_transport::Publisher pub_img_right;

void undistortRectifyImage(Mat& src, Mat& dst, FileStorage& calib_file, int 
left = 1) {
  if (left == 1) {
    remap(src, dst, lmapx, lmapy, cv::INTER_LINEAR);
  } else {
    remap(src, dst, rmapx, rmapy, cv::INTER_LINEAR);
  }
}

void findRectificationMap(FileStorage& calib_file, Size finalSize) {
  Rect validRoi[2];
  cout << "starting rectification" << endl;
  stereoRectify(K1, D1, K2, D2, calib_img_size, R, Mat(T), R1, R2, P1, P2, Q, 
                CV_CALIB_ZERO_DISPARITY, 0, finalSize, &validRoi[0], 
&validRoi[1]);
  cout << "done rectification" << endl;
  calib_file["R1"] >> R1;
  cv::initUndistortRectifyMap(K1, D1, R1, P1, finalSize, CV_32F, lmapx, 
lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, finalSize, CV_32F, rmapx, 
rmapy);
}

void imgLeftCallback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    Mat tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
    //Mat tmp = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_COLOR);
    if (tmp.empty()) return;
    Mat dst;
    undistortRectifyImage(tmp, dst, calib_file, 1);
    sensor_msgs::ImagePtr img_left;
    img_left = cv_bridge::CvImage(msg->header, "bgr8", 
dst).toImageMsg();
    pub_img_left.publish(img_left);
  }
  catch (cv_bridge::Exception& e)
  {
  }
}

void imgRightCallback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    Mat tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
    //Mat tmp = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_COLOR);
    if (tmp.empty()) return;
    Mat dst;
    undistortRectifyImage(tmp, dst, calib_file, 0);
    sensor_msgs::ImagePtr img_right;
    img_right = cv_bridge::CvImage(msg->header, "bgr8", 
dst).toImageMsg();
    pub_img_right.publish(img_right);
  }
  catch (cv_bridge::Exception& e)
  {
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_rectify");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  calib_file = FileStorage(argv[3], FileStorage::READ);
  calib_file["K1"] >> K1;
  calib_file["K2"] >> K2;
  calib_file["D1"] >> D1;
  calib_file["D2"] >> D2;
  calib_file["R"] >> R;
  calib_file["T"] >> T;
  findRectificationMap(calib_file, out_img_size);

  ros::Subscriber sub_img_left = nh.subscribe(argv[1], 1, imgLeftCallback);
  ros::Subscriber sub_img_right = nh.subscribe(argv[2], 1, imgRightCallback);
 
  pub_img_left = it.advertise("/camera_left_rect/image_color", 1);
  pub_img_right = it.advertise("/camera_right_rect/image_color", 1);
  
  ros::spin();
  return 0;
}
