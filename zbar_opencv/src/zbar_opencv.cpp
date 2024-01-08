#include "ros/time.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <zbar.h>

ros::Publisher chatter_pub;

using namespace std;
using namespace cv;
using namespace zbar;

bool forready = true;

void zbarscanner(cv_bridge::CvImagePtr cv_ptr) {
  if (forready) {
    ROS_INFO("zbar_opencv ready");
    forready = false;
  }
  // Create a zbar reader
  ImageScanner scanner;

  // Configure the reader
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

  // Capture an OpenCV frame
  cv::Mat frame, frame_grayscale;
  frame = cv_ptr->image;
  // Convert to grayscale
  cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

  // Obtain image data
  int width = frame_grayscale.cols;
  int height = frame_grayscale.rows;
  uchar *raw = (uchar *)(frame_grayscale.data);

  // Wrap image data
  Image image(width, height, "Y800", raw, width * height);

  // Scan the image for barcodes
  scanner.scan(image);

  // Extract results
  int counter = 0;
  for (Image::SymbolIterator symbol = image.symbol_begin();
       symbol != image.symbol_end(); ++symbol) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << symbol->get_data();
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();

    // Draw location of the symbols found
    if (symbol->get_location_size() == 4) {

      line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)),
           Point(symbol->get_location_x(1), symbol->get_location_y(1)),
           Scalar(0, 255, 0), 2, 8, 0);
      line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)),
           Point(symbol->get_location_x(2), symbol->get_location_y(2)),
           Scalar(0, 255, 0), 2, 8, 0);
      line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)),
           Point(symbol->get_location_x(3), symbol->get_location_y(3)),
           Scalar(0, 255, 0), 2, 8, 0);
      line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)),
           Point(symbol->get_location_x(0), symbol->get_location_y(0)),
           Scalar(0, 255, 0), 2, 8, 0);
    }
    counter++;
  }
}

class ImageConverter {
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;

public:
  ImageConverter() : it(nh) {
    image_sub =
        it.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub = it.advertise("zbar_opencv", 1);
  }
  ~ImageConverter() {}

  void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    zbarscanner(cv_ptr);
    image_pub.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "zbar_opencv");
  ros::NodeHandle nh;
  chatter_pub = nh.advertise<std_msgs::String>("zbar_opencv_code", 1000);
  ImageConverter ic;
  ros::spin();
  return 0;
}
