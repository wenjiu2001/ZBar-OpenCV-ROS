#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "zbar.h"

using namespace std;
using namespace cv;
using namespace zbar;

ros::Publisher chatter_pub;

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
      : nh_(), it_(nh_), image_sub_(), image_pub_() {
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("zbar_opencv", 1);
    chatter_pub = nh_.advertise<std_msgs::String>("zbar_opencv_code", 1000);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      processImage(cv_ptr);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void processImage(cv_bridge::CvImagePtr &cv_ptr) {
    Mat frame, frame_grayscale;
    frame = cv_ptr->image;
    cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    int width = frame_grayscale.cols;
    int height = frame_grayscale.rows;
    uchar *raw = (uchar *)(frame_grayscale.data);

    Image image(width, height, "Y800", raw, width * height);
    scanner.scan(image);

    for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
      std_msgs::String msg;
      msg.data = symbol->get_data();
      ROS_INFO("%s", msg.data.c_str());
      chatter_pub.publish(msg);

      drawSymbol(frame, *symbol);
    }

    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void drawSymbol(Mat &frame, const Symbol &symbol) {
    if (symbol.get_location_size() == 4) {
      for (int i = 0; i < 4; i++) {
        line(frame, Point(symbol.get_location_x(i), symbol.get_location_y(i)),
             Point(symbol.get_location_x((i+1) % 4), symbol.get_location_y((i+1) % 4)),
             Scalar(0, 255, 0), 2, 8, 0);
      }
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "zbar_opencv");
  ImageConverter ic;
  ros::spin();
  return 0;
}
