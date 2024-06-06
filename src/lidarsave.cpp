#include "lidarsave/lidarsave.hpp"
#include <opencv2/opencv.hpp>

LIDARSAVE::LIDARSAVE() : Node("lidar")
{
  writer1.open("lidarsave.mp4", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 30, cv::Size(500, 500));
  lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&LIDARSAVE::scanCb, this, _1));
}
void LIDARSAVE::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  int count = scan->scan_time / scan->time_increment;
  int x, y;
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
  
  for (int i = 0; i < count; i++) {
    float degree = scan->angle_min + scan->angle_increment * i;
    printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", RAD2DEG(degree), scan->ranges[i]);
    x = 250 + scan->ranges[i] * 50 * sin(degree);
    y = 250 + scan->ranges[i] * 50 * cos(degree);
    cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), 1);
  }
  writer1 << img;
  cv::imshow("img", img);
  cv::waitKey(1);
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LIDARSAVE>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}