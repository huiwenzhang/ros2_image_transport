
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ctime>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <termio.h>
#include <fcntl.h>

std::string path;
namespace bfs = boost::filesystem;
bool save = false;

int monitorKeyboard()
{
  fcntl(0, F_SETFL, O_NONBLOCK);
  int in;
  struct termios new_settings;
  struct termios stored_settings;
  tcgetattr(0, &stored_settings);
  new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(0, &stored_settings);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &new_settings);

  in = getchar();
  tcsetattr(0, TCSANOW, &stored_settings);
  return in;
}

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  try
  {
    auto img = cv_bridge::toCvShare(msg, "8UC3")->image;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%H-%M-%S");
    std::string stt = oss.str();
    if (save)
    {
      cv::imwrite(path + "/toilet-" + stt + ".png", img);
    }
    cv::imshow("view", img);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.",
                 msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("image_listener", options);

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  // oss << std::put_time(&tm, "%m-%d-%H-%M-%S");
  oss << std::put_time(&tm, "%m-%d");
  std::string stt = oss.str();
  path = getenv("HOME") + std::string("/data/img/") + stt;
  if (!bfs::exists(path))
  {
    bfs::create_directories(path);
  }

  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub =
      it.subscribe("/ob/rgb/image_raw", 1, imageCallback);

  rclcpp::WallRate loop_rate(5);
  while (rclcpp::ok())
  {
    int flag = monitorKeyboard();
    if (115 == flag)
    {
      save = true;
      std::cout << "\nstart collect image\n";
    }
    if (100 == flag)
    {
      save = false;
      std::cout << "\nstop collect image\n";
    }
    loop_rate.sleep();
    rclcpp::spin_some(node);
  }
  // rclcpp::spin(node);
  cv::destroyWindow("view");
}
