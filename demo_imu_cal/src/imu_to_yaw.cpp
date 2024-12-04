#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cmath>
#include <sstream>
#include <vector>
#include <string>

class ImuSubscriber : public rclcpp::Node
{
public:
  ImuSubscriber() : Node("imu_subscriber"), prev_yaw_(0.0), alpha_(0.1)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "imu/data_demo", 10, std::bind(&ImuSubscriber::topicCallback, this, std::placeholders::_1));

    yaw_publisher_ = this->create_publisher<std_msgs::msg::Int32>("imu/yaw_int", 10);

    RCLCPP_INFO(this->get_logger(), "IMU 서브스크라이버 init");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr yaw_publisher_;
  double prev_yaw_;
  double alpha_;

  void topicCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string cleaned_data = removeHeader(msg->data);

    std::vector<double> imu_data = parseImuData(cleaned_data);

    if (imu_data.size() >= 4) {
      double imu_x = imu_data[2];
      double imu_y = imu_data[1];
      double imu_z = imu_data[0];
      double imu_w = imu_data[3];

      double yaw = atan2(2.0 * (imu_x * imu_y + imu_w * imu_z), imu_w * imu_w + imu_x * imu_x - imu_y * imu_y - imu_z * imu_z);

      double yaw_angle = yaw * (180.0 / M_PI);

      if (yaw_angle < 0.0) {
        yaw_angle += 360.0;
      }

      double filtered_yaw = applyLowPassFilter(yaw_angle);

      int yaw_int = static_cast<int>(filtered_yaw);

      RCLCPP_INFO(this->get_logger(), "YAW(도): %d", yaw_int);

      // 퍼블리시
      auto yaw_msg = std_msgs::msg::Int32();
      yaw_msg.data = yaw_int;
      yaw_publisher_->publish(yaw_msg);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "잘못된 값 받아옴");
    }
  }

  double applyLowPassFilter(double raw_yaw)
  {
    double filtered_yaw = alpha_ * raw_yaw + (1.0 - alpha_) * prev_yaw_;
    prev_yaw_ = filtered_yaw;
    return filtered_yaw;
  }

  std::vector<double> parseImuData(const std::string &data)
  {
    std::vector<double> parsed_data;
    std::stringstream ss(data);

    std::string token;
    while (std::getline(ss, token, ',')) {
      try {
        parsed_data.push_back(std::stod(token));
      }
      catch (const std::out_of_range &e) {
        // 예외 무시
      }
    }
    return parsed_data;
  }

  std::string removeHeader(const std::string &data)
  {
    if (!data.empty() && data[0] == '*') {
      return data.substr(1);
    }
    return data;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
