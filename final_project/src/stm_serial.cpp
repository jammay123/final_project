#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

class SerialNode : public rclcpp::Node
{
public:
  SerialNode() : Node("serial_node")
  {
    // 시리얼 포트 설정
    try {
      serial_.setPort("/dev/ttyUSB0");
      serial_.setBaudrate(115200);  // 보드레이트 설정
      serial_.open();
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    if (serial_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      rclcpp::shutdown();
      return;
    }

    // 구독자 및 퍼블리셔 생성
    sub_ = this->create_subscription<std_msgs::msg::String>("serial_node/input", 10, std::bind(&SerialNode::writeCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::String>("/stm32/serial_node/output", 10);
    pub_stm32_psd_adc_right_ = this->create_publisher<std_msgs::msg::Int32>("/stm32/psd_adc_value_right", 10);
    pub_stm32_psd_adc_front_ = this->create_publisher<std_msgs::msg::Int32>("/stm32/psd_adc_value_front", 10);
    pub_stm32_psd_adc_left_ = this->create_publisher<std_msgs::msg::Int32>("/stm32/psd_adc_value_left", 10);

    sub_stm32_dxl_linear_vel_ = this->create_subscription<std_msgs::msg::Int32>("/stm32/dxl_linear_vel", 10, std::bind(&SerialNode::linearVelCallback, this, std::placeholders::_1));
    sub_stm32_dxl_angular_vel_ = this->create_subscription<std_msgs::msg::Int32>("/stm32/dxl_angular_vel", 10, std::bind(&SerialNode::angularVelCallback, this, std::placeholders::_1));

    // 타이머 설정
    read_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialNode::readCallback, this));
  }

private:
  serial::Serial serial_;  // 시리얼 포트 객체
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_stm32_psd_adc_right_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_stm32_psd_adc_front_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_stm32_psd_adc_left_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_stm32_dxl_linear_vel_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_stm32_dxl_angular_vel_;
  int32_t linear_vel_ = 0;
  int32_t angular_vel_ = 0;
  rclcpp::TimerBase::SharedPtr read_timer_;

  // ========== [메시지 수신 콜백: 수신한 데이터를 시리얼 포트를 통해 전송] ==========
  void writeCallback(const std_msgs::msg::String::SharedPtr msg) {
    serial_.write(msg->data);
  }

  void linearVelCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    linear_vel_ = msg->data;
    sendVelocityData();
  }

  void angularVelCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    angular_vel_ = msg->data;
    sendVelocityData();
  }

  void sendVelocityData() {
    uint8_t packet[10];

    packet[0] = 0x19;

    packet[1] = (linear_vel_ >> 24) & 0xFF; // 상위 8비트
    packet[2] = (linear_vel_ >> 16) & 0xFF;
    packet[3] = (linear_vel_ >> 8) & 0xFF;
    packet[4] = linear_vel_ & 0xFF;         // 하위 8비트

    packet[5] = (angular_vel_ >> 24) & 0xFF; // 상위 8비트
    packet[6] = (angular_vel_ >> 16) & 0xFF;
    packet[7] = (angular_vel_ >> 8) & 0xFF;
    packet[8] = angular_vel_ & 0xFF;         // 하위 8비트

    packet[9] = 0x03;

    serial_.write(packet, sizeof(packet));

    RCLCPP_INFO(this->get_logger(), "Sent packet: linear_vel: %d, angular_vel: %d", linear_vel_, angular_vel_);
  }

  void readCallback() {
    try {
      size_t available_bytes = serial_.available();
      if (available_bytes >= 14) {
        std::vector<uint8_t> buffer(available_bytes);
        serial_.read(buffer.data(), buffer.size());

        if(buffer[0] == 0x08 && buffer[13] == 0x20) {
          int32_t adc_value_1 = (buffer[1] << 24) | (buffer[2] << 16) | (buffer[3] << 8) | buffer[4];
          int32_t adc_value_2 = (buffer[5] << 24) | (buffer[6] << 16) | (buffer[7] << 8) | buffer[8];
          int32_t adc_value_3 = (buffer[9] << 24) | (buffer[10] << 16) | (buffer[11] << 8) | buffer[12];

          std_msgs::msg::Int32 msg1;
          std_msgs::msg::Int32 msg2;
          std_msgs::msg::Int32 msg3;

          msg1.data = adc_value_1;
          msg2.data = adc_value_2;
          msg3.data = adc_value_3;

          pub_stm32_psd_adc_right_->publish(msg1);
          pub_stm32_psd_adc_front_->publish(msg2);
          pub_stm32_psd_adc_left_->publish(msg3);
        }
      }
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "IOException: %s", e.what());
      serial_.close();
      rclcpp::shutdown();
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
