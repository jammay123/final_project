#include "../include/udp_connection/laptop/master_node.hpp"

MasterNode::MasterNode() : isDetectYellowLine(false), isDetectWhiteLine(false), isRobotRun_(false), linear_vel_(0), angular_vel_(0), yellow_line_x_(0.0), white_line_x_(0.0), stage_number_(0), imu_yaw_(0.0), psd_adc_left_(0), psd_adc_front_(0), psd_adc_right_(0), white_line_points_(8, 0.0), yellow_line_points_(8, 0.0), white_line_angle_(0.0), yellow_line_angle_(0.0)
{
    node = rclcpp::Node::make_shared("master_node");

    RCLCPP_INFO(node->get_logger(), "master_node 초기화 완료");

    initialized_ = true;

    // ========== [Line Detect 서브스크라이버] ==========
    sub_yellow_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/yellow_line_detected", 10, std::bind(&MasterNode::detectYellowLine, this, std::placeholders::_1));
    sub_white_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/white_line_detected", 10, std::bind(&MasterNode::detectWhiteLine, this, std::placeholders::_1));
    sub_yellow_line_x_ = node->create_subscription<std_msgs::msg::Float32>("/vision/yellow_line_x", 10, std::bind(&MasterNode::getYellowLineX, this, std::placeholders::_1));
    sub_white_line_x_ = node->create_subscription<std_msgs::msg::Float32>("/vision/white_line_x", 10, std::bind(&MasterNode::getWhiteLineX, this, std::placeholders::_1));

    // ========== [Dynamixel 퍼블리셔 생성] ==========
    pub_dxl_linear_vel_ = node->create_publisher<std_msgs::msg::Int32>("/stm32/dxl_linear_vel", 10);
    pub_dxl_angular_vel_ = node->create_publisher<std_msgs::msg::Int32>("/stm32/dxl_angular_vel", 10);

    // ========== [IMU 서브스크라이버] ==========
    sub_imu_yaw_ = node->create_subscription<std_msgs::msg::Float32>("/imu/yaw", 10, std::bind(&MasterNode::getImuYaw, this, std::placeholders::_1));

    // ========== [STM32의 PSD(ADC) Value 서브스크라이브] ==========
    sub_stm32_psd_adc_right_ = node->create_subscription<std_msgs::msg::Int32>("/stm32/psd_adc_value_right", 10, std::bind(&MasterNode::psdRightCallback, this, std::placeholders::_1));
    sub_stm32_psd_adc_front_ = node->create_subscription<std_msgs::msg::Int32>("/stm32/psd_adc_value_front", 10, std::bind(&MasterNode::psdFrontCallback, this, std::placeholders::_1));
    sub_stm32_psd_adc_left_ = node->create_subscription<std_msgs::msg::Int32>("/stm32/psd_adc_value_left", 10, std::bind(&MasterNode::psdLeftCallback, this, std::placeholders::_1));

    // ========== [White & Yellow Line 감지 및 좌표 서브스크라이브] ==========
    sub_white_line_points_ = node->create_subscription<std_msgs::msg::Float32MultiArray>("/vision/white_line_points", 10, std::bind(&MasterNode::getWhiteLinePoints, this, std::placeholders::_1));
    sub_yellow_line_points_ = node->create_subscription<std_msgs::msg::Float32MultiArray>("/vision/yellow_line_points", 10, std::bind(&MasterNode::getYellowLinePoints, this, std::placeholders::_1));

    // ========== [White & Yellow Angle(각도) 서브스크라이브] ==========
    sub_white_angle_ = node->create_subscription<std_msgs::msg::Float32>("/vision/white_line_angle", 10, std::bind(&MasterNode::getWhiteLineAngle, this, std::placeholders::_1));
    sub_yellow_angle_ = node->create_subscription<std_msgs::msg::Float32>("/vision/yellow_line_angle", 10, std::bind(&MasterNode::getYellowLineAngle, this, std::placeholders::_1));

    stage_number_ = 1; // 최초 시작: 스테이지1
}

MasterNode::~MasterNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void MasterNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        /* 주행 로직
        * 각 line별 detected랑 position X값 사용해서 주행 로직 구현하기
        * 노란색 처리 우선(빛 반사 적음)
        */
        emit updateCurrentStage(stage_number_);
        RCLCPP_INFO(node->get_logger(), "Y Angle: %.2f | W Angle: %.2f", yellow_line_angle_, white_line_angle_);

        if (stage_number_ == 1) {
            runRobotStage1();
        } else if (stage_number_ == 2) {
            runRobotStage2();
        }

        // 현재 상태를 유지하며 지속적으로 퍼블리시
        auto msg_linear_ = std_msgs::msg::Int32();
        auto msg_angular_ = std_msgs::msg::Int32();

        msg_linear_.data = linear_vel_;
        msg_angular_.data = angular_vel_;

        pub_dxl_linear_vel_->publish(msg_linear_);
        pub_dxl_angular_vel_->publish(msg_angular_);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}

bool MasterNode::isInitialized() const
{
    return initialized_; // 초기화 상태 반환
}

// ========== [스테이지별 이동 처리 메서드] ==========
void MasterNode::runRobotStage1() {
    if (yellow_line_x_ <= -0.5 && white_line_x_ >= 0.5) { // 직진 주행
        ctlDxlFront(17, 0);
    } else if (((0.15 >= yellow_line_x_) && (yellow_line_x_ >= -0.5))) { // 우회전
        // 첫 번째 오른쪽 코너
        if (yellow_line_angle_ < 10) {
            ctlDxlRight(7, 3);
        } else if (yellow_line_angle_ < 20) {
            ctlDxlRight(5, 5);
        } else if (yellow_line_angle_ < 25) {
            ctlDxlRight(3, 7);
        } else if (yellow_line_angle_ < 35) {
            ctlDxlRight(1, 12);
        }

    } else if (((0.95 >= white_line_x_) && (white_line_x_ >= 0.35))) { // 좌회전
        // 주행 도중 라인 유지
        ctlDxlLeft(7, 2);
    } else {
        ctlDxlFront(7, 0);
    }

    // Stage2 감지
    if (psd_adc_left_ >= 2450 && (!isDetectYellowLine && (white_line_angle_ < 200))) {
        stage_number_ = 2;
    }
}

void MasterNode::runRobotStage2() {
    // stopDxl();
    // RCLCPP_INFO(node->get_logger(), "스테이지2");
    // 흰 선에 대한 코너링 처리
    if ((!isDetectYellowLine && isDetectWhiteLine) && white_line_points_[0] < 100) {
        ctlDxlLeft(3, 5);
    } else if ((!isDetectYellowLine && isDetectWhiteLine) && white_line_points_[0] < 300) {
        ctlDxlLeft(5, 3);
    } else if ((!isDetectYellowLine && isDetectWhiteLine) && (white_line_points_[0] > 500 && psd_adc_front_ < 2500)) {
        ctlDxlFront(7, 0);
        // if (psd_adc_left_ < 3000 && psd_adc_right_ < 3000) {
        //     ctlDxlLeft(5, 3);
        // } else {
        //     RCLCPP_INFO(node->get_logger(), "7");
        //     ctlDxlFront(7, 0);
        // }
    }
    if (psd_adc_front_ > 2500) {
        stopDxl();
    }
}


// ========== [Line Detect 서브스크라이브] ==========
void MasterNode::detectYellowLine(const std_msgs::msg::Bool::SharedPtr msg) {
    isDetectYellowLine = msg->data;
    if (isDetectYellowLine) {
        isDetectYellowLine = true;
        // RCLCPP_INFO(node->get_logger(), "노란색 라인 감지");
        // ctlDxlRight();
    } else {
        isDetectYellowLine = false;
    }
}

void MasterNode::detectWhiteLine(const std_msgs::msg::Bool::SharedPtr msg) {
    isDetectWhiteLine = msg->data;
    if (isDetectWhiteLine) {
        isDetectWhiteLine = true;
        // RCLCPP_INFO(node->get_logger(), "흰색 라인 감지");
        // ctlDxlLeft();
    } else {
        isDetectWhiteLine = false;
    }
}

void MasterNode::getYellowLineX(const std_msgs::msg::Float32::SharedPtr msg) {
    yellow_line_x_ = msg->data;
    // RCLCPP_INFO(node->get_logger(), "value: %.2f", yellow_line_x_);
}

void MasterNode::getWhiteLineX(const std_msgs::msg::Float32::SharedPtr msg) {
    white_line_x_ = msg->data;
    // RCLCPP_INFO(node->get_logger(), "value: %.2f", white_line_x_);
}

void MasterNode::getWhiteLinePoints(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    for (size_t i = 0; i < white_line_points_.size(); i++) {
        white_line_points_[i] = msg->data[i];
    }
    // RCLCPP_INFO(node->get_logger(), "Updated white_line_points_: [%f, %f, %f, %f, %f, %f, %f, %f]", white_line_points_[0], white_line_points_[1], white_line_points_[2], white_line_points_[3], white_line_points_[4], white_line_points_[5], white_line_points_[6], white_line_points_[7]);
}

void MasterNode::getYellowLinePoints(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    for (size_t i = 0; i < yellow_line_points_.size(); i++) {
        yellow_line_points_[i] = msg->data[i];
    }
    // RCLCPP_INFO(node->get_logger(), "Updated yellow_line_points_: [%f, %f, %f, %f, %f, %f, %f, %f] | %f", yellow_line_points_[0], yellow_line_points_[1], yellow_line_points_[2], yellow_line_points_[3], yellow_line_points_[4], yellow_line_points_[5], yellow_line_points_[6], yellow_line_points_[7], yellow_line_angle_);
}

void MasterNode::getWhiteLineAngle(const std_msgs::msg::Float32::SharedPtr msg) {
    white_line_angle_ = msg->data;
    // RCLCPP_INFO(node->get_logger(), "White value: %.2f", yellow_line_angle_);
}

void MasterNode::getYellowLineAngle(const std_msgs::msg::Float32::SharedPtr msg) {
    yellow_line_angle_ = msg->data;
    // RCLCPP_INFO(node->get_logger(), "Yellow value: %.2f", yellow_line_angle_);
}


// ========== [IMU 서브스크라이브] ==========
void MasterNode::getImuYaw(const std_msgs::msg::Float32::SharedPtr msg) {
    imu_yaw_ = msg->data;
    // RCLCPP_INFO(node->get_logger(), "IMU value: %.2f", imu_yaw_);
}





// ========== [Dxl Control 메서드] ==========
void MasterNode::ctlDxlFront(int linearVel, int angularVel) {
    if (isRobotRun_) {
        linear_vel_ = linearVel;  // 상태 갱신
        angular_vel_ = angularVel;

        // auto msg_linear_ = std_msgs::msg::Int32();
        // auto msg_angular_ = std_msgs::msg::Int32();

        // msg_linear_.data = linearVel;
        // msg_angular_.data = 0;

        // pub_dxl_linear_vel_->publish(msg_linear_);
        // pub_dxl_angular_vel_->publish(msg_angular_);
        RCLCPP_INFO(node->get_logger(), "직진 이동");
    }
}

void MasterNode::ctlDxlLeft(int linearVel, int angularVel) {
    if (isRobotRun_) {
        linear_vel_ = linearVel;  // 상태 갱신
        angular_vel_ = angularVel;

        // auto msg_linear_ = std_msgs::msg::Int32();
        // auto msg_angular_ = std_msgs::msg::Int32();

        // msg_linear_.data = linearVel;
        // msg_angular_.data = angularVel;

        // pub_dxl_linear_vel_->publish(msg_linear_); // 퍼블리시
        // pub_dxl_angular_vel_->publish(msg_angular_);
        RCLCPP_INFO(node->get_logger(), "왼쪽으로 이동");
    }
}


void MasterNode::ctlDxlRight(int linearVel, int angularVel) {
    if (isRobotRun_) {
        linear_vel_ = linearVel;  // 상태 갱신
        angular_vel_ = -angularVel;

        // auto msg_linear_ = std_msgs::msg::Int32();
        // auto msg_angular_ = std_msgs::msg::Int32();

        // msg_linear_.data = linearVel;
        // msg_angular_.data = -angularVel;


        // pub_dxl_linear_vel_->publish(msg_linear_); // 퍼블리시
        // pub_dxl_angular_vel_->publish(msg_angular_);
        RCLCPP_INFO(node->get_logger(), "오른쪽으로 이동");
    }
}

void MasterNode::ctlDxlBack(int linearVel, int angularVel) {
    if (isRobotRun_) {
        // linear_vel_ = linearVel;  // 상태 갱신
        // angular_vel_ = angularVel;

        // auto msg_linear_ = std_msgs::msg::Int32();
        // auto msg_angular_ = std_msgs::msg::Int32();

        // msg_linear_.data = -linearVel;
        // msg_angular_.data = 0;

        // pub_dxl_linear_vel_->publish(msg_linear_);
        // pub_dxl_angular_vel_->publish(msg_angular_);
        RCLCPP_INFO(node->get_logger(), "후진 이동");
    }
}

void MasterNode::ctlDxlYaw(float target_yaw) {
    if(playYawFlag) {
        RCLCPP_WARN(node->get_logger(), "Yaw 제어가 이미 실행중");
        return;
    }

    playYawFlag = true;
    RCLCPP_INFO(node->get_logger(), "현재 yaw 값: %.2f, 목표 yaw 값: %.2f", imu_yaw_, target_yaw);

    const float kP = 1.5f;
    const float kI = 0.05f;
    const float yaw_ok = 1.0f;
    const float max_angular_vel = 10.0f;
    const float min_angular_vel = 1.0f;

    float cal_pi = 0.0f;

    while(playYawFlag) {
        float yaw_error = target_yaw - imu_yaw_;

        if(yaw_error > 180.0f)
        yaw_error -= 360.0f;
        if(yaw_error < -180.0f)
        yaw_error += 360.0f;

        if(fabs(yaw_error) < yaw_ok) {
            stopDxl();
            break;
        }

        cal_pi += yaw_error;

        float angular_vel_pi = (kP * yaw_error) + (kI * cal_pi);

        if(angular_vel_pi > max_angular_vel)
            angular_vel_pi = max_angular_vel;
        if(angular_vel_pi < max_angular_vel)
            angular_vel_pi = -max_angular_vel;

        if ((angular_vel_pi > 0) && (angular_vel_pi < min_angular_vel))
            angular_vel_pi = min_angular_vel;
        if ((angular_vel_pi < 0) && (angular_vel_pi > -min_angular_vel))
            angular_vel_pi = -min_angular_vel;

        runDxl(0, static_cast<int>(angular_vel_pi));
        rclcpp::spin_some(node);
    }

    playYawFlag = false;
}


void MasterNode::stopDxl() {
    isRobotRun_ = !isRobotRun_;

    linear_vel_ = 0;  // 상태 갱신
    angular_vel_ = 0;

    auto msg_linear_ = std_msgs::msg::Int32();
    auto msg_angular_ = std_msgs::msg::Int32();

    msg_linear_.data = 0;
    msg_angular_.data = 0;

    pub_dxl_linear_vel_->publish(msg_linear_);
    pub_dxl_angular_vel_->publish(msg_angular_);

    RCLCPP_INFO(node->get_logger(), "STOPSTOPSTOP!");
}

// 수동 조작 모드일 때, GUI에서 입력한 linear와 angular 데이터 반영
void MasterNode::updateDxlData(int linearVel, int angularVel) {
    linear_vel_ = linearVel;
    angular_vel_ = angularVel;
    RCLCPP_INFO(node->get_logger(), "UPDATEUPDATE!");
}

void MasterNode::runDxl(int linearVel, int angularVel) {
    linear_vel_ = linearVel;  // 상태 저장
    angular_vel_ = angularVel;  // 상태 저장

    auto msg_linear_ = std_msgs::msg::Int32();
    auto msg_angular_ = std_msgs::msg::Int32();

    msg_linear_.data = linearVel;
    msg_angular_.data = angularVel;

    // RCLCPP_INFO(node->get_logger(), "runDxl - Publishing Linear: %d | Angular: %d", msg_linear_.data, msg_angular_.data);

    pub_dxl_linear_vel_->publish(msg_linear_);
    pub_dxl_angular_vel_->publish(msg_angular_);

    RCLCPP_INFO(node->get_logger(), "RUNRUNRUNRUN!");
}


// ========== [STM32 PSD Value Callback Method] ==========
void MasterNode::psdRightCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // RCLCPP_INFO(node->get_logger(), "PSD Right: %d", msg->data);
    psd_adc_right_ = msg->data;
    emit stmPsdRightReceived(msg->data);
}

void MasterNode::psdFrontCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // RCLCPP_INFO(node->get_logger(), "PSD Front: %d", msg->data);
    psd_adc_front_ = msg->data;
    emit stmPsdFrontReceived(msg->data);
}

void MasterNode::psdLeftCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // RCLCPP_INFO(node->get_logger(), "PSD Left: %d", msg->data);
    psd_adc_left_ = msg->data;
    emit stmPsdLeftReceived(msg->data);
}
