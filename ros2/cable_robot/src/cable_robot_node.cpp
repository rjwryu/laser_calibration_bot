#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class CableRobotNode : public rclcpp::Node {
public:
    CableRobotNode() :
        Node("cable_robot_node")
    {
        // publishers for motor commands
        cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/effort_controller/commands", 10);

        // subscriber for joint states
        state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&CableRobotNode::onJointData, this, std::placeholders::_1));

        // control loop timer
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&CableRobotNode::onControlLoop, this));

        // register my shutdown handler before exit
        rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
                std::bind(&CableRobotNode::onShutdown, this));

        RCLCPP_INFO(this->get_logger(),
                "Cable Driven Node initialized");
    }

    // stop the motor (velocity zero) before exit
    void onShutdown() {
        timer_->cancel();

        // stop motor at exit
        std_msgs::msg::Float64MultiArray msg;
        msg.data = { 0.0 };
        cmd_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                "Stopped motor");
    }

private:
    static constexpr double RAMP_STEP_ = 0.01;
    static constexpr double MOTION_THRESHOLD_ = 0.1;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_program_started_ = false;
    bool is_program_completed_ = false;
    double effort_ = 0.0;
    double position_change_ = 0.0;
    unsigned throttle_counter_ = 0;

    void onControlLoop() {
        if (!is_program_started_ || is_program_completed_) {
            publishEffort(0.0);
            return;
        }

        if (position_change_ > MOTION_THRESHOLD_) {
            RCLCPP_INFO(this->get_logger(),
                    "Movement detected at effort %.3f (Δpos=%.3f)",
                    effort_, position_change_);
            is_program_completed_ = true;
            publishEffort(0.0);
            return;
        }

        publishEffort(effort_);
        RCLCPP_INFO(this->get_logger(),
                "No movement detected, applying effort %.3f",
                effort_);

        if (throttle_counter_ == 1) {
            effort_ += RAMP_STEP_;
            throttle_counter_ = 0;
        } else {
            ++throttle_counter_;
        }
    }

    void onJointData(const sensor_msgs::msg::JointState::SharedPtr msg) {
        static double original_position = 0.0;

        if (msg->name.empty()) return;

        double effort_feedback = msg->effort[0];
        double current_position = msg->position[0];
        if (!is_program_completed_) {
            RCLCPP_INFO(this->get_logger(),
                    "Joint data: cur=%+8.3f, pos=%+8.3f", effort_feedback, current_position);
        }

        if (!is_program_started_) {
            original_position = current_position;
            is_program_started_ = true;
        }

        position_change_ = current_position - original_position;
    }

    void publishEffort(double effort) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = { effort };
        cmd_pub_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CableRobotNode>();
    rclcpp::spin(node);
    return 0;
}
