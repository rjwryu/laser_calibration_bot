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
                "/velocity_controller/commands", 10);

        // subscriber for joint states
        state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&CableRobotNode::jointStateCallback, this, std::placeholders::_1));

        // control loop timer
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&CableRobotNode::controlLoop, this));

        // register my shutdown handler before exit
        // using rclcpp::contexts::get_global_default_context;
        // get_global_default_context()->add_pre_shutdown_callback([this]() { this->shutdown(); });
        rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
                std::bind(&CableRobotNode::shutdown, this));

        RCLCPP_INFO(this->get_logger(),
                "Cable Driven Node initialized");
    }

    // stop the motor (velocity zero) before exit
    void shutdown() {
        timer_->cancel();

        // stop motor at exit
        std_msgs::msg::Float64MultiArray msg;
        msg.data = { 0.0 };
        cmd_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                "Stopped motor");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void controlLoop() {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = { 0.5 };
        cmd_pub_->publish(msg);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->name.empty()) return;

        double currentPos = msg->position[0];
        RCLCPP_INFO(this->get_logger(), 
                "Joint pos: %+8.3f", currentPos);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CableRobotNode>();
    rclcpp::spin(node);
    return 0;
}
