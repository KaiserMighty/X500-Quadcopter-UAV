#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

class MotorController : public rclcpp::Node
{
public:
    MotorController() : Node("motor_controller")
    {
        motor_pub_ = gz_node_.Advertise<gz::msgs::Actuators>("/x500/command/motor_speed");

        if (!motor_pub_)
            RCLCPP_ERROR(this->get_logger(), "Failed to create Gazebo publisher.");
        else
            RCLCPP_INFO(this->get_logger(), "Gazebo publisher created successfully.");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MotorController::publish_motor_speeds, this));
    }

private:
    void publish_motor_speeds()
    {
        gz::msgs::Actuators msg;
        msg.add_velocity(500.0);
        msg.add_velocity(500.0);
        msg.add_velocity(500.0);
        msg.add_velocity(500.0);

        bool result = motor_pub_.Publish(msg);
        if (!result)
            RCLCPP_WARN(this->get_logger(), "Failed to publish motor speeds");
    }

    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::sleep_for(std::chrono::seconds(10));
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}
