#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <array>
#include <algorithm>

struct PID
{
    double kp, ki, kd;
    double prev_error = 0.0;
    double integral = 0.0;

    double update(double error, double dt)
    {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

class FlightController : public rclcpp::Node
{
public:
    FlightController() : Node("flight_controller")
    {
        motor_pub_ = gz_node_.Advertise<gz::msgs::Actuators>("/x500/command/motor_speed");
        if (!motor_pub_)
            RCLCPP_ERROR(this->get_logger(), "Failed to create Gazebo motor publisher.");

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/x500/imu", 10,
            std::bind(&FlightController::imu_callback, this, std::placeholders::_1));

        pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>("/x500/air_pressure", 10,
            std::bind(&FlightController::pressure_callback, this, std::placeholders::_1));

        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&FlightController::control_loop, this));

        last_time_ = this->now();

        pid_roll_ = {6.0, 0.0, 0.3};
        pid_pitch_ = {6.0, 0.0, 0.3};
        pid_yaw_ = {1.0, 0.0, 0.1};
        pid_altitude_ = {50.0, 0.0, 5.0};

        target_altitude_ = 2.0;   // meters
        hover_speed_ = 700.0;     // baseline throttle for hover
        max_motor_speed_ = 1000.0;
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_imu_ = *msg;
        /*
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "IMU Accel: [%.2f %.2f %.2f]",
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);
        */
    }

    void pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
    {
        latest_pressure_ = *msg;
        /*
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Pressure: %.2f Pa", msg->fluid_pressure);
        */
    }

    double estimate_altitude()
    {
        const double P0 = 101325.0;   // Sea level standard pressure (Pa)
        const double T0 = 288.15;     // Standard temperature (K)
        const double g = 9.80665;     // Acceleration due to gravity (m/s^2)
        const double L = 0.0065;      // Temperature lapse rate (K/m)
        const double R = 8.31447;     // Gas constant
        const double M = 0.0289644;   // Molar mass of Earth's air (kg/mol)

        if (latest_pressure_.fluid_pressure <= 0.0)
            return 0.0;

        double ratio = latest_pressure_.fluid_pressure / P0;
        return (T0 / L) * (1.0 - std::pow(ratio, (R * L) / (g * M)));
    }

    void control_loop()
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0)
            dt = 0.01;
        last_time_ = current_time;

        // Get orientation from IMU quaternion
        tf2::Quaternion q(
            latest_imu_.orientation.x,
            latest_imu_.orientation.y,
            latest_imu_.orientation.z,
            latest_imu_.orientation.w);
        tf2::Matrix3x3 m(q);
        double current_roll, current_pitch, current_yaw;
        m.getRPY(current_roll, current_pitch, current_yaw);

        double current_altitude = estimate_altitude();

        double desired_roll = 0.0;
        double desired_pitch = 0.0;
        double desired_yaw = 0.0;  // No Rudder
        double desired_altitude = target_altitude_;

        double error_roll = desired_roll - current_roll;
        double error_pitch = desired_pitch - current_pitch;
        double error_yaw = desired_yaw - current_yaw;
        double error_altitude = desired_altitude - current_altitude;

        double control_roll = pid_roll_.update(error_roll, dt);
        double control_pitch = pid_pitch_.update(error_pitch, dt);
        double control_yaw = pid_yaw_.update(error_yaw, dt);
        double control_altitude = pid_altitude_.update(error_altitude, dt);

        double base_throttle = hover_speed_ + control_altitude;

        // Motor mixing for X-configuration quadrotor
        std::array<double, 4> motor_speeds;
        motor_speeds[0] = base_throttle - control_roll - control_pitch + control_yaw; // Front right
        motor_speeds[1] = base_throttle + control_roll - control_pitch - control_yaw; // Front left
        motor_speeds[2] = base_throttle + control_roll + control_pitch + control_yaw; // Rear left
        motor_speeds[3] = base_throttle - control_roll + control_pitch - control_yaw; // Rear right

        for (auto& speed : motor_speeds)
            speed = std::clamp(speed, 0.0, max_motor_speed_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Alt: %.2f Err: %.2f | RPY: [%.2f, %.2f, %.2f] | Motor0: %.1f",
            current_altitude, error_altitude, current_roll, current_pitch, current_yaw, motor_speeds[0]);

        gz::msgs::Actuators msg;
        for (const auto& speed : motor_speeds)
            msg.add_velocity(speed);

        if (!motor_pub_.Publish(msg))
            RCLCPP_WARN(this->get_logger(), "Failed to publish motor speeds.");
    }

    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher motor_pub_;

    sensor_msgs::msg::Imu latest_imu_;
    sensor_msgs::msg::FluidPressure latest_pressure_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    PID pid_roll_;
    PID pid_pitch_;
    PID pid_yaw_;
    PID pid_altitude_;
    rclcpp::Time last_time_;
    double target_altitude_;
    double hover_speed_;
    double max_motor_speed_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightController>());
    rclcpp::shutdown();
    return 0;
}
