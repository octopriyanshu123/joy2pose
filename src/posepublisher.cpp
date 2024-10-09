#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include "/home/octobotics/ros2_ws/install/pubjoy2pose/include/pubjoy2pose/pubjoy2pose/srv/resetpose.hpp"

class PosePublisher : public rclcpp::Node {
public:
    PosePublisher() : Node("pose_publisher") {
        // Subscribe to joystick input
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&PosePublisher::joy_callback, this, std::placeholders::_1));

        // Publisher for the pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/pose", 10);
        
        // // Create the reset pose service
        // reset_pose_service_ = this->create_service<pubjoy2pose::srv::Resetpose>(
        //     "resetpose", std::bind(&PosePublisher::reset_pose_callback, this, std::placeholders::_1, std::placeholders::_2));

        // // Initialize pose values
        // x_ = y_ = z_ = 1.0;
        // roll_ = pitch_ = yaw_ = 0.0;

        // Timer to update the pose
        update_position_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PosePublisher::update_position_timer_callback, this)
        );
    }

private:
    float x_ = 0.0, y_ = 0.0, z_ = 0.0; // Initial position
    float roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0; // Orientation
    float button_value_X_PLUS_MINUS = 0.0;
    float button_value_Y_PLUS_MINUS = 0.0;
    float button_value_Z_PLUS_MINUS = 0.0;
    bool button_value_PITCH_PLUS = false, button_value_PITCH_MINUS = false;
    bool button_value_ROLL_PLUS = false, button_value_ROLL_MINUS = false;
    bool button_value_YAW_PLUS = false, button_value_YAW_MINUS = false;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::TimerBase::SharedPtr update_position_timer_;
    // rclcpp::Service<pubjoy2pose::srv::ResetPose>::SharedPtr reset_pose_service_; 

    void update_position_timer_callback() {
        // Update position based on joystick input
        x_ += button_value_X_PLUS_MINUS * 0.01; // Update X position
        y_ += button_value_Y_PLUS_MINUS * 0.01; // Update Y position
        z_ += button_value_Z_PLUS_MINUS * 0.01; // Update Z position

        // Create a Pose message
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = x_;
        pose_msg.position.y = y_;
        pose_msg.position.z = z_;

        // Set the orientation from Euler angles
        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();
        pose_msg.orientation.w = q.w();

        // Publish the updated pose
        pose_pub_->publish(pose_msg);
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        // Read joystick axes for position control
        button_value_X_PLUS_MINUS = joy_msg->axes[0]; // X-axis
        button_value_Y_PLUS_MINUS = joy_msg->axes[1]; // Y-axis
        button_value_Z_PLUS_MINUS = joy_msg->axes[3]; // Z-axis

        // Read joystick buttons for orientation control
        button_value_PITCH_PLUS = joy_msg->buttons[0]; // Button for pitch + (increase)
        button_value_PITCH_MINUS = joy_msg->buttons[2]; // Button for pitch - (decrease)
        button_value_ROLL_PLUS = joy_msg->buttons[1]; // Button for roll + (increase)
        button_value_ROLL_MINUS = joy_msg->buttons[3]; // Button for roll - (decrease)
        button_value_YAW_PLUS = joy_msg->buttons[4]; // Button for yaw + (increase)
        button_value_YAW_MINUS = joy_msg->buttons[5]; // Button for yaw - (decrease)

        // Update orientation based on button presses
        if (button_value_PITCH_PLUS) pitch_ += 0.1;
        if (button_value_PITCH_MINUS) pitch_ -= 0.1;
        if (button_value_ROLL_PLUS) roll_ += 0.1;
        if (button_value_ROLL_MINUS) roll_ -= 0.1;
        if (button_value_YAW_PLUS) yaw_ += 0.1;
        if (button_value_YAW_MINUS) yaw_ -= 0.1;
    }

    // void reset_pose_callback(const std::shared_ptr<your_package_name::srv::ResetPose::Request> request,
    //                          std::shared_ptr<your_package_name::srv::ResetPose::Response> response) {
    //     // Reset pose values to default
    //     x_ = y_ = z_ = 1.0;
    //     roll_ = pitch_ = yaw_ = 0.0;

    //     RCLCPP_INFO(this->get_logger(), "Pose values reset to default.");
    // }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePublisher>());
    rclcpp::shutdown();
    return 0;
}