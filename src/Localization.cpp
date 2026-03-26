#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";

    odometry_.pose.pose.position.x = 0.0;
    odometry_.pose.pose.position.y = 0.0;
    odometry_.pose.pose.position.z = 0.0;

    odometry_.pose.pose.orientation.x = 0.0;
    odometry_.pose.pose.orientation.y = 0.0;
    odometry_.pose.pose.orientation.z = 0.0;
    odometry_.pose.pose.orientation.w = 1.0;

    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 
        10, 
        std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
    );

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

    // tf_broadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    if (msg.velocity.size() < 2) return; 

    if (std::isnan(msg.velocity[0]) || std::isnan(msg.velocity[1])) return;
    
    auto current_time = this->get_clock()->now();
    
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    
    if (dt <= 0.0 || dt > 1.0) return; 
    

    // msg.velocity[0] je levé a msg.velocity[1] pravé kolo
    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    
    publishOdometry();
    publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    double linear = (robot_config::WHEEL_RADIUS / 2.0) * (left_wheel_vel + right_wheel_vel);
    double angular = (robot_config::WHEEL_RADIUS / 2.0 / robot_config::HALF_DISTANCE_BETWEEN_WHEELS) * (left_wheel_vel - right_wheel_vel);

    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = angular;


    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    double delta_x = linear * std::cos(theta) * dt;
    double delta_y = linear * std::sin(theta) * dt;
    double delta_theta = angular * dt;

    odometry_.pose.pose.position.x += delta_x;
    odometry_.pose.pose.position.y += delta_y;
    theta += delta_theta;

    theta = std::atan2(std::sin(theta), std::cos(theta));

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);


}

void LocalizationNode::publishOdometry() {
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = odometry_.header.frame_id;
    t.child_frame_id = odometry_.child_frame_id;

    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0;

    t.transform.rotation = odometry_.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
}
