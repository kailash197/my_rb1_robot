#include "ros/duration.h"
#include "ros/ros.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <my_rb1_ros/Rotate.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class RotateRB1Robot {
public:
  RotateRB1Robot()
      : nh_(),
        cmd_vel_pub_(nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000)),
        odom_sub_(
            nh_.subscribe("/odom", 1000, &RotateRB1Robot::odomCallback, this)),
        service_(nh_.advertiseService("/rotate_robot",
                                      &RotateRB1Robot::serviceCallback, this)),
        current_yaw_(0.0) {
    ROS_INFO("Service Ready: /rotate_robot");
  }

  void spin() { ros::spin(); }

private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;
  ros::ServiceServer service_;
  double current_yaw_;

  // Function to convert degrees to radians
  double degreesToRadians(double degrees) { return degrees * (M_PI / 180.0); }

  // Function to normalize angle to the range -pi to pi
  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  bool serviceCallback(my_rb1_ros::Rotate::Request &req,
                       my_rb1_ros::Rotate::Response &res) {
    int degrees = req.degrees;
    ROS_INFO("Service Requested: %d degrees", degrees);
    double rotation_yaw = degreesToRadians(static_cast<double>(fabs(degrees)));
    ros::Rate rate(10); // Frequency of 10 Hz

    double stop_yaw = 0.0;
    // Determine direction (clockwise or counter-clockwise)
    double direction = (degrees < 0) ? -1.0 : 1.0;

    // Set target yaw based on direction
    if (degrees < 0) {
      stop_yaw = normalizeAngle(current_yaw_ - rotation_yaw);
    } else {
      stop_yaw = normalizeAngle(current_yaw_ + rotation_yaw);
    }

    geometry_msgs::Twist command;
    double error = fabs(normalizeAngle(current_yaw_ - stop_yaw));

    while (error >= 0.01) {
      ROS_DEBUG("Current: %f, Target: %f", current_yaw_, stop_yaw);

      // Adjust angular speed based on the error
      if (error > 0.5) {
        command.angular.z = 0.3 * direction;
      } else if (error > 0.4) {
        command.angular.z = 0.2 * direction;
      } else if (error > 0.3) {
        command.angular.z = 0.1 * direction;
      } else {
        command.angular.z = 0.05 * direction;
      }

      cmd_vel_pub_.publish(command);

      ros::spinOnce(); // Ensure callbacks are processed
      rate.sleep();    // Sleep to maintain the loop frequency

      // Update current_yaw and error
      error = fabs(normalizeAngle(current_yaw_ - stop_yaw));
    }

    // Stop the robot
    command.angular.z = 0.0;
    cmd_vel_pub_.publish(command);
    ROS_INFO("Service Completed.");
    res.result = "success";
    return true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);

    // Convert the quaternion to roll, pitch, and yaw (RPY)
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = normalizeAngle(yaw);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rb1_rotate_service_server");

  RotateRB1Robot rb1_robot;
  rb1_robot.spin();

  return 0;
}
