#include <ros/ros.h>
#include <ros/time.h>
#include <trimode_control/FloatList.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <boost/assign.hpp>

double left_back_velocity = 0, right_back_velocity = 0, left_front_velocity = 0, right_front_velocity = 0;

void back_messageCb( const trimode_control::FloatList & msg) {

    left_back_velocity = msg.data[1];
    right_back_velocity = msg.data[3];
}

void front_messageCb( const trimode_control::FloatList & msg) {

    left_front_velocity = msg.data[0];
    right_front_velocity = msg.data[2];
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("encoder_back_vel", 100, back_messageCb);
  ros::Subscriber sub2 = n.subscribe("encoder_front_vel", 100, front_messageCb);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/trimode_velocity_controller/odom", 100);
  tf::TransformBroadcaster odom_broadcaster;

  double wheel_base = 0.565;
  double wheel_radius = 0.12;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    //ROS_WARN("vx_loop: %3f", vx);

  // Average from the two wheels on the same side, is there a better way?
//    double omega_left = (left_back_velocity + left_front_velocity) / 2;
//    double omega_right = (right_back_velocity + right_front_velocity) / 2;
    double omega_left = left_front_velocity;
    double omega_right = right_front_velocity;
//    ROS_ERROR("omega_left: %f", omega_left);
//    ROS_ERROR("omega_right: %f", omega_right);
    double v_left = omega_left * wheel_radius;
    double v_right = omega_right * wheel_radius;

    double vx = ((v_right + v_left) / 2);
    double vy = 0;
    double vth = ((v_right - v_left) / wheel_base);

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    // no need, we use odometry_filtered
//    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set position covariance
    odom.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                         (0) (1e-3)  (0)  (0)  (0)  (0)
                                                         (0)   (0)  (1e6) (0)  (0)  (0)
                                                         (0)   (0)   (0) (1e6) (0)  (0)
                                                         (0)   (0)   (0)  (0) (1e6) (0)
                                                         (0)   (0)   (0)  (0)  (0)  (0.1) ;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //set velocity covariance
    odom.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                                                        (0) (1e-3)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (1e6) (0)  (0)  (0)
                                                        (0)   (0)   (0) (1e6) (0)  (0)
                                                        (0)   (0)   (0)  (0) (1e6) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (0.1) ;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
