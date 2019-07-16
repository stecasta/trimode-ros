#include <ros/ros.h>
#include <ros/time.h>
#include <trimode_control/FloatList.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>

double w;

void messageCb( const trimode_control::FloatList & msg) {

w = msg.data[1];

}

int main(int argc, char** argv){
  ros::init(argc, argv, "transf_odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("encoder_vel_transf", 1000, messageCb);
//  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);

  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_w = w * dt;

    th += delta_w;

    ROS_WARN("theta: %3f", th);

    //publish the message
//    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
