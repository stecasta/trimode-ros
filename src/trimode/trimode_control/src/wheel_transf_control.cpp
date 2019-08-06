#include <ros/ros.h>
#include <ros/time.h>
#include <trimode_control/FloatList.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
//#include <trimode_control/BoolList.h>
#include <trimode_control/TransformWheels.h>

/*
 * 855 pulses from claw to round
 * 2148 pulses from round to arc
 */

// To do: add checks if you loose connection

const int PWM_VALUE = 100;

//const int CLAW2ROUND = 855;
//const int CLAW2ARC = 2148;

const int CLAW2ROUND[4] = {726, 421, 350, 420};
const int CLAW2ARC[4] = {2026, 1061, 1070, 1165};

float pulses[4] = {0};
int command_ = 0;
//long int position = 0;
trimode_control::FloatList pwm;
bool home_registered = false;
bool single_home_registered[4] = {0};
bool home_[4] = {0};
bool msg_received = false;

// I think I can leave this global varibales!?

class Robot {
public:
    Robot(ros::NodeHandle& n) {

        sub_right = n.subscribe("encoder_right_vel_transf", 1000, &Robot::right_messageCb, this);
        sub_left = n.subscribe("encoder_left_vel_transf", 1000, &Robot::left_messageCb, this);

        sub_joy = n.subscribe("joy", 1000, &Robot::Joy_messageCb, this);
        sub_home_right = n.subscribe("home_right_position", 1000, &Robot::home_right_messageCb, this);
        sub_home_left = n.subscribe("home_left_position", 1000, &Robot::home_left_messageCb, this);

        pwm_pub = n.advertise<trimode_control::FloatList>("pwms_wheel", 100, this);
        service = n.advertiseService("transform_wheels", &Robot::transform_wheels_srv, this);

        ros::Time current_time, last_time;
        current_time = ros::Time::now();
        last_time = ros::Time::now();
    }

    bool transform_wheels_srv(trimode_control::TransformWheels::Request  &req,
                              trimode_control::TransformWheels::Response &res){

        ROS_INFO("request: command = %ld", (long int)req.command);

        if (msg_received && home_registered){
            transform_wheels_(req.command);
            res.done = true;
        }
        else{
            res.done = false;
        }
        ROS_INFO("sending back response: [%ld]", (long int)res.done);
        return true;
    }

    void transform_wheels_(int command){

        ros::Rate r(20);

        bool transformation_done = false;

        while (!transformation_done){
            // you are not checking for the command anymore!!
            // Initialize with zero values.
            pwm.data[0] = 0;
            pwm.data[1] = 0;
            pwm.data[2] = 0;
            pwm.data[3] = 0;

            ros::spinOnce();               // check for incoming messages

            int sum = 0;
            for (int idx = 0; idx < 4; idx++){
                if (command == 1){
                    if (pulses[idx] >= -CLAW2ROUND[idx]){
                        pwm.data[idx] = PWM_VALUE;
                    }
                    else sum++;
                }
                if (command == -1){
                    if (pulses[idx] <= 0){
                        pwm.data[idx] = -PWM_VALUE;
                    }
                    else sum++;
                }
                if (command == 2){
                    if(pulses[idx] >= -CLAW2ARC[idx]){
                        pwm.data[idx] = PWM_VALUE;
                    }
                    else sum++;
                }
                if (command == -2){
                    if(pulses[idx] <= -CLAW2ROUND[idx]){
                        pwm.data[idx] = -PWM_VALUE;
                    }
                    else sum++;
                }
            }
            if (sum == 4) transformation_done = true;

            pwm_pub.publish(pwm);
            r.sleep();
        }
    }

    void right_messageCb( const trimode_control::FloatList & msg) {

        msg_received = true;

        pulses[2] = msg.data[2];
        pulses[3] = msg.data[3];

        // Initializing pwm msg. CHANGE THIS LIKE ARDUINO
        pwm = msg;
        pwm.data[0] = 0;
        pwm.data[1] = 0;
        pwm.data[2] = 0;
        pwm.data[3] = 0;

    }

    void left_messageCb( const trimode_control::FloatList & msg) {
        //ADD CONTROL
        //    msg_received = true;

        pulses[0] = msg.data[0];
        pulses[1] = msg.data[1];
    }

    void home_right_messageCb( const trimode_control::FloatList & home_msg) {
        home_[2] = home_msg.data[2];
        home_[3] = home_msg.data[3];
    }
    void home_left_messageCb( const trimode_control::FloatList & home_msg) {

        home_[0] = home_msg.data[0];
        home_[1] = home_msg.data[1];
    }

    void Joy_messageCb( const sensor_msgs::Joy & joy_msg) {

        command_ = 0;
        if (joy_msg.buttons[1] == 1 && joy_msg.buttons[7] == 1){
            command_ = 1;  // From claw to round.
        }
        else if ((joy_msg.buttons[1] == 1 && joy_msg.buttons[6] == 1)){
            command_ = -1;  // Transform to claw mode.
        }
        else if ((joy_msg.buttons[3] == 1 && joy_msg.buttons[7] == 1)){
            command_ = 2;  // Transform in arc mode.
        }
        else if ((joy_msg.buttons[3] == 1 && joy_msg.buttons[6] == 1)){
            command_ = -2;  // From arc to round.
        }
    }

    void run() {

        ros::Rate r(20);
        while (ros::ok)
        {
            ros::spinOnce();               // check for incoming messages
            //pwm.data[idx] = 0;    //initialize at start of node

            //          current_time = ros::Time::now();
            //          double dt = (current_time - last_time).toSec();

            //add check on transf encoder msg too!!
            if (msg_received){
                // Move wheel motor to home position on initialize.
                if (!home_registered){
                    int sum_of_home = 0;
                    for (int idx = 0; idx < 4; idx++){
                        if(!single_home_registered[idx]){
                            if(!home_[idx]){
                                pwm.data[idx] = -PWM_VALUE;
                            }
                            else {single_home_registered[idx] = true;
                            }
                        }
                        sum_of_home += single_home_registered[idx];
                    }
                    if(sum_of_home == 4){
                        home_registered = true;
                    }
                }
                else{
                    transform_wheels_(command_);
                }
            }

            //publish the message
            pwm_pub.publish(pwm);

            //          last_time = current_time;
            r.sleep();
        }
        //    }
    }
protected:

    ros::Subscriber sub_right;
    ros::Subscriber sub_left;

    ros::Subscriber sub_joy;
    ros::Subscriber sub_home_right;
    ros::Subscriber sub_home_left;

    ros::Publisher pwm_pub;
    ros::ServiceServer service;
};

int main(int argc, char** argv){

    ros::init(argc, argv, "transf_publisher");
    ros::NodeHandle n;
    Robot robot(n);
    robot.run();
}
