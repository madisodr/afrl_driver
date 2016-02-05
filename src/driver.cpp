/**************************************************************************************
 * Author: Daniel R Madison
 * Last Updated: 04/2/2016
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "kobuki_msgs/BumperEvent.h"
#include <vector>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

#include "afrl_driver/driver.h"

using namespace std;
// Consstruct a new RandomWalk object and hook up this ROS node
// to the simulated robot's velocity control and laser topics
AFRL_Driver::AFRL_Driver(ros::NodeHandle& nh) {
    PIDTimer = nh.createTimer(ros::Duration(0.1), &AFRL_Driver::PID_control, this);
    commandPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); 
    laserSub = nh.subscribe("scan", 1, &AFRL_Driver::commandCallBack, this);
    bumperSub = nh.subscribe("/mobile_base/events/bumper", 1, &AFRL_Driver::bumperCallBack, this);

    // Populate the error vector with a bunch preset data;
    for(int i = 0; i < PID_VECTOR_SIZE; i++) { errors.push_back(0.0); }

    ros::spinOnce();
}

// Check the bumpers for objects and backup to turn around if encountered.
void AFRL_Driver::bumperCallBack(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
    if(msg->state == kobuki_msgs::BumperEvent::PRESSED) {
        switch(msg->bumper) {
            case kobuki_msgs::BumperEvent::LEFT:
                turnOdom(true, 25, true);
                break;
            case kobuki_msgs::BumperEvent::RIGHT:
                turnOdom(false, 25, true);
                break;
            case kobuki_msgs::BumperEvent::CENTER:
                turnOdom(true, 25, true);
                break;
        }
    }
}

// Send a velocity command
void AFRL_Driver::move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
}

// Process the incoming laser scan message
void AFRL_Driver::commandCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {

        this->rangesMin = 0;
        this->rangesMax = msg->ranges.size() - 1;

        double midDist = msg->ranges[this->rangesMax / 2]; // Half of max distance

        // Since the hokuyo sensor has a 180 degree range, we device the max range
        // by a number to get a degree offset. To get the number, just divide your 
        // sensors max angle by the degree you want to pick up from, then replace
        // the result into MSG_ANGLE_OFFSET
        double rightDist = msg->ranges[this->rangesMin + (this->rangesMax * MSG_RANGES_OFFSET / MSG_RANGES_ANGLE)];
        double leftDist = msg->ranges[this->rangesMax - (this->rangesMax * MSG_RANGES_OFFSET / MSG_RANGES_ANGLE)];

        // If our middle most distances is less then 0.5, then there is something right infront
        // of the bot and we should stop and turn.
        if(midDist < PROXIMITY_RANGE_MIN)
            turnOdom(true, 45, true);

        // if the left or the rihght distance from the laser scan goes to infinity, 
        // then we set this to the greatest scan range possible, otherwise it could
        // make the PID controller go funky. 
        if(isinf(rightDist)) rightDist = PROXIMITY_RANGE_MAX;
        if(isinf(leftDist)) leftDist = PROXIMITY_RANGE_MAX;

        if(leftDist == PROXIMITY_RANGE_MAX || rightDist == PROXIMITY_RANGE_MAX)
            ROS_INFO_STREAM("MAXED");

#ifdef DEBUG_MSG
        ROS_INFO_STREAM("\n\n[##############################################");
        ROS_INFO_STREAM("Left Dist: " << leftDist);
        ROS_INFO_STREAM("Right Dist: " << rightDist);
        ROS_INFO_STREAM("Middle Dist: " << midDist);
#endif

        // Throw out NANs. You may not want to use this if you use a sensor with a minimum range
        // such as the kinect. TODO add this as a toggleable condition. 
        if(!isnan(leftDist) && !isnan(rightDist)) {
            double e = leftDist - rightDist;
            errors.pop_back();
            errors.insert(errors.begin(), e);
        }
}


/* Method for turning the robot a certain distance, with the enableing of going
 * backwards and turning either clockwise or counter clockwise. 
 *
 * TODO: Make this the main movement method and remove void move(). Also take this
 * out of a sleeping loop and turn it into a callback method.
 */
bool AFRL_Driver::turnOdom(bool clockwise, double angle, bool backtrack) {
    bool done = false;
    double radians = angle * M_PI/180;
    
#ifdef DEBUG_MSG
    ROS_INFO_STREAM("Turning " << angle);
#endif

    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    listener_.waitForTransform("base_footprint", "odom", 
            ros::Time(0), ros::Duration(1.0));

    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    listener_.lookupTransform("base_footprint", "odom", 
            ros::Time(0), start_transform);

    geometry_msgs::Twist base_cmd;


    if(backtrack == true) base_cmd.linear.x = base_cmd.linear.y = REVERSE_SPEED_MPS;
    else base_cmd.linear.x = base_cmd.linear.y = 0.0;

    base_cmd.angular.z = ROTATE_SPEED_RADPS;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;

    tf::Vector3 desired_turn_axis(0,0,1);
    if (clockwise == false) desired_turn_axis = -desired_turn_axis;

    while (!done && ros::ok()) {
        commandPub.publish(base_cmd);

        listener_.lookupTransform("base_footprint", "odom", ros::Time(0), current_transform);
        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();

        double angle_turned = relative_transform.getRotation().getAngle();

        if ( fabs(angle_turned) < 1.0e-2) continue;

        if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
            angle_turned = 2 * M_PI - angle_turned;

        if (angle_turned > radians) return true;
    }

    return false;
}

/* Method for quick summation of the errors vector. */
double AFRL_Driver::summation() {
    double tot = 0;
    for(int i = 0; i < PID_VECTOR_SIZE; i++)
        tot += errors[i];

    return tot;
}

/* Method for calculating all the gains for the PID controller and based on the resulting error,
 * determine the control variable of where the robot needs to adjust.
 */
void  AFRL_Driver::PID_control(const ros::TimerEvent& e) {
    double previousError = errors[0] * Kp;
    double integral = summation() * Ki;
    double derivative = (errors[0] - errors[1]) * Kd;

    // Calculate error of the PID controller.
    error = derivative + previousError + integral;

    // Based on the calculated error in relation to the threshold, decide
    // whether to increase, decrease, or reset our control. Resetting the control
    // happens when the error's sign is suddenly flipped. The control is also
    // limited to be between a max rotation speed, else it begins spinning to quickly
    // and can't recover. 
    if (error > CONTROL_THRESHOLD && control < ROTATE_SPEED_MAX) {
        if(pError < 0) 
            control = 0; // reset our control
        else
            control = control + ROTATE_CONTROL_STEP; // 
    } else if (error < CONTROL_THRESHOLD && control > (-1*ROTATE_SPEED_MAX)) {
        if(pError > 0) 
            control = 0; // reset our control.
        else
            control = control - ROTATE_CONTROL_STEP; // lower the control.
    } else if(fabs(error < CONTROL_THRESHOLD)) {
        control = 0; // If the error is between a threshhold, then set our control to 0 and go forward.
    }

    // Store the previous total error. Used for when the error suddenly flips 
    // signs and the control needs to be reset to the base of 0.
    pError = error;
#ifdef DEBUG_MSG
    ROS_INFO_STREAM("Error: " << error);
    ROS_INFO_STREAM("Control: " << control);
#endif
}

/*  Main spin method. Drives the robot forward by default and activates the PID controller. 
 *  Various triggers in the call backs will pause the loop and have the robot rotate or 
 *  reverse its direction.
 */
void  AFRL_Driver::spin() {
    ros::Rate rate(20); // Specify the looprate in Hz

    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
        move(FORWARD_SPEED_MPS, control);
        ros::spinOnce(); // Need to call this function often to allow ROS to process incomingmessages
        rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "AFRL_Driver");
    ros::NodeHandle n;
    AFRL_Driver walker(n); 
    walker.spin(); 
    return 0;
}
