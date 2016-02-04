/****************************************
 * Author: Daniel R. Madison
 * Last Updated: Feb 1st 2016
 *
 * Public Liscense: 
 *
 ***************************************/

#ifndef AFRL_DRIVER_H
#define AFRL_DRIVER_H

// PID Controller and tuning paramaters
static const int PID_VECTOR_SIZE = 10; // How big the error vector should be.
const static double Kp = 0.8; // Proportional gain
const static double Kd = 0.5; // Derivative gain
const static double Ki = 0.005; // Integral gain
const static double CONTROL_THRESHOLD = 0.1;

enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
const static double MIN_SCAN_ANGLE_RAD = -1.57079637051;
const static double MAX_SCAN_ANGLE_RAD = 1.57079637051;        
const static double PROXIMITY_RANGE_M = 5.5; // Should be smaller than   sensor_msgs::LaserScan::range_max

const static double FORWARD_SPEED_MPS = 0.2;
const static double ROTATE_SPEED_RADPS = 0.3;
const static double ROTATE_SPEED_MAX = 1;
const static double ROTATE_CONTROL_STEP = 0.1;

const static int MSG_RANGES_ANGLE = 180;
const static int MSG_RANGES_OFFSET = 25;
const static int MSG_RANGES_MAX = 509;
const static int MSG_RANGES_MIN = 0;

using namespace std;

class AFRL_Driver {
    public:
        // Constructor and Destructor
        AFRL_Driver(ros::NodeHandle& nh);

        // Callback Functions
        void bumperCallBack(const kobuki_msgs::BumperEvent::ConstPtr& msg);
        void commandCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);

        // Movement Functions
        bool turnOdom(bool clockwise, double radians, bool backtrack = false);
        void move(double linearVelMPS, double angularVelRadPS);

        // PID Functions and Utilitys
        void PID_control(const ros::TimerEvent& e);
        double summation();
        
        // ROS Functions
        void spin();


    private:
        int bumperState;
        double control;
        double error;
        double pError;
        vector<double> errors;

    protected:
        ros::Publisher commandPub; // Publisher to the robot's velocity command topic
        ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
        ros::Subscriber bumperSub; // Subscriber to the robot's bump sensors topic.
        tf::TransformListener listener_; // Listener for the transform topic

        enum FSM fsm; // Finite state machine for the random walk algorithm
        ros::Time rotateStartTime; // Start time of the rotation
        ros::Duration rotateDuration; // Duration of the rotation
        ros::Timer PIDTimer;

};

#endif

