#ifndef __PLANNER_H
#define __PLANNER_H

#include <ros/ros.h>
/*
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

class Planner{
    private:




    public:
        // constructor
        Planner();

        //methods
        void run();
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &);
        void odom_callback(const geometry_msgs::PoseStamped::ConstPtr &);
        

        // members
        ros::Subscriber scan_sub;
        ros::Subscriber odom_sub;

        //global vars
        bool scan_updated_ = false;
        bool pose_updated_ = false;
        double curr_vel_ = 0.0;

};


class Window{
    public:
        Window(const double, const double, const double, const double);
        double min_vel;
        double max_vel;
        double min_omega;
        double max_omega;
};
*/
#endif //__PLANNER_H