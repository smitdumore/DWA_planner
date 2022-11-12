#ifndef __PLANNER_H
#define __PLANNER_H

#include <ros/ros.h>
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
#include "dwa_planner/state.h"
#include <math.h>

class Window{
    public:
        double min_vel;
        double max_vel;
        double min_omega;
        double max_omega;

        //constructor
        Window(double min_vel, double max_vel, double min_omega, double max_omega){
            this->min_vel = min_vel;
            this->max_vel = max_vel;
            this->min_omega = min_omega;
            this->max_omega = max_omega;
        }
        
};

class Planner{
    private:
        ros::NodeHandle nh_;
        double TARGET_VELOCITY;
        double MAX_VELOCITY;
        double MIN_VELOCITY;
        double MAX_OMEGA;
        double MAX_ACCELERATION;
        double MAX_ANG_ACCELERATION;
        //double MAX_DIST;
        double VELOCITY_RES;
        double OMEGA_RES;
        //double ANGLE_RESOLUTION;
        double SIM_TIME;
        double TO_GOAL_COST_GAIN;
        double SPEED_COST_GAIN;
        double OBSTACLE_COST_GAIN;
        double DT;
        double HZ;
        double GOAL_THRESHOLD;
        double GOAL_X;
        double GOAL_Y;
        //double TURN_DIRECTION_THRESHOLD;


    public:
        // constructor
        Planner(ros::NodeHandle &nh);

        //methods
        void run();
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &);
        void odom_callback(const nav_msgs::Odometry::ConstPtr &);
        std::vector<State> best_dwa_selection(const Window &,const Eigen::Vector3d &);
        Window get_window();
        void show_dwa_trajectories(const std::vector<std::vector<State>> &);
        void simulate_dynamics(State &s, double , double );
        void visualize_trajectories(const std::vector<std::vector<State>>& );
        double get_goal_cost(const std::vector<State> &, const Eigen::Vector3d &);
        double get_speed_cost(const std::vector<State> &, const double );
        void show_best_trajectory(const std::vector<State> &);
        double get_obstacle_cost(const std::vector<State> &, const std::vector<std::vector<double>> &);
        void show_goal(Eigen::Vector3d &);
        void show_robot_path();

        // members
        ros::Subscriber scan_sub;
        ros::Subscriber odom_sub;
        ros::Publisher trajectories_viz_pub;
        ros::Publisher best_traj_viz_pub;
        ros::Publisher goal_vis_pub;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher path_viz_pub;

        //global vars
        bool scan_updated_ = false;
        bool pose_updated_ = false;
        geometry_msgs::Twist curr_vel_;
        geometry_msgs::PoseWithCovariance curr_pose_;
        int count_ = INT_MIN;
        bool dwa_converged_ = false;
        std::vector<std::vector<double>> local_obstacles_;
        tf2_ros::TransformListener tf2_listener_;
        tf2_ros::Buffer tf_buffer_;
        geometry_msgs::TransformStamped tf_base_to_odom_;
        //tf::TransformListener listener_;
};

#endif //__PLANNER_H
