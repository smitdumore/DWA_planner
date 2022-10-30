#include "dwa_planner/planner.h"
#include "dwa_planner/state.h"


Planner::Planner():nh("~"){

    scan_sub = nh.subscribe("/scan", 1, &Planner::scan_callback, this);
    odom_sub = nh.subscribe("/odom", 10, &Planner::pose_callback, this);
}

Planner::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
    scan_updated_ = true;
}

Planner::odom_callback(const geometry_msgs::PoseStamped::ConstPtr &odom_msg){
    pose_updated_ = true;

    curr_pose_ = odom_msg->pose.pose; 
    curr_vel_ = odom_msg->twist.twist;
}

Planner::get_window(const geometry_msgs::Twist& curr_vel){

    Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_OMEGA, MAX_OMEGA);

    window.min_vel = std::max((curr_vel.linear.x - MAX_ACCELERATION*DT), MIN_VELOCITY);
    window.max_vel = std::min((curr_vel.linear.x + MAX_ACCELERATION*DT), MAX_VELOCITY);
    window.min_omega = std::max((current_vel.angular.z - MAX_D_OMEGA*DT), -MAX_OMEGA);
    window.max_omega = std::min((current_vel.angular.z + MAX_D_OMEGA*DT),  MAX_OMEGA);
    /// MAX D YAWTRATE ??

    return window;
}

Planner::best_dwa_selection(const Window &window,const Eigen::Vector3d &goal){

    //change names
    double min_cost = INT32_MIN;
    double min_obs_cost = min_cost;
    double min_goal_cost = min_cost;
    double min_speed_cost = min_cost;

    std::vector<std::vector<State>> trajectory_table;
    std::vector<State> best_trajectory;

    for(double v=window.min_vel; v <= window.max_vel; v+=VELOCITY_RES){
        for(double w=window.min_omega; w<=window.max_omega; w+=OMEGA_RES){
            // CURR_POSE IS ZERO ?????
            State state(0.0, 0.0, 0.0, curr_vel_.linear.x, curr_vel_.angular.z);
            std::vector<State> temp_traj;
            for(double t=0; t<=SIM_TIME; t+=DT){
                simulate_dynamics(state, v, w);
                temp_traj.push_back(state);
            }
            trajectory_table.push_back(temp_traj);
            double cost_on_goal = get_goal_cost(traj, goal);
            double cost_on_speed = get_speed_cost(traj, TARGET_VELOCITY);
            double cost_on_obs = get_obstacle_cost(traj, obs_list);
            //double cumulative_cost = TO_GOAL_COST_GAIN*to_goal_cost + SPEED_COST_GAIN*speed_cost + OBSTACLE_COST_GAIN*obstacle_cost;

}

Planner::get_goal_cost(const vector<State> &trajectory, const Eigen::Vector3d &goa){
    //getting end-point of the trajectory
    Eigen::Vector3d last_position(trajectory.back().x, trajectory.back().y, trajectory.back().omega);

    //WHAT IS NORM ???????
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

Planner::run(){
    if(!scan_updated_){
        ROS_ERROR("Waiting for scan ...");
        return;
    }

    if(!pose_updated_){
        ROS_ERROR("Waiting for odom ...");
        return;
    }

    Window dynamic_window = get_window(curr_vel_);
    // GOAL x,y,heading in robot frame
    Eigen::Vector3d goal(10.0, 0.0, 0.0);

    std::vector<State> best_trajectory = best_dwa_selection(dynamic_window, goal);


}