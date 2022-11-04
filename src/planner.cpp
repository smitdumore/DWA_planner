#include "dwa_planner/planner.h"

Planner::Planner(ros::NodeHandle &nh):nh_(nh){

    scan_sub = nh_.subscribe("/scan", 1, &Planner::scan_callback, this);
    odom_sub = nh_.subscribe("/odom", 10, &Planner::odom_callback, this);
    trajectories_viz_pub = nh_.advertise<visualization_msgs::MarkerArray>("/dwa_trajs", 10);
    best_traj_viz_pub = nh_.advertise<visualization_msgs::MarkerArray>("/best_traj", 10);

    nh_.param("HZ", HZ, 20.0);
    nh_.param("TARGET_VELOCITY", TARGET_VELOCITY, 0.8);
    nh_.param("MAX_VELOCITY", MAX_VELOCITY, 1.0);
    nh_.param("MIN_VELOCITY", MIN_VELOCITY, 0.0);
    nh_.param("MAX_OMEGA", MAX_OMEGA, 0.8);
    nh_.param("MAX_ACCELERATION", MAX_ACCELERATION, 1.0);
    nh_.param("MAX_ANG_ACCELERATION", MAX_ANG_ACCELERATION, 2.0);
    //local_nh.param("MAX_DIST", MAX_DIST, {10.0});
    nh_.param("VELOCITY_RES", VELOCITY_RES, 0.05);
    nh_.param("OMEGA_RES", OMEGA_RES, 0.05);
    //local_nh.param("ANGLE_RESOLUTION", ANGLE_RESOLUTION, {0.2});
    nh_.param("SIM_TIME", SIM_TIME, 3.0);
    nh_.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, 1.0);
    nh_.param("SPEED_COST_GAIN", SPEED_COST_GAIN, 1.0);
    //local_nh.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, {1.0});
    //local_nh.param("GOAL_THRESHOLD", GOAL_THRESHOLD, {0.3});
    //local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {1.0});
    DT = 1.0 / HZ;
}

void Planner::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
    scan_updated_ = true;
    //ROS_INFO("scan coming");
}


void Planner::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg){
    pose_updated_ = true;

    curr_pose_ = odom_msg->pose;
    curr_vel_ = odom_msg->twist.twist;

    /********************/
    curr_vel_.linear.x = 0.2;
    curr_vel_.angular.z = 0.2;
    /********************/
     
}

Window Planner::get_window(const geometry_msgs::Twist& curr_vel){

    Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_OMEGA, MAX_OMEGA);

    window.min_vel = std::max((curr_vel.linear.x - MAX_ACCELERATION*DT), MIN_VELOCITY);
    window.max_vel = std::min((curr_vel.linear.x + MAX_ACCELERATION*DT), MAX_VELOCITY);
    window.min_omega = std::max((curr_vel.angular.z - MAX_ANG_ACCELERATION*DT), -MAX_OMEGA);
    window.max_omega = std::min((curr_vel.angular.z + MAX_ANG_ACCELERATION*DT),  MAX_OMEGA);

    return window;
}

std::vector<State> Planner::best_dwa_selection(const Window &window,const Eigen::Vector3d &goal){

    dwa_converged_ = false;
    double min_cost = INT_MAX;

    //double min_obs_cost = min_cost;
    //double min_goal_cost = min_cost;
    //double min_speed_cost = min_cost;

    std::vector<std::vector<State>> trajectory_table;
    std::vector<State> best_trajectory;

    
    for(double v=window.min_vel; v <= window.max_vel; v+=VELOCITY_RES){
        for(double w=window.min_omega; w<=window.max_omega; w+=OMEGA_RES){
             
            State state(0.0, 0.0, 0.0, curr_vel_.linear.x, curr_vel_.angular.z);
            std::vector<State> temp_traj;
            for(double t=0.0; t<=SIM_TIME; t+=DT){
                simulate_dynamics(state, v, w);
                temp_traj.push_back(state);
            }
            trajectory_table.push_back(temp_traj);

            double cost_on_goal = get_goal_cost(temp_traj, goal);
            double cost_on_speed = get_speed_cost(temp_traj, TARGET_VELOCITY);
            //double cost_on_obs = get_obstacle_cost(traj, obs_list);

            double cumulative_cost = TO_GOAL_COST_GAIN*cost_on_goal + SPEED_COST_GAIN*cost_on_speed;// + OBSTACLE_COST_GAIN*obstacle_cost;

            if(min_cost >= cumulative_cost){
                //min_goal_cost = TO_GOAL_COST_GAIN*cost_on_goal;
                //min_speed_cost = SPEED_COST_GAIN*cost_on_speed;
                min_cost = cumulative_cost;
                best_trajectory = temp_traj;
                dwa_converged_ = true;
            }
        }

    }

    //trajectory table populated
    show_dwa_trajectories(trajectory_table);

    if(min_cost == 1e6){
        std::vector<State> traj;
        State state(0.0, 0.0, 0.0, curr_vel_.linear.x, curr_vel_.angular.z);
        traj.push_back(state);
        dwa_converged_  = false;
        best_trajectory = traj;
    }


    return best_trajectory;

}

void Planner::show_dwa_trajectories(const std::vector<std::vector<State>> &trajectory_table) {

    visualization_msgs::MarkerArray traj_list;
    visualization_msgs::Marker traj;
    geometry_msgs::Point p;

    traj.header.frame_id = "/base_footprint";
    traj.header.stamp = ros::Time::now();

    traj.id = 0;
    traj.type = visualization_msgs::Marker::LINE_STRIP;
    //traj.action = visualization_msgs::Marker::ADD;

    traj.pose.position.x = 0.0;//curr_pose_.pose.position.x;
    traj.pose.position.y = 0.0;//curr_pose_.pose.position.y;
    traj.pose.position.z = 0;
    traj.pose.orientation.x = 0.0;//curr_pose_.pose.orientation.x;
    traj.pose.orientation.y = 0.0;//curr_pose_.pose.orientation.y;
    traj.pose.orientation.z = 0.0;//curr_pose_.pose.orientation.z;
    traj.pose.orientation.w = 1.0;//curr_pose_.pose.orientation.w;

    traj.scale.x = traj.scale.y = traj.scale.z = 0.005;
    
    traj.color.r = 1.0;
    traj.color.a = 1.0;

    //ROS_INFO("publishing traj viz");
    //ROS_INFO("size table : %d", trajectory_table.size());
    //ROS_INFO("traj size : %d", trajectory_table[0].size());
    
    for (int i=0; i< trajectory_table.size(); i++) {

        traj.id += 1;
        traj.color.b += 0.1;
        traj.points.clear();
        
        for (int j = 0; j < trajectory_table[i].size(); j++) {
            p.x = trajectory_table[i][j].x;
            p.y = trajectory_table[i][j].y;
            traj.points.push_back(p);
        }
        traj_list.markers.push_back(traj);
    }
    //std::cout << "vis pubbing" << "\n";
    trajectories_viz_pub.publish(traj_list);
    
}

void Planner::show_best_trajectory(const std::vector<State> &best_trajectory) {

    visualization_msgs::MarkerArray best_list;
    visualization_msgs::Marker best_traj;
    geometry_msgs::Point point;

    best_traj.header.frame_id = "base_footprint";
    best_traj.id = 1;
    best_traj.type = visualization_msgs::Marker::LINE_STRIP;
    best_traj.scale.x = best_traj.scale.y = 0.01;
    best_traj.action = visualization_msgs::Marker::ADD;
    best_traj.pose.orientation.w = 1.0;
    best_traj.color.g = 1.0;
    best_traj.color.a = 1.0;

    
    for(int i=0; i< best_trajectory.size(); i++) {
        point.x = best_trajectory[i].x;
        point.y = best_trajectory[i].y;
        best_traj.points.push_back(point);
    }
    best_list.markers.push_back(best_traj);

    best_traj_viz_pub.publish(best_list);           //publish best trajectory
}

void Planner::simulate_dynamics(State &state, double velocity, double omega){
    state.heading += omega*DT;
    state.x += velocity*cos(state.heading)*DT;
    state.y += velocity*sin(state.heading)*DT;
    state.velocity = velocity;
    state.omega = omega;
}


double Planner::get_goal_cost(const std::vector<State> &current_trajectory, const Eigen::Vector3d &goal){

    Eigen::Vector2d traj_end(current_trajectory.back().x, current_trajectory.back().y);
    Eigen::Vector2d goal_res(0.0, 0.0);

    goal_res[0] = traj_end[0] - goal[0];
    goal_res[1] = traj_end[1] - goal[1];

    return goal_res.norm();

}

double Planner::get_speed_cost(const std::vector<State> &current_trajectory, const double target_vel){

    double cost = fabs( target_vel - current_trajectory.end()->velocity);

    return cost;
}

void Planner::run(){
    ros::Rate loop_rate(HZ);
    while(ros::ok()){
        
        if(!scan_updated_){
            ROS_ERROR("Waiting for scan ...");
            ros::Duration(1.0).sleep();
        }
        
        if(!pose_updated_){
            ROS_ERROR("Waiting for odom ...");
            ros::Duration(1.0).sleep();
        }
    
        Window dynamic_window = get_window(curr_vel_);

        //get goal in odom frame 
        // using tf, convert the goal to robot frame

        // GOAL x,y,heading in robot frame
        Eigen::Vector3d goal(10.0, 10.0, 0.0);

        std::vector<State> best_trajectory = best_dwa_selection(dynamic_window, goal);
        if(dwa_converged_) ROS_INFO("DWA Solution found ");

        show_best_trajectory(best_trajectory);


        ros::spinOnce();
        loop_rate.sleep();
    }

}
