#include "dwa_planner/planner.h"

Planner::Planner(ros::NodeHandle &nh):nh_(nh), tf2_listener_(tf_buffer_){

    scan_sub = nh_.subscribe("scan", 10, &Planner::scan_callback, this);
    odom_sub = nh_.subscribe("odom", 10, &Planner::odom_callback, this);
    trajectories_viz_pub = nh_.advertise<visualization_msgs::MarkerArray>("/dwa_trajs", 10);
    best_traj_viz_pub = nh_.advertise<visualization_msgs::MarkerArray>("/best_traj", 10);
    goal_vis_pub = nh_.advertise<visualization_msgs::Marker>("/goal", 10);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // change var names
    nh_.param("HZ", HZ, 20.0);
    nh_.param("TARGET_VELOCITY", TARGET_VELOCITY, 0.8);
    nh_.param("MAX_VELOCITY", MAX_VELOCITY, 1.0);
    nh_.param("MIN_VELOCITY", MIN_VELOCITY, 0.0);
    nh_.param("MAX_OMEGA", MAX_OMEGA, 0.8);
    nh_.param("MAX_ACCELERATION", MAX_ACCELERATION, 2.0);
    nh_.param("MAX_ANG_ACCELERATION", MAX_ANG_ACCELERATION, 3.0);
    //local_nh.param("MAX_DIST", MAX_DIST, {10.0});
    nh_.param("VELOCITY_RES", VELOCITY_RES, 0.05);
    nh_.param("OMEGA_RES", OMEGA_RES, 0.05);
    //local_nh.param("ANGLE_RESOLUTION", ANGLE_RESOLUTION, {0.2});
    nh_.param("SIM_TIME", SIM_TIME, 3.0);
    nh_.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, 1.0);
    nh_.param("SPEED_COST_GAIN", SPEED_COST_GAIN, 1.0);
    nh_.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, 1.0);
    nh_.param("GOAL_THRESHOLD", GOAL_THRESHOLD, 0.3);
    //local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {1.0});
    nh_.param("GOAL_X", GOAL_X, 0.0);
    nh_.param("GOAL_Y", GOAL_Y, 0.0);
    DT = 1.0 / HZ;
}

void Planner::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){

    local_obstacles_.clear();
    double angle = scan_msg->angle_min;
    for(auto r : scan_msg->ranges){

        //if(std::isinf(r) or std::isnan(r) or r > 20.0) continue;

        double x = r * cos(angle);
        double y = r * sin(angle);
        std::vector<double> hit_point = {x, y};
        local_obstacles_.push_back(hit_point);
        angle += scan_msg->angle_increment;
    }
    scan_updated_ = true;
    std::cout << "Local Obtacles updated\n";
}


void Planner::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg){
    
    pose_updated_ = true;
    curr_pose_ = odom_msg->pose;
    curr_vel_ = odom_msg->twist.twist;
}

Window Planner::get_window(){

    Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_OMEGA, MAX_OMEGA); 
    
    // edit presentation 
    window.min_vel = std::max((curr_vel_.linear.x - MAX_ACCELERATION*DT), MIN_VELOCITY);
    window.max_vel = std::min((curr_vel_.linear.x + MAX_ACCELERATION*DT), MAX_VELOCITY);
    window.min_omega = std::max((curr_vel_.angular.z - MAX_ANG_ACCELERATION*DT), -MAX_OMEGA);
    window.max_omega = std::min((curr_vel_.angular.z + MAX_ANG_ACCELERATION*DT),  MAX_OMEGA);

    return window;
}

std::vector<State> Planner::best_dwa_selection(const Window &window,const Eigen::Vector3d &goal){

    dwa_converged_ = false;
    double min_cost = DBL_MAX;

    std::vector<std::vector<State>> trajectory_table;
    std::vector<State> best_trajectory;

    // creating candidate trajectories
    for(double v=window.min_vel; v <= window.max_vel; v+=VELOCITY_RES){
        for(double w=window.min_omega; w<=window.max_omega; w+=OMEGA_RES){
             
            State state(0.0, 0.0, 0.0, curr_vel_.linear.x, curr_vel_.angular.z);
            std::vector<State> temp_traj;
            for(double t=0.0; t<=SIM_TIME; t+=DT){
                simulate_dynamics(state, v, w);
                temp_traj.push_back(state);
            }
            trajectory_table.push_back(temp_traj);

            // edit var names and method names
            double cost_on_goal = get_goal_cost(temp_traj, goal);
            double cost_on_speed = get_speed_cost(temp_traj, TARGET_VELOCITY);
            double cost_on_obs = get_obstacle_cost(temp_traj, local_obstacles_);

            double cumulative_cost = TO_GOAL_COST_GAIN*cost_on_goal + SPEED_COST_GAIN*cost_on_speed + OBSTACLE_COST_GAIN*cost_on_obs;

            if(cumulative_cost <= min_cost){
                min_cost = cumulative_cost;
                best_trajectory = temp_traj;
                dwa_converged_ = true;
            }
        }

    }

    //trajectory table populated
    show_dwa_trajectories(trajectory_table);
    ROS_INFO("Traj Table Created and Published");

    // updating minimum cost trajectory
    if(min_cost == DBL_MAX){
        std::vector<State> traj;
        State state(0.0, 0.0, 0.0, curr_vel_.linear.x, curr_vel_.angular.z);
        traj.push_back(state);
        dwa_converged_  = false;
        ROS_ERROR("INFINITE COST");
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

    traj.pose.position.x = 0.0;
    traj.pose.position.y = 0.0;
    traj.pose.position.z = 0;
    traj.pose.orientation.x = 0.0;
    traj.pose.orientation.y = 0.0;
    traj.pose.orientation.z = 0.0;
    traj.pose.orientation.w = 1.0;

    traj.scale.x = traj.scale.y = traj.scale.z = 0.01;
    traj.color.r = 1.0;
    traj.color.a = 0.5;
    
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
    trajectories_viz_pub.publish(traj_list);
    
}

void Planner::show_best_trajectory(const std::vector<State> &best_trajectory) {

    visualization_msgs::MarkerArray best_list;
    visualization_msgs::Marker best_traj;
    geometry_msgs::Point point;

    best_traj.header.frame_id = "base_footprint";
    best_traj.id = 1;
    best_traj.type = visualization_msgs::Marker::LINE_STRIP;
    best_traj.scale.x = best_traj.scale.y = 0.02;
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
    best_traj_viz_pub.publish(best_list); 
}

void Planner::show_goal(Eigen::Vector3d &goal) {
    
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "/base_footprint";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    //marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = goal[0];
    marker.pose.position.y = goal[1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    goal_vis_pub.publish(marker);
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

double Planner::get_obstacle_cost(const std::vector<State> &trajectory, const std::vector<std::vector<double>> &local_obs){
    double cost = 0.0;
    double min_dist = DBL_MAX;

    for(const auto& state : trajectory){
        for(const auto& scan_pt : local_obs){
            double current_distance = sqrt((state.x - scan_pt[0])*(state.x - scan_pt[0]) + 
                                                                (state.y - scan_pt[1])*(state.y - scan_pt[1]));
            if(current_distance <= 0.01){

                cost = DBL_MAX;
                return cost;
            }
            min_dist = std::min(min_dist, current_distance);
        }
    }

    if(min_dist == 0.0) return DBL_MAX;

    cost = 1.0 / min_dist;
    return cost;
}

void Planner::run(){
    ros::Rate loop_rate(HZ);
    while(ros::ok()){
        
        
        if(scan_updated_ == false){
            ROS_ERROR("Waiting for scan init...");
            ros::Duration(1.0).sleep();
        }
        
        if(pose_updated_ == false){
            ROS_ERROR("Waiting for odom init...");
            ros::Duration(1.0).sleep();
        }
        
        
        try{
            tf_base_to_odom_ = tf_buffer_.lookupTransform("base_footprint", "odom",ros::Time(0));
        }
        catch (tf::TransformException& ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }  

        //goal in odom
        double x_odom = GOAL_X;
        double y_odom = GOAL_Y;
        
        const auto translation = tf_base_to_odom_.transform.translation;
        const double yaw = tf::getYaw(tf_base_to_odom_.transform.rotation);

        double x_base = x_odom*cos(yaw) - y_odom*sin(yaw) + translation.x;
        double y_base = y_odom*sin(yaw) + y_odom*cos(yaw) + translation.y;
        
        Eigen::Vector3d goal_in_base(x_base, y_base, 0.0);
        show_goal(goal_in_base);

        Window dynamic_window = get_window();

        std::vector<State> best_trajectory = best_dwa_selection(dynamic_window, goal_in_base);
        
        if(!dwa_converged_) ROS_ERROR("DWA Solution NOT found "); // exit code

        show_best_trajectory(best_trajectory);

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = best_trajectory[0].velocity;
        vel_msg.angular.z = best_trajectory[0].omega;

        if(sqrt((x_base*x_base) + (y_base*y_base)) < GOAL_THRESHOLD){
            ROS_WARN("Reached.........");
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            cmd_vel_pub_.publish(vel_msg);
            return;
        }

        cmd_vel_pub_.publish(vel_msg);


        dwa_converged_ =false;
        ros::spinOnce();
        loop_rate.sleep();
    }

}
