#include "dwa_planner/planner.h"

Planner::Planner(ros::NodeHandle &nh):nh_(nh){

    scan_sub = nh_.subscribe("/scan", 1, &Planner::scan_callback, this);
    odom_sub = nh_.subscribe("/odom", 10, &Planner::odom_callback, this);
    trajectories_viz_pub = nh_.advertise<visualization_msgs::MarkerArray>("/dwa_trajs", 10);
    //test_pub = nh_.advertise<visualization_msgs::Marker>("/test_vis", 10);
    points_pub = nh_.advertise<visualization_msgs::Marker>("/test_vis", 10);

    nh_.param("HZ", HZ, 20.0);
    //nh_.param("ROBOT_FRAME", ROBOT_FRAME, {"base_footprint"});
    nh_.param("TARGET_VELOCITY", TARGET_VELOCITY, 0.8);
    nh_.param("MAX_VELOCITY", MAX_VELOCITY, 1.0);
    nh_.param("MIN_VELOCITY", MIN_VELOCITY, 0.0);
    nh_.param("MAX_OMEGA", MAX_OMEGA, 0.8);
    nh_.param("MAX_ACCELERATION", MAX_ACCELERATION, 1.0);
    nh_.param("MAX_D_OMEGA", MAX_D_OMEGA, 2.0);
    //local_nh.param("MAX_DIST", MAX_DIST, {10.0});
    nh_.param("VELOCITY_RES", VELOCITY_RES, 0.05);
    nh_.param("OMEGA_RES", OMEGA_RES, 0.05);
    //local_nh.param("ANGLE_RESOLUTION", ANGLE_RESOLUTION, {0.2});
    nh_.param("SIM_TIME", SIM_TIME, 3.0);
    //local_nh.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, {1.0});
    //local_nh.param("SPEED_COST_GAIN", SPEED_COST_GAIN, {1.0});
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
}

Window Planner::get_window(const geometry_msgs::Twist& curr_vel){

    Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_OMEGA, MAX_OMEGA);

    window.min_vel = std::max((curr_vel.linear.x - MAX_ACCELERATION*DT), MIN_VELOCITY);
    window.max_vel = std::min((curr_vel.linear.x + MAX_ACCELERATION*DT), MAX_VELOCITY);
    window.min_omega = std::max((curr_vel.angular.z - MAX_D_OMEGA*DT), -MAX_OMEGA);
    window.max_omega = std::min((curr_vel.angular.z + MAX_D_OMEGA*DT),  MAX_OMEGA);
    /// MAX D OMEGA is the angular acceleration alpha ??

    return window;
}

void Planner::best_dwa_selection(const Window &window,const Eigen::Vector3d &goal){

    //change names
    //double min_cost = INT32_MIN;
    //double min_obs_cost = min_cost;
    //double min_goal_cost = min_cost;
    //double min_speed_cost = min_cost;

    std::vector<std::vector<State>> trajectory_table;
    //std::vector<State> best_trajectory;

    
    for(double v=window.min_vel; v <= window.max_vel; v+=VELOCITY_RES){
        for(double w=window.min_omega; w<=window.max_omega; w+=OMEGA_RES){
             
            State state(curr_pose_.pose.position.x, curr_pose_.pose.position.y, tf::getYaw(curr_pose_.pose.orientation), curr_vel_.linear.x, curr_vel_.angular.z);
            std::vector<State> temp_traj;
            for(double t=0; t<=SIM_TIME; t+=DT){
                simulate_dynamics(state, v, w);
                temp_traj.push_back(state);
            }
            trajectory_table.push_back(temp_traj);
            //double cost_on_goal = get_goal_cost(traj, goal);
            //double cost_on_speed = get_speed_cost(traj, TARGET_VELOCITY);
            //double cost_on_obs = get_obstacle_cost(traj, obs_list);
            //double cumulative_cost = TO_GOAL_COST_GAIN*to_goal_cost + SPEED_COST_GAIN*speed_cost + OBSTACLE_COST_GAIN*obstacle_cost;
        }

    }

    //trajectory table populated
    //visulaize
    //std::cout << "max vel : " << window.max_vel << " min vel : " << window.min_vel << "\n";
    //std::cout << "max omega : " << window.max_omega << " min omega : " << window.min_omega << "\n";

    show_dwa_trajectories(trajectory_table);
    //visualize_trajectories(trajectory_table);

    return;
    //std::cout << "SIZE : "  << trajectory_table.size() << trajectory_table[0].size() << "\n";

}
void Planner::visualize_trajectories(const std::vector<std::vector<State>>& trajectory_table)
{
    visualization_msgs::Marker Blob;

    Blob.header.frame_id = "/base_footprint";
    Blob.header.stamp = ros::Time::now();

    
    Blob.type = visualization_msgs::Marker::SPHERE;
    Blob.action = visualization_msgs::Marker::ADD;

    Blob.scale.x = Blob.scale.y = Blob.scale.z = 0.01;
    Blob.color.r = 1.0;
    Blob.color.a = 1.0;

    for (int i=0; i< trajectory_table.size(); i++) {
        Blob.id = count_++;

        for (int j = 0; j < trajectory_table[i].size(); j++) {
            Blob.pose.position.x =  trajectory_table[i][j].x;
            Blob.pose.position.y = trajectory_table[i][j].y;
            Blob.pose.orientation.x = 0.0;
            Blob.pose.orientation.y = 0.0;
            Blob.pose.orientation.z = 0.0;
            Blob.pose.orientation.w = 1.0;
            points_pub.publish(Blob);

        }
        
    }

    std::cout << "viz pubbing points" << "\n";

}

void Planner::show_dwa_trajectories(const std::vector<std::vector<State>> &trajectory_table) {

    visualization_msgs::MarkerArray traj_list;
    visualization_msgs::Marker traj;
    geometry_msgs::Point p;

    traj.header.frame_id = "/base_footprint";
    traj.header.stamp = ros::Time::now();

    traj.id = count_++;
    traj.type = visualization_msgs::Marker::LINE_STRIP;
    traj.action = visualization_msgs::Marker::ADD;

    traj.pose.position.x = 0;
    traj.pose.position.y = 0;
    traj.pose.position.z = 0;
    traj.pose.orientation.x = 0.0;
    traj.pose.orientation.y = 0.0;
    traj.pose.orientation.z = 0.0;
    traj.pose.orientation.w = 1.0;

    traj.scale.x = traj.scale.y = traj.scale.z = 0.01;
    
    traj.color.r = 1.0;
    traj.color.a = 1.0;

    ROS_INFO("publishing traj viz");
    ROS_INFO("size table : %d", trajectory_table.size());
    ROS_INFO("traj size : %d", trajectory_table[0].size());
    
    for (int i=0; i< trajectory_table.size(); i++) {

        traj.id += 1;
        traj.color.b += 0.1;
        traj.points.clear();
        
        for (int j = 0; j < trajectory_table[i].size(); j++) {
            p.x = trajectory_table[i][j].x;
            p.y = trajectory_table[i][j].y;
            traj.points.push_back(p);
            //std::cout << trajectory_table[i][j].x << "," << trajectory_table[i][j].y << " ";
        }
        traj_list.markers.push_back(traj);
        //std::cout << "\n";
    }
    std::cout << "vis pubbing" << "\n";
    trajectories_viz_pub.publish(traj_list);
    
}

void Planner::simulate_dynamics(State &state, double velocity, double omega){
    state.heading += omega*DT;
    state.x += velocity*cos(state.heading)*DT;
    state.y += velocity*sin(state.heading)*DT;
    state.velocity = velocity;
    state.omega = omega;
}

/*
Planner::get_goal_cost(const vector<State> &trajectory, const Eigen::Vector3d &goal){

    Eigen::Vector2d traj_end(trajectory.back().x, trajectory.back().y);
    Eigen::Vector2d goal_res(0.0, 0.0);

    goal_res[0] = traj_end[0] - goal0];
    goal_res[1] = traj_end[1] - goal[1];

    return goal_res.norm();

}

Planner::get_speed_cost(const vector<State> &trajectory, const double target_vel){

    double cost = fabs( target_vel - trajectory.end().velocity);

    return cost;

}
*/

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

        // GOAL x,y,heading in robot frame
        Eigen::Vector3d goal(10.0, 10.0, 0.0);

        best_dwa_selection(dynamic_window, goal);

        ros::spinOnce();
        loop_rate.sleep();
    }

}
