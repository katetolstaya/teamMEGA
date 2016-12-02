/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32MultiArray.h>

#define PATH_SIZE 10

mavros_msgs::State current_state;

nav_msgs::Odometry current_location;
nav_msgs::Odometry home_location;

int init = 0;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;

}

void gps_cb(const nav_msgs::Odometry::ConstPtr& msg) {
     current_location = *msg;
        if (init == 0) {
        home_location = current_location;
        init ++;
        }
     
}


int curr_path_size = 0;
double r = 1.0;
double theta;
int count=0;
int path_idx = 0;
double wn= 1.0;
float path[PATH_SIZE][2];
ros::ServiceClient landing_client;
ros::Subscriber path_listener;
bool land_detected = false;


void path_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {

       // float temp[2*PATH_SIZE];
        //temp = msg->data;
        curr_path_size = msg->data.size()/2;
        if (!msg->data.empty()) {       
                for(int i = 0; i < msg->data.size()/2; i++) {
                        path[i][0] = msg->data.at(i);
                        path[i][1] = msg->data.at(msg->data.size()/2+i);
                 }
              
        }
}


bool land(mavros_msgs::CommandTOL::Request & req,
                mavros_msgs::CommandTOL::Response &res) 
{
        mavros_msgs::CommandTOL land_cmd;
           land_cmd.request.altitude = 1.0;
        ROS_INFO("Land detected");
           //land_cmd.request.yaw = 0.0;
          // land_cmd.request.latitude =
           
           land_detected = true;
           
           if( landing_client.call(land_cmd) && land_cmd.response.success) {
                ROS_INFO("Vehicle landed");
                }
         res.success = true;


}


bool customLand(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
     
        mavros_msgs::CommandTOL land_cmd;
        land_cmd.request.altitude = 1.0;
        land_cmd.request.latitude = 0.0;
        land_cmd.request.longitude = 0.0;
        land_cmd.request.yaw = 0.0;
        land_cmd.request.min_pitch = 0.0;
//takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        land_detected = true;
        ROS_INFO("Land detected");
        if( landing_client.call(land_cmd) && land_cmd.response.success) {
        ROS_INFO("Vehicle landed");
        }

        res.success = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
     landing_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
     path_listener = nh.subscribe<std_msgs::Float32MultiArray>
            ("mavros_custom_path",10, path_cb);
    ros::ServiceServer landing_listener = nh.advertiseService("mavros_custom_msgs",customLand);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber gps_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local",10, gps_cb);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ROS_INFO("Attempting to Connect to FCU\n");
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
        ROS_INFO("Connected\n");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
   for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
        ROS_INFO("Debug2\n");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";  
  //    offb_set_mode.request.base_mode = 6;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now();

    while(ros::ok()){
    
    ROS_INFO("PATH size %d\n", curr_path_size);
    for(int i = 0; i < curr_path_size; i++) {
        ROS_INFO("x: %f, y: %f\n", path[i][0], path[i][1]);

    }
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    ros::Time start_time = ros::Time::now();
                }
                last_request = ros::Time::now();
            }
        } 
       // if (ros::Time::now() - start_time > ros::Duration(30.0)) {
        if (0){
           mavros_msgs::CommandTOL land_cmd;
           land_cmd.request.altitude = 2.0;
           //land_cmd.request.yaw = 0.0;
          // land_cmd.request.latitude =
           
           if( landing_client.call(land_cmd) && land_cmd.response.success) {
                ROS_INFO("Vehicle landed");
                break;
                }
        }
        
        if (land_detected) {
               // ROS_INFO("Vehicle Landing\n");
 ROS_INFO("LOCAL POSE x: %f y: %f z: %f\n", current_location.pose.pose.position.x -home_location.pose.pose.position.x, current_location.pose.pose.position.y - home_location.pose.pose.position.y, current_location.pose.pose.position.z); 
        if (current_location.pose.pose.position.z < 0.05)
                break;
                
        }
        else {
 
        theta = wn*count*0.05;
        if (curr_path_size ==0) {
        pose.pose.position.x = r*sin(theta);//+ home_location.pose.pose.position.x;
        pose.pose.position.y =  r*cos(theta); //home_location.pose.pose.position.y +;
        pose.pose.position.z = 1.5;
        count++;
        }
        
        // Hover in circle, unless path is present
        if (curr_path_size >=1 && path_idx < curr_path_size) {
           
                pose.pose.position.x = path[path_idx][0];
                pose.pose.position.y = path[path_idx][1];  
                pose.pose.position.z = 1.5;
                path_idx++;
        }
       
        
        local_pos_pub.publish(pose);
        ROS_INFO("Sending a POSE x: %f y: %f z: %f\n", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
       
            ROS_INFO("LOCAL POSE x: %f y: %f z: %f\n", current_location.pose.pose.position.x -home_location.pose.pose.position.x, current_location.pose.pose.position.y - home_location.pose.pose.position.y, current_location.pose.pose.position.z); 
      
        }
          ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
