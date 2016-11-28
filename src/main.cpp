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


double r = 1.0;
double theta;
double count=0.0;
double wn= 1.0;
ros::ServiceClient landing_client;
bool land_detected = false;





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

        pose.pose.position.x = r*sin(theta);//+ home_location.pose.pose.position.x;
        pose.pose.position.y =  r*cos(theta); //home_location.pose.pose.position.y +;
        pose.pose.position.z = 1.5;
        count++;
       
        
        local_pos_pub.publish(pose);
        ROS_INFO("Sending a POSE x: %f y: %f z: %f\n", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        ROS_INFO("GPS POSE x: %f y: %f z: %f\n", current_location.pose.pose.position.x, current_location.pose.pose.position.y, current_location.pose.pose.position.z); 
      ROS_INFO("HOME POSE x: %f y: %f z: %f\n", home_location.pose.pose.position.x, home_location.pose.pose.position.y, home_location.pose.pose.position.z); 
            ROS_INFO("LOCAL POSE x: %f y: %f z: %f\n", current_location.pose.pose.position.x -home_location.pose.pose.position.x, current_location.pose.pose.position.y - home_location.pose.pose.position.y, current_location.pose.pose.position.z); 
      
        }
          ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
