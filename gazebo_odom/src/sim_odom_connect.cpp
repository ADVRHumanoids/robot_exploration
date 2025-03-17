#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "tf_conversions/tf_eigen.h"
#include <eigen_conversions/eigen_msg.h>
#include <gazebo_msgs/GetModelState.h>

#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "simOdomGazebo");
    ros::NodeHandle nh;
    
    Eigen::Affine3d temp = Eigen::Affine3d::Identity();
    
    tf::TransformBroadcaster br;
    tf::StampedTransform tr;
    tf::Transform transform;
    
    ros::ServiceClient get_model_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    std::string model_name, world_frame, base_frame;

    ros::param::get("sim_odom_connect/model_name", model_name);
    ros::param::get("sim_odom_connect/world_frame", world_frame);
    ros::param::get("sim_odom_connect/base_frame", base_frame);

    gazebo_msgs::GetModelState get_model_state_msg;
    get_model_state_msg.request.model_name = model_name;

    ros::Time now;
    ros::Rate loop_rate(200);

    std_msgs::Header prev_heder;


    while(ros::ok()){
                
        get_model_client.call(get_model_state_msg);

        if(get_model_state_msg.response.header.stamp.nsec != prev_heder.stamp.nsec ||
           get_model_state_msg.response.header.stamp.sec != prev_heder.stamp.sec){

            transform.setOrigin(tf::Vector3(
                                get_model_state_msg.response.pose.position.x,
                                get_model_state_msg.response.pose.position.y,
                                get_model_state_msg.response.pose.position.z- 0.90f));

            transform.setRotation(tf::Quaternion(
                                    get_model_state_msg.response.pose.orientation.x,
                                    get_model_state_msg.response.pose.orientation.y,
                                    get_model_state_msg.response.pose.orientation.z,
                                    get_model_state_msg.response.pose.orientation.w));

            br.sendTransform(tf::StampedTransform(transform, 
                                                ros::Time(get_model_state_msg.response.header.stamp.sec, 
                                                            get_model_state_msg.response.header.stamp.nsec),
                                                world_frame,
                                                base_frame));
        }

        prev_heder = get_model_state_msg.response.header;
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
